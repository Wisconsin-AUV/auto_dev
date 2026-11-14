import os
import socket

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pyzed.sl as sl
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # NEW
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry  # NEW
from ament_index_python.packages import get_package_share_directory


class YoloDetectorZedNative(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        # CHANGED: ROS convention (REP-103): x forward, y left, z up.
        # This applies to the pose AND to obj.position.
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

        init_params.depth_maximum_distance = 10.0
        init_params.camera_fps = 15

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Camera Open Error: {err}')

        # NEW: Positional tracking. It must be enabled BEFORE object detection.
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_imu_fusion = True      # ZED X Mini has an IMU; keep on
        tracking_params.set_floor_as_origin = False   # no floor plane underwater
        tracking_params.enable_area_memory = False    # no loop closure; avoids pose jumps
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        err = self.zed.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Positional Tracking Enable Error: {err}')
        self.pose = sl.Pose()
        self.last_track_state = None

        model_name = self.declare_parameter('model', 'krill_gate_v1.onnx').value
        package_share = get_package_share_directory('wauv_perception')
        onnx_path = os.path.join(package_share, 'models', model_name)
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(f'ONNX file not found at: {onnx_path}')

        self.get_logger().info(f'Loading {onnx_path} into ZED TensorRT...')
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_YOLOLIKE_BOX_OBJECTS
        obj_param.custom_onnx_file = onnx_path
        obj_param.enable_tracking = False
        obj_param.enable_segmentation = False

        err = self.zed.enable_object_detection(obj_param)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Object Detection Enable Error: {err}')
        self.get_logger().info('ZED YOLO TensorRT Engine Ready!')

        self.runtime_params = sl.RuntimeParameters()
        # NEW: keep detections camera-relative. Otherwise they would come out in the world frame.
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA
        self.od_runtime_params = sl.CustomObjectDetectionRuntimeParameters()
        self.od_runtime_params.object_detection_properties.detection_confidence_threshold = 30

        self.image = sl.Mat()
        self.objects = sl.Objects()
        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # CHANGED: gate position in the camera frame (x fwd, y left, z up), meters
        self.gate_pub = self.create_publisher(Vector3, 'vision/gate_raw', 10)
        # NEW
        self.odom_pub = self.create_publisher(Odometry, '/mavros/odometry/out', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    # NEW
    def publish_pose(self):
        state = self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        if state != self.last_track_state:
            self.get_logger().info(f'Tracking state: {state}')
            self.last_track_state = state
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            return  # SEARCHING / OFF / FPS_TOO_LOW: covariance is not meaningful

        t = self.pose.get_translation(sl.Translation()).get()  # [x, y, z]
        q = self.pose.get_orientation(sl.Orientation()).get()  # [ox, oy, oz, ow]

        msg = Odometry()
        msg.header.stamp = Time(nanoseconds=self.pose.timestamp.get_nanoseconds()).to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, t)
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = map(float, q)

        # 36 floats, row-major 6x6, order [x, y, z, rot_x, rot_y, rot_z] (same order as ROS)
        msg.pose.covariance = [float(c) for c in self.pose.pose_covariance]

        # Twist and its covariance (SDK >= 4.0)
        tw = self.pose.twist
        (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
         msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z) = map(float, tw)
        msg.twist.covariance = [float(c) for c in self.pose.twist_covariance]

        self.odom_pub.publish(msg)

    def timer_callback(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        self.publish_pose()  # NEW

        status = self.zed.retrieve_custom_objects(self.objects, self.od_runtime_params)

        self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
        frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)

        best = None

        if status == sl.ERROR_CODE.SUCCESS:
            for obj in self.objects.object_list:
                if obj.raw_label != 0:
                    continue
                bbox_2d = obj.bounding_box_2d
                x1, y1 = int(bbox_2d[0][0]), int(bbox_2d[0][1])
                x2, y2 = int(bbox_2d[2][0]), int(bbox_2d[2][1])

                # 3D position [x fwd, y left, z up] in meters
                pos = obj.position
                x, y, z = float(pos[0]), float(pos[1]), float(pos[2])

                class_id = obj.raw_label
                score = obj.confidence

                # CHANGED: forward is now x, not z
                if not np.isfinite(x) or x <= 0.0:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), thickness=2)
                    continue

                if best is None or score > best[0]:
                    best = (score, x, y, z)

                self.get_logger().info(f'{class_id} {score:.2f}: x={x:.2f} y={y:.2f} z={z:.2f}')
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)
                cv2.putText(
                    frame, f'{x:.2f} {y:.2f} {z:.2f}m', (x1, max(24, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA
                )

        if best is not None:
            msg = Vector3()
            _, msg.x, msg.y, msg.z = best
            self.gate_pub.publish(msg)

        # stream_frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_LINEAR)
        # ok, buf = cv2.imencode(".jpg", stream_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        # if ok:
        #     if len(buf) >= 65000:
        #         self.get_logger().warn(f"frame too big: {len(buf)} bytes, dropped")
        #     else:
        #         try:
        #             self.sock.sendto(buf.tobytes(), ("192.168.137.1", 5000))
        #         except OSError:
        #             pass


def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetectorZedNative()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.zed.disable_object_detection()
        yolo_detector.zed.disable_positional_tracking()  # NEW
        yolo_detector.zed.close()
        yolo_detector.sock.close()
        yolo_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()