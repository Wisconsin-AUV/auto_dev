import faulthandler
import socket
import os
import time
import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pyzed.sl as sl
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

faulthandler.enable()  # print Python stack on segfault

LAPTOP_ADDR = ('192.168.137.1', 5000)
STREAM_SIZE = (640, 360)
JPEG_QUALITY = 70

GREEN = (0, 255, 0)
RED = (0, 0, 255)


class YoloDetectorZedNative(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.zed = sl.Camera()

        # ---- Parameters ----
        self.svo_file = self.declare_parameter('svo_file', '').value
        self.yolo = self.declare_parameter('yolo', True).value
        self.vslam = self.declare_parameter('vslam', True).value
        self.model_name = self.declare_parameter('model', 'gate_bins_v1.onnx').value
        self.stream = self.declare_parameter('stream', True).value
        self.get_logger().info(
            f'svo_file="{self.svo_file}" yolo={self.yolo} vslam={self.vslam} '
            f'model={self.model_name} stream={self.stream}')

        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        # ROS convention (REP-103): x forward, y left, z up.
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_maximum_distance = 10.0
        init_params.camera_fps = 15
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        if self.svo_file:
            init_params.set_from_svo_file(self.svo_file)
            init_params.svo_real_time_mode = True

        self.output_path = "data/"
        self.recordingParameters = sl.RecordingParameters()
        self.recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
        os.makedirs(self.output_path, exist_ok=True)
        self.recordingParameters.video_filename = os.path.join(
            self.output_path, time.strftime('run_%Y%m%d_%H%M%S.svo2'))

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Camera Open Error: {err}')
        res = self.zed.get_camera_information().camera_configuration.resolution
        self.get_logger().info(f'{res.width}x{res.height}')
        # Scale factors from camera image -> stream image (boxes drawn after resize).
        self.sx = STREAM_SIZE[0] / res.width
        self.sy = STREAM_SIZE[1] / res.height

        # Positional tracking (VSLAM). Must be enabled BEFORE object detection.
        if self.vslam:
            tracking_params = sl.PositionalTrackingParameters()
            tracking_params.enable_imu_fusion = True
            tracking_params.set_floor_as_origin = False
            tracking_params.enable_area_memory = False
            tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
            err = self.zed.enable_positional_tracking(tracking_params)
            if err != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f'Positional Tracking Enable Error: {err}')
        else:
            self.get_logger().info('VSLAM disabled: no odometry published')

        # Don't write a duplicate .svo2 when playing a recording.
        if not self.svo_file:
            err = self.zed.enable_recording(self.recordingParameters)
            if err != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f'Recording Enable Error: {err}')
        self.pose = sl.Pose()
        self.last_track_state = None

        # ---- YOLO (only loaded when yolo:=true) ----
        if self.yolo:
            package_share = get_package_share_directory('wauv_perception')
            onnx_path = os.path.join(package_share, 'models', self.model_name)
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

            self.od_runtime_params = sl.CustomObjectDetectionRuntimeParameters()
            self.od_runtime_params.object_detection_properties.detection_confidence_threshold = 50
            self.objects = sl.Objects()
            self.gate_pub = self.create_publisher(Vector3, 'vision/gate_raw', 10)
        else:
            self.get_logger().info('YOLO disabled: odometry + stream only')

        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA

        self.image = sl.Mat()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.vslam:
            self.odom_pub = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def publish_pose(self):
        state = self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        if state != self.last_track_state:
            self.get_logger().info(f'Tracking state: {state}')
            self.last_track_state = state
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            return

        t = self.pose.get_translation(sl.Translation()).get()
        q = self.pose.get_orientation(sl.Orientation()).get()

        msg = Odometry()
        msg.header.stamp = Time(nanoseconds=self.pose.timestamp.get_nanoseconds()).to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, t)
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = map(float, q)
        msg.pose.covariance = [float(c) for c in self.pose.pose_covariance]

        tw = self.pose.twist
        (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
         msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z) = map(float, tw)
        msg.twist.covariance = [float(c) for c in self.pose.twist_covariance]

        self.odom_pub.publish(msg)

    def run_yolo(self):
        """Run detection, publish best valid detection. Returns list of
        (x1, y1, x2, y2, color, label) in full-res image pixels for drawing."""
        draws = []
        status = self.zed.retrieve_custom_objects(self.objects, self.od_runtime_params)
        if status != sl.ERROR_CODE.SUCCESS:
            return draws

        best = None
        for obj in self.objects.object_list:
            bb = obj.bounding_box_2d
            x1, y1 = bb[0][0], bb[0][1]
            x2, y2 = bb[2][0], bb[2][1]

            x, y, z = (float(v) for v in obj.position)
            class_id = obj.raw_label
            score = obj.confidence

            if not np.isfinite(x) or x <= 0.0:
                # Detected but no valid depth -> red box, not published.
                draws.append((x1, y1, x2, y2, RED, f'{class_id} {score:.0f} no depth'))
                continue

            if best is None or score > best[0]:
                best = (score, x, y, z)

            self.get_logger().info(f'{class_id} {score:.2f}: x={x:.2f} y={y:.2f} z={z:.2f}')
            draws.append((x1, y1, x2, y2, GREEN,
                          f'{class_id} {score:.0f} | {x:.2f} {y:.2f} {z:.2f}m'))

        if best is not None:
            msg = Vector3()
            _, msg.x, msg.y, msg.z = best
            self.gate_pub.publish(msg)
        return draws

    def send_stream(self, draws):
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
        frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)
        # Resize first, then draw, so boxes/text stay crisp at stream resolution.
        frame = cv2.resize(frame, STREAM_SIZE, interpolation=cv2.INTER_AREA)

        for x1, y1, x2, y2, color, label in draws:
            p1 = (int(x1 * self.sx), int(y1 * self.sy))
            p2 = (int(x2 * self.sx), int(y2 * self.sy))
            cv2.rectangle(frame, p1, p2, color, 2)
            cv2.putText(frame, label, (p1[0], max(12, p1[1] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return
        if len(buf) >= 65000:
            self.get_logger().warn(f'frame too big: {len(buf)} bytes, dropped')
            return
        try:
            self.sock.sendto(buf.tobytes(), LAPTOP_ADDR)
        except OSError:
            pass

    def timer_callback(self):
        err = self.zed.grab(self.runtime_params)
        if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            # Loop the recording and reset tracking so odometry doesn't jump.
            self.get_logger().info('End of SVO, looping to frame 0')
            self.zed.set_svo_position(0)
            if self.vslam:
                self.zed.reset_positional_tracking(sl.Transform())
            return
        if err != sl.ERROR_CODE.SUCCESS:
            return

        if self.vslam:
            self.publish_pose()

        draws = self.run_yolo() if self.yolo else []

        if self.stream:
            self.send_stream(draws)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorZedNative()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.svo_file:
            node.zed.disable_recording()
        if node.yolo:
            node.zed.disable_object_detection()
        if node.vslam:
            node.zed.disable_positional_tracking()
        node.zed.close()
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()