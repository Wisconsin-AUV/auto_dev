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

LAPTOP_ADDR = ('192.168.137.1', 5000)  # NEW
STREAM_SIZE = (640, 360)               # NEW
JPEG_QUALITY = 70                      # NEW


class YoloDetectorZedNative(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.zed = sl.Camera()
        # CHANGED: added for recording playback. Empty = live camera (same as before);
        # a path = play that .svo2 instead. Declared here because zed.open() needs it.
        self.svo_file = self.declare_parameter('svo_file', '').value

        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        # ROS convention (REP-103): x forward, y left, z up.
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

        init_params.depth_maximum_distance = 10.0
        init_params.camera_fps = 15
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        # CHANGED: added. Open the recording instead of the camera (per the Stereolabs
        # docs); depth, tracking and YOLO are recomputed from it. Real-time mode plays it
        # at the recorded 15 fps instead of as fast as the 20 Hz timer pulls frames.
        if self.svo_file:
            init_params.set_from_svo_file(self.svo_file)
            init_params.svo_real_time_mode = True
        self.output_path = "data/"
        self.recordingParameters = sl.RecordingParameters()
        self.recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
        # Optional (ZED SDK >= 5.3): favor encoding speed over quality
        os.makedirs(self.output_path, exist_ok=True)
        self.recordingParameters.video_filename = os.path.join(
            self.output_path, time.strftime('run_%Y%m%d_%H%M%S.svo2'))

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Camera Open Error: {err}')
        res = self.zed.get_camera_information().camera_configuration.resolution
        self.get_logger().info(f'{res.width}x{res.height}')
        # Positional tracking. It must be enabled BEFORE object detection.
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_imu_fusion = True
        tracking_params.set_floor_as_origin = False
        tracking_params.enable_area_memory = False
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        err = self.zed.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Positional Tracking Enable Error: {err}')
        # CHANGED: indented under "if not self.svo_file" so playing a recording
        # doesn't write a duplicate .svo2 copy of it into data/.
        if not self.svo_file:
            err = self.zed.enable_recording(self.recordingParameters)
            if err != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f'Recording Enable Error: {err}')
        self.pose = sl.Pose()
        self.last_track_state = None

        # CHANGED: uncommented the YOLO setup below. Default model krill_gate_v1 -> gate_bins_v1
        # (combined gate + bins model, already on this branch). Override with -p model:=<file>.onnx
        model_name = self.declare_parameter('model', 'gate_bins_v1.onnx').value
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
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA
        # CHANGED: uncommented; confidence 30 -> 50. 50 gave zero false positives on
        # pool footage with no props; 30 is where false hits start to get through.
        self.od_runtime_params = sl.CustomObjectDetectionRuntimeParameters()
        self.od_runtime_params.object_detection_properties.detection_confidence_threshold = 50

        self.image = sl.Mat()
        # self.depth = sl.Mat()  # NEW
        # CHANGED: uncommented; retrieve_custom_objects() fills this each frame.
        self.objects = sl.Objects()

        # NEW: streaming + recording toggles
        self.stream = self.declare_parameter('stream', True).value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


        self.gate_pub = self.create_publisher(Vector3, 'vision/gate_raw', 10)
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

    def timer_callback(self):
        # CHANGED: grab result is kept so the end of a recording can be detected.
        # At the end, loop back to frame 0 (as in the Stereolabs example) and reset
        # tracking so odometry doesn't jump. A live camera never hits this branch.
        err = self.zed.grab(self.runtime_params)
        if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            self.zed.set_svo_position(0)
            self.zed.reset_positional_tracking(sl.Transform())
            return
        if err != sl.ERROR_CODE.SUCCESS:
            return

        self.publish_pose()

        # CHANGED: uncommented; gets this frame's YOLO detections with 3D positions.
        status = self.zed.retrieve_custom_objects(self.objects, self.od_runtime_params)

        self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
        frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)

        # CHANGED: uncommented through gate_pub.publish(); draws boxes (red = no depth)
        # and publishes the most confident detection with valid depth. Logic unchanged.
        best = None

        if status == sl.ERROR_CODE.SUCCESS:
            for obj in self.objects.object_list:
                bbox_2d = obj.bounding_box_2d
                x1, y1 = int(bbox_2d[0][0]), int(bbox_2d[0][1])
                x2, y2 = int(bbox_2d[2][0]), int(bbox_2d[2][1])

                pos = obj.position
                x, y, z = float(pos[0]), float(pos[1]), float(pos[2])

                class_id = obj.raw_label
                score = obj.confidence

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

        # CHANGED: re-enabled; streams the annotated frame
        if self.stream:
            stream_frame = cv2.resize(frame, STREAM_SIZE, interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', stream_frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                if len(buf) >= 65000:
                    self.get_logger().warn(f'frame too big: {len(buf)} bytes, dropped')
                else:
                    try:
                        self.sock.sendto(buf.tobytes(), LAPTOP_ADDR)
                    except OSError:
                        pass


def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetectorZedNative()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.zed.disable_recording()
        yolo_detector.zed.disable_positional_tracking()
        yolo_detector.zed.close()
        yolo_detector.sock.close()
        yolo_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
