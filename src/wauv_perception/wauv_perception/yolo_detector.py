"""YOLO detector on the ZED camera.

Runs a YOLO .onnx model inside the ZED SDK (TensorRT) and publishes:
  vision/gate_raw         Vector3   position of the most confident detection (meters)
  /mavros/odometry/out    Odometry  ZED visual odometry pose

Positions use the ROS convention (REP-103): x forward, y left, z up.

Pick the model with the `model` parameter, for example:
  ros2 run wauv_perception yolo_detector --ros-args -p model:=gate_bins_v1.onnx

gate_bins_v1.onnx is the combined gate + bins model. Its classes are:
  0 gate_sos, 1 gate_hammer, 2 bins, 3 bin_blood, 4 bin_fire
"""
import os

import pyzed.sl as sl
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time

CONFIDENCE_PERCENT = 30   # ZED detection threshold, 0-100
MAX_DEPTH_M = 10.0


class YoloDetectorZedNative(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.zed = sl.Camera()
        self.open_camera()
        self.enable_tracking()
        self.enable_detection(self.declare_parameter('model', 'krill_gate_v1.onnx').value)

        self.pose = sl.Pose()
        self.last_track_state = None
        self.objects = sl.Objects()

        self.gate_pub = self.create_publisher(Vector3, 'vision/gate_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        self.create_timer(0.05, self.timer_callback)  # 20 Hz

    # ---------- setup ----------

    def open_camera(self):
        params = sl.InitParameters()
        params.depth_mode = sl.DEPTH_MODE.NEURAL
        params.coordinate_units = sl.UNIT.METER
        params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        params.depth_maximum_distance = MAX_DEPTH_M
        params.camera_fps = 15
        check(self.zed.open(params), 'Camera open')

    def enable_tracking(self):
        """Positional tracking. Must be enabled before object detection."""
        params = sl.PositionalTrackingParameters()
        params.enable_imu_fusion = True       # the ZED X Mini has an IMU
        params.set_floor_as_origin = False    # no floor plane underwater
        params.enable_area_memory = False     # no loop closure, so no pose jumps
        params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        check(self.zed.enable_positional_tracking(params), 'Positional tracking')

    def enable_detection(self, model_name):
        onnx_path = os.path.join(
            get_package_share_directory('wauv_perception'), 'models', model_name)
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(f'ONNX file not found at: {onnx_path}')

        self.get_logger().info(f'Loading {onnx_path} into ZED TensorRT...')
        params = sl.ObjectDetectionParameters()
        params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_YOLOLIKE_BOX_OBJECTS
        params.custom_onnx_file = onnx_path
        params.enable_tracking = False
        params.enable_segmentation = False
        check(self.zed.enable_object_detection(params), 'Object detection')
        self.get_logger().info('ZED YOLO TensorRT engine ready')

        # Detections stay camera-relative instead of world-frame.
        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA
        self.od_params = sl.CustomObjectDetectionRuntimeParameters()
        self.od_params.object_detection_properties.detection_confidence_threshold = \
            CONFIDENCE_PERCENT

    # ---------- every frame ----------

    def timer_callback(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return
        self.publish_pose()
        self.publish_best_detection()

    def publish_best_detection(self):
        """Publish the position of the highest-confidence detection (any class)."""
        if self.zed.retrieve_custom_objects(self.objects, self.od_params) != sl.ERROR_CODE.SUCCESS:
            return

        best = None
        for obj in self.objects.object_list:
            x, y, z = (float(v) for v in obj.position)
            self.get_logger().info(
                f'{obj.raw_label} {obj.confidence:.2f}: x={x:.2f} y={y:.2f} z={z:.2f}')
            if best is None or obj.confidence > best[0]:
                best = (obj.confidence, x, y, z)

        if best is not None:
            _, x, y, z = best
            self.gate_pub.publish(Vector3(x=x, y=y, z=z))

    def publish_pose(self):
        state = self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        if state != self.last_track_state:
            self.get_logger().info(f'Tracking state: {state}')
            self.last_track_state = state
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            return  # SEARCHING / OFF / FPS_TOO_LOW: covariance is not meaningful

        msg = Odometry()
        msg.header.stamp = Time(nanoseconds=self.pose.timestamp.get_nanoseconds()).to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        p = msg.pose.pose
        p.position.x, p.position.y, p.position.z = (
            float(v) for v in self.pose.get_translation(sl.Translation()).get())
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
            float(v) for v in self.pose.get_orientation(sl.Orientation()).get())
        # 6x6 row-major, order [x, y, z, rot_x, rot_y, rot_z] (same as ROS)
        msg.pose.covariance = [float(c) for c in self.pose.pose_covariance]

        t = msg.twist.twist
        (t.linear.x, t.linear.y, t.linear.z,
         t.angular.x, t.angular.y, t.angular.z) = (float(v) for v in self.pose.twist)
        msg.twist.covariance = [float(c) for c in self.pose.twist_covariance]

        self.odom_pub.publish(msg)

    def shutdown(self):
        self.zed.disable_object_detection()
        self.zed.disable_positional_tracking()
        self.zed.close()


def check(err, what):
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f'{what} error: {err}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorZedNative()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
