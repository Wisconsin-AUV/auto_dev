"""
Filename: perception_node.py
Description: ROS2 node that finds competition objects in the front colour camera, gets
             their distance from the depth camera, and publishes them for the behavior tree.

Output: vision_msgs/Detection3DArray on /wauv/perception/objects. class_id is the Krill
class name ('gate', 'role_sign-compass', ...), pose is in the colour camera's optical
frame (x right, y down, z forward), header copied from the colour image.

The 2D detector is picked with the `backend` param:
    yolo  - Ultralytics weights in `yolo_weights` (the 19-class competition model)
    hough - HoughCircles, needs no weights; handy for checking the plumbing in sim
Topic defaults match the Stonefish sim (WAUV_Stonefish bluerov2.scn); remap for the sub.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

from wauv_perception import perception_core as core


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception')

        self.bridge = CvBridge()

        # topics (Stonefish: ros_publisher topic + "/image_color", "/image_depth", "/camera_info")
        self.declare_parameter('image_topic', '/bluerov2/left/image_color')
        self.declare_parameter('image_info_topic', '/bluerov2/left/camera_info')
        self.declare_parameter('depth_topic', '/bluerov2/depth_image/image_depth')
        self.declare_parameter('depth_info_topic', '/bluerov2/depth_image/camera_info')

        # colour camera position along the depth camera's optical x axis (metres).
        # Stonefish: left camera is 0.0725 m left of the depth camera -> -0.0725.
        # 0.0 if depth is already registered to the colour image (ZED SDK default).
        self.declare_parameter('color_in_depth_x', -0.0725)

        # trusted depth range (Stonefish depth camera: 0.1 to 8 m)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 8.0)

        self.declare_parameter('backend', 'yolo')
        self.declare_parameter('yolo_weights', '')
        self.declare_parameter('yolo_conf', 0.25)
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('yolo_device', '')
        self.declare_parameter('config_path', '')
        self.declare_parameter('publish_debug_image', True)

        self.cfg = core.load_config(self.get_parameter('config_path').value or None)
        self.model = None
        if self.get_parameter('backend').value == 'yolo':
            self.load_yolo()

        self.image_k = None
        self.depth_k = None
        self.latest_depth = None

        p = self.get_parameter
        self.create_subscription(CameraInfo, p('image_info_topic').value, self.image_info_cb, 10)
        self.create_subscription(CameraInfo, p('depth_info_topic').value, self.depth_info_cb, 10)
        self.create_subscription(Image, p('depth_topic').value, self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(Image, p('image_topic').value, self.image_cb, qos_profile_sensor_data)

        self.detection_pub = self.create_publisher(Detection3DArray, '/wauv/perception/objects', 10)
        self.debug_pub = self.create_publisher(Image, '/wauv/perception/debug_image', 10)

        self.get_logger().info(f"Perception started (backend={p('backend').value})")

    def load_yolo(self):
        weights = self.get_parameter('yolo_weights').value
        if not weights:
            self.get_logger().error("backend=yolo needs yolo_weights (or run with backend:=hough)")
            raise SystemExit
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error("backend=yolo needs `pip install ultralytics`")
            raise SystemExit

        self.model = YOLO(weights)
        names = [self.model.names[i] for i in sorted(self.model.names)]
        unknown = [n for n in names if n not in self.cfg['classes']]
        if unknown:
            self.get_logger().warn(f"model classes not in perception.yaml: {unknown}")
        self.get_logger().info(f"Loaded {weights} ({len(names)} classes)")

    @staticmethod
    def k_of(info):
        # (fx, fy, cx, cy)
        return (info.k[0], info.k[4], info.k[2], info.k[5])

    def image_info_cb(self, msg):
        self.image_k = self.k_of(msg)

    def depth_info_cb(self, msg):
        self.depth_k = self.k_of(msg)

    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"depth convert failed: {e}")

    def detect(self, bgr):
        if self.model is not None:
            p = self.get_parameter
            results = self.model.predict(
                bgr,
                conf=p('yolo_conf').value,
                imgsz=p('yolo_imgsz').value,
                device=p('yolo_device').value or None,
                verbose=False,
            )
            return core.yolo_result_to_detections(results[0])
        return core.detect_circles(bgr)

    def image_cb(self, msg):
        if self.image_k is None or self.depth_k is None or self.latest_depth is None:
            self.get_logger().warn("Waiting for camera info and depth", throttle_duration_sec=5.0)
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"image convert failed: {e}")
            return

        p = self.get_parameter
        fx, fy, cx, cy = self.image_k
        out = Detection3DArray()
        out.header = msg.header
        debug = bgr.copy() if p('publish_debug_image').value else None

        for d in self.detect(bgr):
            z = core.estimate_depth_across(
                self.latest_depth, d, self.image_k, self.depth_k,
                color_in_depth_x=p('color_in_depth_x').value,
                min_d=p('min_depth').value, max_d=p('max_depth').value,
            )
            if debug is not None:
                colour = (0, 200, 0) if z is not None else (0, 0, 255)
                label = d.class_id if z is None else f"{d.class_id} {z:.2f}m"
                cv2.rectangle(debug, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), colour, 2)
                cv2.putText(debug, label, (int(d.x1), max(12, int(d.y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)
            if z is None:
                # out of depth range (e.g. gate beyond 8 m): nothing to steer to yet
                continue

            u, v = d.center
            x, y, _ = core.pixel_to_optical(u, v, z, fx, fy, cx, cy)

            det = Detection3D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.class_id
            hyp.hypothesis.score = d.score
            hyp.pose.pose.position.x = float(x)
            hyp.pose.pose.position.y = float(y)
            hyp.pose.pose.position.z = float(z)
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)
            det.bbox.center = hyp.pose.pose
            det.bbox.size.x = (d.x2 - d.x1) * z / fx
            det.bbox.size.y = (d.y2 - d.y1) * z / fy
            out.detections.append(det)

        if out.detections:
            self.detection_pub.publish(out)
        if debug is not None:
            dbg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg.header = msg.header
            self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PerceptionNode()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
