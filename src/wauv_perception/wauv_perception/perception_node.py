"""
Filename: perception_node.py
Description: ROS2 node that grabs colour + depth straight from the ZED SDK, finds the
             competition objects, gets their distance and 3D point, and publishes them
             for the behavior tree.

No ROS2 image topics: frames come from pyzed (same calls as record.py in WAUV-tools), so
the ZED computes depth and nothing image-sized goes over ROS. Two outputs:
  - vision_msgs/Detection3DArray on /wauv/perception/objects, one message per frame (an
    empty array means nothing was seen). class_id is the Krill class name ('gate',
    'role_sign-compass', ...), pose is the object's point in the left camera's optical
    frame (x right, y down, z forward), bbox.size is its width/height in metres.
  - the image with boxes drawn, streamed as JPEG over UDP to the laptop, same format and
    port as record.py, so Aarav's streamer.py shows it.

The 2D detector is picked with the `backend` param:
    yolo  - Ultralytics weights in `yolo_weights` (the 19-class competition model)
    hough - HoughCircles, needs no weights; handy for checking the plumbing
Set `exr_dir` to replay the EXR frames record.py saves instead of opening the camera.
"""

import glob
import math
import os
import re
import socket
import time

os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '1'  # before cv2 is imported, for exr_dir replay

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose  # noqa: E402

from wauv_perception import perception_core as core  # noqa: E402

STREAM_WIDTH = 640   # streamed image width; height keeps the aspect ratio
UDP_LIMIT = 65000    # one JPEG per UDP packet


# ---------------------------------------------------------------------------
# frame sources: read() gives (bgr, depth in metres) or None; k = (fx, fy, cx, cy)
# ---------------------------------------------------------------------------

class ZedSource:
    """Live ZED camera. Depth is registered to the left image by the SDK, so a pixel in
    the colour image indexes the same pixel in the depth map (no parallax search)."""

    def __init__(self, depth_mode, max_depth):
        import pyzed.sl as sl
        self.sl = sl
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
        init_params.coordinate_units = sl.UNIT.METER
        # x right, y down, z forward: the optical frame perception_core uses
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        init_params.depth_maximum_distance = max_depth

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Camera Open Error: {err}')

        info = self.zed.get_camera_information()
        # SDK 4.x keeps calibration under camera_configuration, 3.x on info itself
        conf = getattr(info, 'camera_configuration', info)
        cam = conf.calibration_parameters.left_cam
        self.k = (cam.fx, cam.fy, cam.cx, cam.cy)

        self.runtime_params = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.depth = sl.Mat()

    def read(self):
        sl = self.sl
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return None
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
        self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
        # copy out: sl.Mat buffers get reused on the next grab
        bgr = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)
        depth = self.depth.get_data().copy()
        return bgr, depth

    def close(self):
        self.zed.close()


class ExrSource:
    """Replays the out_rgbd*.exr frames record.py saves (BGR + depth in one file), so the
    node can be tested on a laptop without the camera.

    The EXRs don't store the calibration, so k is estimated from the horizontal field
    of view: distances are real, x/y are approximate."""

    def __init__(self, folder, hfov_deg, fps):
        def frame_number(path):
            m = re.search(r'_(\d+)\.exr$', path)
            return int(m.group(1)) if m else 0

        self.paths = sorted(glob.glob(os.path.join(folder, '*.exr')), key=frame_number)
        if not self.paths:
            raise RuntimeError(f'No .exr files in {folder}')
        self.hfov = hfov_deg
        self.period = 1.0 / fps if fps > 0 else 0.0
        self.i = 0
        self.k = None
        self.finished = False

    def read(self):
        if self.i >= len(self.paths):
            self.finished = True
            return None
        path = self.paths[self.i]
        self.i += 1
        if self.period:
            time.sleep(self.period)
        flags = cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH | cv2.IMREAD_UNCHANGED
        image = cv2.imread(path, flags)
        if image is None or image.ndim != 3 or image.shape[2] < 4:
            return None
        bgr = np.clip(image[:, :, :3], 0, 255).astype(np.uint8)
        depth = image[:, :, -1].astype(np.float32)
        if self.k is None:
            h, w = depth.shape
            f = (w / 2) / math.tan(math.radians(self.hfov) / 2)
            self.k = (f, f, w / 2, h / 2)
        return bgr, depth

    def close(self):
        pass


# ---------------------------------------------------------------------------
# one frame -> 3D objects, and the debug picture
# ---------------------------------------------------------------------------

def locate(dets, depth, k, cfg, min_d, max_d):
    """Distance from the depth map (nearest surface in the box, so hollow props like the
    gate work), then 3D point and size in the optical frame. xyz is None when there is
    no usable depth (e.g. a gate beyond max_depth)."""
    fx, fy, cx, cy = k
    objects = []
    for d in dets:
        z = core.estimate_depth(depth, d, min_d=min_d, max_d=max_d)
        obj = {'det': d, 'class_id': d.class_id, 'role': core.role_of(d.class_id, cfg),
               'score': d.score, 'xyz': None, 'size': None}
        if z is not None:
            u, v = d.center
            obj['xyz'] = core.pixel_to_optical(u, v, z, fx, fy, cx, cy)
            obj['size'] = ((d.x2 - d.x1) * z / fx, (d.y2 - d.y1) * z / fy)
        objects.append(obj)
    return objects


def draw(bgr, objects, fps):
    out = bgr.copy()
    for o in objects:
        d = o['det']
        found = o['xyz'] is not None
        colour = (0, 200, 0) if found else (0, 0, 255)   # red = no depth for it
        label = f"{o['class_id']} {o['score']:.2f}"
        if found:
            label += f" {o['xyz'][2]:.2f}m"
        cv2.rectangle(out, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), colour, 3)
        cv2.putText(out, label, (int(d.x1), max(24, int(d.y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2, cv2.LINE_AA)
    cv2.putText(out, f'{fps:.1f} fps  {len(objects)} objects', (10, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 0), 2, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------
# node
# ---------------------------------------------------------------------------

class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception')

        self.declare_parameter('backend', 'yolo')
        self.declare_parameter('yolo_weights', '')
        self.declare_parameter('yolo_conf', 0.25)
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('yolo_device', '')
        self.declare_parameter('config_path', '')

        # ZED: PERFORMANCE, ULTRA (what record.py uses), NEURAL, ...
        self.declare_parameter('depth_mode', 'ULTRA')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 8.0)
        self.declare_parameter('frame_id', 'zed_left_camera_optical_frame')

        # debug stream to the laptop; same address and port as record.py
        self.declare_parameter('stream', True)
        self.declare_parameter('stream_host', '192.168.137.1')
        self.declare_parameter('stream_port', 5000)
        self.declare_parameter('jpeg_quality', 70)

        # replay record.py's EXR frames instead of opening the camera
        self.declare_parameter('exr_dir', '')
        self.declare_parameter('exr_hfov', 110.0)
        self.declare_parameter('exr_fps', 15.0)

        p = self.get_parameter
        self.cfg = core.load_config(p('config_path').value or None)
        self.model = None
        if p('backend').value == 'yolo':
            self.load_yolo()

        if p('exr_dir').value:
            self.source = ExrSource(p('exr_dir').value, p('exr_hfov').value,
                                    p('exr_fps').value)
            self.get_logger().info(
                f"Replaying {len(self.source.paths)} EXR frames from {p('exr_dir').value} "
                f"(x/y use an estimated {p('exr_hfov').value} deg field of view)")
        else:
            self.source = ZedSource(p('depth_mode').value, p('max_depth').value)
            fx, fy, _, _ = self.source.k
            self.get_logger().info(f'ZED open, left camera fx={fx:.1f} fy={fy:.1f}')

        self.sock = None
        if p('stream').value:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.stream_addr = (p('stream_host').value, p('stream_port').value)
            self.get_logger().info(
                f'Streaming annotated frames to {self.stream_addr[0]}:{self.stream_addr[1]} '
                '(run streamer.py there)')

        self.detection_pub = self.create_publisher(
            Detection3DArray, '/wauv/perception/objects', 10)

        self.frames = 0
        self.fps = 0.0
        self.last = time.time()
        self.get_logger().info(f"Perception started (backend={p('backend').value})")

    def load_yolo(self):
        weights = self.get_parameter('yolo_weights').value
        if not weights:
            self.get_logger().error('backend=yolo needs yolo_weights (or run with backend:=hough)')
            raise SystemExit
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error('backend=yolo needs `pip install ultralytics`')
            raise SystemExit

        self.model = YOLO(weights)
        names = [self.model.names[i] for i in sorted(self.model.names)]
        unknown = [n for n in names
                   if n not in self.cfg['classes'] and n not in self.cfg['roles']]
        if unknown:
            self.get_logger().warn(f'model classes not in perception.yaml: {unknown}')
        self.get_logger().info(f'Loaded {weights} ({len(names)} classes)')

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

    def step(self):
        """Grab one frame, detect, publish, stream. Returns False when a replay ends."""
        got = self.source.read()
        if got is None:
            return not getattr(self.source, 'finished', False)
        bgr, depth = got

        p = self.get_parameter
        objects = locate(self.detect(bgr), depth, self.source.k, self.cfg,
                         p('min_depth').value, p('max_depth').value)

        now = time.time()
        if self.frames:
            inst = 1.0 / max(now - self.last, 1e-6)
            self.fps = inst if self.fps == 0.0 else 0.9 * self.fps + 0.1 * inst
        self.last = now
        self.frames += 1

        self.publish(objects)
        if self.sock is not None:
            self.stream(draw(bgr, objects, self.fps))

        seen = ', '.join(
            f"{o['class_id']}@{o['xyz'][2]:.2f}m" if o['xyz'] is not None
            else f"{o['class_id']}@?" for o in objects) or 'nothing'
        self.get_logger().info(f'{self.fps:.1f} fps  {seen}', throttle_duration_sec=1.0)
        return True

    def publish(self, objects):
        out = Detection3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('frame_id').value
        for o in objects:
            if o['xyz'] is None:
                continue  # no distance (out of range): nothing to steer to yet
            x, y, z = (float(v) for v in o['xyz'])
            det = Detection3D()
            det.header = out.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = o['class_id']
            hyp.hypothesis.score = float(o['score'])
            hyp.pose.pose.position.x = x
            hyp.pose.pose.position.y = y
            hyp.pose.pose.position.z = z
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)
            det.bbox.center = hyp.pose.pose
            det.bbox.size.x = float(o['size'][0])
            det.bbox.size.y = float(o['size'][1])
            out.detections.append(det)
        self.detection_pub.publish(out)

    def stream(self, frame):
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (STREAM_WIDTH, round(h * STREAM_WIDTH / w)),
                           interpolation=cv2.INTER_AREA)
        quality = self.get_parameter('jpeg_quality').value
        ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return
        if len(buf) >= UDP_LIMIT:
            self.get_logger().warn(f'frame too big: {len(buf)} bytes, dropped',
                                   throttle_duration_sec=5.0)
            return
        self.sock.sendto(buf.tobytes(), self.stream_addr)

    def close(self):
        self.source.close()
        if self.sock is not None:
            self.sock.close()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PerceptionNode()
    except (SystemExit, RuntimeError) as e:
        if isinstance(e, RuntimeError):
            print(e)
        rclpy.shutdown()
        return
    try:
        # the camera paces the loop (grab blocks until the next frame); spin_once keeps
        # parameter services etc. alive without a separate thread
        while rclpy.ok() and node.step():
            rclpy.spin_once(node, timeout_sec=0.0)
        if getattr(node.source, 'finished', False):
            node.get_logger().info('Replay finished.')
    except KeyboardInterrupt:
        pass
    node.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
