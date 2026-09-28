"""
Filename: perception_core.py
Description: ROS-free perception logic for the perception node (and any behavior tree
             that needs the transforms or the gate-side vote). Everything here is plain numpy / OpenCV so
             it can be unit tested on any laptop without a ROS install.

Frames used in this file:
    optical  - camera optical frame (x right, y down, z forward), what pinhole math gives
    body     - vehicle base_link, ROS FLU (x forward, y left, z up)
    local    - MAVROS local_position frame (ENU), what motion_controller steers in
"""

from dataclasses import dataclass, field
import math
import os

import cv2
import numpy as np
import yaml


# default config ships next to the package (installed to share/wauv_perception/config)
DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'perception.yaml')

# where the colour camera sits on base_link, FLU. Default is the Stonefish left camera
# (bluerov2.scn origin xyz="0.16 -0.0725 0.15" is FRD, so y and z flip). Override per sub.
CAMERA_MOUNT_XYZ = (0.16, 0.0725, -0.15)


# ---------------------------------------------------------------------------
# class taxonomy and role config
# ---------------------------------------------------------------------------

def default_config_path():
    """Installed share/wauv_sim/config when running under ROS, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('wauv_perception'), 'config', 'perception.yaml')
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return os.path.normpath(DEFAULT_CONFIG)


def load_config(path=None):
    """Load the perception config (class list + symbol -> role map)."""
    with open(path or default_config_path(), 'r') as f:
        cfg = yaml.safe_load(f)

    classes = []
    for entry in cfg['label_types']:
        symbols = entry.get('symbols') or []
        if symbols:
            classes.extend(f"{entry['name']}-{s}" for s in symbols)
        else:
            classes.append(entry['name'])
    cfg['classes'] = classes

    symbol_to_role = {}
    for role, symbols in cfg['roles'].items():
        for s in symbols:
            symbol_to_role[s] = role
    cfg['symbol_to_role'] = symbol_to_role
    return cfg


def split_class(class_id):
    """'role_sign-compass' -> ('role_sign', 'compass'); 'gate' -> ('gate', None)."""
    if '-' in class_id:
        label_type, symbol = class_id.split('-', 1)
        return label_type, symbol
    return class_id, None


def role_of(class_id, cfg):
    """Role a detection belongs to, or None for role-free classes like 'gate'."""
    _, symbol = split_class(class_id)
    if symbol is None:
        return None
    return cfg['symbol_to_role'].get(symbol)


# ---------------------------------------------------------------------------
# 2D detections
# ---------------------------------------------------------------------------

@dataclass
class Detection2D:
    class_id: str
    score: float
    # pixel box, x1 < x2, y1 < y2
    x1: float
    y1: float
    x2: float
    y2: float
    # 'ring' for Hough circles (hollow hoop), 'box' for everything else
    shape: str = 'box'
    extra: dict = field(default_factory=dict)

    @property
    def center(self):
        return 0.5 * (self.x1 + self.x2), 0.5 * (self.y1 + self.y2)

    @property
    def width(self):
        return self.x2 - self.x1


def detect_circles(bgr, dp=1.2, min_dist=50.0, param1=100.0, param2=30.0,
                   min_radius=5, max_radius=200):
    """The original HoughCircles detector, returned as ring-shaped Detection2Ds."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=dp, minDist=min_dist,
        param1=param1, param2=param2, minRadius=min_radius, maxRadius=max_radius,
    )
    dets = []
    if circles is None:
        return dets
    for (u, v, r) in circles[0, :]:
        u, v, r = float(u), float(v), float(r)
        # Hough gives no calibrated confidence
        dets.append(Detection2D('circle', 1.0, u - r, v - r, u + r, v + r, shape='ring'))
    return dets


def yolo_result_to_detections(result, class_names=None):
    """Convert one ultralytics Results object into Detection2Ds (no torch types leak out)."""
    names = class_names or result.names
    dets = []
    boxes = result.boxes
    if boxes is None:
        return dets
    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy()
    cls = boxes.cls.cpu().numpy().astype(int)
    for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
        dets.append(Detection2D(names[k], float(c), float(x1), float(y1), float(x2), float(y2)))
    return dets


# ---------------------------------------------------------------------------
# depth sampling
# ---------------------------------------------------------------------------

def _valid(values, min_d, max_d):
    values = np.asarray(values, dtype=np.float32)
    return values[np.isfinite(values) & (values > min_d) & (values < max_d)]


def ring_depth_samples(depth, det, n=72, inner=0.85, outer=1.1):
    """Depth values on an annulus around a circle. The hoop centre is a hole, so the
    centre pixel sees the pool wall (or nothing, inf) instead of the hoop."""
    h, w = depth.shape[:2]
    u, v = det.center
    r = 0.5 * det.width
    out = []
    for scale in np.linspace(inner, outer, 4):
        for a in np.linspace(0.0, 2.0 * math.pi, n, endpoint=False):
            x = int(round(u + scale * r * math.cos(a)))
            y = int(round(v + scale * r * math.sin(a)))
            if 0 <= x < w and 0 <= y < h:
                out.append(depth[y, x])
    return np.asarray(out, dtype=np.float32)


def box_depth_samples(depth, det, shrink=0.0):
    """All depth values inside the box. No trim by default: on a tight gate box the only
    object pixels are the thin PVC frame right at the edge, and background bleeding in at
    the edge is further away, which estimate_depth already ignores."""
    h, w = depth.shape[:2]
    dx = shrink * (det.x2 - det.x1)
    dy = shrink * (det.y2 - det.y1)
    x1 = max(0, int(det.x1 + dx))
    x2 = min(w, int(math.ceil(det.x2 - dx)))
    y1 = max(0, int(det.y1 + dy))
    y2 = min(h, int(math.ceil(det.y2 - dy)))
    if x2 <= x1 or y2 <= y1:
        return np.empty(0, dtype=np.float32)
    return depth[y1:y2, x1:x2].ravel()


def estimate_depth(depth, det, min_d=0.05, max_d=30.0, near_pct=3.0, band=0.5,
                   min_samples=5):
    """Robust distance to a detected object, in metres, or None.

    Many of our objects are hollow (gate frame, hoops, torpedo holes), so a box also
    contains background that is further away. We anchor on the near side of the depth
    distribution (a low percentile, not the min, so single noisy pixels don't win) and
    take the median of everything within `band` metres of that anchor. `near_pct` must
    stay below the share of box pixels that are object: a far gate's PVC frame can be
    only ~5% of its box.
    """
    if det.shape == 'ring':
        samples = ring_depth_samples(depth, det)
    else:
        samples = box_depth_samples(depth, det)
    valid = _valid(samples, min_d, max_d)
    if valid.size < min_samples:
        return None
    anchor = float(np.percentile(valid, near_pct))
    near = valid[valid <= anchor + band]
    return float(np.median(near))


def estimate_depth_across(depth, det, color_k, depth_k, color_in_depth_x=0.0,
                          min_d=0.05, max_d=30.0, n_candidates=24, tol=0.2, **kw):
    """estimate_depth for a box found in a *different* camera than the depth image.

    Stonefish (and the real ZED if depth isn't registered to the left image) gives a
    colour camera and a depth camera with different resolution, FOV and position, so a
    colour pixel can't index the depth image directly. We map the box through normalised
    image coordinates. The cameras' baseline adds a parallax of fx_depth * baseline / Z
    pixels, which depends on the distance we're trying to measure: at 0.4 m it is ~120
    depth pixels, enough to miss a small target and read the wall behind it instead.
    So we try candidate distances and keep the nearest one whose measured depth agrees
    with the guess (within `tol`, relative).
    color_in_depth_x: colour camera position along the depth camera's optical x, metres.
    """
    kw.update(min_d=min_d, max_d=max_d)
    plain = estimate_depth(depth, map_box(det, color_k, depth_k), **kw)
    if color_in_depth_x == 0.0:
        return plain

    best = None
    for zc in np.geomspace(max(min_d, 0.1), max_d, n_candidates):
        shift = depth_k[0] * color_in_depth_x / zc
        z = estimate_depth(depth, map_box(det, color_k, depth_k, shift_px=shift), **kw)
        if z is None:
            continue
        # consistent if the measured distance would have produced (nearly) this shift
        if abs(z - zc) <= tol * zc or abs(depth_k[0] * color_in_depth_x * (1 / z - 1 / zc)) < 2.0:
            if best is None or z < best:
                best = z
    return best if best is not None else plain


def map_box(det, src_k, dst_k, shift_px=0.0):
    """Re-express a pixel box from camera src in camera dst's pixels.
    k = (fx, fy, cx, cy). Assumes parallel optical axes (true for a stereo rig)."""
    sfx, sfy, scx, scy = src_k
    dfx, dfy, dcx, dcy = dst_k

    def mx(u):
        return (u - scx) / sfx * dfx + dcx + shift_px

    def my(v):
        return (v - scy) / sfy * dfy + dcy

    return Detection2D(det.class_id, det.score, mx(det.x1), my(det.y1), mx(det.x2), my(det.y2),
                       shape=det.shape, extra=det.extra)


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

def pixel_to_optical(u, v, z, fx, fy, cx, cy):
    """Pinhole inverse projection into the camera optical frame."""
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float64)


def optical_to_body(p_opt, mount=CAMERA_MOUNT_XYZ):
    """Optical (x right, y down, z fwd) -> base_link FLU, for a forward-facing camera."""
    x, y, z = p_opt
    return np.array([z + mount[0], -x + mount[1], -y + mount[2]], dtype=np.float64)


def quat_to_matrix(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def body_to_local(p_body, position, orientation):
    """Body-frame point -> MAVROS local frame, given the vehicle pose there.
    position is (x, y, z), orientation is (qx, qy, qz, qw)."""
    R = quat_to_matrix(*orientation)
    return R @ np.asarray(p_body, dtype=np.float64) + np.asarray(position, dtype=np.float64)


# ---------------------------------------------------------------------------
# gate side decision
# ---------------------------------------------------------------------------

class GateSideVoter:
    """Decide which half of the gate belongs to our role.

    Each frame, every role_sign within the gate's width votes for the half it is in:
    +1 for our role's symbol on that side, -1 for the other role's. The side is the sum
    over the last `window` frames (the Perception Plan asks for a 10 frame vote), so one
    misread symbol can't flip the decision.

    Works in any horizontal coordinate that grows to the right: pixel u, or optical x.
    """

    def __init__(self, our_role, cfg, window=10, min_votes=3):
        self.our_role = our_role
        self.cfg = cfg
        self.window = window
        self.min_votes = min_votes
        self.history = []

    def update(self, gate, signs):
        """gate: (centre_x, half_width) or None. signs: [(class_id, x), ...]."""
        vote = 0
        if gate is not None:
            mid, half = gate
            for class_id, x in signs:
                if split_class(class_id)[0] != 'role_sign' or abs(x - mid) > half:
                    continue
                side = -1 if x < mid else 1  # -1 = left half, +1 = right half
                role = role_of(class_id, self.cfg)
                if role == self.our_role:
                    vote += side
                elif role is not None:
                    vote -= side
        self.history.append(vote)
        self.history = self.history[-self.window:]
        return self.decision()

    def decision(self):
        total = sum(self.history)
        if abs(total) < self.min_votes:
            return None
        return 'left' if total < 0 else 'right'
