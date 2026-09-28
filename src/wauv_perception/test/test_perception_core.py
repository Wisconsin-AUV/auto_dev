"""Tests for perception_core. Pure numpy/OpenCV, so they run without ROS:

    python -m pytest src/wauv_perception/test/test_perception_core.py
"""

import math

import numpy as np
import pytest

from wauv_perception import perception_core as core



@pytest.fixture(scope='module')
def cfg():
    return core.load_config()


# --- config ---------------------------------------------------------------

def test_config_has_19_classes_in_fixed_order(cfg):
    assert len(cfg['classes']) == 19
    assert cfg['classes'][:3] == ['gate', 'role_sign-compass', 'role_sign-hammer_pick']
    assert cfg['classes'][-1] == 'basket-helmet_cross'


def test_every_symbol_has_exactly_one_role(cfg):
    symbols = {core.split_class(c)[1] for c in cfg['classes']} - {None}
    assert symbols == set(cfg['symbol_to_role'])
    roles = cfg['roles']
    assert not set(roles['survey_repair']) & set(roles['search_rescue'])


def test_role_of(cfg):
    assert core.role_of('role_sign-compass', cfg) == 'survey_repair'
    assert core.role_of('torpedo_hole-ambulance', cfg) == 'search_rescue'
    # same symbol on a different object keeps its role
    assert core.role_of('bin-fire', cfg) == core.role_of('torpedo_hole-fire', cfg)
    assert core.role_of('gate', cfg) is None
    assert core.role_of('circle', cfg) is None


# --- depth sampling ---------------------------------------------------------

def hollow_scene(obj_z=3.0, bg_z=float('inf'), noise=0.0, seed=0, thickness=20):
    """A 3 m-away square frame (like the gate) with a see-through middle."""
    rng = np.random.default_rng(seed)
    depth = np.full((480, 640), bg_z, dtype=np.float32)
    t = thickness
    depth[100:300, 200:400] = obj_z
    depth[100 + t:300 - t, 200 + t:400 - t] = bg_z  # hole
    if noise:
        depth += rng.normal(0, noise, depth.shape).astype(np.float32)
    return depth, core.Detection2D('gate', 0.9, 200, 100, 400, 300)


@pytest.mark.parametrize('bg', [float('inf'), 12.0, float('nan')])
def test_hollow_object_depth_ignores_background(bg):
    depth, det = hollow_scene(bg_z=bg)
    assert core.estimate_depth(depth, det) == pytest.approx(3.0, abs=0.01)


def test_thin_pvc_frame_far_away():
    # 3 px frame in a 200 px box: ~6% of the box is gate, the rest is pool wall
    depth, det = hollow_scene(obj_z=7.0, bg_z=11.0, thickness=3)
    assert core.estimate_depth(depth, det) == pytest.approx(7.0, abs=0.01)


def test_depth_survives_noise_and_speckle():
    depth, det = hollow_scene(bg_z=9.0, noise=0.02)
    # a few too-close speckles (bubbles) must not become "the object"
    depth[150, 250] = depth[151, 251] = 0.4
    assert core.estimate_depth(depth, det) == pytest.approx(3.0, abs=0.05)


def test_depth_none_when_nothing_valid():
    depth = np.full((480, 640), np.inf, dtype=np.float32)
    det = core.Detection2D('gate', 0.9, 10, 10, 50, 50)
    assert core.estimate_depth(depth, det) is None


def test_box_clipped_to_image():
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    det = core.Detection2D('bin-fire', 0.9, -50, 400, 100, 600)
    assert core.estimate_depth(depth, det) == pytest.approx(2.0)


# --- geometry ---------------------------------------------------------------

def test_pixel_to_optical_centre_and_edge():
    fx = fy = 320.0
    assert core.pixel_to_optical(319.5, 239.5, 4.0, fx, fy, 319.5, 239.5) == pytest.approx([0, 0, 4])
    # right image edge of a 90 deg camera is 45 deg off axis: x == z
    x, _, z = core.pixel_to_optical(639.5, 239.5, 4.0, fx, fy, 319.5, 239.5)
    assert x == pytest.approx(z)


def test_optical_to_body_axes():
    mount = (0.16, 0.0725, -0.15)  # Stonefish left camera, FLU
    # straight ahead, 2 m -> forward 2 m plus the mount offset
    assert core.optical_to_body([0, 0, 2], mount) == pytest.approx([2.16, 0.0725, -0.15])
    # to the right in the image -> negative y (FLU: y is left)
    assert core.optical_to_body([1, 0, 2], mount)[1] == pytest.approx(0.0725 - 1)
    # down in the image -> negative z
    assert core.optical_to_body([0, 1, 2], mount)[2] == pytest.approx(-0.15 - 1)


def test_body_to_local_with_yaw():
    yaw = math.pi / 2  # vehicle facing +y in the local frame
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    p = core.body_to_local([2, 0, 0], (10, 5, -3), q)
    assert p == pytest.approx([10, 7, -3])


# --- gate side --------------------------------------------------------------

def test_gate_side_votes_for_our_symbols(cfg):
    voter = core.GateSideVoter('search_rescue', cfg)
    gate = (320.0, 150.0)  # centre x, half width (pixels)
    signs = [('role_sign-compass', 250.0), ('role_sign-sos', 400.0)]
    side = None
    for _ in range(3):
        side = voter.update(gate, signs)
    assert side == 'right'
    assert core.GateSideVoter('survey_repair', cfg).update(gate, signs * 3) == 'left'


def test_gate_side_resists_single_misread(cfg):
    voter = core.GateSideVoter('survey_repair', cfg)
    gate = (0.0, 1.5)  # optical-frame metres work too
    for _ in range(9):
        voter.update(gate, [('role_sign-hammer_pick', -0.7)])
    # one frame where the symbol is misread as the other role
    assert voter.update(gate, [('role_sign-life_ring', -0.7)]) == 'left'


def test_gate_side_ignores_signs_outside_gate_and_no_gate(cfg):
    voter = core.GateSideVoter('survey_repair', cfg)
    assert voter.update(None, [('role_sign-compass', 0.0)] * 5) is None
    # octagon wall image far off to the side is not a gate marker
    assert voter.update((0.0, 1.5), [('role_sign-compass', 4.0)] * 5) is None


# --- colour camera -> depth camera (Stonefish rig) ---------------------------

def k_for(width, height, hfov_deg):
    """Stonefish CameraInfo: square pixels, principal point at the image centre."""
    fx = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    return (fx, fx, width / 2, height / 2)


COLOR_K = k_for(640, 480, 75.0)      # camera_left
DEPTH_K = k_for(1920, 1200, 110.0)   # zed_x_mini_depth
BASELINE = -0.0725                   # left camera sits 7.25 cm left of the depth camera


def render_depth(z_obj, half_w, half_h, thickness, z_bg=float('inf'), cx_obj=0.0):
    """Depth image of a hollow rectangle (gate-like frame) centred at x=cx_obj in the
    COLOUR camera's frame, as the depth camera would see it."""
    fx, fy, cx, cy = DEPTH_K
    depth = np.full((1200, 1920), z_bg, dtype=np.float32)
    x0 = cx_obj + BASELINE  # object position in the depth camera's frame

    def px(x):
        return int(round(fx * x / z_obj + cx))

    def py(y):
        return int(round(fy * y / z_obj + cy))

    depth[py(-half_h):py(half_h), px(x0 - half_w):px(x0 + half_w)] = z_obj
    t = thickness
    depth[py(-half_h + t):py(half_h - t), px(x0 - half_w + t):px(x0 + half_w - t)] = z_bg
    return depth


def box_in_color(z_obj, half_w, half_h, cx_obj=0.0):
    fx, fy, cx, cy = COLOR_K
    return core.Detection2D('gate', 0.9,
                            fx * (cx_obj - half_w) / z_obj + cx, fy * -half_h / z_obj + cy,
                            fx * (cx_obj + half_w) / z_obj + cx, fy * half_h / z_obj + cy)


def test_map_box_identity_and_scale():
    det = core.Detection2D('gate', 0.9, 100, 100, 200, 150)
    same = core.map_box(det, COLOR_K, COLOR_K)
    assert (same.x1, same.y1, same.x2, same.y2) == pytest.approx((100, 100, 200, 150))
    # the image centre maps to the image centre
    c = core.map_box(core.Detection2D('x', 1, 320, 240, 320, 240), COLOR_K, DEPTH_K)
    assert (c.x1, c.y1) == pytest.approx((960, 600))


def test_gate_at_5m_through_stonefish_rig():
    # 3 m x 1.5 m gate, 5 cm PVC, 5 m ahead, pool wall 12 m behind the camera
    depth = render_depth(5.0, 1.5, 0.75, 0.05, z_bg=12.0)
    det = box_in_color(5.0, 1.5, 0.75)
    z = core.estimate_depth_across(depth, det, COLOR_K, DEPTH_K, BASELINE, max_d=30.0)
    assert z == pytest.approx(5.0, abs=0.01)


def test_baseline_correction_matters_up_close():
    # parallax at 0.4 m is fx_depth * 7.25 cm / 0.4 m ~ 122 depth pixels. Bigger objects
    # still overlap and the near-surface anchor copes; a 4 cm target (67 px) is missed.
    depth = render_depth(0.4, 0.02, 0.02, 0.005, z_bg=3.0, cx_obj=0.1)
    det = box_in_color(0.4, 0.02, 0.02, cx_obj=0.1)
    ignore_baseline = core.estimate_depth_across(depth, det, COLOR_K, DEPTH_K, 0.0, max_d=30.0)
    corrected = core.estimate_depth_across(depth, det, COLOR_K, DEPTH_K, BASELINE, max_d=30.0)
    assert corrected == pytest.approx(0.4, abs=0.01)
    # without the correction the box lands beside the target and reads the background
    assert ignore_baseline is None or abs(ignore_baseline - 0.4) > 1.0


def test_gate_beyond_depth_range_gives_none():
    # Stonefish depth camera tops out at 8 m: a 10 m gate has no distance yet
    depth = render_depth(10.0, 1.5, 0.75, 0.05)
    depth[depth > 8.0] = np.inf
    det = box_in_color(10.0, 1.5, 0.75)
    assert core.estimate_depth_across(depth, det, COLOR_K, DEPTH_K, BASELINE, max_d=8.0) is None
