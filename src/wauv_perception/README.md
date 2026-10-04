# wauv_perception

Finds competition props in the front colour camera, gets their distance from the depth
camera, and publishes them for the behavior tree.

- Output: `vision_msgs/Detection3DArray` on `/wauv/perception/objects`. `class_id` is the
  Krill class name (`gate`, `role_sign-compass`, ...); pose is in the colour camera's
  optical frame. Debug overlay on `/wauv/perception/debug_image`.
- `config/perception.yaml` is the only place that knows the 19 classes and which symbol
  belongs to which role.
- Topic defaults match WAUV_Stonefish; remap them for the sub's ZED.

```bash
# plumbing check in the sim, no weights needed
ros2 launch wauv_perception perception_launch.py backend:=hough
# competition model
ros2 launch wauv_perception perception_launch.py yolo_weights:=/path/to/best.pt
# gate role-sign model that ships with this package (use conf 0.5)
ros2 launch wauv_perception perception_launch.py yolo_conf:=0.5 \
  yolo_weights:=$(ros2 pkg prefix wauv_perception)/share/wauv_perception/models/role_signs_v3/role_signs_v3.pt
# tests (no ROS needed)
python -m pytest src/wauv_perception/test/test_perception_core.py
```

`models/role_signs_v3/` is the 2-class gate role-sign detector (`survey_repair`,
`search_rescue`, one box per printed sign). Its `MODEL_CARD.md` covers the ZED launch,
test results, known limits and what to record at the pool test.

Depth: many props are hollow (gate, torpedo openings), so depth is taken from the nearest
surface inside the box, not the centre pixel. The colour and depth cameras differ in
resolution, FOV and position, so boxes are mapped between them, including the
distance-dependent parallax from the 7.25 cm baseline.
