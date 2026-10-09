# wauv_perception

Finds competition props with the ZED, gets their distance from the ZED depth, and publishes
them for the behavior tree.

- Input: colour + depth straight from the ZED SDK (`pyzed`, same calls as `record.py` in
  WAUV-tools). No ROS2 image topics, so nothing image-sized goes over ROS.
- Output: `vision_msgs/Detection3DArray` on `/wauv/perception/objects`, one message per frame
  (empty = nothing seen). `class_id` is the Krill class name (`gate`, `role_sign-compass`, ...);
  pose is the object's point in the left camera's optical frame (x right, y down, z forward);
  `bbox.size` is its width and height in metres.
- Debug view: the image with boxes is streamed as JPEG over UDP to `192.168.137.1:5000`, the
  same as `record.py`, so WAUV-tools' `streamer.py` on the laptop shows it.
- `config/perception.yaml` is the only place that knows the 19 classes and which symbol
  belongs to which role.
- Needs the ZED SDK's Python API (`pyzed`) on the Jetson; it isn't a rosdep.

```bash
# plumbing check, no weights
ros2 launch wauv_perception perception_launch.py backend:=hough
# gate role-sign model that ships with this package (use conf 0.5)
ros2 launch wauv_perception perception_launch.py yolo_conf:=0.5 \
  yolo_weights:=$(ros2 pkg prefix wauv_perception)/share/wauv_perception/models/role_signs_v3/role_signs_v3.pt
# no camera: replay the EXR frames record.py saved, view on this machine
ros2 launch wauv_perception perception_launch.py exr_dir:=/path/to/data stream_host:=127.0.0.1 \
  yolo_weights:=/path/to/best.pt
# tests (no ROS needed)
python -m pytest src/wauv_perception/test/test_perception_core.py
```

`models/role_signs_v3/` is the 2-class gate role-sign detector (`survey_repair`,
`search_rescue`, one box per printed sign). Its `MODEL_CARD.md` covers the ZED launch,
test results, known limits and what to record at the pool test.

Depth: many props are hollow (gate, torpedo openings), so depth is taken from the nearest
surface inside the box, not the centre pixel. The ZED SDK registers depth to the left image,
so a box indexes the depth map directly. (`perception_core.estimate_depth_across` still
handles separate colour and depth cameras, like the Stonefish sim's, if that's needed again.)
