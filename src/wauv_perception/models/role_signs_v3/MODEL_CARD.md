# role_signs_v3: 2026 gate role-sign detector

Lokesh Sai Dasari, 2026-10-01. Gate Task (Lokesh, Josef, Shivanth).

## What it does

Finds the two printed 2026 Start Assessment signs and says which is which:

| id | class | sign |
|---|---|---|
| 0 | `survey_repair` | compass + hammer & wrench |
| 1 | `search_rescue` | SOS + life ring |

One box per whole sign. It does **not** detect the gate frame (PVC + panels). That
needs a separate model or geometry, so the gate-side voter in `wauv_perception`
won't make a left/right decision from this model alone.

YOLO11s, 640x640 input, ~9.4M params. Files:

| file | use |
|---|---|
| `role_signs_v3.pt` | Ultralytics / the `wauv_perception` node (`yolo_weights:=`) |
| `role_signs_v3.card.json` | class order + sha256, so you can check you have the right file |

The `.onnx` isn't committed (38 MB). For TensorRT on the Orin, make it from the `.pt` and
build the engine there:
`yolo export model=role_signs_v3.pt format=onnx imgsz=640 opset=17 simplify=True`, then
`trtexec --onnx=role_signs_v3.onnx --saveEngine=role_signs_v3.engine --fp16`.

**Use conf 0.5.**

## Run it

On the sub. `colcon build` installs these files to
`share/wauv_perception/models/role_signs_v3/`. The node reads the ZED through the SDK, so
there are no topics to remap:

```
ros2 launch wauv_perception perception_launch.py yolo_conf:=0.5 \
  yolo_weights:=$(ros2 pkg prefix wauv_perception)/share/wauv_perception/models/role_signs_v3/role_signs_v3.pt
```

- Watch the boxes on the laptop with WAUV-tools' `streamer.py` (UDP port 5000).
  Detections: `/wauv/perception/objects`.
- Signs with no usable depth (too far, out of the ZED's range) are drawn red and not published.

On a laptop, on a recording: `yolo predict model=role_signs_v3.pt source=clip.mp4 conf=0.5 save=True`

## How it was trained

All synthetic: the official RoboNation artwork rendered as printed acrylic onto real pool
frames (`python -m wauv_yolo synth-signs`), with varied distance, angle, lighting, colour
grade, blur, noise and lens distortion. Started from COCO `yolo11s.pt`, plus:
- 2000 **hard negatives**: real frames with no 2026 signs (last season's gate, red arrow
  cards, buoys, bins), so it learns look-alikes aren't signs.
- The same 2000 scenes **with** a sign rendered in, so "unusual scene" doesn't mean "no sign".

Recipe and code: Lokesh's `wauv-yolo` training repo (not on GitHub yet) (`configs/role_signs_graded.yaml`, then `role_signs_v3.yaml`).

## How good it is

Tested on data it never trained on. Recall and precision at conf 0.5, IoU 0.5:

| test | v1 (first model) | **v3** |
|---|---|---|
| Synthetic val (800 imgs) | 0.996 / 0.984 | **0.997 / 0.991** |
| New scenes: fresh signs on held-out sessions with look-alikes in frame (600) | 0.895 / 0.807 | **0.977 / 0.975** |
| Real sign-free frames that get a false sign (1704 held-out frames) | 19.8% | **0.4%** (0.2% at conf 0.7) |
| Turbid water, 60% haze | 0.965 / 0.942 | **0.990 / 0.986** |
| Sensor noise σ=15 | 0.878 / 0.916 | **0.955 / 0.989** |
| Heavy blur σ=4 | **0.941** / 0.982 | 0.832 / 0.988 |
| 2x farther (signs half size) | 0.993 / 0.990 | 0.986 / 0.993 |

Overfitting checks: val loss kept falling to the last epoch, and train-set recall (0.999)
≈ val recall (0.997), so it isn't memorizing. v1 *was* overfit in a way synthetic val hid:
it called anything red-on-white a sign (fired on 2025 arrow cards at conf 0.95).

## Known limits

- **No real photos of the 2026 signs have been tested yet.** Every number above is
  synthetic signs or real frames without signs. The pool test is the first real check.
- Heavy blur is the weak spot (recall 0.83 at σ=4). Expect misses when the camera is out of
  focus, moving fast, or the sign is far in murky water.
- Detects signs only, not the gate frame.

## Pool test: what to bring back

1. **Record raw camera** (`record.py` in WAUV-tools saves the RGB video and RGB + depth EXR frames).
   Cover 1-8 m, straight-on and angled (up to ~60°), a few depths, with the signs in and out
   of view, and with last year's props nearby if they're in the pool.
2. Note when it misses or fires on something wrong (time + what it saw).
3. Those frames become the first **real** val set. Label ~200 in Krill and we retrain
   with real data in the mix.
