# krill_gate_v1

The gate model that `yolo_detector.py` loads (`models/krill_gate_v1.pt`, installed to
`share/wauv_perception/models/` by the `models/*.pt` line in `setup.py`).

## What it detects

The classes are Krill's label types, in Krill's order. That means a later Krill export
trains with the same class IDs.

| id | class | boxes in training |
|---|---|---|
| 0 | `gate_sos` | 1348 |
| 1 | `gate_hammer` | 1033 |
| 2 | `bins` | 395 (no val video, so untested) |
| 3 | `bin_blood` | 146 (one video, untested) |
| 4-16 | torpedo, table, basket types | none yet, never predicted |

## Training

- **Data:** Krill export 6 `gate_Task_data` (2026-10-18). It has 1945 training images
  from 4 videos. Krill splits by video, so the validation set is one whole held-out
  video, `gate_pvc_imgs2` (459 frames). The training set also includes 484 frames from
  the sub's own ZED footage of the Oct 4 pool test (`recovered.mp4`). Those frames show
  no signs, so they teach the model what is *not* a sign. They come from alternating
  60-second blocks of that video, and the other blocks were held out for testing.
- **Settings:** `yolo11s.pt` start, imgsz 960, 40 epochs, no early stopping,
  `fliplr 0` (a mirrored sign doesn't exist), ±10° rotation. The checkpoint is the best
  epoch (35). The config is `wauv-yolo/configs/krill_gate_960.yaml`.

## Results

These were measured by calling `model(frame)` exactly as `yolo_detector.py` does, so
they use imgsz 960 from the checkpoint and conf 0.25 by default.

| | conf 0.25 | conf 0.5 |
|---|---|---|
| `gate_sos` recall / precision (val video) | 0.82 / 0.99 | 0.77 / 1.00 |
| `gate_hammer` recall / precision (val video) | 0.69 / 0.89 | 0.56 / 0.97 |
| frames with any detection, held-out sub pool footage (370 frames, no signs) | 0.3% | 0.0% |

For comparison, the same recipe at 640 px without the pool frames reached 0.88 / 0.78
recall, but it fired on 4.9% of the pool frames at conf 0.25. Nearly all of those
false alarms were `gate_hammer` on people's white clothing. That is about one false
sign every 1.4 s at 15 fps.

Speed: 21 ms per frame on an RTX 3050 Ti laptop GPU (PyTorch, 960 px). On the Jetson,
expect roughly 2x the cost of a 640 px model. If it can't keep up with the 0.05 s
timer, export to TensorRT (`yolo export format=engine imgsz=960 half=True`).

## Known limits

- **Rolled camera.** Most missed hammer signs are on frames where the camera is rolled
  20-30° or more. The val video is handheld, and training only rotates ±10°. A level
  sub should do better than the val numbers suggest. The next retrain should raise
  `degrees`.
- **Hammer is the weaker class.** That sign is mostly white, so it is easier to lose
  against bright backgrounds than the red SOS sign.
- **Underwater.** The only underwater labels are one GoPro video, and it is in
  training. There is no held-out underwater score with signs present yet. Record the
  ZED at the next pool test with the gate in view.
- **Bins** have no validation video, so treat bin detections as untested.
