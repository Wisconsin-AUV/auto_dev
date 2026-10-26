# bins_v1

The bins model (`models/bins_v1.onnx` for the ZED SDK, `models/bins_v1.pt` for
ultralytics). Both are installed to `share/wauv_perception/models/` by the `models/*.onnx`
and `models/*.pt` lines in `setup.py`. It is a separate model from `krill_gate_v1`, so
the gate model is unchanged.

To run it instead of the gate model:

```bash
ros2 run wauv_perception yolo_dector --ros-args -p model:=bins_v1.onnx
```

## What it detects

| id | class | boxes in training (real) |
|---|---|---|
| 0 | `bins` | 395 + crates rendered into synthetic scenes |
| 1 | `bin_blood` | 146 + synthetic |
| 2 | `bin_fire` | none yet, never predicted |

The names match Krill's label types. `bin_fire` is in the list so the IDs stay the same
once fire sheets are labelled in Krill.

## Training

- **Data:** the bins labels from Krill export 6, plus 2500 synthetic images
  (`bins_synth_2026-10-23`). The synthetic images paste crate cut-outs from the
  training frames onto other scenes. About 60% of the scenes are pool frames, and the
  pasted crates vary in position, rotation, size, aspect, lighting, shadow noise and
  blue tint, following Shivanth's list. The validation set is whole held-out segments
  of three Krill videos, with a buffer around each, so near-duplicate frames don't
  leak between training and validation.
- **Settings:** fresh `yolo11s.pt` start (not the gate model, because the gate model
  had seen the validation clips), imgsz 960, 30 epochs, best epoch 14. The configs are
  `wauv-yolo/configs/bins_v1.yaml` and `bins_real_only.yaml` (the ablation below).
- **ONNX:** static 1x3x960x960, opset 17, no NMS. These are the same export settings as
  `krill_gate_v1.onnx`.

## Results

Measured on the held-out validation segments at imgsz 960.

| | precision | recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| all | 0.92 | 0.85 | 0.92 | 0.63 |
| `bins` | 0.95 | 0.76 | 0.89 | 0.68 |
| `bin_blood` | 0.88 | 0.94 | 0.95 | 0.58 |

- **Overfitting:** the training-set mAP50 is 0.90, slightly *below* validation, so the
  model is not memorising the training frames.
- **Stress tests on validation (mAP50):** blur 0.91, dark 0.92, blue tint 0.90.
- **False alarms on the sub's pool footage** (370 held-out frames, no bins): 0.3% of
  frames at conf 0.25, 0% at conf 0.5.
- **Real-only ablation:** the same recipe without the synthetic images reached mAP50
  0.87 and started to overfit once mosaic closed. Under blue tint it dropped to 0.04,
  so most of the synthetic set's value is robustness to the water's colour, not the
  peak score.

## Known limits

- **`bin_fire` is untrained.** It needs the Oct 20 phone clips labelled in Krill first.
- **Side views.** On unseen phone clips the model misses crates seen from eye level on
  shelves. The sub will mostly look down at the bins, which is closer to the training
  data, but this hasn't been checked underwater.
- **Blood sheet inside a crate.** On the phone clips `bin_blood` didn't fire at conf
  0.5 for sheets lying inside a crate. Try a lower threshold before judging it.
- **No underwater bins footage yet.** All real labels are from the dry lab. Record the
  ZED over the bins at the next pool test.
