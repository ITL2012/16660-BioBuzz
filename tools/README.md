# BioBuzz computer tools (Python + OpenCV)

Two desktop tools that support the robot's vision and shooter calibration.
They run on any machine with Python 3 — your Mac is fine.

```bash
cd tools
python3 -m pip install -r requirements.txt
```

## 1. `hsv_calibrator.py` — tune the ball/flower/robot color ranges

A GUI with HSV sliders, live mask preview, automatic bounding boxes, and
mouse-drawn bounding boxes. Run it on a video of the field or a live webcam:

```bash
python3 hsv_calibrator.py --camera 0                 # live webcam
python3 hsv_calibrator.py field_video.mp4            # a recorded video
python3 hsv_calibrator.py video.mp4 --name POLLEN_HSV_RANGE
```

- **Sliders** (H/S/V min & max) live in the "HSV Controls" window. OpenCV's hue
  runs **0–180 — the exact same scale the robot's Kotlin `ColorRange` uses**, so
  values transfer 1:1.
- **Bounding boxes**: the tool draws green/yellow boxes around everything the
  current range matches. To build a range from a sample instead, **drag a box
  around a ball** with the mouse, then press **`f`** — the sliders jump to the
  min/max HSV inside your box. Nudge from there.
- Press **`s`** to print a paste-ready `ColorRange(...)` snippet for `globals.kt`.
- Press **`p`** to save the range as a JSON preset (default `hsv_presets.json`) —
  the flywheel analyzer can reuse it.
- `SPACE` pause, `u` undo box, `q` quit.

## 2. `flywheel_analyzer.py` — automatic shooter calibration from video

Films of the calibration shots are analyzed automatically: it tracks the ball,
finds where each shot lands, converts pixels to cm, and prints the numbers
ready to paste into `globals.kt` (`SHOOTER_CALIBRATION_DISTANCE_CM`).

### Camera setup (do this once)

1. **Fix the camera** (tripod / propped phone). It must not move during filming.
2. Point it **roughly side-on** to the ball's flight path, so the ball's
   *lowest point in the frame* is where it touches the floor.
3. Put **two tape marks on the floor a known distance apart** (e.g. 100 cm),
   in view of the camera, roughly along the flight direction.
4. Tune the ball's HSV first with `hsv_calibrator.py` and save the preset
   (`p`), or plan to pass `--hsv` numbers.

### Robot + filming procedure

1. Run the **"Flywheel Calibration"** OpMode (group *Test* on the Driver
   Station). Pick a RPM step, press **A** to spin up, wait for `AT SPEED`,
   press **X** to feed one ball. Film each shot.
2. Film the shots **in order 6000 → 5000 → 4000 → 3000 → 2000 → 1000** (one
   video is easiest; one video per shot also works — run the analyzer once per
   video with a single `--rpm` value).

### Analyze

```bash
# one video with all six shots (default order 6000..1000):
python3 flywheel_analyzer.py calib.mp4

# ball color from a preset saved by hsv_calibrator.py:
python3 flywheel_analyzer.py calib.mp4 --preset hsv_presets.json --preset-name POLLEN_HSV_RANGE

# or pass the range inline (H_MIN S_MIN V_MIN H_MAX S_MAX V_MAX):
python3 flywheel_analyzer.py calib.mp4 --hsv 14 100 100 35 255 255

# one video per shot:
python3 flywheel_analyzer.py shot_6000.mp4 --rpm 6000
```

The first frame opens a **click-calibration** window:

- Click **1**: the launch point (where the ball exits the flywheel).
- Clicks **2 & 3**: your two tape marks (`--ref-cm 100` by default — change it
  if your marks are a different distance apart).
- `c` confirms, `u` undoes.

It then plays the annotated video while tracking (press `q` to fast-forward
without display) and prints:

- the measured landing distance per RPM,
- a paste-ready `SHOOTER_CALIBRATION_RPM` / `SHOOTER_CALIBRATION_DISTANCE_CM`
  block for `globals.kt` (sorted 6000 → 1000),
- a `*_calibration.json` next to the video with the raw results.

### Finish on the robot

Paste the printed distances into `globals.kt`. That's it — `ShooterModel`
immediately starts interpolating the curve, the shooting window widens to the
calibrated reach, and teleop/autonomous spin the flywheel to the RPM the live
hive distance needs (firing only once **both** wheels are at speed). While
idle the flywheel stays primed at `FLYWHEEL_PRIME_RPM` so spin-up is quick.

### Analyzer assumptions & knobs

- Side-on, static camera (landing = lowest tracked point).
- One ball visible at a time; other same-colored objects near the launcher can
  confuse tracking (move them away while calibrating).
- `--launch-radius` (default 120 px): how near the launch point a new shot
  must appear; raise it if shots aren't detected, lower it if tracks merge.
- `--lost-frames` (default 25): how long the ball can vanish before a track
  ends; raise for low frame-rate videos.
- `--min-area` (default 80 px²): noise floor for the ball blob.

## 3. `blackbox_builder.py` — turn recorded matches into datasets

Every OpMode on the `feature/superhuman-autonomy` branch records a full-match
Black Box session to the Robot Controller's storage (passive, onboard-only —
see `docs/LEGALITY-REVIEW.md`). A session is **ONE SQLite file**: the robot
state (pose, flywheel RPMs, battery, gamepad inputs), the world-model tracks,
the planner decisions, the events, and the subsampled JPEG frames from BOTH
robot cameras (stored as BLOBs inside the file). Nothing is JSON any more, so
nothing can be half-parsed.

**Pull the data off the phone** over USB (MTP) from:
`Android/data/org.firstinspires.ftc.robotcontroller/files/blackbox/<label>_<ts>.db`

**Build the dataset** (pure standard library — no installs):

```bash
python3 blackbox_builder.py ../blackbox/teleop_1760000000000.db
python3 blackbox_builder.py SESSION.db --out my_match --copy-frames
```

Outputs `state.csv`, `events.csv`, `frames.csv`, `report.txt` (duration,
distance traveled, battery curve, flywheel peaks, planner decision histogram,
event timeline) and, with `--copy-frames`, sequential `UP_000000.jpg` /
`FRONT_000000.jpg` images synced to the log timestamps.

The shadow-mode planner records what the AI **would** have done each second
while the human drove — compare its decision histogram in `report.txt` against
what actually happened to quantify "is the AI better than us yet?"

## 4. Learning tools — the robot gets better between matches

These close the Black Box loop: matches record data, these tools turn it into
better constants, you paste the output back into `globals.kt`.

### `shooter_refit.py` — the shooter curve learns from every confirmed shot

Every `fire_start` → `shot_confirmed` pair with a live hive distance is a free
calibration sample. Buckets them by distance, averages the RPM that worked,
prints a refined `SHOOTER_CALIBRATION_*` block (any number of points), plus
success-per-RPM-band and battery-voltage-on-misses diagnostics:

```bash
python3 shooter_refit.py ../blackbox
python3 shooter_refit.py ../blackbox --bin-cm 15 --min-samples 3
```

### `planner_fit.py` — planner costs measured, not guessed

Measures real robot speed (from pose deltas), real collect cycles
(`COLLECT_BALL` decision → `ball_collected`), real score cycles, and loader
stroke→confirm timing, then prints suggested `ROBOT_SPEED_CM_PER_SEC`,
`PLANNER_COLLECT_HANDLE_SEC`, `PLANNER_SCORE_TOTAL_SEC`:

```bash
python3 planner_fit.py ../blackbox
```

### `hsv_autotune.py` — color ranges learned from confirmed pickups

Opens the FRONT frames nearest each confirmed `ball_collected` event, runs
your current HSV range, accumulates the ball blobs' real pixel values, and
proposes tightened saturation/value bounds (hue stays). Needs
`opencv-python`. Always sanity-check the proposal with `hsv_calibrator.py`
before pasting:

```bash
python3 hsv_autotune.py ../blackbox --hsv "14 100 100 35 255 255"
python3 hsv_autotune.py ../blackbox --label "Nectar Blue"
```

### The weekly loop

1. Play matches/practice (Black Box on).
2. `blackbox_builder.py` — review the report.
3. `shooter_refit.py` + `planner_fit.py` (+ `hsv_autotune.py` if colors drift).
4. Paste suggested constants into `globals.kt` (after eyeballing them).
5. Repeat — the robot gets sharper every session, and the data proves it.

## 5. Legality

Before using the Black Box frame capture or the SelfPlay OpMode in an
official match, read `docs/LEGALITY-REVIEW.md` (rule-by-rule analysis +
verification checklist). Kill switches: `BLACKBOX_ENABLED` and
`SHADOW_PLANNER_ENABLED` in `globals.kt`.

## 6. Neural nets (all on-robot: TFLite detector + pure-Kotlin shot predictor)

Staged plan: **B (shot predictor) first, then A (ball detector), C (imitation)
later**. Everything runs INSIDE the Robot Controller app on the robot — no
co-processor, no network, lowest latency. Trained models live in the app's
external storage (the same folder that contains `blackbox/`): make an `nn/`
folder there and drop in:

    nn/shot_predictor.json     tiny MLP weights — pure Kotlin runs it
    nn/ball_detector.tflite    tiny YOLO, float-I/O TFLite export

### Stage B — shot-outcome predictor

1. Play matches / run the calibration OpMode with the Black Box on.
2. `python3 train_shot_predictor.py ../blackbox`
   (needs >= 30 logged shots with live hive distances and MIXED outcomes;
   needs `numpy`)
3. Copy the printed JSON to `<app files>/nn/shot_predictor.json`.
4. Run the **Neural Net Test** OpMode — the P(hit) table should look like a
   sensible curve, high where the shooter is dialed in.
5. Keep `SHOT_PREDICTOR_MODE = ADVISORY` for a few matches (every prediction
   is logged as `nn_shot` records — review with blackbox_builder), then flip
   to `REFEREE` to let it nudge low-probability shots by at most +/- 5% RPM.

### Stage A — ball/flower detector (tiny YOLO)

1. `python3 auto_label.py ../blackbox --out dataset`
   The HSV pipeline acts as the teacher: every saved FRONT frame gets
   labeled; background frames are kept as negatives.
2. `python3 label_review.py dataset`
   Fix the ~5% the teacher got wrong: `a` = add-mode then drag to draw,
   click + `d` = delete, `c` = cycle class, `s` = save, `n`/`p` = navigate.
3. `pip install ultralytics` (LAPTOP only), then:
   `python3 train_detector.py dataset --model yolov8n.pt --imgsz 320`
   Any tiny ultralytics model works — use the newest nano your install has.
   Keep `--imgsz 320`: the Control Hub must run this in real time next to
   the rest of vision.
4. Copy the exported `.tflite` to `<app files>/nn/ball_detector.tflite`.
5. Bench-check with the **Neural Net Test** OpMode — detections draw cyan on
   the Driver Station preview. Authority ladder (`NN_DETECTOR_MODE`):
   **ADVISORY** (default: preview + logs only) -> **REFEREE** (removes HSV
   blobs the detector doesn't corroborate) -> **PRIMARY** (also adds
   detector-only finds as targets).

### Stage C — imitation policy (data collection already running)

Shadow-mode logs (planner decisions + full human inputs + outcomes) are the
training set. When there's a pile of sessions, the trainer gets built the
same way: auto-label from logs, train heavy on the laptop, deploy small.

### Performance notes

* `NN_DETECT_HZ = 3` on the Control Hub is a good start — the CPU also runs
  the HSV pipeline, AprilTags, and frame capture. Watch the preview FPS.
* The shot predictor costs microseconds (it's ~50 multiply-adds in Kotlin).
* Both nets log everything (`nn_shot`, `nn_det`, `nn_status` records), so
  ADVISORY mode doubles as the evaluation harness before flipping authority.


