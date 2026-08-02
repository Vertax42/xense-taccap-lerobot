# `taccap_gripper` — Handheld data-collection device

The **TacCap-Gripper** (**TacCap** = _Tactile Capture_ Gripper) is a handheld
**UMI** leader gripper for tactile data collection.

Single-arm handheld data-collection pipeline. The device is **self-driven**:
there is no separate teleoperator — the robot itself produces both the
observation (state being recorded) and the demonstration action.
`lerobot-record` allows `teleop=None` for this self-driven device, so no
`--teleop.*` flags are needed on the CLI.

This is the **single-gripper** device: one unit, two tactile pads, one wrist camera, one
Pico4 tracker, and **unprefixed** observation keys (`tcp.*`, `gripper.pos`,
`tactile_left` / `tactile_right`, `wrist_cam`). For two units driven as one robot — with
`left_` / `right_` prefixes and the optional Insight head camera — see
[`bi_taccap_gripper`](../bi_taccap_gripper/README.md). Both sides of a two-gripper rig can
also be run one at a time through this device by passing `--robot.side`.

Components:

- **Gripper:** TacCap-Gripper handheld unit (motor jaw, two embedded
  visuotactile sensors, wrist UVC camera, encoder, IMU). Driven by the
  `xense.taccap` SDK (`taccap-gripper` PyPI package, ≥ 0.1.0).
- **Pose:** Pico4 Ultra **independent motion tracker** mounted on top
  of the gripper. Reached via `xensevr_pc_service_sdk` and read by
  `lerobot.teleoperators.pico4.tracker.Pico4TrackerReader`.
- **Cameras:** plain LeRobot `cameras/` framework, read asynchronously.
  The two tactile sensors and the wrist UVC camera are **auto-discovered by
  serial rule** (`serial_discovery.py`) from `/dev/v4l/by-id` — no serials are
  supplied in config. See "End-to-end recording" below.

The device is passive: `send_action()` is a no-op; the motor is never
enabled. The operator drives the jaw mechanically and walks the device
through demonstrations.

## Coordinate frame

Recorded pose is in **our world frame by default** (X forward away from base,
Y left, Z up, gravity-aligned): `Pico4TrackerReader` applies the same Pico→world
remap the `teleop_pico4` controller flow uses (`pico_to_world=True`). The world
origin is the headset position the moment the Unity VR Client app started.

This world frame is **not** any specific robot's base frame, and deliberately so
— recorded poses stay robot-agnostic, and the base difference is cancelled at
training/deployment time by the relative pose representation rather than baked in
here. See "Why there is no init-pose alignment" below.

The axis convention (handedness, Z direction) is documented
inconsistently upstream — Pico docs claim right-handed, the SDK's
`rerun_dual_with_tracker.py` notes left-handed. **TBD pending live
verification on real hardware.**

**Do not restart the Unity client between episodes** or all subsequent
recordings will be in a different origin.

### Tracker → EEF TCP mount transform

The Pico4 tracker is bolted to the gripper, so the pose it reports is the
**tracker's**, not the TCP's — on this unit the two are ~195 mm apart. The
constant rigid offset lives in [`ee_transform.py`](ee_transform.py) and is applied
body-fixed by `Pico4TrackerReader`:

```
T_world_tcp = T_world_tracker @ X
```

Because `X` is body-fixed it rides along with the gripper, so it holds at any
orientation — starting a UMI session with the gripper pointing anywhere is fine.

- **TCP** is the two-finger midpoint. Symmetric jaws keep that point still as the
  jaw opens, so the transform is a constant and does not depend on `gripper.pos`.
- **Both sides are measured**, straight out of the SolidWorks macro as
  `^Tracker T_EE`. They are near-mirrors about the XZ plane but not exactly —
  rotation agrees with the mirror to 0.03°, translation differs by 1.27 mm — so
  neither side is derived from the other. `mirror_xz()` is kept only as a
  consistency check.
- **Leader bodies only.** The follower gripper is a different design (URDF shows
  different joint origins, a flipped jaw axis, fingertips 21 mm further out), so
  `--robot.role=follower` warns and falls back to the leader value.
- `--robot.tracker_to_ee_pos` / `_quat` default to `None` = use this side's
  built-in value. Set either one to override; they are independent, so a
  re-machined mount can pin just the translation.

> **Cross-check on the numbers.** At the CAD reference pose the EE frame is
> aligned to world, so `delta_world == R_tracker_eeᵀ · t_tracker_ee`. The macro's
> right-side values reproduce the independent measurement in
> `media/right_eef_tcp.jpg` to **0.000 mm**, which also settles that screenshot's
> sign question — its Y and Z really are negative.

> **Datasets recorded before this landed hold the tracker pose in `tcp.*`, not the
> TCP.** They are off by the handle offset and must not be mixed with newer
> episodes without re-transforming.

### Checking the mount transform in Rerun

`lerobot-teleoperate --display_data=true` draws **both frames** in the `/world` view:
the EE frame (large marker, 10 cm axes, labelled `EE`) and the tracker's own frame
(small dim marker, 6 cm axes, labelled `TRACKER`), joined by a thin dashed yellow
construction line labelled with its length in mm. The tracker pose is published as
display-only `tracker.*` keys — absent from `observation_features`, so it never
reaches a dataset — and a `tracker pose` tab appears in the scalar panel next to
`tcp pose`.

The scene declares `rr.ViewCoordinates.FLU` (X forward, Y left, Z up) rather than
the weaker `RIGHT_HAND_Z_UP`, so the viewer knows which axis is _forward_ and aims
its initial camera down +X. The origin triad is labelled `+X forward` / `+Y left` /
`+Z up` so the orientation stays readable after you orbit.

What to look for, with the gripper lying flat:

| Check              | Expected                                                                           |
| ------------------ | ---------------------------------------------------------------------------------- |
| Segment length     | **≈195 mm** (right) / **≈194 mm** (left), and **constant** as you wave the gripper |
| EE marker position | at the **two-finger midpoint**                                                     |
| EE axes when flat  | X forward, Y left, Z up — i.e. **level**                                           |

**Checked on 2026-08-02 and correct**, so this is a regression check now rather
than an open question. The middle row is the one that matters: it is the only one
that distinguishes `APPLY_G_REBASE` — both settings put the EE the right 195 mm
from the tracker, just 51° apart, so the segment length looks fine either way.
If the EE ever lands somewhere unrelated to the fingers, flip
`APPLY_G_REBASE` in [`ee_transform.py`](ee_transform.py).

Verify a mount with the pivot check — no extra hardware needed:

```bash
python -m lerobot.robots.taccap_gripper.calibrate_tracker --side right
```

Rest the two-finger midpoint on a fixed point and sweep the handle through as
many orientations as it allows. `ee xyz` should stay put while `raw xyz` swings;
whatever drift remains is the transform's error. Run it for **both** sides — a
left value mirrored the wrong way shows up as `ee` moving about twice as much as
it should.

### Why there is no init-pose alignment

`Pico4TrackerReader` can latch a rigid transform at connect time so every later
pose comes out in the frame of a supplied robot TCP pose:

```
T_align  = T_ee_init · (T_world_tracker(0) · T_tracker_ee)⁻¹
T_out(t) = T_align · T_world_tracker(t) · T_tracker_ee
```

That is a **live-teleoperation** formula, and this device does not use it. Two
reasons:

- `T_ee_init` is in a robot's base frame, so it needs that robot **present and
  localised at connect time** — which a handheld capture rig does not have.
- Even with a value, it would bake one arm's base into the dataset; a second arm
  would mean re-recording.

Base-frame differences are handled downstream instead: with the poses expressed
relative to the current EE frame, any global transform `M` between capture and
deployment cancels exactly —

```
inv(M·T(t)) · M·T(t+k) = inv(T(t)) · T(t+k)
```

— so the deployment arm's base, and the initial EE orientation it happens to sit
at, drop out on their own. The robot end only has to calibrate its TCP to the
gripper's EEF.

The reader keeps the mechanism (it is generic, and a live teleoperator would want
it); it is simply not wired to the capture path, and the config fields that used
to expose it are gone. Stacking it with a relative representation would re-base
twice.

## Hardware bring-up sequence

1. Plug the TacCap-Gripper into the host (USB).
2. Power on the Pico4 Ultra headset; pair the motion tracker.
3. Launch the Unity VR Client app on the headset (this freezes the
   coordinate origin).
4. Start the XenseVR PC Service on the host.
5. Run any of the scripts below.

> **Serial-port permissions (one-time host setup).** The gripper MCU enumerates
> as `/dev/ttyACM*`, owned by the `dialout` group. If your user is not in
> `dialout`, the SDK can _list_ the grippers but cannot open the port to read
> the firmware SN — `scan_grippers()` then reports `role=Unknown` / empty
> `firmware_sn`, and `connect()` fails with
> `RuntimeError: No <role> gripper discovered for the <side> side.` (the
> underlying error is `IoError: ... Permission denied`). Fix it once with
> `sudo usermod -aG dialout "$USER"`, then start a fresh session (or
> `newgrp dialout`) and replug. See the
> [top-level README → Installation Step 6](../../../../README.md#-installation)
> for the full check.

> **`Device or resource busy` on the gripper serial after replug (ModemManager).**
> The gripper MCU is a CH343 USB-serial (`1a86:55d2`) that enumerates as a
> CDC-ACM port. On every hot-plug, **ModemManager** (the cellular-modem service
> shipped by default on Ubuntu/GNOME) probes the fresh port with AT commands and
> holds it open for a few seconds — so a `connect()` in that window fails with
> `IoError: SerialBus: open(/dev/serial/by-id/...): Device or resource busy`.
> Classic symptom: the **first** launch works (the port has settled), but
> unplug → move to another port → relaunch immediately **busy**. (`brltty`, the
> braille driver, grabs `1a86` devices the same way if installed.) Quick
> workaround: wait ~3 s after replug. Permanent fix — tell ModemManager to ignore
> these devices via a udev rule (does **not** disable it for real modems):
>
> ```bash
> sudo tee /etc/udev/rules.d/99-taccap-ignore-modemmanager.rules >/dev/null <<'EOF'
> # TacCap-Gripper MCUs are CH343 USB-serial (1a86:55d2) — keep ModemManager off them
> ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"
> EOF
> sudo udevadm control --reload-rules && sudo udevadm trigger
> ```
>
> Verify: `udevadm info -q property -n /dev/ttyACM0 | grep ID_MM_DEVICE_IGNORE`
> shows `ID_MM_DEVICE_IGNORE=1`, and `mmcli -L` no longer lists the grippers.
> Revert by deleting the rule file and reloading. (Alternatively, on a dedicated
> robot PC with no cellular modem: `sudo systemctl disable --now ModemManager`.)

## Calibration workflow (do once per device)

### 1. Encoder zero + travel span

The SDK ships the calibration CLI. It pins by firmware SN — so with both sides
plugged in you cannot zero the wrong one — and does **both** ends in one pass:

```bash
python third_party/taccap-gripper/python/examples/calibrate.py TCGU01A28Z0024m
```

1. Hold the gripper **fully closed** → latched as the encoder zero
   (`Encoder.set_zero()`), then re-read to confirm the post-zero residual.
2. Open to the **mechanical limit** → written to MCU flash as that unit's
   encoder-max (`Cmd::EncoderMaxCal`, firmware ≥ V2.1).

List available firmware SNs:

```bash
python -c "from xense.taccap import scan_grippers, Side; \
  [print(f'{\"L\" if g.side==Side.Left else \"R\"} fw={g.firmware_sn} mcu={g.mcu_serial}') for g in scan_grippers()]"
```

**Do this on every unit.** Step 2 is what `gripper.pos` is normalised against:
with it, the SDK reports the true opening of _that_ gripper in [0, 1]; without
it the robot falls back to dividing by `gripper_open_rad`, a single config
constant (default 1.7) standing in for every gripper ever built. Real travel
varies — a measured unit came out at **1.1486 rad (65.8°)**, which under the
fallback tops out at `1.1486 / 1.7 = 0.676` and never reaches 1.0.

In a bimanual rig, calibrating only one side is the worst case: the two
`{side}_gripper.pos` channels end up on different scales, so the same physical
grip reads differently left and right. The connect log says which path each side
took — `Jaw normalised by the firmware's encoder-max calibration` versus
`Firmware encoder-max calibration unavailable …`.

Closed is always 0 — there is no `gripper_closed_rad` config, the zero lives in
firmware. `gripper_open_rad` is now only the fallback for uncalibrated or
pre-V2.1 units, and for followers (`Cmd::EncoderMaxCal` is leader-only).

### 2. Sanity-check the Pico4 tracker

```bash
python -m lerobot.robots.taccap_gripper.calibrate_tracker
# or, pin to a specific tracker SN:
python -m lerobot.robots.taccap_gripper.calibrate_tracker PC2310MLL3200496G
```

Watch the `raw xyz` move smoothly when you wave the gripper. The
`ee xyz` is `raw` after the rigid `tracker_to_ee_*` mount transform —
identity by default. Measure your physical mount offset and put it in
the config (`tracker_to_ee_pos`, `tracker_to_ee_quat`).

## Live visualization (`lerobot-teleoperate`)

Before recording anything, stream the device to Rerun to confirm both tactile pads, the
wrist camera and the pose are alive. The device is self-driven, so **no `--teleop.*`
flags** — `lerobot-teleoperate` just pumps `get_observation()` into the viewer:

```bash
lerobot-teleoperate \
    --robot.type=taccap_gripper \
    --fps=30 \
    --display_data=true
```

Add `--robot.side=left|right` only when both grippers are connected; a single unit
auto-resolves. Add `--robot.enable_tracker=false` to skip the Pico4 / XenseVR PC service
entirely and watch tactile + gripper only.

### Viewer layout

`--display_data=true` sends a blueprint rather than letting Rerun auto-lay-out, which would
give each tactile pad the same screen area as everything else. For a single rig (no head
camera) that resolves to:

- **Left, largest**: the `/world` 3D trajectory view.
- **Right**: the wrist camera, then the two tactile pads in their own grid.
- **Bottom**: scalars in tabs by unit — `gripper.pos`, `tcp pose`, `imu` (when
  `--robot.enable_imu=true`), plus a catch-all `all` tab. Metres, unit-length rotation
  components and accelerations share no y-axis, so one merged plot would be unreadable.

With `--robot.enable_tracker=false` there is no 3D view, and the wrist + tactile views take
the whole top half.

The tactile pads you see are the **display-only** `difference` view
(`tactile_{left,right}_difference`), not the `rectify` stream being recorded: the viewer is
laid out from the robot's `display_features` rather than `observation_features` — same
schema, with each tactile sensor's recorded stream swapped in place for its display one —
and `log_rerun_data` is fed the matching subset of the observation
(`select_display_observation`), so the recorded stream never reaches Rerun. The tile count
is unchanged; what is easiest to read live is simply not what lands in the dataset.

### 3D trajectory

In the `/world` view the gripper is drawn as a labelled ellipsoid + axis triad at its live
Pico4 pose (`tcp.*`), trailing a breadcrumb of where it has been — mirroring the SDK's
[`rerun_dual_with_tracker.py`](../../../third_party/taccap-gripper/python/examples/rerun_dual_with_tracker.py)
example. Our pose is already in the gravity-aligned world frame, so the scene is
`FLU` — X forward, Y left, Z up (the example shows the raw Pico `LEFT_HAND_Y_UP` frame).

On by default; `--show_trajectory=false` drops **that view only** (the rest of the layout
still applies), and it auto-skips when `--robot.enable_tracker=false` (no pose to draw).
Implemented in [`visualization.py`](visualization.py) and shared by both teleoperate and
record — same flags on `lerobot-record`.

## Standalone smoke test

Verifies the robot stack independently of `lerobot-record`. Devices are
auto-discovered; pass `--side` only when both grippers are connected:

```bash
# Gripper + tactile + wrist, all auto-discovered (pick a side if both present):
python -m lerobot.robots.taccap_gripper.taccap_gripper_example --side left

# Cameras + gripper only (no wrist camera):
python -m lerobot.robots.taccap_gripper.taccap_gripper_example --side left --no-wrist

# + Pico4 tracker (pose). --tracker-sn is optional — omit it to auto-discover
# by the second-to-last-digit side rule:
python -m lerobot.robots.taccap_gripper.taccap_gripper_example \
    --side left --tracker --tracker-sn PC2310MLL3200496G

# + IMU channels, and a different jaw-open calibration:
python -m lerobot.robots.taccap_gripper.taccap_gripper_example \
    --side left --imu --open-rad 1.7 --frames 30
```

## End-to-end recording

`taccap_gripper` runs without a teleoperator (`RecordConfig.__post_init__`
allows `teleop=None` for it). Recording is handled by the dedicated
`self_driven_record_loop` in `lerobot_record.py` (the device is routed there
via `SELF_DRIVEN_RECORD_ROBOTS`). Each recorded row uses **shifted-frame**
pairing: the observation from step _t-1_ is paired with the pose at step _t_
(Pico4 pose + normalised `gripper.pos`) as the action, so the action leads
its observation by one step — a real "move-to-next" target rather than the
degenerate same-frame pose. One frame is dropped per episode (the first
sample has no predecessor). The between-episode reset phase is a passive
wait: reposition the device, no teleop needed. **No `--teleop.*` flags.**

Devices are **auto-discovered by serial rule** — no gripper/tactile/camera serials
are listed. With a single gripper connected it is picked up automatically; when both
are connected, set `--robot.side=left|right`:

```bash
lerobot-record \
    --robot.type=taccap_gripper \
    --robot.id=right \
    --robot.side=right \
    --dataset.repo_id=<your_org>/<your_dataset> \
    --dataset.num_episodes=1 \
    --dataset.episode_time_s=10 \
    --dataset.single_task='Pick up the object'
```

With the Pico4 tracker powered on, 6-DoF pose is recorded automatically — the tracker
is auto-discovered and matched to this unit's side by its serial's second-to-last digit
(odd → left, even → right). Add `--robot.enable_tracker=false` to record tactile +
gripper only.

To bypass the side rule — e.g. a tracker whose serial does not follow it, or when PC-service
enumeration is flaky — pin the serial directly with `--robot.tracker_serial=<SN>`. A pinned
serial is used **verbatim**: no enumeration, no rule check (a typo surfaces as a device-not-found
at connect). Leave it unset (default) to keep auto-discovery.

- **Tactile** → obs keys `tactile_left` / `tactile_right`; landscape `(400,700,3)` uint8
  (width/height auto-derive — don't hard-code). Each sensor is read once per frame for
  **two views with two destinations**:
  - **recorded** — `--robot.tactile_output_types`, default `rectify` (exactly one type).
    The unsubtracted image; this is the only tactile key in `observation_features`, so
    it is the only one that lands in the dataset.
  - **displayed** — `--robot.tactile_display_output_types`, default `difference` (SDK
    `OutputType.AugDifference`), published as `tactile_{left,right}_difference`. It
    amplifies deformation the raw `rectify` image barely shows, which is what you want
    to watch live, but it is deliberately kept out of `observation_features` and never
    recorded: its baseline is taken at sensor init, so a finger loaded at connect would
    have that pressure subtracted out of the whole run. Keep the fingers **unloaded** at
    connect for a readable live view. Set to `'[]'` to skip the second read; Rerun then
    shows the recorded stream.

  Rerun is fed `display_features` (recorded stream swapped for the displayed one), so
  the viewer shows only `difference` and the tile count is unchanged.
  `--robot.tactile_diff_gain` (default `1.0`, sensors ship at `1.5`) is the linear gain on
  that difference: 1.0 cuts per-pixel temporal noise from ~1.77 to ~1.18 grey levels and
  stops the image clipping, but scales signal down with it — raise it if light contact
  becomes invisible, set it to `None` to keep whatever the sensor was flashed with.
  Tune `--robot.tactile_fps`; `--robot.expected_tactiles_per_side` validates the count.
  The two sensors are paired to this unit's gripper by **USB hub**; `left`/`right` finger
  comes from the GSPS serial's **last digit** (odd → `left`, even → `right`, 单左双右).

- **Wrist** → obs key `wrist_cam`; `--robot.enable_wrist_camera=false` skips. Tune
  `--robot.wrist_camera_width/_height/_fps`.
- **Role**: `--robot.role=follower` binds the Slave units (default `leader`).

### Streaming video encoding & encoder warmup

Video keys (tactile + wrist) are encoded **in real time during capture** rather than
written as PNGs and encoded at episode end, so `save_episode()` is near-instant. This is
on by default (`--dataset.streaming_encoding=true`); pair it with
`--dataset.encoder_threads=2` and optionally `--dataset.vcodec=auto` for hardware
encoding. One `_CameraEncoderThread` runs per camera, fed raw frames through a bounded
queue (`--dataset.encoder_queue_maxsize`, ~1 s of frames); if the encoder can't keep up
the oldest frames are dropped with a warning rather than stalling the loop.

**Encoder warmup.** Opening the PyAV container + codec context is expensive (~25 ms), and
doing it lazily on the episode's first frame made that frame badly overrun the `fps`
budget. To avoid this, `dataset.prepare_episode_recording()` is called once before each
episode's record loop: it starts the encoder threads and opens their codec contexts up
front (using each video key's declared `(H, W, C)` shape), blocking until every encoder
reports ready. By the time the first frame is recorded the encoders are hot, so the first
`add_frame()` no longer pays the init cost. The lazy first-frame start remains as a
defensive fallback for callers that don't pre-warm.

## What gets recorded per frame

Keys are **unprefixed** — unlike [`bi_taccap_gripper`](../bi_taccap_gripper/README.md),
which prefixes everything `left_` / `right_`. A single-arm dataset is therefore not a
column subset of a bimanual one.

| Key                             | When                  | Source                                                 | Shape / type                                         |
| ------------------------------- | --------------------- | ------------------------------------------------------ | ---------------------------------------------------- |
| `tcp.x`, `tcp.y`, `tcp.z`       | `enable_tracker`      | Pico4 tracker → EE                                     | float (m)                                            |
| `tcp.r1`..`tcp.r6`              | `enable_tracker`      | 6-D rotation of EE                                     | float                                                |
| `gripper.pos`                   | `enable_gripper`      | TacCap encoder, normalised                             | float ∈ [0, 1]                                       |
| `imu.accel.{x,y,z}`             | `enable_imu`          | TacCap IMU                                             | float (m/s²)                                         |
| `imu.gyro.{x,y,z}`              | `enable_imu`          | TacCap IMU                                             | float (rad/s)                                        |
| `imu.mag.{x,y,z}`               | `enable_imu`          | TacCap IMU                                             | float (µT)                                           |
| `tactile_left`, `tactile_right` | auto-discovered       | Xense sensor on that finger, recorded view (`rectify`) | uint8 (H, W, 3), landscape — currently (400, 700, 3) |
| `wrist_cam`                     | `enable_wrist_camera` | wrist UVC via `cameras/`                               | uint8 (H, W, 3)                                      |

The flags are all `--robot.*` (e.g. `--robot.enable_imu=true`); the tracker and gripper
ones are on by default, `enable_imu` is off. Disabling one **removes** its keys from the
schema rather than zero-filling them, so `observation.state` is 10-D by default
(9 pose + 1 jaw) and 19-D with the IMU on.

Tactile cameras contribute their **recorded** view only (`rectify` by default). The
`tactile_{left,right}_difference` keys `get_observation()` also returns are display-only:
they are absent from `observation_features`, so `build_dataset_frame` never sees them and
nothing is written for them.

`action_features` is the `tcp.*` + `gripper.pos` subset — images are observation-only.

The 6-D rotation convention matches `vive_tracker`:
`r1..r3` is the first column of the rotation matrix, `r4..r6` is the
second column.

## Files in this package

- `taccap_gripper.py` — the `Robot` subclass. `get_observation()` and
  `get_action()` both surface pose + gripper + optional IMU + cameras.
- `config_taccap_gripper.py` — `RobotConfig` dataclass.
- `taccap_gripper_example.py` — standalone smoke test (above).
- `calibrate_tracker.py` — sanity-check the Pico4 tracker.

Encoder zero calibration lives in the SDK itself
(`third_party/taccap-gripper/python/examples/calibrate.py`) — we no
longer ship a duplicate.

The Pico4 tracker reader is shared with future devices and lives at
`src/lerobot/teleoperators/pico4/tracker.py`.

The integration point in the record script is
`src/lerobot/scripts/lerobot_record.py`:

- `RecordConfig.__post_init__` allows `teleop=None` for the self-driven
  robots (`SELF_DRIVEN_RECORD_ROBOTS`).
- The dispatch in `record()` routes `taccap_gripper` / `bi_taccap_gripper`
  to the dedicated `self_driven_record_loop` (shifted-frame pairing).
