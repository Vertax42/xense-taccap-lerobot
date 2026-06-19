# `taccap_gripper` — Handheld data-collection device

Single-arm handheld data-collection pipeline. The device is **self-driven**:
there is no separate teleoperator — the robot itself produces both the
observation (state being recorded) and the demonstration action.
`lerobot-record` allows `teleop=None` for this self-driven device, so no
`--teleop.*` flags are needed on the CLI.

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

This world frame is **not** any specific robot's base frame — for cross-robot
transfer, use the opt-in init-pose alignment below to rebase recorded poses into
a deployment robot's frame.

The axis convention (handedness, Z direction) is documented
inconsistently upstream — Pico docs claim right-handed, the SDK's
`rerun_dual_with_tracker.py` notes left-handed. **TBD pending live
verification on real hardware.**

**Do not restart the Unity client between episodes** or all subsequent
recordings will be in a different origin.

### Opt-in: UMI-style init-pose alignment (reserved for deployment)

Mirrors the `vive_tracker` UMI flow. When enabled,
`Pico4TrackerReader.connect()` snapshots the first valid tracker pose
and computes a rigid transform so all subsequent recorded poses are
in the **same frame as `init_tcp_pose`** (typically the deployment
robot's base frame at its home configuration).

Config fields (default OFF — needs live deployment-hardware verification first):

```python
enable_init_pose_alignment: bool = False
init_tcp_pose: tuple[float, ...] = (
    0.693307, -0.114902, 0.14589,
    0.004567, 0.003238, 0.999984, 0.001246,
)  # example deployment-robot home pose
```

Workflow when ready to enable:
1. Place the gripper in its "init" stance — physically matching the
   robot's home configuration on a workbench (same orientation +
   roughly the height the robot's EE would reach).
2. Set `--robot.enable_init_pose_alignment=true`.
3. Run `lerobot-record`. The first valid tracker pose is latched and
   alignment is computed automatically. From frame 0 the recorded
   `tcp.x/y/z/r1-r6` are in the robot's base frame and can train
   directly without post-processing.

When alignment is off (default), records stay in the raw xrt frame
and downstream tooling must reframe.

## Hardware bring-up sequence

1. Plug the TacCap-Gripper into the host (USB).
2. Power on the Pico4 Ultra headset; pair the motion tracker.
3. Launch the Unity VR Client app on the headset (this freezes the
   coordinate origin).
4. Start the XenseVR PC Service on the host.
5. Run any of the scripts below.

## Calibration workflow (do once per device)

### 1. Latch the encoder zero

The SDK ships a complete calibration CLI. Use it directly — it pins by
firmware SN, latches the zero via `Encoder.set_zero()`, verifies the
post-zero raw residual, and optionally sanity-checks the open angle:

```bash
python third_party/taccap-gripper/python/examples/calibrate.py SN000003
```

List available firmware SNs:

```bash
python -c "from xense.taccap import scan_grippers, Side; \
  [print(f'{\"L\" if g.side==Side.Left else \"R\"} fw={g.firmware_sn} mcu={g.mcu_serial}') for g in scan_grippers()]"
```

After zero is latched, the SDK's `position_rad` reads 0 when closed
and rises to ~1.7 rad (~97°) at the mechanical limit. There is **no
`gripper_closed_rad` config** — closed is always 0. Only the
`gripper_open_rad` config field (default 1.7) is configurable per unit.

### 2. Sanity-check the Pico4 tracker

```bash
python -m lerobot.robots.taccap_gripper.calibrate_tracker
# or, pin to a specific tracker SN:
python -m lerobot.robots.taccap_gripper.calibrate_tracker LHR-XXXXXXXX
```

Watch the `raw xyz` move smoothly when you wave the gripper. The
`ee xyz` is `raw` after the rigid `tracker_to_ee_*` mount transform —
identity by default. Measure your physical mount offset and put it in
the config (`tracker_to_ee_pos`, `tracker_to_ee_quat`).

## 3D trajectory visualization

With `--display_data=true`, the Rerun viewer adds a `/world` 3D view: the gripper is drawn
as a labelled ellipsoid + axis triad at its live Pico4 pose (`tcp.*`), trailing a breadcrumb
of where it has been — mirroring the SDK's
[`rerun_dual_with_tracker.py`](../../../third_party/taccap-gripper/python/examples/rerun_dual_with_tracker.py)
example. Our pose is already in the gravity-aligned world frame, so the scene is
`RIGHT_HAND_Z_UP` (the example shows the raw Pico `LEFT_HAND_Y_UP` frame).

On by default; `--show_trajectory=false` suppresses it, and it auto-skips when
`--robot.enable_tracker=false` (no pose to draw). Implemented in
[`visualization.py`](visualization.py) and shared by both teleoperate and record.

## Standalone smoke test

Verifies the robot stack independently of `lerobot-record`. Devices are
auto-discovered; pass `--side` only when both grippers are connected:

```bash
# Gripper + tactile + wrist, all auto-discovered (pick a side if both present):
python -m lerobot.robots.taccap_gripper.taccap_gripper_example --side left

# Cameras + gripper only (no wrist camera):
python -m lerobot.robots.taccap_gripper.taccap_gripper_example --side left --no-wrist

# + Pico4 tracker (pose); pin its PT- serial:
python -m lerobot.robots.taccap_gripper.taccap_gripper_example \
    --side left --tracker --tracker-sn PT-XXXXXXXXXXXX
```

## End-to-end recording

`taccap_gripper` runs without a teleoperator (`RecordConfig.__post_init__`
allows `teleop=None` for it). Recording is handled by the dedicated
`self_driven_record_loop` in `lerobot_record.py` (the device is routed there
via `SELF_DRIVEN_RECORD_ROBOTS`). Each recorded row uses **shifted-frame**
pairing: the observation from step *t-1* is paired with the pose at step *t*
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

- **Tactile** → obs keys `tactile_0` / `tactile_1`; the rectify image is landscape
  `(400,700,3)` (width/height auto-derive — don't hard-code). Tune `--robot.tactile_fps`
  / `--robot.tactile_output_types`; `--robot.expected_tactiles_per_side` validates the count.
- **Wrist** → obs key `wrist_cam`; `--robot.enable_wrist_camera=false` skips. Tune
  `--robot.wrist_camera_width/_height/_fps`.
- **Role**: `--robot.role=follower` binds the Slave units (default `leader`).

## What gets recorded per frame

| Key | Source | Shape / type |
|---|---|---|
| `tcp.x`, `tcp.y`, `tcp.z` | Pico4 tracker → EE | float (m) |
| `tcp.r1`..`tcp.r6` | 6-D rotation of EE | float |
| `gripper.pos` | TacCap encoder, normalised | float ∈ [0, 1] |
| `imu.accel.{x,y,z}` (opt) | TacCap IMU | float (m/s²) |
| `imu.gyro.{x,y,z}` (opt) | TacCap IMU | float (rad/s) |
| `imu.mag.{x,y,z}` (opt) | TacCap IMU | float (µT) |
| `<camera_name>` per camera | `cameras/` framework | uint8 (H, W, 3) |

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
