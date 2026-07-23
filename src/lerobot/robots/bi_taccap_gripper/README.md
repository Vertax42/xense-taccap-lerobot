# bi_taccap_gripper

Bimanual TacCap-Gripper handheld data-collection rig — two `taccap_gripper` units
(left + right) driven as one robot. Passive/self-driven: `send_action()` is a no-op
(jaw motors stay disabled, encoders read-only); pose comes from a per-side Pico4
Ultra tracker, tactile + wrist cameras go through the standard `cameras` framework.
An optional Insight9 head camera uses the same robot observation path and contributes
RGB plus a raw-frame VIO pose; no head-to-gripper extrinsic calibration is required
at capture time.

Implemented with the **reimplement-with-prefixes** pattern (cf. `bi_elite_cs66_rt`):
one `Robot` class, per-side handles in dicts keyed `"left"`/`"right"`, and every
observation/action key is `left_`/`right_` prefixed. Per-side reading logic matches
the single [`taccap_gripper`](../taccap_gripper/README.md).

## Observation / action schema

Per side `{s}` ∈ {left, right}:

| Key | When | Meaning |
|---|---|---|
| `{s}_tcp.x/y/z`, `{s}_tcp.r1..r6` | `{s}_enable_tracker` | Pico4 → EE 6D pose |
| `{s}_gripper.pos` | `{s}_enable_gripper` | normalised jaw, 0=closed / 1=open |
| `{s}_imu.{accel,gyro,mag}.{x,y,z}` | `{s}_enable_imu` | IMU |
| `{s}_wrist` | `{s}_enable_wrist_camera` | wrist UVC frame |
| `{s}_tactile_left` / `{s}_tactile_right` | auto-discovered | tactile frame from the left / right finger sensor |
| `head_rgb` | `enable_head_camera` | latest decoded Insight9 RGB frame |
| `head_camera.x/y/z` | `enable_head_camera` | raw Insight9 VIO position in the Insight9 coordinate frame |
| `head_camera.r1..r6` | `enable_head_camera` | raw Insight9 VIO orientation as the first two rotation-matrix columns |

`action_features` = the per-side gripper pose + `{s}_gripper.pos` subset; the head
camera pose and all images remain observation-only. With both Pico4 trackers, both
grippers and Insight9 enabled, `observation.state` has 29 dimensions (20 + 9).

For each fixed-rate robot sample, the adapter calls the Insight9 SDK's `latest()`
exactly once and takes the newest cached RGB and VIO values. The source XYZW quaternion
is converted inside the camera adapter with the shared 6D conversion used by the Pico4
trackers. A corrupt new JPEG holds the previous good RGB frame, and stale RGB/VIO caches
produce a rate-limited runtime warning; no timing, age, status or IMU fields are stored
in the dataset.

## Config — auto-discovered by serial rule

**No device serials are listed.** The two grippers, four tactile sensors and two wrist
cameras are scanned from the connected hardware and assigned to `left`/`right` by the
Xense serial rule:

- **Side** — last sequence digit odd → left, even → right (单左双右).
- **Role** — patch `m` → Master/Leader, `s` → Slave/Follower (`--robot.role`, default
  `leader`).

**Tactile left/right** (`{side}_tactile_{left,right}`) is resolved by **USB hub**,
not by the tactile serial alone: the two GSPS sensors sharing a gripper's USB hub
are that gripper's pair, and the gripper's `side` is read from its **firmware SN**
over the wire (`scan_grippers()` → `ep.side`, i.e. `Cmd::GetSn` — *not* the CH343
`mcu_serial`). Within the pair, the **finger** is the GSPS serial's last digit
(odd → `left` sensor, even → `right`, 单左双右). Because this needs the gripper SDK
scan, tactile discovery runs at construction (grippers must be powered then).

A non-conforming serial, or a side with a missing / duplicated / mis-counted device,
raises a clear error so the config and the physical serials can't drift out of
alignment. See [`serial_discovery.py`](../taccap_gripper/serial_discovery.py).

The **Pico4 motion trackers are auto-discovered too** (no SNs): with `enable_tracker`
on (default), the XenseVR PC service is queried at startup and each tracker is assigned
to left/right by its serial's **second-to-last digit** (odd → left, even → right; e.g.
`PC2310MLL3200496G` → `6` → right). A bimanual rig needs one tracker per side; a
missing/duplicate/malformed tracker raises a clear error. Set `--robot.enable_tracker=false`
to record tactile + gripper only (no PC service needed). Other knobs: `--robot.role`,
`--robot.gripper_open_rad`, `--robot.tactile_fps`, `--robot.wrist_camera_{width,height,fps}`,
`--robot.expected_tactiles_per_side`.

To bypass the tracker side rule, pin serials directly with `--robot.left_tracker_serial=<SN>`
and/or `--robot.right_tracker_serial=<SN>`. A pinned side uses its serial **verbatim** (no
enumeration, no rule check); un-pinned sides still auto-discover by the second-to-last-digit
rule. Use this for a tracker whose serial does not follow the rule, or when enumeration is flaky.

Enable the head camera with `--robot.enable_head_camera=true`. The defaults match the
currently observed native stream (`width=1088`, `height=1920`, dataset FPS 30). Width
and height define and validate the dataset schema; the adapter intentionally does not
write resolution settings back to the device. Override the native library lookup with
`--robot.head_camera_library_path=/path/to/libinsight9.so` when needed. During recording,
RGB or VIO staleness first warns after 0.2 s; if either stream remains unchanged for more
than 3 s, recording aborts with a timeout instead of silently repeating old head data.

Raw acquisition is isolated in
[`../../cameras/insight9/camera_insight9.py`](../../cameras/insight9/camera_insight9.py).
The camera adapter keeps the original Insight9 VIO coordinate frame and converts only
the quaternion representation to `r1..r6`; the robot adapter applies no
head-to-gripper extrinsic.

## Usage

Self-driven — **no `--teleop`**. Prerequisite: `xense.taccap` importable in the
`xense-taccap` env (`bash ./setup_env.sh --install`).

**Live Rerun visualization** (cameras + gripper only — both grippers, 4 tactiles and
2 wrist cameras are discovered automatically):

```bash
lerobot-teleoperate \
    --robot.type=bi_taccap_gripper \
    --fps=30 \
    --display_data=true
```

**Record a dataset** (`self_driven_record_loop`, shifted-frame). With the trackers
powered on, 6-DoF pose is recorded automatically (both trackers auto-assigned by SN);
add `--robot.enable_tracker=false` to record tactile + gripper only:

```bash
lerobot-record \
    --robot.type=bi_taccap_gripper \
    --robot.enable_head_camera=true \
    --dataset.repo_id=Xense/<dataset_name> \
    --dataset.single_task="Pick up the cube" \
    --dataset.num_episodes=20 \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --display_data=true
```

Before enabling it, run `insight9-check-env --hidraw`; the Insight9 HID node must be
readable and writable by the recording user.

## 3D trajectory visualization

With `--display_data=true`, the Rerun viewer adds a `/world` 3D view: each gripper is a
labelled marker (red = left, blue = right) at its live Pico4 pose (`{side}_tcp.*`), trailing a
breadcrumb of its swept path — the same effect as the SDK's `rerun_dual_with_tracker.py`
example, but in our gravity-aligned `RIGHT_HAND_Z_UP` world frame. On by default;
`--show_trajectory=false` suppresses it, and it auto-skips when `--robot.enable_tracker=false`.
Shared implementation in [`../taccap_gripper/visualization.py`](../taccap_gripper/visualization.py).

More variants in [`../../scripts/client_commands.md`](../../scripts/client_commands.md).
