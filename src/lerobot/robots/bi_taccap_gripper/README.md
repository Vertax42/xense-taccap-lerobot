# bi_taccap_gripper

Bimanual TacCap-Gripper handheld data-collection rig — two `taccap_gripper` units
(left + right) driven as one robot. Passive/self-driven: `send_action()` is a no-op
(jaw motors stay disabled, encoders read-only); pose comes from a per-side Pico4
Ultra tracker, tactile + wrist cameras go through the standard `cameras` framework.
An optional Pico headset camera uses the same robot observation path and contributes
a stereo RGB frame plus the headset pose; no head-to-gripper extrinsic calibration is
required at capture time.

Implemented with the **reimplement-with-prefixes** pattern (cf. `bi_elite_cs66_rt`):
one `Robot` class, per-side handles in dicts keyed `"left"`/`"right"`, and every
observation/action key is `left_`/`right_` prefixed. Per-side reading logic matches
the single [`taccap_gripper`](../taccap_gripper/README.md).

## Observation / action schema

Per side `{s}` ∈ {left, right}:

| Key                                      | When                           | Meaning                                                                                   |
| ---------------------------------------- | ------------------------------ | ----------------------------------------------------------------------------------------- |
| `{s}_tcp.x/y/z`, `{s}_tcp.r1..r6`        | `{s}_enable_tracker`           | Pico4 → EE 6D pose                                                                        |
| `{s}_gripper.pos`                        | `{s}_enable_gripper`           | normalised jaw, 0=closed / 1=open                                                         |
| `{s}_imu.{accel,gyro,mag}.{x,y,z}`       | `{s}_enable_imu`               | IMU                                                                                       |
| `{s}_wrist`                              | `{s}_enable_wrist_camera`      | wrist UVC frame                                                                           |
| `{s}_tactile_left` / `{s}_tactile_right` | auto-discovered                | **recorded** tactile frame from the left / right finger sensor (`rectify`)                |
| `{s}_tactile_{left,right}_difference`    | `tactile_display_output_types` | **display-only** amplified-deformation view of the same read — Rerun only, never recorded |
| `left_head` / `right_head`               | `enable_head_camera`           | headset camera, one key per **eye** (not per arm) — `head_camera_eyes` can select one     |
| `head_camera.x/y/z`                      | `enable_head_camera`           | headset position, same world frame as `{s}_tcp.*`                                         |
| `head_camera.r1..r6`                     | `enable_head_camera`           | headset orientation as the first two rotation-matrix columns                              |

`action_features` = the per-side gripper pose + `{s}_gripper.pos` subset; the head
camera pose and all images remain observation-only. With both Pico4 trackers, both
grippers and the head camera enabled, `observation.state` has 29 dimensions (20 + 9).

**Two tactile streams, two destinations.** Each sensor is read once per frame for two
views: `rectify` (recorded) and the amplified `difference` (displayed). Only the
recorded one is in `observation_features`, so only it reaches the dataset; the
`*_difference` keys live in `display_features`, which is what the Rerun layout and
`log_rerun_data` are fed — the recorded stream is not sent to the viewer at all, so
the tactile grid stays at four tiles. `difference` is the easier view to read contact
from live, but its baseline is captured at sensor init, so a finger resting on
something at connect would have that pressure subtracted out of the whole recording —
which is exactly why the dataset gets `rectify`.

For each fixed-rate robot sample, the adapter takes the newest cached frame for each
eye and the current headset pose. The source XYZW quaternion is converted with the shared
6D conversion used by the Pico4 trackers. A corrupt JPEG holds the previous good frame,
as does a left/right pair that did not come from the same capture; both are counted and
surfaced in a rate-limited warning. No timing, age, status or IMU fields are stored
in the dataset.

## Config — auto-discovered by serial rule

**No device serials are listed.** The two grippers, four tactile sensors and two wrist
cameras are scanned from the connected hardware and assigned to `left`/`right` by the
Xense serial rule:

- **Side** — last sequence digit odd → left, even → right (单左双右).
- **Role** — patch `m` → leader, `s` → follower (`--robot.role`, default
  `leader`).

**Tactile left/right** (`{side}_tactile_{left,right}`) is resolved by **USB hub**,
not by the tactile serial alone: the two GSPS sensors sharing a gripper's USB hub
are that gripper's pair, and the gripper's `side` is read from its **firmware SN**
over the wire (`scan_grippers()` → `ep.side`, i.e. `Cmd::GetSn` — _not_ the CH343
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
`--robot.{side}_gripper_open_rad` (fallback only — see below), `--robot.tactile_fps`,
`--robot.wrist_camera_{width,height,fps}`,
`--robot.expected_tactiles_per_side`, `--robot.enable_tactile`.

**Jaw normalisation.** `{side}_gripper.pos` comes from each leader's own encoder-max
calibration in MCU flash (firmware ≥ V2.1), so 1.0 is _that_ unit's real full-open rather
than a shared constant. Calibrate every unit with
`python third_party/taccap-gripper/python/examples/calibrate.py <left|right>` — it sets the zero and
the travel span in one pass. Uncalibrated units, and followers (`Cmd::EncoderMaxCal` is
leader-only), fall back to dividing by `{side}_gripper_open_rad`. Calibrating only one side
is the case to avoid: the two channels then sit on different scales and the same grip reads
differently left and right. The connect log names the path each side took.

**Tracker → EEF TCP.** Each side's tracker is bolted to its gripper, so the raw pose is
the tracker's, not the TCP's (~195 mm apart). The constant body-fixed offset comes from
[`../taccap_gripper/ee_transform.py`](../taccap_gripper/ee_transform.py): **both sides
are measured** off the CAD assembly (they are near-mirrors about the XZ plane, but the
translations differ by 1.27 mm, so neither is derived from the other).
`--robot.{left,right}_tracker_to_ee_pos` / `_quat` default to `None` = built-in value; set
either to override. TCP is the two-finger midpoint, which symmetric jaws keep still, so the
transform does not vary with `{side}_gripper.pos`. Both trackers are drawn next to their EE
frames in the Rerun `/world` view — that check, which also confirmed the mount
convention on hardware, is in the
[single-gripper README](../taccap_gripper/README.md#checking-the-mount-transform-in-rerun).
**Episodes recorded before this landed hold the tracker pose in `{side}_tcp.*`** and must
not be mixed with newer ones without re-transforming.

Tactile streams: `--robot.tactile_output_types` (recorded, default `rectify`, exactly
one type) and `--robot.tactile_display_output_types` (Rerun-only, default `difference`;
set to `'[]'` to drop the second read and show the recorded stream instead).
`--robot.tactile_diff_gain` (default `1.0`, sensors ship at `1.5`) scales the difference
image, i.e. the displayed one only.

To bypass the tracker side rule, pin serials directly with `--robot.left_tracker_serial=<SN>`
and/or `--robot.right_tracker_serial=<SN>`. A pinned side uses its serial **verbatim** (no
enumeration, no rule check); un-pinned sides still auto-discover by the second-to-last-digit
rule. Use this for a tracker whose serial does not follow the rule, or when enumeration is flaky.

Enable the head camera with `--robot.enable_head_camera=true`. It records `width=1024`,
`height=768` at dataset FPS 30 as **two keys, one per eye** — `left_head` and
`right_head`, each 768x1024. `--robot.head_camera_eyes=left` (or `right`) records a
single eye, halving both the JPEG decoding and the encoder load.

> These names refer to the headset's **eyes**, not to the left/right arm. There is one
> headset on a bimanual rig, so they are not per-arm the way `{s}_wrist` is.

Only `1024x768` and `1280x960` are accepted, via `--robot.head_camera_width/_height`.
Both are 4:3, matching the sensor: PICO's camera-access API caps a frame at 2328x1748,
which is 4:3, so a 16:9 request would crop or stretch rather than widen the field of
view. An unlisted size is an error rather than a silent fallback. Changing the size or
the eye selection changes the recorded frame, so episodes either side of such a change
are not comparable.

The two eyes arrive as separate messages, each with its own sequence number and
timestamp, and recording them under separate keys means a mismatched pair leaves no
trace in the data. So each frame the two cameras' newest frames are compared: identical
sequence numbers are a definitive match, otherwise their timestamps must agree within
`--robot.head_camera_pair_max_skew_ms` (default 20 ms, against a ~33 ms frame period at
30 fps). Exceeding it does not stop recording — it raises a rate-limited warning naming
the measured skew, so the condition is visible rather than silent.

Raw acquisition is isolated in
[`../../cameras/pico/camera_pico.py`](../../cameras/pico/camera_pico.py). The headset
pose is remapped into the same gravity-aligned world frame as `{s}_tcp.*` — unlike the
Insight VIO pose it replaces, which lived in its own frame — so head and gripper poses
are directly comparable. The robot adapter applies no head-to-gripper extrinsic.

## Usage

Self-driven — **no `--teleop`**. Prerequisite: `xense.taccap` importable in the
`xense-taccap` env (`bash ./setup_env.sh --install`).

**Live Rerun visualization** (cameras + gripper only — both grippers, 4 tactiles and
2 wrist cameras are discovered automatically):

```bash
lerobot-teleoperate \
    --robot.type=bi_taccap_gripper \
    --robot.id=taccap_0 \
    --fps=30 \
    --display_data=true
```

**Record a dataset** (`self_driven_record_loop`, shifted-frame). With the trackers
powered on, 6-DoF pose is recorded automatically (both trackers auto-assigned by SN);
add `--robot.enable_tracker=false` to record tactile + gripper only:

```bash
lerobot-record \
    --robot.type=bi_taccap_gripper \
    --robot.id=taccap_0 \
    --robot.enable_head_camera=true \
    --dataset.repo_id=Xense/<dataset_name> \
    --dataset.single_task="Pick up the cube" \
    --dataset.num_episodes=20 \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --display_data=true
```

The head camera shares the XenseVR SDK connection with the Pico4 trackers, so the
headset app must be running and streaming before enabling it. Turning the camera off
does not drop the trackers' connection, and vice versa.

`--robot.id` is a **required station label** (`taccap_0`, `taccap_1`, … — one per rig,
and this bimanual rig is one rig): the config rejects a missing or blank one at
CLI-parse time, before any device is touched. It is not a dataset column. What is
recorded instead is `meta/hardware.json`, written at connect: each unit's gripper
firmware SN plus the two tactile serials on that gripper, keyed by `side` (which
gripper) and `finger` (which sensor on it) and carrying the observation key each sensor
feeds. See
[`../taccap_gripper/README.md`](../taccap_gripper/README.md#--robotid-required-and-the-hardware-manifest).

## 3D trajectory visualization

With `--display_data=true`, the Rerun viewer adds a `/world` 3D view: each gripper is a
labelled marker (red = left, blue = right) at its live Pico4 pose (`{side}_tcp.*`), trailing a
breadcrumb of its swept path — the same effect as the SDK's `rerun_dual_with_tracker.py`
example, but in our gravity-aligned `FLU` world frame (X forward, Y left, Z up).
On by default;
`--show_trajectory=false` suppresses it, and it auto-skips when `--robot.enable_tracker=false`.
Shared implementation in [`../taccap_gripper/visualization.py`](../taccap_gripper/visualization.py).

More variants in [`../../scripts/client_commands.md`](../../scripts/client_commands.md).
