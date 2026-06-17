# bi_taccap_gripper

Bimanual TacCap-Gripper handheld data-collection rig — two `taccap_gripper` units
(left + right) driven as one robot. Passive/self-driven: `send_action()` is a no-op
(jaw motors stay disabled, encoders read-only); pose comes from a per-side Pico4
Ultra tracker, tactile + wrist cameras go through the standard `cameras` framework.

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
| `{s}_tactile_0` / `{s}_tactile_1` | auto-discovered | tactile frames |

`action_features` = the pose + `{s}_gripper.pos` subset (no cameras).

## Config — auto-discovered by serial rule

**No device serials are listed.** The two grippers, four tactile sensors and two wrist
cameras are scanned from the connected hardware and assigned to `left`/`right` by the
Xense serial rule:

- **Side** — last sequence digit odd → left, even → right.
- **Role** — patch `m` → Master/Leader, `s` → Slave/Follower (`--robot.role`, default
  `leader`).

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

## Usage

Self-driven — **no `--teleop`**. Prerequisite: `xense.taccap` importable in the
`lerobot-xense` env (`bash ./setup_env.sh --install`).

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
    --dataset.repo_id=Xense/<dataset_name> \
    --dataset.single_task="Pick up the cube" \
    --dataset.num_episodes=20 \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --display_data=true
```

More variants in [`../../scripts/client_commands.md`](../../scripts/client_commands.md).
