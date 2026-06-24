# Lerobot client commands — TacCap-Gripper

This branch is slimmed to the **TacCap-Gripper** robot (single + bimanual) and the
**Pico4** teleoperator. All TacCap devices — grippers, tactile sensors, wrist cameras
and Pico4 motion trackers — are **auto-discovered by serial rule**, so no device serials
are passed on the CLI. The one optional override is the Pico4 tracker serial
(`--robot.tracker_serial` / `--robot.{left,right}_tracker_serial`); see Teleoperate below.

## Prerequisites

### Hugging Face CLI login

Required before any command with `--dataset.push_to_hub=true`:

```bash
huggingface-cli login
```

Paste your HuggingFace access token (write permission) when prompted; it is stored at
`~/.cache/huggingface/token` and persists across sessions.

Also ensure `xense.taccap` is importable (`bash ./setup_env.sh --install`) and, for
6-DoF pose, the XenseVR PC service + Pico4 trackers are running.

## Teleoperate (live Rerun visualization)

`taccap_gripper` / `bi_taccap_gripper` are **self-driven** (sensors only) — there is no
taccap teleoperator, so **no `--teleop` is required**. `lerobot-teleoperate` just streams
`get_observation()` to Rerun.

With the tracker on, the viewer adds a **3D pose + breadcrumb trajectory** view (`/world`):
each gripper is a labelled marker at its live Pico4 pose, trailing the path it has swept —
the same effect as the SDK's `rerun_dual_with_tracker.py` example. It is **on by default**
under `--display_data=true`; add `--show_trajectory=false` to suppress it (or it auto-skips
when `--robot.enable_tracker=false`, since there is no pose to draw). Same flag on `lerobot-record`.

### Bimanual (`bi_taccap_gripper`)

```bash
lerobot-teleoperate \
    --robot.type=bi_taccap_gripper \
    --fps=30 \
    --display_data=true
```

['PC2310MLL4150713G', 'PC2310MLL4150387G']

Both leader grippers, all four tactiles, both wrist cameras **and both Pico4 trackers**
are discovered automatically. Sides are assigned by serial: Xense devices by the last
sequence digit (odd → left, even → right) plus the role patch (`m`=Master, `s`=Slave);
Pico4 trackers by the **second-to-last digit** (e.g. `PC2310MLL3200496G` → `6` → right).
A bimanual rig needs one tracker per side. To record tactile + gripper only (no Pico4 /
PC service), add:

```bash
    --robot.enable_tracker=false \
```

To bypass the tracker side rule (a tracker whose serial doesn't follow it, or flaky
enumeration), pin serials directly — bimanual takes one per side, single takes one:

```bash
    --robot.left_tracker_serial=PC2310MLL4150713G \
    --robot.right_tracker_serial=PC2310MLL4150387G \   # bi_taccap_gripper
    --robot.tracker_serial=PC2310MLL4150387G \         # taccap_gripper (single)
```

A pinned side is used verbatim (no enumeration, no rule check); un-pinned sides still
auto-discover. Other knobs: `--robot.role=follower` (bind Slave units), `--robot.gripper_open_rad`,
`--robot.tactile_fps`, `--robot.wrist_camera_width/height/fps`.

### Single (`taccap_gripper`)

```bash
lerobot-teleoperate \
    --robot.type=taccap_gripper \
    --robot.side=left \
    --fps=30 \
    --display_data=true
```

`--robot.side` is only needed when both grippers are connected; a single unit auto-resolves.

## Record a dataset

Recording is self-driven (`self_driven_record_loop`, shifted-frame: `action[t]` paired with
`obs[t-1]`) — **no `--teleop`**. Same robot flags as teleop, plus `--dataset.*`.

### Bimanual (`bi_taccap_gripper`)

```bash
lerobot-record \
    --robot.type=bi_taccap_gripper \
    --dataset.repo_id=Xense/taccap-g1-test-0624 \
    --dataset.single_task="Pick up the cube" \
    --dataset.num_episodes=2 \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --dataset.streaming_encoding=true \
    --dataset.push_to_hub=false \
    --display_data=true
```

### Single (`taccap_gripper`)












```bash
lerobot-record \
    --robot.type=taccap_gripper \
    --robot.side=left \
    --dataset.repo_id=Xense/<dataset_name> \
    --dataset.single_task="Pick up the cube" \
    --dataset.num_episodes=20 \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --display_data=true
```

> Notes: discovery reads `/dev/v4l/by-id` (tactile + wrist serials), `scan_grippers()`
> (gripper side/role), and the XenseVR PC service (Pico4 tracker SNs, side from the
> 2nd-to-last digit). Obs keys: `tactile_0/1` + `wrist_cam` + `tcp.*` (single), or
> `left_/right_tactile_0/1` + `{side}_wrist` + `{side}_tcp.*` (bi). Tactile rectify is
> landscape `(400,700,3)` — width/height auto-derive, don't hard-code.
