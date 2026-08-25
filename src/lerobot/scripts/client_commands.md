# Lerobot client commands — TacCap-Gripper

This branch is slimmed to the **TacCap-Gripper** robot (single + bimanual) and the
**Pico4** teleoperator. All TacCap devices — grippers, tactile sensors, wrist cameras
and Pico4 motion trackers — are **auto-discovered by serial rule**, so no device serials
are passed on the CLI. The one optional override is the Pico4 tracker serial
(`--robot.tracker_serial` / `--robot.{left,right}_tracker_serial`); see Teleoperate below.

`--robot.id` **is required** on every command below — the station label for the rig
(`taccap_0`, `taccap_1`, … one per rig). See
[`--robot.id` and the hardware manifest](#--robotid-required-and-the-hardware-manifest).

## Prerequisites

### Hugging Face CLI login

Required before any command with `--dataset.push_to_hub=true`:

```bash
huggingface-cli login
```

Paste your HuggingFace access token (write permission) when prompted; it is stored at
`~/.cache/huggingface/token` and persists across sessions.

Also ensure `xense.taccap` is importable (`bash ./setup_env.sh --install`) and, for
6-DoF pose, the XenseVR PC service + Pico4 trackers are running. The optional head
camera shares that same connection, so it needs the headset app streaming too.

## Teleoperate (live Rerun visualization)

`taccap_gripper` / `bi_taccap_gripper` are **self-driven** (sensors only) — there is no
taccap teleoperator, so **no `--teleop` is required**. `lerobot-teleoperate` just streams
`get_observation()` to Rerun.

`--display_data=true` applies a blueprint rather than letting Rerun auto-lay-out, which
would give a tactile pad the same screen area as the head camera. The layout adapts to what
the rig actually reports:

- **Left, largest**: the 3D trajectory view — gripper EE frames with fading trails, plus the
  headset when the head camera is on.
- **Right, top to bottom**: the head cameras (`left_head` / `right_head`), the wrist cameras
  under them, then the tactile pads in their own grid.
- **Bottom**: scalars split into tabs by unit — `gripper.pos`, head pose, tcp pose, tracker
  pose, imu. One shared plot would be unreadable, since metres, unit-length
  rotation components and accelerations do not share an axis.

With the tracker on, the 3D view (`/world`) shows each gripper as a labelled marker at its
live Pico4 pose, trailing the path it has swept — the same effect as the SDK's
`rerun_dual_with_tracker.py` example. It is **on by default**; `--show_trajectory=false`
drops that view only, leaving the rest of the layout in place, and it auto-skips when
`--robot.enable_tracker=false` since there is no pose to draw. Same flag on `lerobot-record`.

### Bimanual (`bi_taccap_gripper`)

```bash
lerobot-teleoperate \
    --robot.type=bi_taccap_gripper \
    --robot.id=0 \
    --fps=30 \
    --display_data=true \
    --robot.enable_tracker=false \
    --robot.enable_head_camera=false
```

['PC2310MLL4150713G', 'PC2310MLL4150387G']

Both leader grippers, all four tactiles, both wrist cameras **and both Pico4 trackers**
are discovered automatically. Sides are assigned by serial: Xense devices by the last
sequence digit (odd → left, even → right) plus the role patch (`m`=leader, `s`=follower);
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
auto-discover. Other knobs: `--robot.role=follower` (bind follower units), `--robot.gripper_open_rad`,
`--robot.tactile_fps`, `--robot.wrist_camera_width/height/fps`.

The bimanual rig can add the Pico headset camera with:

```bash
    --robot.enable_head_camera=true \
    --robot.head_camera_eyes=both \
```

`head_camera_width`/`height` are **per eye** and default to 640x480, the headset app's
own default. Only that, 1024x768 and 1280x960 are accepted — all 4:3, matching the sensor
— and an unlisted size is an error rather than a silent resize, as is a first frame whose
size disagrees with the config, since rescaling would quietly change the recorded field of
view. The app's Resolution setting is what produces the frames, so the two must agree.

Capture stores `left_head` and `right_head` — one video key per eye, not a merged frame —
plus the headset pose as `head_camera.x/y/z/r1..r6`. That pose goes through the same
Pico→world remap as the gripper trackers, so it shares their world frame and the same
rotation-matrix first-two-columns representation; no IMU or timing/age/status metadata is
stored. `--robot.head_camera_eyes=left` (or `right`) records a single eye.

### Single (`taccap_gripper`)

```bash
lerobot-teleoperate \
    --robot.type=taccap_gripper \
    --robot.id=0 \
    --robot.side=left \
    --fps=30 \
    --display_data=true
```

`--robot.side` is only needed when both grippers are connected; a single unit auto-resolves.

## Record a dataset

Recording is self-driven (`self_driven_record_loop`, shifted-frame: `action[t]` paired with
`obs[t-1]`) — **no `--teleop`**. Same robot flags as teleop, plus `--dataset.*`.

### `--robot.id` (required) and the hardware manifest

`--robot.id` is a **required station label** — one per rig, and a bimanual rig is
one rig. It names the seat, not the hardware in it, so it survives a gripper swap.

**Pass a number.** `--robot.id=0` is stored as `taccap_0` on a single rig and
`bi_taccap_0` on a bimanual one: a bare number is expanded against `--robot.type`
minus its `_gripper` suffix, so the label cannot disagree with the rig it names.
Anything not all digits is taken verbatim, so an existing `--robot.id=taccap_0`
keeps working.

Upstream leaves it optional; both TacCap configs reject a
missing or blank one in `__post_init__`, so the run stops at CLI-parse time rather
than a rig spinning up and recording anonymously:

```
ValueError: --robot.id is required: the station label for this rig, e.g. --robot.id=0 …
```

It reaches the log prefix, the calibration filename and `str(robot)`, and it is copied
into the manifest below — but it is **not** a dataset column, and `meta/info.json`
never sees it.

Device identity is carried instead by `meta/hardware.json`, written into the dataset
right after connect — gripper firmware SN plus the tactile sensors on that gripper,
each tied to the observation key it feeds:

```json
{
  "robot_type": "bi_taccap_gripper",
  "robot_id": "taccap_0",
  "role": "leader",
  "units": [
    {
      "side": "left",
      "gripper_sn": "TCGU01A24Z0001m",
      "tactile_sensors": [
        {
          "finger": "left",
          "observation_key": "left_tactile_left",
          "serial": "GSPS01A25Z0011"
        },
        {
          "finger": "right",
          "observation_key": "left_tactile_right",
          "serial": "GSPS01A25Z0012"
        }
      ]
    }
  ]
}
```

`side` is which gripper, `finger` is which sensor on it — independent left/rights, hence
the explicit `observation_key`. `gripper_sn` is the firmware SN read over the wire, not
the CH343 `mcu_serial`. Single-arm writes the same shape with one entry. Resuming a
dataset on different hardware keeps the original file and warns rather than overwriting
it. Details: [`robots/taccap_gripper/README.md`](../robots/taccap_gripper/README.md).

### Recording with the viewer on — `[slow_frame]`

`--display_data=true` costs loop time, and a bimanual rig with the head camera is
eight images per frame. The defaults below are already the fast ones; reach for
them only if the log shows `[slow_frame] ... overrun=`.

| Flag                          | Default | What it costs                                                                                                                                                                                                                                                                         |
| ----------------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--display_compressed_images` | `false` | `true` JPEG-encodes every image inline on the record loop — measured at 13.2 ms/frame against a 33.3 ms budget at 30 fps, versus 3.1 ms off. Turn it on only when the viewer is on **another machine** (`--display_ip`), where saving IPC bandwidth is worth more than the loop time. |
| `--display_image_every_n`     | `1`     | Log camera images every N-th frame. Scalars stay at full rate, so `tcp.*` and `gripper.pos` curves are unaffected — only the camera tiles get sparser. Every 3rd frame brings the image cost to ~1 ms. Last resort: it is the only one of these that changes what you see.            |

The `[slow_frame]` line carries a `top_obs=` suffix naming the slowest cameras of
that frame, so check it before reaching for either flag — a single slow sensor is
a different problem from the viewer being expensive.

### Recording on a machine with no GPU

On a GPU-less host — a CPU-only server, a VM, a laptop with no discrete GPU —
turn streaming encoding off:

```bash
lerobot-record \
    ... \
    --dataset.streaming_encoding=false
```

**The codec you can leave alone.** `--dataset.vcodec=auto` (the default) probes
by opening a real encode session, so on a host with no NVIDIA driver it reports
no hardware encoder and falls back to `libsvtav1` — AV1 on the CPU, which is what
the offline dataset tools already default to. Measured with
`libnvidia-encode.so.1` masked out: `auto` resolves to `libsvtav1` in 2 ms.

Passing `--dataset.vcodec=libsvtav1` explicitly is still fine, and worth doing if
you want the recording command to be self-documenting about where it can run.

> This did not use to work. The probe was `av.codec.Codec(name, "w")` —
> `avcodec_find_encoder_by_name`, a static lookup in the codec table FFmpeg was
> _built_ with, which never touches a driver. PyAV ships with nvenc compiled in,
> so the lookup succeeded on machines with no GPU at all, `auto` selected
> `h264_nvenc`, and recording died at the first frame with
> `avcodec_open2(h264_nvenc)`. If you are on a build from before that fix, pass
> `--dataset.vcodec=libsvtav1` by hand.

**`--dataset.streaming_encoding=false`.** Streaming encoding runs the encoder
inline with capture, which is a win when the encoder is a dedicated ASIC on the
GPU and the CPU only hands it frames. With `libsvtav1` the encoder _is_ the CPU,
and it competes with the capture loop for the same cores — on a bimanual rig
that is six to eight images per frame inside a 33.3 ms budget at 30 fps, so the
first thing you see is `[slow_frame] ... overrun=`. With it off, frames are
written out during capture and encoded in a batch at `save_episode()` instead:
episode saves become slow and visible, capture stays on time. That is the right
trade — a late save costs you patience, a starved capture loop costs you data
you cannot re-record.

Two knobs worth knowing if you keep streaming encoding on anyway, e.g. on a
many-core server:

| Flag                              | Default | Why you would touch it                                                                                                                            |
| --------------------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--dataset.encoder_threads`       | `None`  | `None` lets the codec pick, which on a big machine means libsvtav1 helping itself to cores the capture loop needs. `2` per encoder is a sane cap. |
| `--dataset.encoder_queue_maxsize` | `30`    | ~1 s of buffer at 30 fps. It is the backpressure valve: when the encoder falls behind, capture blocks here rather than growing memory forever.    |

`lerobot-record` prints a reminder when `streaming_encoding=false`, suggesting you
turn it back on if the hardware is capable. On a GPU-less host it is not, so the
suggestion does not apply — leaving it off is deliberate.

### Bimanual (`bi_taccap_gripper`)

```bash
lerobot-record \
    --robot.type=bi_taccap_gripper \
    --robot.id=0 \
    --robot.enable_head_camera=true \
    --dataset.repo_id=Xense/taccap-g1-test-0722 \
    --dataset.single_task="Pick up the cube" \
    --dataset.num_episodes=2 \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --dataset.streaming_encoding=true \
    --dataset.push_to_hub=false \
    --display_data=false
```

> `--dataset.streaming_encoding=true` is the default and assumes an NVIDIA card.
> On a GPU-less host, pass `--dataset.streaming_encoding=false` instead — the
> codec picks itself. See
> [Recording on a machine with no GPU](#recording-on-a-machine-with-no-gpu).

### Single (`taccap_gripper`)

One gripper, its two tactile pads and its wrist camera, recorded through the same
`self_driven_record_loop`. Keys are **unprefixed** (`tcp.*`, `gripper.pos`,
`tactile_left` / `tactile_right`, `wrist_cam`), so a single-arm dataset is not a
column subset of a bimanual one. `--robot.enable_head_camera` works here too, and the
head keys are unprefixed the same way on both robots (`left_head` / `right_head` name the
headset's eyes, not the arms).

`--robot.side` is only needed when both grippers are plugged in; a lone unit
auto-resolves, and so does its Pico4 tracker (side from the serial's 2nd-to-last
digit). Add `--robot.enable_tracker=false` to record tactile + gripper only — the
`tcp.*` columns then disappear from the dataset instead of recording zeros.

```bash
lerobot-record \
    --robot.type=taccap_gripper \
    --robot.id=0 \
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
> (gripper side/role; also pairs each tactile to a gripper by USB hub), and the XenseVR
> PC service (Pico4 tracker SNs, side from the 2nd-to-last digit). Obs keys:
> `tactile_left/right` + `wrist_cam` + `tcp.*` (single), or
> `left_/right_tactile_left/right` + `{side}_wrist` + `{side}_tcp.*` (bi). Tactile
> rectify is landscape `(400,700,3)` — width/height auto-derive, don't hard-code.
> Each tactile key records `rectify`, and Rerun shows that same stream by default. A
> `--robot.tactile_display_output_types` other than the recorded type (e.g.
> `'["difference"]'`) adds a display-only `*_<type>` key that Rerun shows instead of the
> recorded one and that never enters the dataset.
