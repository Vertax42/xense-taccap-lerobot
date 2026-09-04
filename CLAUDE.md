# lerobot-xense — Claude working notes

Fork of HuggingFace `lerobot` (tracks upstream v5.1), slimmed to the
**TacCap-Gripper** (**TacCap** = _Tactile Capture_ Gripper) — a handheld **UMI**
leader gripper for tactile data collection (single + bimanual) — and the
**Pico4** teleoperator/tracker, with Xense tactile cameras and an optional
Pico headset camera. See `src/lerobot/robots/taccap_gripper/README.md` for usage.

## TacCap serial / topology rules (device auto-discovery)

Devices are auto-discovered and assigned to `left`/`right` by serial + USB
topology rules — **no serials are hand-listed**. Source of truth:
`src/lerobot/robots/taccap_gripper/serial_discovery.py`.

### Serial grammar

- Gripper : `TCGU01<batch><line><seq><m|s>` — e.g. `TCGU01A24Z0002m`
- Tactile : `GSPS01<batch><line><seq>` — e.g. `GSPS01A25Z0011`
- Camera : `XC<batch><line><seq><m|s>` — e.g. `XCA24Z0007m`
- `<seq>` is 4 digits; patch `m` → leader, `s` → follower.

### Side rule — 单左双右 (`side_of_sequence`)

The **last digit** of the 4-digit sequence: **odd → left, even → right**.
Applies to gripper / camera side and to tactile _finger_ (below).

### Tactile left/right → `{side}_tactile_{left,right}`

Combines USB topology with the side rule (`discover_tactiles_by_hub`):

- **side** (which gripper a sensor pair belongs to): the two GSPS sensors
  sharing a gripper's **USB hub** are that gripper's pair. The gripper's side is
  read from its **firmware SN** over the wire (`scan_grippers()` → `ep.side`,
  i.e. `Cmd::GetSn`) — **NOT** the CH343 `mcu_serial`. So: hub → gripper → side.
- **finger** (left/right sensor on that gripper): the GSPS serial's **last
  digit** (单左双右).
- Hubs are matched via `/dev/v4l/by-path` (tactiles) ↔ `/dev/serial/by-path`
  (gripper `mcu_device`); a device's hub = its USB port path minus its own port.
- Needs the gripper SDK scan, so tactile discovery runs at **construction**
  (grippers must be powered then), keeping the obs schema ready before
  `connect()`.

### Pico4 motion tracker — different serial system

Tracker serials (e.g. `PC2310MLL3200496G`) are **not** Xense serials. Side is
the **second-to-last digit**: odd → left, even → right (`pico_tracker_side`),
e.g. `…496G` → `6` → right. Trackers enumerate from the XenseVR PC service at
connect (pin with `--robot.{left_,right_,}tracker_serial=<SN>` to bypass).

### Dataset provenance — `meta/hardware.json`, not `robot.id`

`--robot.id` is the station label (`taccap_0`, `taccap_1`; one per rig) — it
reaches the logger prefix, the calibration filename and `str(robot)`, and is
**not a dataset column** (`LeRobotDataset.create` takes only `robot_type`).
Unlike upstream's optional `RobotConfig.id` it is **required**: both TacCap
configs run `validate_robot_id()` in `__post_init__`, so a missing/blank id
fails at CLI-parse time instead of a rig recording anonymously. Enforced there
rather than by changing the base dataclass, which is upstream's.
Identity travels in `meta/hardware.json`, written by `lerobot-record` right
after `connect()` from `robot.hardware_manifest`: per unit, the gripper's
**firmware** SN (`Cmd::GetSn`, not `mcu_serial`) plus its tactile serials, each
tagged with `side` (which gripper), `finger` (which sensor on it) and the
`observation_key` it feeds. Keep it a separate file — `meta/info.json` is
upstream's schema and a fork-local key there collides on the next v5.x sync.
Helpers live in `taccap_gripper/common.py`.

The file is a list of **`epochs`**, not one flat `units`: each carries
`from_episode` / `to_episode` (half-open, matching `dataset_from_index` /
`dataset_to_index`) and `recorded_at`, so a rig swapped **mid-dataset** closes
the open epoch at the current episode count and opens the next one. It used to
warn and keep the original file; that warning went to the log and never reached
the dataset, so afterwards nothing on disk said the rig had changed while the
manifest quietly misattributed every episode recorded after the swap. Only a
`robot_type` mismatch is still keep-and-warn — single vs bimanual changes the
observation keys, so it is not the same dataset and epochs do not model it.
A pre-epoch file reads back as one open epoch (`manifest_epochs`), but an open
single epoch means _"nothing here says the rig changed"_, **not** _"it didn't"_.

**`robot_id` is the exception: a wall, not an epoch boundary.** One dataset is
one station, so `--resume` with a `--robot.id` the manifest disagrees with
raises (`check_dataset_station`) — identical `units` do not make it one rig,
because the label names the seat, not the hardware in it. Checked twice: in
`lerobot-record` **before `connect()`** (the id comes from the config, so no
device has to spin up to be turned away) and again inside
`write_hardware_manifest`, the choke point every writer goes through. Being a
dataset-level invariant it is written at the top level beside `robot_type` and
repeated per epoch so readers of older files keep working; `manifest_robot_ids`
reads both. An unlabelled dataset (recorded before `--robot.id` was required) is
not a mismatch — same reading as the open epoch above.

Tactile **runtime bundles are no longer recorded** (they used to sit at
`meta/runtimes/<serial>-<time>.bin` with a `runtime` key per sensor). The only
per-sensor part of a bundle is the reference image, and the dataset already
carries it — the first `rectify` frame of each episode; everything else in the
solver is fixed per sensor _model_. Downstream (TacFlow) rebuilds depth / force /
difference from the stream plus the serial in this manifest, so the manifest is
still the one thing that must survive every conversion.

### On mis-burned / mis-installed hardware

Every discovery helper raises `ValueError` naming the offending hub/serial
(non-conforming serial, wrong per-side count, two sensors on one hub mapping to
the same finger, a tactile hub with no matching gripper) so the physical rig and
the schema can't silently drift.

### Host gotcha — `Device or resource busy` on the gripper serial (ModemManager)

The gripper MCU is a CH343 USB-serial (`1a86:55d2`, CDC-ACM). On every hot-plug
**ModemManager** probes the fresh port with AT commands and holds it open for a
few seconds, so `connect()` in that window dies with
`IoError: SerialBus: open(/dev/serial/by-id/...): Device or resource busy`.
Tell: **first** launch works, but unplug → other port → relaunch _immediately_
is busy. **Not** a tactile/camera/bandwidth issue. Permanent fix is a udev rule
ignoring `1a86` (`ID_MM_DEVICE_IGNORE=1`) — see README → "Hardware bring-up
sequence". (`brltty` grabs `1a86` the same way if installed.)

## Pico headset camera (bimanual: `--robot.type=xtac_umi_g1`; single-arm: `--robot.enable_head_camera`)

Records `left_head` / `right_head` (one key per **eye** — on a bimanual rig
`{side}_wrist` is per-arm, but there is one headset, so the prefix means
something different) plus `head_camera.*`, the headset pose.

- **On the bimanual rig the head is a robot type, not a flag.** `robot_type` in
  `meta/info.json` is written from `robot.name`, a class attribute bound to the
  draccus registry key, so a config field can change what is recorded but never
  what the recording claims to be. A head-enabled run under `bi_taccap_gripper`
  wrote 29 state dims and 8 cameras under a label meaning 20 and 6; twelve
  datasets on disk were mislabelled that way. `XtacUmiG1Config` inherits
  `BiTaccapGripperConfig` and pins `records_head = True`; the base
  `__post_init__` refuses any `enable_head_camera` that contradicts the class,
  in both directions, naming the type to switch to.
  `taccap_gripper` (single-arm) is unchanged — it keeps the flag, and has no
  headset variant type, because no single-arm head recording exists.
- **Removing the head must move the label back.** `convert_8_to_6_cameras`
  relabels `xtac_umi_g1` → `bi_taccap_gripper`, and `modify_features` does the
  same when the head image keys are among those removed
  (`robot_type_without_head` / `robot_type_after_removing` in `dataset_tools.py`).
  Without that, the same mismatch reappears from the other direction.

- **`head_camera.*` is remapped like the tracker.** Same `PICO_TO_WORLD_R`
  conjugation, same xyzw→wxyz reorder, so it lands in the world frame `tcp.*`
  uses. The SDK hands out `HeadsetPose` / `ControllerPose` / `MotionTrackerPose`
  through one `stringToPoseArray`, so all three share a layout — do not assume a
  different convention for any of them.
- **Do not read the SDK frame cache from the record loop.** The eyes arrive as
  separate messages, left first, so sampling at loop rate catches one updated
  and not the other (measured 7% of frames). `cameras/pico/stereo_poller.py`
  polls at 60 Hz and publishes only pairs whose `frame_sequence` agrees. The
  poll rate is margin, not throughput: the SDK holds one slot per eye, so a
  frame not collected before the next lands is lost, and its counterpart then
  ages out unpaired. 60 Hz against a ~30 fps stream is 2x — `StereoPoller.stop`
  reports `dropped_unpaired`, which is the tell if that margin is too thin.
- **Resolution is a whitelist**, `640x480` / `1024x768` / `1280x960`, all 4:3
  like the sensor and all three offered by the headset app's Resolution setting.
  The default is `640x480` because that is the app's default, so the two line up
  untouched. Unlisted sizes and a first frame that disagrees with the config are
  errors, deliberately — silent rescaling would change the recorded field of
  view without trace. Do not "fix" this with a resize.

## Rerun logging is off the record loop — keep it there

`log_rerun_data` runs on a worker thread owned by `RerunLogSink`
(`utils/visualization_utils.py`); `lerobot_record` and `lerobot_teleoperate` call
`submit()` and return. Inline it measured **27.2 ms/frame against a 24.1 ms
viewer-off baseline** on a bimanual rig with the head camera (eight images), i.e.
the ~3 ms that turned frames into `[slow_frame] ... overrun=` inside a 33.3 ms
budget at 30 fps — which is how the data team ended up recording with
`--display_data=false`. Off-loop it measures 24.1 ms, the baseline exactly.

It works because rerun's Rust bindings **release the GIL** for the heavy part of
`rr.log`, and on Linux `busy_wait` is `time.sleep`, so the worker runs inside the
window the loop is idle anyway. Both halves matter: a pure-Python sink, or a
spinning `busy_wait` (which is what macOS/Windows get), would not buy this.

The hand-off is **one frame deep and latest-wins** — a slow viewer drops display
frames instead of blocking capture, counted and reported once at `close()`.
Consequences, both display-only and deliberate: `log_time` lags capture by up to a
loop period, and the worker holds the images the loop just read (the same arrays
`dataset.add_frame` owns), so a camera recycling its buffer could tear a
_displayed_ frame — never a recorded one.

Do not move either call back inline "for ordering", and do not give the sink a
deeper queue: depth is latency, and a viewer that has fallen a second behind is
showing the operator the wrong thing more usefully than it is showing it late.
`traj_viz` belongs to the sink now — `reset_trajectory()` between episodes, not
`traj_viz.reset()`.

## `xrt.init()` is a process singleton — go through `xrt_session`

`teleoperators/pico4/xrt_session.py` owns it, with a hold count and an atexit
fallback. Both `Pico4TrackerReader` and `PicoCamera` hold it, and either can be
off, so **never call `xrt.init()` / `xrt.close()` directly** from a new consumer
— closing it out from under a live subscriber is the failure this prevents.
Discovery deliberately loads without holding (`ensure_loaded`), so one-shot
enumeration cannot leak a hold. Closing matters: the SDK's joinable
`std::thread`s call `std::terminate()` if the process exits without it.

## Logging goes through spdlog — including the stdlib

`utils/robot_utils.py:get_logger()` is the logger for this fork. Two things
about it are load-bearing:

- **`init_logging()` installs a stdlib→spdlog bridge, not a `StreamHandler`.**
  Upstream lerobot, `xensesdk` and libav all log through `logging`, so without
  the bridge they took a different path, to a different stream, in a different
  format, and never reached the session file. Those were the
  `INFO 2026-08-27 16:58:52 eo_utils.py:189` lines — whose location field is the
  _tail_ of the path cut to 15 chars, which is why `video_utils.py` reads as
  `eo_utils.py`. `install_stdlib_bridge()` clears root's handlers on purpose:
  leaving one in place is how every line gets printed twice. The bridge names
  each record by its **module**, because upstream logs via the root logger and
  `root` identifies nothing.
- **`get_logger()` caches per `(name, level)` and shares its sinks.** Calling it
  twice for one name used to stack a second stdout sink onto the same terminal.
  Sinks in this pybind expose only `set_level` — no `Sink.set_pattern` — so the
  pattern is set on the logger and both sinks render `SPDLOG_PATTERN`;
  `FILE_LOG_PATTERN` is aspirational until the binding grows one.

Console level is `$XENSE_LOG_LEVEL` (default INFO), the session file is
`$XENSE_LOG_DIR/session_<ts>.log` (default `~/xenselogs`, 15 files kept). The
bridge filters at the console level _before_ spdlog sees a record, so the
DEBUG-level file does not fill with third-party chatter.

### A camera stall is a stale frame, not a slow loop — `CaptureStallMonitor`

Every camera backend used to end `read()` with
`logger.debug(f"{self} read took: {ms:.1f}ms")`. On a bimanual rig that is six
cameras at 30 Hz: ~180 records a second, ~1 MiB a minute, none of it on the
console (the sink filters at INFO) and all of it in the DEBUG-level session file.

`CaptureStallMonitor` (`utils/robot_utils.py`) replaces it, and is wired into
each backend's **background `_read_loop`**, not `read()`. That placement is the
whole point:

- The record loop takes frames through `CameraReadGuard.read()` →
  `cam.async_read()`, which returns the cached `latest_frame` **without
  blocking**. A slow capture therefore never stalls the record loop — it means
  the loop is handed the _same_ frame, and roughly `duration / budget` recorded
  frames are duplicates of one image. Read the warning as "the loop was blocked"
  and you will go looking in the wrong place; the message says stale frames
  because that is the actual cost.
- Only `XenseTactileCamera` has `_read_loop` call `self.read()`. OpenCV,
  RealSense and ZMQ capture via `_read_from_hardware()` and their `read()` is not
  in the record path at all — instrumenting `read()` there measured nothing,
  which is why only `[XenseCam]` warnings ever appeared.
- Timing the loop rather than `read()` also keeps the connect-time warmup out of
  it, where "errors are expected" and a slow read means nothing.

Nothing else sees this class of problem. `[slow_frame]` cannot — the loop is not
blocked. `CameraReadGuard`'s freeze detection cannot either: `CAM_FREEZE_TIMEOUT_S`
is 2s, deliberately well above any frame interval so a slow sensor never trips
it, and the stalls that matter are shorter. Measured on the rig: 8-stream nvenc
warm-up and episode save starve the tactile capture threads for 0.3–0.9s, i.e.
~10–25 duplicate frames, every episode boundary.

Reporting is gated on a dataset actually being written (`set_capture_recording`,
bracketed per episode in `lerobot_record.py`). Encoder warm-up, the reset phase
and the save/encode gap all stall captures but record nothing, so a stall there
damages nothing and warning about it is noise nobody can act on — in the measured
run, all eight onset warnings of the session fired outside an episode. The gate
is process-global on purpose: a camera is a leaf with no reference back to the
session, and "is a dataset being written right now" is a property of the process.
It defaults **open**, so a consumer that never manages it (teleoperate, ad-hoc
scripts) still gets the diagnostic.

The first stall after a clean window is logged as it happens; the rest of the
window folds into one summary carrying the worst stall's **wall-clock time** —
that is what tells you whether it landed inside an episode or in the gap between
two. Captures inside budget say nothing, and a camera with no configured `fps`
has no budget and is silent by design. `str(owner)` is resolved only when a
warning is emitted; building it per call is the cost the class exists to avoid.

### Duplicate frames are counted by identity, never by pixels

`CameraReadGuard.stale_frame_report()` reports, per episode, how much of what
went to disk is a repeat of the previous frame:

```
[stale_frames] episode 3  [left_tactile_left] 45/1800 frames served stale (2.5%): 25 gap(s), longest 21 frame(s)
```

The predicate is `frame is prev` — Python object identity, already computed for
freeze detection. **Do not replace it with a pixel or hash comparison.** A
resting tactile gel barely changes between frames, so any content test flags the
normal case; and the recorded stream is lossily encoded, which destroys
bit-exactness in both directions (noise-level differences quantize to identical
output). A real capture allocates a new array — `_format_read_result` returns
`np.ascontiguousarray` of a reversed view — so identity is exact and free.

Run length is what separates the two causes, which is why it is reported
alongside the percentage:

- **a long run** is a capture stall (`CaptureStallMonitor` reports the same event
  from the capture side; the two should agree, and if they do not, something else
  is producing stale frames);
- **many one-frame runs** are the beat between two unsynchronised 30 Hz loops —
  the sensor's background capture and the record loop each free-run at the same
  nominal rate, so the phase drifts and the loop occasionally samples twice
  before a new frame lands. Tolerated by design, but nothing measured it until
  now.

Logged at INFO, not WARNING: some duplication is expected, and warning on it
every episode is the noise this whole change removed. Promote it once a rig's
baseline is known and there is a defensible threshold.

The tally is **drained** per take and the line is **printed after the reset**.
Both halves are load-bearing. `stale_frame_report()` clears `_freshness`, so it
has to be called once per take or two takes add together — which is why a
discarded take cannot simply skip it. But a retake keeps the same index
(`clear_episode_buffer()` leaves `dataset.num_episodes` alone), so printing at
collection time gave three identical `[stale_frames] episode 143` headers for one
saved episode, two of them describing frames that were thrown away; a rig
reported it as "the same episode keeps coming up in the terminal". Disposition is
only known after the reset — the left arrow arrives either during the take or
during the reset that follows, and the second is the common flow (end with right,
decide it was bad while resetting) — so the flag is read there and a dropped take
is labelled `(discarded take)`. Nothing accumulates in between (the
`set_capture_recording` gate is closed in the record loop's `finally`), so the
wait costs only the log lines' position.

### libav noise — `quiet_libav()`, not `setLevel`

`video_utils.py` calls `av.logging.restore_default_callback()` deliberately:
PyAV's Python callback "sometimes doesn't play nicely with multi-threaded
workflows" and the streaming encoder is threaded. But once restored, **only
ffmpeg's own level matters** — the `logging.getLogger("libav").setLevel(...)`
calls that sat next to it did nothing, which is why every episode save printed a
screen of `[mov,mp4,...] Auto-inserting h264_mp4toannexb bitstream filter` and
`Starting second pass: moving the moov atom`. `quiet_libav()` sets
`av.logging.set_libav_level` (the native printer) **and** `set_level` (PyAV's
callback, feeding the bridge), at ERROR — not `None`, because PyAV drops the
message text from raised exceptions when its logging is fully off. Those two
constant scales are different (libav `WARNING` is 24, stdlib's is 30); do not
pass one to the other.

### Keyboard events are a global X hook, not terminal input

`control_utils.init_keyboard_listener()` uses `pynput`, which hooks the whole X
session through XRecord. A right arrow pressed in **any** window — the Rerun
viewer (whose timeline steps on arrow keys), a browser, another shell, and in a
container the whole _host_ desktop — ends the episode exactly as if it had been
typed at the recording terminal. When an operator reports "it exited and I never
touched the arrow key", that is the mechanism; the handler logs through spdlog
so the press carries a timestamp. It used to be a bare `print()`, which is
block-buffered on a redirected stdout and could therefore surface in a captured
log well after the press it described.

`exit_early` is cleared at the top of each episode in `lerobot_record.py`, and
that line is load-bearing. The listener runs for the whole session but only the
record/reset loops consume the flag, and between the reset loop ending and the
next episode starting there is a gap — `save_episode()` plus the encoder warm-up
— with no consumer (~2s with streaming encoding, much longer without).

A right arrow pressed there is still set when the next episode's first iteration
checks it. That episode ends after zero frames, the reset then runs its **full**
duration, and `save_episode()` reaches `validate_episode_buffer`, which raises on
an empty buffer — `You must add one or several frames with add_frame`. The
session dies mid-collection, minutes after the press that caused it, which is
what makes it hard to connect back to a key someone pressed.

`rerecord_episode` is cleared on the same line for a milder version of the same
problem. A **left** arrow in that gap is self-correcting as far as the buffer
goes — it sets `exit_early` too, so the next episode ends at zero frames, but the
empty buffer then reaches `clear_episode_buffer()` rather than `save_episode()`
and the take is simply retried. What it does without the clear is announce
"Re-record episode" for an episode that never started, after sitting through a
full `reset_time_s` for a scene nobody disturbed. `stop_recording` is deliberately
_not_ cleared: that one should survive the gap.

Upstream lerobot and the sister repo `lerobot-xense` both carry the `exit_early`
hole, so the clear is a deliberate divergence, not a fork-local quirk being
papered over. Our loop's break conditions have since diverged further — see "The
event contract" below.

### The event contract — one break signal, two intent flags

A record loop sees three flags and must treat them differently:

| flag               | meaning                    | consumed by                                 |
| ------------------ | -------------------------- | ------------------------------------------- |
| `exit_early`       | leave **this** loop now    | the loop that breaks on it — always         |
| `rerecord_episode` | discard the take, retry it | `record()`, after the reset                 |
| `stop_recording`   | end the session            | `record()`'s outer `while` — never the loop |

So `self_driven_record_loop` breaks on `exit_early` and on **nothing else**,
which is upstream's `record_loop` exactly. Every keypress that should end it sets
`exit_early` too (`control_utils.on_press`), so one condition covers all three —
left arrow ends the loop _and_ leaves `rerecord_episode` standing for the caller,
which is the whole point of an intent flag.

Checking the intent flags as well is not merely redundant, it is how the bug got
in. Nothing ever sets `stop_recording` on its own either — ESC sets both, and the
`device_lost` path sets it and breaks on the next line — so a `stop_recording`
branch here is dead code whose only effect is which line gets logged. Do not add
one back.

There used to be a third writer, and it is worth knowing it is gone: the teleop
refresh (`control_utils.refresh_events_from_teleop`, called per iteration through
`refresh_listener_events`) polled the Pico4 reset button into a `go_start` flag
**nothing ever read**. The whole path — flag, refresh, and the Space key that
also set it — was removed; `poll_buttons` / `get_reset_button` remain on the
teleoperators, just not wired into any loop. So the event table above is the
complete list of flags, not a summary of a longer one.

Breaking on `rerecord_episode` instead is what put reset activity _inside_
episodes. That break left `exit_early` unconsumed; the reset phase is this same
loop with `dataset=None`, so it broke in its own first iteration and the retake
got **0 seconds** of reset while `log_say("Reset the environment")` was still
playing (`spd-say` is non-blocking — "Reset the environment" / "Re-record
episode" / "Recording episode N" simply queue up). Measured 2.35s from that
announcement to "Recording episode 56", twice in one session. Nothing from the
reset phase is ever written to the dataset; the contamination is entirely the
operator putting the scene back while the retake records them doing it.

The `or events["rerecord_episode"]` in the skip-reset condition — there so a
retake of the _last_ episode still gets reset time — was dead for the same
reason, and needed no change of its own to come back to life.

`"Recording episode N"` stays **non-blocking**, as in `lerobot-xense`. Making it
`blocking=True` looks like part of the same fix and is the wrong direction: it
opens a ~1s window in which nothing is recorded while the operator, hearing the
announcement finish, has already started moving — it drops the beginning of the
demonstration and puts speech-dispatcher's latency inside the record loop. The
phrase playing over the take's first second is correct; a second of lead-in at
the head of an episode costs nothing.

`lerobot-xense` has the same defect, but only on its **RT** path:
`run_rt_record_loop` checks `rerecord_episode` before `exit_early` and breaks
without consuming it, so the reset (the generic `record_loop`, whose first check
is `exit_early`) exits at once. Its non-RT robots run `record_loop` for both
phases and that one ignores `rerecord_episode` entirely, so they get a real reset
by accident. Unfixed there as of `2b737e45`.

## The streaming encoder spent more time on stats than on encoding

`_CameraEncoderThread.run` does three things per frame: hand the array to PyAV,
encode it, and fold it into `RunningQuantileStats`. Measured per frame per
camera on a Core Ultra 9 275HX, before the fix:

| step                             | tactile 400x700 | wrist/head 480x640 |
| -------------------------------- | --------------- | ------------------ |
| `stats_tracker.update()`         | **2.22 ms**     | **2.41 ms**        |
| ├ `_update_histograms`           | 1.37 ms         |                    |
| ├ mean + mean-of-squares         | 0.46 ms         |                    |
| └ min/max                        | 0.35 ms         |                    |
| encode (libsvtav1 preset 12)     | 1.16 ms         | 1.00 ms            |
| `Image.fromarray` → `from_image` | 0.21 ms         | 0.24 ms            |

A bimanual rig with the head camera is 8 streams at 30 fps, so the statistics
alone were ~0.53 cores — **twice what the actual video encoding cost**. That is
the budget that decides whether the rig runs on a smaller PC, and none of it was
buying anything: the histogram was a 5000-bin approximation of data that has
only 256 possible values.

- **uint8 goes through `RunningQuantileStats._update_uint8`**, which derives
  every statistic from one `np.bincount` per column. A 256-bin histogram of
  uint8 is a _complete_ description of the batch — mean, mean-of-squares, min
  and max are dot products or searches over it — so the separate mean / square /
  min / max / `np.histogram` passes all collapse into it. **2.22 ms → 0.11 ms**,
  and the quantiles come out _exact_ (verified against `np.quantile`) where the
  5000-bin version interpolated. Bin `i` spans `[i-0.5, i+0.5]`, covering the
  whole domain from the first batch, which is why this path never calls
  `_adjust_histograms`: a later batch can widen min/max but can never fall
  outside the binning. The mode is fixed by the first batch and a later dtype
  change is refused — mixing would mean two binnings feeding one histogram.
- **`av.VideoFrame.from_ndarray` instead of the PIL round-trip**, 0.21 → 0.04 ms.
  It needs a C-contiguous buffer, which the CHW→HWC branch just above it does
  _not_ produce, hence the `flags["C_CONTIGUOUS"]` check — a no-op on the normal
  path, where the camera backends already return contiguous arrays.

Together: ~20.3 ms → ~1.3 ms of per-frame CPU across 8 streams, i.e. **~0.6
cores handed back**.

### …and `std` was exactly 0 for every image and video feature

Same code, a separate bug, found while measuring the above. `update()` computed
`np.mean(batch**2)`, and `batch**2` on a **uint8** batch squares _in uint8_ and
wraps: mean-of-squares came out ~106 instead of ~21442, so
`variance = mean_of_squares - mean**2` went negative, `np.maximum(0, variance)`
in `get_statistics()` clamped it, and `std` landed at exactly 0.0.

This was **not** streaming-only. `compute_episode_stats` feeds `sample_images()`
output — also uint8 — through `get_feature_stats` into the same class, so the
non-streaming path had it too. Every image/video feature in every dataset this
fork has written has `std == 0`, whatever `--dataset.streaming_encoding` said.
The fix is `np.square(batch, dtype=np.float64)` on the general path (uint8 now
bypasses it entirely); `mean`, `min`, `max` and the quantiles were always fine,
because the `/255.0` normalisation happens downstream in `save_episode`.

**Datasets recorded before this fix need their stats recomputed** if anything
downstream normalises by `std`.

## `_last_obs_timing` — the `[slow_frame]` breakdown that never printed

`_format_slow_frame_obs_suffix` (in both `lerobot_record.py` and
`lerobot_teleoperate.py`) renders `obs= arms= grips= cams= top_obs=` off a
`_last_obs_timing` dict on the robot. **Nothing ever wrote that dict**, so the
suffix was always `''`: the warning could say the loop overran but never which
device made it overrun. On a rig sitting at 24.1 ms against a 33.3 ms budget
that is the one number worth having, and its absence is why the load question
kept getting answered by guesswork.

Both TacCap robots now populate it in `get_observation()`. The key names are
what the formatters parse and are not free-form: `<label>_arm_ms` (the pose
source — a _tracker_ on a handheld, not an arm), `<label>_grip_ms`,
`cam[<name>]_ms`, plus `cameras_ms` and `total_ms`. `perf_counter()` is ~40 ns a
call against milliseconds being measured.

`lerobot_record.py`'s formatter also gained an "everything else" bucket so a key
outside those three categories (`head_pose_ms`) appears in `top_obs` instead of
widening an unexplained gap between `obs=` and the parts. When reconstructing
the key for that bucket, note `arm_items`/`grip_items` strip only `_ms` — their
names still carry `_arm`/`_grip`, so the key is `name + "_ms"`; appending the
category again matches nothing and lists every arm and grip twice.

## The jaw encoder is streamed, not polled — `gripper_stream_hz`

`Encoder::read_once` is `send_cmd(GetEncoder)` and a wait for the ACK: a
synchronous command round-trip over the CH343, on the record loop, once per
gripper per frame, on a USB bus that six cameras are saturating with
isochronous traffic. The SDK's own numbers put the _mean_ at 0.5–0.9 ms on a
quiet bus; nothing bounds the tail, and the loop sits at ~24 ms against 33 ms.
It was the last synchronous hardware wait left on the loop — the trackers, the
head pose and every camera are already cached reads.

`Cmd::StartStream` has the firmware push `EncoderData` at `gripper_stream_hz`
(default 100, the firmware default; only divisors of 1000 are exact).
`GripperReadGuard.subscribe` registers `encoder.on_data` — the SDK's transport
thread copies three floats into `_GripperStream` — and `read()` serves the
cache. `0` restores polling. Leaders only: follower firmware streams motor
status and nothing else, and ignores the encoder/IMU rates (measured, per the
SDK's `control_loop.hpp`).

Things that are not obvious from the code:

- **`enable_imu` streams the IMU too, and that is not for symmetry.** Once a
  stream runs on the link, any _command_ on it is exposed to a firmware UART
  defect that corrupts the occasional ACK (tc-gu-01 issue #1). Commands retry,
  ~31 ms each, surfacing as latency. Streaming both sources is what leaves no
  per-frame command on the bus for that to hit; a polled IMU next to a streamed
  encoder would be the worst of both.
- **Silence is the failure mode.** A polled gripper fails by raising; a
  streamed one fails by going quiet. The guard treats "no sample for
  `timeout_s`" as loss and trips it _immediately_ — the clock starts at the
  last sample, not at the read that noticed — because the silence already
  spans the timeout. Same `ENCODER_LOSS_TIMEOUT_S`, same degrade-to-last-good.
- **Bring-up failure falls back to polling, logged, not raised.** A rig that
  records with the old round-trip beats one that refuses to start. The log line
  to look for is `encoder streamed by the firmware at 100 Hz`; its absence with
  a `stream unavailable` warning means you are polling.
- **`unsubscribe_all()` runs first in `_release`**, before `stop_streaming`,
  so the transport thread is not delivering into a guard for a device being
  torn down.

**Measured on the rig, 2026-09-02** (two leaders, fw 1.2.2, `TCGU01A28Z0115m`
/ `0116m`): the stream comes up in 11–21 ms, the firmware delivers exactly 100
Hz (200 distinct samples in 2 s), `read()` is p99 **0.04 ms** with a sample at
most 9.9 ms old, IMU streams alongside, and after `unsubscribe_all()` the bus
answers `read_once` as before. For scale, `read_once` itself was p50 0.27 ms
on a quiet bus and 0.32 ms / max 0.43 ms with six cameras open — so the
round-trip was never the 10 ms; what the stream buys is that no synchronous
bus wait is left on the loop at all. `--robot.gripper_stream_hz=0` is the
fallback, and a full recording on it still runs clean.

### The frame copy in `feed_frame` is off for TacCap

`StreamingVideoEncoder.feed_frame` snapshots each image before queueing it, in
case the camera driver recycles its buffer. The TacCap backends never do: Xense
returns `np.ascontiguousarray` of a reversed view, OpenCV `cv2.cvtColor`, Pico
`cv2.imdecode` — a fresh array each, and nothing downstream mutates one. So
`lerobot_record` sets `copy_frames = False` on the encoder for the self-driven
robots: eight ~0.9 MB copies per frame, **1.7 ms** measured on the record
thread. Left on for every other robot — RealSense hands out views of the
driver's buffer.

### What the main loop does and does not spend time on (measured)

Per frame, bimanual + head camera, on the record thread: `build_dataset_frame`
×2 **0.004 ms**, `validate_frame` **0.004 ms**, `feed_frame` ×8 without the
copy **~0.3 ms**. GC: a simulated 60 s episode triggered **zero** collections —
per-frame garbage is refcount-freed and the episode buffer holds numpy arrays,
which are untracked — so `gc.freeze()`/`gc.disable()` are not a lever here.
The loop's time is in `get_observation`; `_last_obs_timing` (above) says where.

**How to read it.** `cam[...]` reads are `async_read` — a lock and a dict
lookup, microseconds. If they show milliseconds, the record thread is waiting
for the **GIL**, not for a camera: `xensesdk` is Cython
(`__compile__.cpython-312-*.so`), and Cython holds the GIL unless the code
says `nogil`. Four tactile capture threads each calling `selectSensorInfo` at
30 Hz is the candidate. Measured cost of that pattern on this host: 4 threads
holding the GIL in 8 ms bursts put the main thread's re-acquire at **p99 13.6
ms**; 12 threads in 1 ms bursts, p99 4.4 ms. **`sys.setswitchinterval` lower is
not a fix** — 1 ms made the 8 ms-burst case _worse_ (p99 29.6 ms), because it
round-robins the GIL among every runnable thread and the main thread queues
behind all of them. If the probe confirms it, the fix is process isolation
(tactile capture in a subprocess over shared memory), not a knob.

**What a real recording measures without Pico/tracker** (2026-09-02, this
host, 4 tactile + 2 wrist, `episode_time_s=12`): **zero `[slow_frame]`** in
every configuration tried — nvenc on all 24 cores, `libsvtav1` pinned to 4
cores with default threads, with `encoder_threads=1`, with `--display_data`,
and with the polled encoder. The 8-stream overrun the data team hit is
therefore not reproducible from these six streams alone; the untested
difference is the Pico head camera + trackers, and a different host.

Two things the 4-core runs did show, neither of them an overrun:

- **The loop drifts slow under load without ever overrunning.** 349–355 frames
  per 12 s episode on 4 cores (359 unconstrained), 343–345 with Rerun on: the
  body stays inside the budget so nothing is logged, but `time.sleep` wakes late
  when the cores are busy. `add_frame` stamps `frame_index / fps`, so the
  dataset claims 30 fps while capture ran at 28.7–29.5 — a 2–4% time
  compression that nothing on disk records. `precise_sleep` (spin for the last
  10 ms) would hold cadence at the cost of a core; writing the real timestamp
  would be honest but changes the file format. Neither is done; know it exists.
- **Six SVT-AV1 streams on 4 cores cost ~1.8 cores** (`ps`, process-wide,
  incl. the tactile SDK): 182% mean / 211% peak, identical with 522 threads
  (`encoder_threads` unset) and 150 (`=1`). The thread count is not the CPU.

**Encoder threads on a small CPU.** `--dataset.encoder_threads` unset lets
each SVT-AV1 instance size its own pool: 8 streams open **628–632 OS threads**
on a 4–8 core box (measured, `/proc/self/status`). `encoder_threads=1` makes
that 132. In isolation the main loop barely noticed either on 4 cores (p99
18.6 vs 20.7 ms with an 18 ms body) — SVT's threads are native and yield — but
on a machine the tactile SDK is also competing for, fewer is the safe default.
`h264` with `preset=ultrafast, threads=1` was flat (p99 18.0, 4 threads total)
at similar size; the record CLI does not expose a preset for `h264`, and
libx264's default `medium` is _slower_ than AV1 preset 12, so do not switch
codecs without adding one.

## libsvtav1 sessions grew ~1.3 GB per episode — glibc arenas, not a leak

Found by the `[loop_summary]` `rss` field on its first outing (2026-09-02):
six 400x700 SVT streams, 5-second episodes, RSS **2.5 → 8.8 GB over six
episodes**, linear. nvenc flat. Nothing reachable: `gc.collect()` freed 955
objects and returned nothing.

Each episode starts fresh encoder threads. glibc gives a new thread a new
malloc arena (up to 8 × cores of them), libsvtav1 allocates a few hundred MB
per encoder, and when the thread ends that memory is freed **into that
arena** — where it stays, because the next episode's threads land in other
arenas and nothing reuses it. A 16 GB box without a GPU reaches swap inside
an hour of normal recording, and from the record loop swap reads as overruns.
This is the strongest candidate for the field report that started all this.

`StreamingVideoEncoder._return_freed_memory` calls `malloc_trim(0)` after
every `finish_episode` / `cancel_episode`: it walks every arena and returns
its free pages. Same six episodes: **2.55 → 2.67 GB**, ~55 ms per call, paid
in the gap between episodes. Things that were tried and are not the fix:

- releasing the stream before `container.close()` — matters on the main
  thread (+547 vs +21 MB for six encoders) and is kept as hygiene, but inside
  the worker thread it changed nothing;
- `MALLOC_ARENA_MAX=1/2` — slows the growth to a ~3.9 GB plateau, process-
  global, still not flat;
- `encoder_threads=1` — smaller working set (~1.1–1.8 GB) but still creeps.

`TestStreamingEncoderReturnsMemoryBetweenEpisodes` measures RSS across three
episodes rather than trusting the call is still there; with the trim
monkeypatched out it grows ~1 GB and fails.

## Reading a session log — diagnosing an overrun without the machine

`~/xenselogs/session_<ts>.log` (`$XENSE_LOG_DIR`) is written at DEBUG and is
what a rig in the field should send. Since 2026-09-02 it carries enough to
diagnose a `[slow_frame]` without ssh. The tags, in the order they appear,
and what each one settles (`lerobot/utils/loop_diagnostics.py` owns the
formats; `lerobot_record.py` and `video_utils.py` emit them):

**`[session] host …` / `[session] recording …` / `[session] robot …`** —
once, after `connect()`. Read these before the first symptom:
`usable` < `cores` means `taskset` or a container; `governor=powersave` on a
laptop is late `sleep` wake-ups waiting to happen; `gpu=none` +
`vcodec=libsvtav1` is software AV1; `encoder_threads=None` is one SVT pool per
stream (~520 threads for 6–8 streams); `git=` says which code ran;
`gripper_stream_hz=0` means the jaw is polled. `copy_frames=False` is
expected on TacCap.

**`[slow_frame] … loop= budget= overrun= | phases obs= build= add= display= |
obs= arms= grips= cams= top_obs=`** — one frame that overran. The first five
per take are WARN (console and file); the rest go to DEBUG (file only) and a
`[slow_frame_summary]` WARN names the window's worst every 5 s, so a loop that
overruns every frame still leaves a readable log. Which phase is fat says
where to look:

| fat phase | meaning                                                                                                | cross-check                                                    |
| --------- | ------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------- |
| `add`     | `feed_frame` blocked — a queue is full, `put()` waits up to 100 ms before dropping                     | `[encoder_summary]` queue high-water near max, put blocked >0  |
| `obs`     | with `cams=` at milliseconds: GIL / CPU starvation, not a camera — `async_read` is a lock and a lookup | `[loop_summary]` cpu vs usable cores; encoder p99 inflated too |
| `obs`     | with `grips=` at milliseconds: the jaw is polled (`gripper_stream_hz=0`) or the stream fell back       | `[session] robot`, a `stream unavailable` warning at connect   |
| `obs`     | with `arms=` or `head_pose` fat: tracker / headset SDK                                                 | Pico-side logs                                                 |
| `display` | should be microseconds (`RerunLogSink.submit` is a slot swap)                                          | —                                                              |

**`[loop_summary] episode N[ (discarded take)]: F frames in Ts = X fps
(nominal 30…) | body ms … (budget) | overruns … | sleep-late ms … | phases
p99/max ms: … | cpu …% rss …MB threads-peak … load … | gc gen0/1/2 …`** —
one per take, printed with the `[stale_frames]` lines after the reset. What a
per-frame line cannot show:

- **effective fps below nominal with `overruns 0`** is late `sleep` wake-up
  under load (`sleep-late` p99 says so). The body was in budget, nothing
  warned, and `add_frame` stamps `frame_index / fps` — the dataset claims 30
  fps and the recording ran at 29. Measured on a 4-core pin: 29.0 fps,
  sleep-late p99 5.9 ms, zero overruns.
- **`cpu`** is the process over the take; compare with `usable × 100`.
  253% on 4 usable cores is a starved machine even though nothing overran.
- **`rss`** across episodes is the memory trend — it is what exposed the
  arena growth above; a rising series with `vcodec=libsvtav1` on a build
  without the trim is that bug. **`threads-peak`** 520+ is the SVT default
  pools; **`gc`** counts collections (gen0 is cheap; gen2 here is worth
  matching against overrun times).

**`[encoder_summary] <key>: F frames, encode ms p50/p99/max, stats ms p99,
queue high-water h/max, put blocked n×/ms, dropped d`** — one per stream per
saved episode, from the encoder thread itself. Reference points from this
host: unloaded, encode ~1 ms and stats ~0.1 ms; pinned to 4 cores with six
SVT streams, encode p99 13–15 ms and stats p99 7 ms — same code, starved
threads. Encode p99 near the frame period or a high-water mark near the
maximum is the encoder side about to back-pressure the loop.

`[stale_frames]`, the `background capture stalled` warnings and the
`Rerun display: N/M frames dropped` line at exit are the older layers and are
documented above; the stall warnings at connect (before "Recording episode
0") are warm-up and expected.

## Recorded files land 0600 if they come from a temp file

Python's temp-file APIs create restrictively **on purpose**, and `shutil.move` /
`os.replace` carry the mode onto the destination:

| API                  | mode   |
| -------------------- | ------ |
| `NamedTemporaryFile` | `0600` |
| `mkstemp`            | `0600` |
| `mkdtemp`            | `0700` |

So "write to a temp file, then move it into place" quietly produces a dataset
only its writer can read. In the container the writer is root, so exporting as
yourself fails — and only on the files that took that path, which is what makes
it confusing: `dac15f74` was reported as `cp: cannot open '.../file-000.mp4':
Permission denied` on **every video while the parquet and json copied fine**,
because those are written straight through `pq.ParquetWriter` / `write_text`
and land `0644` under the normal umask (`0022` on the host and in the image).

**Anything that moves a temp file into a dataset needs an explicit
`chmod(0o644)` after the move.** `video_utils.py` does this after
`shutil.move(tmp_output_video_path, output_video_path)`. Do not reach for
`os.umask` instead — encoding runs in a thread pool and umask is process-global.

The same trap has now appeared twice: once in the video concatenation path, and
once in a proposed `_atomic_write_parquet` that would have moved data parquet
from `0644` to `0600` (reviewed on PR #19).

### Host gotcha — `import xense.taccap` must precede torchvision

torchvision ships a vendored `libjpeg` that claims the `LIBJPEG_8.0` symbol
version but carries **none** of the `jpeg12_*` symbols conda's `libtiff` needs.
Whichever loads first wins the version slot, so once torchvision is in, every
later `import xense.taccap` — which reaches libtiff via
`libopencv_videoio` → `libopencv_imgcodecs` → `libtiff` — dies with:

```text
ImportError: .../libtiff.so.6: undefined symbol: jpeg12_write_raw_data, version LIBJPEG_8.0
```

`lerobot_record.py`, `lerobot_replay.py` and `lerobot_teleoperate.py` each carry
a `contextlib.suppress(ImportError)` import of `xense.taccap` **above every
lerobot import** for exactly this. Moving it below them puts the bug back.

Only those three need it: they are the entry points that both pull torchvision
(through `lerobot.datasets`) and touch the SDK. `lerobot-calibrate` never loads
torchvision at all, so it is fine as-is — verify before adding the block
elsewhere rather than sprinkling it.

The failure is nasty out of proportion to its cause: nothing fails at startup,
because the SDK import is what breaks and the robots only reach for it later. On
the sister repo `lerobot-xense`, whose `XenseWristCamera` imports
`FisheyeUndistorter` lazily inside `connect()`, the same conflict surfaced as a
recording dying at camera-connect time. `LD_PRELOAD` does not help — torchvision's
copy is auditwheel-renamed (`libjpeg.4af9affd.so.8`), so it is not competing on
soname but on the symbol version, and preloading cannot outrank it.

## Vendored SDK

`third_party/taccap-gripper` is the TacCap-Gripper SDK submodule (has its own
`CLAUDE.md`). `xense.taccap` is gripper-protocol + wrist-camera only; tactile
_imaging_ (rectify) is handled at the Python level via the `xensesdk` wheel.

**There is no XenseVR-PC-Service submodule.** `xensevr_pc_service_sdk` is built
from **our** pybind under
`src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/`, and the C SDK it
links — `PXREARobotSDK.h` + `libPXREARobotSDK.so` — is copied by `setup_env.sh`
out of the installed `xensevr-pc-service` `.deb`
(`/opt/apps/roboticsservice/SDK/{include,x64}`; `arm64` on arm64 hosts). The
pybind's `include/` and `lib/` are gitignored staging, not sources.

A 31 MiB checkout of Qt service sources and prebuilt gRPC archives used to be
cloned purely to rebuild a library that `.deb` already shipped. Removing it took
a recursive clone from ~33 MiB to ~1.6 MiB and dropped a cmake + static-gRPC
link from every install. **The trade:** an SDK _source_ fix now has to travel
through a `.deb` release — bump `debPack/control`, rebuild, publish, and bump
`DEB_VER` in `setup_env.sh`. Re-releasing the same version number does not
work: `install_xensevr_service()` skips a `.deb` whose version already matches
what dpkg reports.

Two consequences worth remembering:

- **nlohmann comes from conda** (`nlohmann_json=3.11.3` in
  `conda_environment.yaml`, `find_package`d by the pybind CMakeLists).
  `py_bindings.cpp` needs `<nlohmann/json.hpp>`; it used to resolve by accident
  because the submodule vendored a copy that `setup_env.sh` shovelled into
  `include/`. The `.deb` does not carry it.
- **`setup.py:sdk_version()` asks dpkg**, not a submodule tag. The `.deb` is not
  a proxy for the SDK, it _is_ where the SDK came from — so `pip list` and the
  installed daemon can no longer disagree.

To work on the C SDK itself, clone `XenseRobotics-AI/XenseVR-PC-Service` separately. Its
Windows and aarch64 trees were pruned; the Linux Unity demo lives as a release
asset that `SDKDemo/UnityBin/fetch_linux_demo.sh` fetches on demand.

The Insight head camera and its `pyinsight` submodule are **gone**, as is
`XenseVR-RobotVision-PC` (the ZED-M passthrough). Head vision is the Pico
headset camera above; do not reintroduce them.
