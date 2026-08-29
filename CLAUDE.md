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

Each tactile sensor's **runtime bundle** goes in beside it, at
`meta/runtimes/<serial>-<local time>.bin` (`RUNTIME_DIR`), with the epoch's
sensors pointing at their own. Deriving depth / force / difference from the
recorded `rectify` stream needs the bundle that was current **when the episodes
were recorded** — it carries the reference image captured at `Sensor.create()`,
and a sensor that comes back from maintenance keeps its serial but produces a
different bundle, which is why the name is timestamped and why a new bundle
opens its own epoch. Solving against the wrong one does not fail: an untouched
gel returns plausible depth and force. So `tactile_runtime_for_key` returning
`None` means **skip derivation**, never "use whichever bundle is nearest".

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

## Pico headset camera (`--robot.enable_head_camera`, off by default)

Records `left_head` / `right_head` (one key per **eye** — on a bimanual rig
`{side}_wrist` is per-arm, but there is one headset, so the prefix means
something different) plus `head_camera.*`, the headset pose.

- **`head_camera.*` is remapped like the tracker.** Same `PICO_TO_WORLD_R`
  conjugation, same xyzw→wxyz reorder, so it lands in the world frame `tcp.*`
  uses. The SDK hands out `HeadsetPose` / `ControllerPose` / `MotionTrackerPose`
  through one `stringToPoseArray`, so all three share a layout — do not assume a
  different convention for any of them.
- **Do not read the SDK frame cache from the record loop.** The eyes arrive as
  separate messages, left first, so sampling at loop rate catches one updated
  and not the other (measured 7% of frames). `cameras/pico/stereo_poller.py`
  polls at 120 Hz and publishes only pairs whose `frame_sequence` agrees.
- **Resolution is a whitelist**, `640x480` / `1024x768` / `1280x960`, all 4:3
  like the sensor and all three offered by the headset app's Resolution setting.
  The default is `640x480` because that is the app's default, so the two line up
  untouched. Unlisted sizes and a first frame that disagrees with the config are
  errors, deliberately — silent rescaling would change the recorded field of
  view without trace. Do not "fix" this with a resize.

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
in. Nothing ever sets `stop_recording` on its own either — ESC sets both, the
teleop refresh (`control_utils.refresh_events_from_teleop`) writes only
`go_start`, and the `device_lost` path sets it and breaks on the next line — so a
`stop_recording` branch here is dead code whose only effect is which line gets
logged. Do not add one back.

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

To work on the C SDK itself, clone `Vertax42/XenseVR-PC-Service` separately. Its
Windows and aarch64 trees were pruned; the Linux Unity demo lives as a release
asset that `SDKDemo/UnityBin/fetch_linux_demo.sh` fetches on demand.

The Insight head camera and its `pyinsight` submodule are **gone**, as is
`XenseVR-RobotVision-PC` (the ZED-M passthrough). Head vision is the Pico
headset camera above; do not reintroduce them.
