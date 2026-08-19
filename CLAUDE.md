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
Helpers live in `taccap_gripper/common.py`; a resume against different hardware
warns and keeps the original file rather than misattributing recorded episodes.

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
