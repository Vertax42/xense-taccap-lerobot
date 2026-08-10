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
- **Resolution is a whitelist**, `1024x768` / `1280x960`, both 4:3 like the
  sensor. Unlisted sizes and a first frame that disagrees with the config are
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

## Vendored SDK

`third_party/taccap-gripper` is the TacCap-Gripper SDK submodule (has its own
`CLAUDE.md`). `xense.taccap` is gripper-protocol + wrist-camera only; tactile
_imaging_ (rectify) is handled at the Python level via the `xensesdk` wheel.

`third_party/XenseVR-PC-Service` is the Pico4 service; `xensevr_pc_service_sdk`
is built from **our** pybind under
`src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/`, not from the
submodule's own packaging, and its `lib/` is gitignored — `setup_env.sh` builds
the client `.so` from submodule source and copies it in, so a source-only fix
there propagates on `--install` with nothing binary to commit. (The `.deb`
ships a prebuilt `SDK/x64/libPXREARobotSDK.so`, but it is **not** a substitute
— it lags the submodule tip, so the from-source build is load-bearing.)

That submodule is **pruned to the x86_64 Linux build** and marked
`shallow = true` in `.gitmodules`. Deleted: the Windows halves
(`Redistributable/win`, `GrpcSDK/lib`, `SDKDemo/UnityBin/RobotWinDemo`,
`SDK/win`, `Package/innosetup` — 417 MiB, all behind `if(WIN32)`) and the whole
aarch64 tree (`Redistributable/linux_aarch64`, `Package/debPackAArch64`,
`PXREAService/linux_aarch64`, the `build_aarch64.sh` scripts — arm64 is not
supported). A recursive clone went from ~313 MiB to ~104 MiB.

Deliberately **kept**, do not "finish the job" on these:

- `SDKDemo/UnityBin/RobotLinuxDemo` and `Redistributable/linux/*` — they ship
  inside the released `.deb`; check `dpkg -L xensevr-pc-service` before touching.
- `GrpcSDK/include` — needed on Linux too (`PXREARobotSDK/CMakeLists.txt:46`);
  only `lib/` was Windows-only.
- the two `stacktrace_aarch64-inl.inc` — absl headers compiled into the x86_64
  build, named for the target absl can unwind on, not for our arch.

The `if(ISA_NAME STREQUAL "aarch64")` CMake branches were left in place and now
point at deleted paths. That is intentional: arm64 is unsupported.

The Insight head camera and its `pyinsight` submodule are **gone**, as is
`XenseVR-RobotVision-PC` (the ZED-M passthrough). Head vision is the Pico
headset camera above; do not reintroduce them.
