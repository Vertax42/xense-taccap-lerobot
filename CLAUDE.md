# lerobot-xense — Claude working notes

Fork of HuggingFace `lerobot` (tracks upstream v5.1), slimmed to the
**TacCap-Gripper** (single + bimanual) handheld device and the **Pico4**
teleoperator/tracker, with Xense tactile cameras. See
`src/lerobot/robots/taccap_gripper/README.md` for usage.

## TacCap serial / topology rules (device auto-discovery)

Devices are auto-discovered and assigned to `left`/`right` by serial + USB
topology rules — **no serials are hand-listed**. Source of truth:
`src/lerobot/robots/taccap_gripper/serial_discovery.py`.

### Serial grammar
- Gripper : `TCGU01<batch><line><seq><m|s>` — e.g. `TCGU01A24Z0002m`
- Tactile : `GSPS01<batch><line><seq>`      — e.g. `GSPS01A25Z0011`
- Camera  : `XC<batch><line><seq><m|s>`     — e.g. `XCA24Z0007m`
- `<seq>` is 4 digits; patch `m` → Master/Leader, `s` → Slave/Follower.

### Side rule — 单左双右 (`side_of_sequence`)
The **last digit** of the 4-digit sequence: **odd → left, even → right**.
Applies to gripper / camera side and to tactile *finger* (below).

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
Tell: **first** launch works, but unplug → other port → relaunch *immediately*
is busy. **Not** a tactile/camera/bandwidth issue. Permanent fix is a udev rule
ignoring `1a86` (`ID_MM_DEVICE_IGNORE=1`) — see README → "Hardware bring-up
sequence". (`brltty` grabs `1a86` the same way if installed.)

## Vendored SDK
`third_party/taccap-gripper` is the TacCap-Gripper SDK submodule (has its own
`CLAUDE.md`). `xense.taccap` is gripper-protocol + wrist-camera only; tactile
*imaging* (rectify) is handled at the Python level via the `xensesdk` wheel.
