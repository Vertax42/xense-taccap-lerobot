# `taccap_gripper` — Handheld data-collection device

The **TacCap-Gripper** (**TacCap** = _Tactile Capture_ Gripper) is a handheld
**UMI** leader gripper for tactile data collection.

Single-arm handheld data-collection pipeline. The device is **self-driven**:
there is no separate teleoperator — the robot itself produces both the
observation (state being recorded) and the demonstration action.
`lerobot-record` allows `teleop=None` for this self-driven device, so no
`--teleop.*` flags are needed on the CLI.

This is the **single-gripper** device: one unit, two tactile pads, one wrist camera, one
Pico4 tracker, and **unprefixed** observation keys (`tcp.*`, `gripper.pos`,
`tactile_left` / `tactile_right`, `wrist_cam`). For two units driven as one robot — with
`left_` / `right_` prefixes and the optional Pico headset camera — see
[`bi_taccap_gripper`](../bi_taccap_gripper/README.md). Both sides of a two-gripper rig can
also be run one at a time through this device by passing `--robot.side`.

Components:

- **Gripper:** TacCap-Gripper handheld unit (motor jaw, two embedded
  visuotactile sensors, wrist UVC camera, encoder, IMU). Driven by the
  `xense.taccap` SDK (`taccap-gripper` PyPI package, ≥ 0.1.0).
- **Pose:** Pico4 Ultra **independent motion tracker** mounted on top
  of the gripper. Reached via `xensevr_pc_service_sdk` and read by
  `lerobot.teleoperators.pico4.tracker.Pico4TrackerReader`.
- **Cameras:** plain LeRobot `cameras/` framework, read asynchronously.
  The two tactile sensors and the wrist UVC camera are **auto-discovered by
  serial rule** (`serial_discovery.py`) from `/dev/v4l/by-id` — no serials are
  supplied in config. See "End-to-end recording" below.

The device is passive: `send_action()` is a no-op; the motor is never
enabled. The operator drives the jaw mechanically and walks the device
through demonstrations.

## Coordinate frame

Recorded pose is in **our world frame by default** (X forward away from base,
Y left, Z up, gravity-aligned): `Pico4TrackerReader` applies the same Pico→world
remap the `teleop_pico4` controller flow uses (`pico_to_world=True`). The world
origin is the headset position the moment the Unity VR Client app started.

This world frame is **not** any specific robot's base frame, and deliberately so
— recorded poses stay robot-agnostic, and the base difference is cancelled at
training/deployment time by the relative pose representation rather than baked in
here. See "Why there is no init-pose alignment" below.

The axis convention (handedness, Z direction) is documented
inconsistently upstream — Pico docs claim right-handed, the SDK's
`rerun_dual_with_tracker.py` notes left-handed. **TBD pending live
verification on real hardware.**

**Do not restart the Unity client between episodes** or all subsequent
recordings will be in a different origin.

### Tracker → EEF TCP mount transform

The Pico4 tracker is bolted to the gripper, so the pose it reports is the
**tracker's**, not the TCP's — on this unit the two are ~195 mm apart. The
constant rigid offset lives in [`ee_transform.py`](ee_transform.py) and is applied
body-fixed by `Pico4TrackerReader`:

```
T_world_tcp = T_world_tracker @ X
```

Because `X` is body-fixed it rides along with the gripper, so it holds at any
orientation — starting a UMI session with the gripper pointing anywhere is fine.

- **TCP** is the two-finger midpoint. Symmetric jaws keep that point still as the
  jaw opens, so the transform is a constant and does not depend on `gripper.pos`.
- **Both sides are measured**, straight out of the SolidWorks macro as
  `^Tracker T_EE`. They are near-mirrors about the XZ plane but not exactly —
  rotation agrees with the mirror to 0.03°, translation differs by 1.27 mm — so
  neither side is derived from the other. `mirror_xz()` is kept only as a
  consistency check.
- **Leader bodies only.** The follower gripper is a different design (URDF shows
  different joint origins, a flipped jaw axis, fingertips 21 mm further out), so
  `--robot.role=follower` warns and falls back to the leader value.
- `--robot.tracker_to_ee_pos` / `_quat` default to `None` = use this side's
  built-in value. Set either one to override; they are independent, so a
  re-machined mount can pin just the translation.

> **Cross-check on the numbers.** At the CAD reference pose the EE frame is
> aligned to world, so `delta_world == R_tracker_eeᵀ · t_tracker_ee`. The macro's
> right-side values reproduce the independent measurement in
> `media/right_eef_tcp.jpg` to **0.000 mm**, which also settles that screenshot's
> sign question — its Y and Z really are negative.

> **Datasets recorded before this landed hold the tracker pose in `tcp.*`, not the
> TCP.** They are off by the handle offset and must not be mixed with newer
> episodes without re-transforming.

### Checking the mount transform in Rerun

`lerobot-teleoperate --display_data=true` draws the EE frame in the `/world` view —
a marker with 10 cm axes, labelled `EE` — trailing a breadcrumb of where it has been.

With the head camera on, the headset is drawn too — a smaller amber marker
labelled `HEAD`, no trail. It shares the gripper's world frame (the same
Pico→world remap is applied to `head_camera.*` as to `tcp.*`), so the two can be
read against each other: where the operator was looking versus what their hands
were doing. A trail is deliberately omitted — the head wanders continuously and
its breadcrumb would bury the gripper trails.

The scene declares `rr.ViewCoordinates.FLU` (X forward, Y left, Z up) rather than
the weaker `RIGHT_HAND_Z_UP`, so the viewer knows which axis is _forward_ and aims
its initial camera down +X. The origin triad is labelled `+X forward` / `+Y left` /
`+Z up` so the orientation stays readable after you orbit.

What to look for, with the gripper lying flat:

| Check              | Expected                                 |
| ------------------ | ---------------------------------------- |
| EE marker position | at the **two-finger midpoint**           |
| EE axes when flat  | X forward, Y left, Z up — i.e. **level** |

**Checked on 2026-08-02 and correct**, so this is a regression check now rather
than an open question. The marker position is the row that matters: it is the only
one that distinguishes `APPLY_G_REBASE` — both settings put the EE the right 195 mm
from the tracker, just 51° apart, so a distance check alone looks fine either way.
If the EE ever lands somewhere unrelated to the fingers, flip
`APPLY_G_REBASE` in [`ee_transform.py`](ee_transform.py).

> The viewer used to also draw the tracker's own frame and a dashed line between the
> two, labelled with its length. That was scaffolding for verifying the mount
> transform; with the transform confirmed it was just clutter over the EE frame, so
> it is gone. The tracker pose is still published as display-only `tracker.*` keys —
> absent from `observation_features`, so it never reaches a dataset — and still has a
> `tracker pose` tab in the scalar panel next to `tcp pose`, which is enough to see
> the raw numbers if the transform is ever in question again.

Verify a mount with the pivot check — no extra hardware needed:

```bash
python -m lerobot.robots.taccap_gripper.check_tracker --side right
```

Rest the two-finger midpoint on a fixed point and sweep the handle through as
many orientations as it allows. `ee xyz` should stay put while `raw xyz` swings;
whatever drift remains is the transform's error. Run it for **both** sides — a
left value mirrored the wrong way shows up as `ee` moving about twice as much as
it should.

### Why there is no init-pose alignment

`Pico4TrackerReader` can latch a rigid transform at connect time so every later
pose comes out in the frame of a supplied robot TCP pose:

```
T_align  = T_ee_init · (T_world_tracker(0) · T_tracker_ee)⁻¹
T_out(t) = T_align · T_world_tracker(t) · T_tracker_ee
```

That is a **live-teleoperation** formula, and this device does not use it. Two
reasons:

- `T_ee_init` is in a robot's base frame, so it needs that robot **present and
  localised at connect time** — which a handheld capture rig does not have.
- Even with a value, it would bake one arm's base into the dataset; a second arm
  would mean re-recording.

Base-frame differences are handled downstream instead: with the poses expressed
relative to the current EE frame, any global transform `M` between capture and
deployment cancels exactly —

```
inv(M·T(t)) · M·T(t+k) = inv(T(t)) · T(t+k)
```

— so the deployment arm's base, and the initial EE orientation it happens to sit
at, drop out on their own. The robot end only has to calibrate its TCP to the
gripper's EEF.

The reader keeps the mechanism (it is generic, and a live teleoperator would want
it); it is simply not wired to the capture path, and the config fields that used
to expose it are gone. Stacking it with a relative representation would re-base
twice.

## Hardware bring-up sequence

1. Plug the TacCap-Gripper into the host (USB).
2. **Check the USB bandwidth budget** — see below. Do this before blaming
   anything else; on a bimanual rig it is the single most common reason a
   camera will not open, and it is decided by which physical ports you used.
3. Power on the Pico4 Ultra headset; pair the motion tracker.
4. Launch the Unity VR Client app on the headset (this freezes the
   coordinate origin).
5. Start the XenseVR PC Service on the host.
6. Run any of the scripts below.

### Step 2 — check the USB bandwidth budget

Every UVC camera reserves _isochronous_ bandwidth for as long as it is open, and
that budget is **per USB 2.0 bus**: 480 Mbit/s, of which ~384 is available to
periodic transfers. The cameras are USB 2.0 devices, so plugging them into a blue
USB 3 port changes nothing — they still land on that controller's USB 2.0 bus.

Count what shares a bus:

```bash
lsusb -t
```

Each `480M` `root_hub` line is one budget. A bimanual rig puts **six** cameras
(four tactile + two wrist) on it, plus the laptop's built-in webcam if it has
one, and six is more than one bus can carry (measured below). Two `480M`
root_hubs, three cameras each, is comfortable.

Watch the kernel while you start, in a second terminal:

```bash
sudo dmesg -w | grep --line-buffered -iE "uvcvideo|bandwidth|disconnect"
```

`--line-buffered` is not optional: without it `grep` buffers and appears to hang.
`Not enough bandwidth for altsetting N` at the moment a camera fails is the
diagnosis, full stop — nothing in this repo can work around it.

If the budget is short, the fix is a **second USB host controller**, not another
hub. A Thunderbolt/USB4 dock brings its own xHCI controller; a plain hub does
not. Confirm by plugging one gripper into it and checking `lsusb -t` shows a
**new `480M` root_hub**, not another nested hub.

> **Serial-port permissions (one-time host setup).** The gripper MCU enumerates
> as `/dev/ttyACM*`, owned by the `dialout` group. If your user is not in
> `dialout`, the SDK can _list_ the grippers but cannot open the port to read
> the firmware SN — `scan_grippers()` then reports `role=Unknown` / empty
> `firmware_sn`, and `connect()` fails with
> `RuntimeError: No <role> gripper discovered for the <side> side.` (the
> underlying error is `IoError: ... Permission denied`). Fix it once with
> `sudo usermod -aG dialout "$USER"`, then start a fresh session (or
> `newgrp dialout`) and replug. See the
> [top-level README → Installation Step 6](../../../../README.md#-installation)
> for the full check.

> **`Device or resource busy` on the gripper serial after replug (ModemManager).**
> The gripper MCU is a CH343 USB-serial (`1a86:55d2`) that enumerates as a
> CDC-ACM port. On every hot-plug, **ModemManager** (the cellular-modem service
> shipped by default on Ubuntu/GNOME) probes the fresh port with AT commands and
> holds it open for a few seconds — so a `connect()` in that window fails with
> `IoError: SerialBus: open(/dev/serial/by-id/...): Device or resource busy`.
> Classic symptom: the **first** launch works (the port has settled), but
> unplug → move to another port → relaunch immediately **busy**. (`brltty`, the
> braille driver, grabs `1a86` devices the same way if installed.) Quick
> workaround: wait ~3 s after replug. Permanent fix — tell ModemManager to ignore
> these devices via a udev rule (does **not** disable it for real modems):
>
> ```bash
> sudo tee /etc/udev/rules.d/99-taccap-ignore-modemmanager.rules >/dev/null <<'EOF'
> # TacCap-Gripper MCUs are CH343 USB-serial (1a86:55d2) — keep ModemManager off them
> ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"
> EOF
> sudo udevadm control --reload-rules && sudo udevadm trigger
> ```
>
> Verify: `udevadm info -q property -n /dev/ttyACM0 | grep ID_MM_DEVICE_IGNORE`
> shows `ID_MM_DEVICE_IGNORE=1`, and `mmcli -L` no longer lists the grippers.
> Revert by deleting the rule file and reloading. (Alternatively, on a dedicated
> robot PC with no cellular modem: `sudo systemctl disable --now ModemManager`.)

> **`Cannot open camera N` on one tactile sensor (USB bandwidth).** One gripper
> hub carries **three** UVC devices — two tactile sensors plus the wrist camera —
> behind a single xHCI root port, and a root port has only ~384 Mbit/s of
> isochronous budget. An uncompressed YUYV 640x480@30 stream is ~147 Mbit/s of
> data and reserves a top altsetting (~196 Mbit/s), so three of them overrun the
> port: whichever camera opens last dies with
> `ConnectionError: Failed to connect to XenseTactileCamera(GSPS…). Error: Cannot
open camera N`, even though its `/dev/video*` node is present and
> `find_cameras()` lists it. Which sensor loses is a race, so the failure moves
> between runs. The tell is in `dmesg`:
>
> ```
> usb 1-11.2: Not enough bandwidth for new device state.
> usb 1-11.2: Not enough bandwidth for altsetting 6
> ```
>
> This is **not** the ModemManager problem above (that one is the gripper's
> serial port, not a camera) and not a bad cable. The wrist camera defaults to
> `wrist_camera_fourcc="MJPG"` for exactly this reason — left to itself OpenCV's
> V4L2 backend prefers YUYV. If a rig still overruns after that, measure before
> changing anything else: run it and read the negotiated endpoints with
>
> ```bash
> # I:* marks the ACTIVE altsetting; the class string is lowercase "(video)"
> sudo grep -E "^(T:|I:\*.*video|E:.*Isoc)" /sys/kernel/debug/usb/devices
> ```
>
> Each video interface's `Alt=` and its `E:` line's `MxPS=` give the reservation
> (`MxPS × 8000 × 8` bit/s); sum them per **bus** and compare against ~384
> Mbit/s. The next lever is the tactile side — `raw_size` down to `(320, 240)`
> quarters its bandwidth, but the sensor's rectify calibration is made at
> 640x480, so verify the tactile output before recording with it.

> **Whether a bimanual rig fits on one USB 2.0 bus depends on the sensors.** It
> is not a fixed answer, and that is the confusing part. Three customer machines
> with exactly one `480M` root*hub could not open all six cameras; the
> development machine, also six cameras on one `480M` root_hub, does. The
> difference is how much each device \_asks* for, which varies between sensor
> batches — so measure this rig rather than assuming either outcome. On the
> machines that failed:
>
> | Cameras open                                                                                 | Reserved              | Result    |
> | -------------------------------------------------------------------------------------------- | --------------------- | --------- |
> | 4 tactile (`--robot.left_enable_wrist_camera=false --robot.right_enable_wrist_camera=false`) | 242 Mbit/s            | works     |
> | 2 wrist (`--robot.enable_tactile=false`)                                                     | fits                  | works     |
> | all 6 (default)                                                                              | 6 × 60.4 = 362 Mbit/s | **fails** |
>
> Those three commands are the bisect: run each half, and "a camera is broken"
> becomes arithmetic. `altsetting 6` is 944 B per microframe, i.e. 60.4 Mbit/s
> per sensor — against ~37 Mbit/s of actual 320x240 YUYV@30 data, so the devices
> over-request by ~64% and the wrist cameras over-request far more. It fails at
> the margin, which is why _which_ camera loses moves between runs and why it can
> look intermittent.
>
> The working development rig carries `GSPS01A28Z…` sensors; the three that
> failed carry `GSPS01A29Z…` and `GSPS01A31Z…`. If a newer batch reserves more
> for the same stream, that is a UVC descriptor question for firmware, not
> something this repo can configure around — worth measuring both and comparing
> the `Alt=` values before accepting "the host is too small".
>
> Things that do **not** help, so that nobody spends a day on them again:
>
> - **A different physical port.** The cameras are USB 2.0; every port on one
>   controller shares one bus.
> - **`--robot.tactile_fps`.** It throttles the Python read loop only. The Xense
>   SDK's `Sensor.create` takes no fps argument, so the USB stream is unchanged.
> - **`uvcvideo quirks=128`** (`UVC_QUIRK_FIX_BANDWIDTH`). Tried on a failing rig;
>   no change. It recomputes the reservation from `width × height × bpp`, and a
>   compressed format has no meaningful bpp — so it has nothing to work with for
>   the MJPEG wrist cameras, which are the biggest over-requesters.
>
> The fix is a second USB host controller — see
> [Step 2](#step-2--check-the-usb-bandwidth-budget). Until one is fitted, record
> with the wrist cameras off: tactile, pose and jaw are all still there, but the
> dataset then has no `{side}_wrist` key, so decide before recording rather than
> mixing two incompatible observation schemas.

## Calibration workflow (do once per device)

### 0. What state is a unit already in?

Both questions below — what firmware is on it, and is it calibrated — are
answered by one command. The name is about fisheye, but `show` prints the
firmware version and **both** stored calibrations:

```bash
python third_party/taccap-gripper/python/examples/fisheye_cal.py show --sn TCGU01A28Z0023m
```

```
Firmware 1.2.2  (fisheye needs cmd set >= V2.0, encoder-max >= V2.1: leader >= 1.2.0 / follower >= 1.1.0)

Fisheye camera calibration (Cmd 0x2B)
  not calibrated — firmware returned CalNotSet

Encoder max travel angle (Cmd 0x2C, leader only)
  max_rad = 1.1582 rad (66.4°)
```

`max_rad` present means step 1 is done and `gripper.pos` is normalised against
this unit's real travel; `CalNotSet` there means it still falls back to
`gripper_open_rad`.

If you only want the version, ask the MCU directly — `scan_grippers()` cannot
tell you, since `GripperEndpoints` carries `firmware_sn` / `side` / `role` but
no version, and the C++ `fw_version_str` is not exposed through pybind:

```bash
python -c "
from xense.taccap import scan_grippers, LeaderGripper, Cmd
for ep in scan_grippers():
    g = LeaderGripper(mcu_device=ep.mcu_device)
    ack = g.transport.send_cmd(Cmd.GetVersion, b'', 500)
    print(f'{ep.firmware_sn}  {ep.side.name:5}  fw={ack.data[0]}.{ack.data[1]}.{ack.data[2]}')
"
```

Opening by `mcu_device` leaves the cameras off and `normalize_position` at its
default `False`, so this works on an uncalibrated unit — the normalising
constructor would throw instead.

`Cmd::GetVersion` returns the constant compiled into the running image, not OTA
bank metadata, so it is proof of what actually landed. Two things it is not:
`xense.taccap.__version__` is the **SDK** version (`0.1.9`), unrelated to
firmware; and `ack.data[3]`, the fourth "build" byte, is pinned to 0 and
meaningless — versions are `MAJOR.MINOR.PATCH` everywhere, so do not write the
trailing zero into a version comparison.

### 1. Encoder zero + travel span

The SDK ships the calibration CLI. Select the gripper by side — it resolves the
firmware SN itself and prints it, so with both sides plugged in you cannot zero
the wrong one — and it does **both** ends in one pass:

```bash
python third_party/taccap-gripper/python/examples/calibrate.py left
python third_party/taccap-gripper/python/examples/calibrate.py right
```

Side is read from the firmware-burned SN over the wire (`Cmd::GetSn`), the same
rule the robot uses, so `left` here is the same gripper as `left_gripper.pos`.
An explicit SN still works if you want to pin it:
`calibrate.py TCGU01A28Z0024m`.

1. Hold the gripper **fully closed** → latched as the encoder zero
   (`Encoder.set_zero()`), then re-read to confirm the post-zero residual.
2. Open to the **mechanical limit** → written to MCU flash as that unit's
   encoder-max (`Cmd::EncoderMaxCal`, firmware ≥ V2.1).

List available firmware SNs:

```bash
python -c "from xense.taccap import scan_grippers, Side; \
  [print(f'{\"L\" if g.side==Side.Left else \"R\"} fw={g.firmware_sn} mcu={g.mcu_serial}') for g in scan_grippers()]"
```

**Do this on every unit.** Step 2 is what `gripper.pos` is normalised against:
with it, the SDK reports the true opening of _that_ gripper in [0, 1]; without
it the robot falls back to dividing by `gripper_open_rad`, a single config
constant (default 1.7) standing in for every gripper ever built. Real travel
varies — a measured unit came out at **1.1486 rad (65.8°)**, which under the
fallback tops out at `1.1486 / 1.7 = 0.676` and never reaches 1.0.

In a bimanual rig, calibrating only one side is the worst case: the two
`{side}_gripper.pos` channels end up on different scales, so the same physical
grip reads differently left and right. The connect log says which path each side
took — `Jaw normalised by the firmware's encoder-max calibration` versus
`Firmware encoder-max calibration unavailable …`.

Closed is always 0 — there is no `gripper_closed_rad` config, the zero lives in
firmware. `gripper_open_rad` is now only the fallback for uncalibrated or
pre-V2.1 units, and for followers (`Cmd::EncoderMaxCal` is leader-only).

#### If the gripper reports pre-V2.1 firmware

`calibrate.py` exits without changing anything when the unit's command set is
older than V2.1 (i.e. leader < 1.2.0) — the encoder-max command does not exist
there. Since 0.1.7 the SDK ships the released images, so flashing no longer
needs the firmware source:

```bash
# Which role is this unit?  The LAST character of the firmware SN decides —
# 'm' = the leader image, 's' = the follower one.  NOT which hand it is on:
# two grippers on opposite sides of a rig are routinely both leaders.
# (The image FILES keep the vendor's master/slave names — see below.)
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.firmware_sn, '->', 'leader' if g.firmware_sn.endswith('m') else 'follower')"

python third_party/taccap-gripper/python/examples/ota_update.py \
    tc-gu-01-master.bin --side left --target-version 1.2.2
```

Naming the image is enough — `ota_update.py` finds it in the SDK's own
`firmware/`, so the command is the same from here and from inside the
submodule.

Two things that bite here:

- **Update the SDK before the firmware, not after.** Pre-0.1.7 `OtaSession`
  read a firmware-side error on the echoed-command path as a 1-byte success,
  so a failed update reported success. The current SDK talks to old firmware
  unchanged, so this ordering is always safe.
- **The wrong role bricks the MCU** into needing an SWD probe. `ota_update.py`
  identifies the image by CRC32 against `firmware/manifest.json` and refuses a
  mismatch outright rather than warning — `--force` overrides.

Re-run `calibrate.py` once the unit comes back up (~1–3 s for USB
re-enumeration).

### 2. Sanity-check the Pico4 tracker

```bash
python -m lerobot.robots.taccap_gripper.check_tracker
# or, pin to a specific tracker SN:
python -m lerobot.robots.taccap_gripper.check_tracker PC2310MLL3200496G
```

Watch the `raw xyz` move smoothly when you wave the gripper. The
`ee xyz` is `raw` after the rigid `tracker_to_ee_*` mount transform —
identity by default. Measure your physical mount offset and put it in
the config (`tracker_to_ee_pos`, `tracker_to_ee_quat`).

## Live visualization (`lerobot-teleoperate`)

Before recording anything, stream the device to Rerun to confirm both tactile pads, the
wrist camera and the pose are alive. The device is self-driven, so **no `--teleop.*`
flags** — `lerobot-teleoperate` just pumps `get_observation()` into the viewer:

```bash
lerobot-teleoperate \
    --robot.type=taccap_gripper \
    --fps=30 \
    --display_data=true
```

Add `--robot.side=left|right` only when both grippers are connected; a single unit
auto-resolves. Add `--robot.enable_tracker=false` to skip the Pico4 / XenseVR PC service
entirely and watch tactile + gripper only.

### Viewer layout

`--display_data=true` sends a blueprint rather than letting Rerun auto-lay-out, which would
give each tactile pad the same screen area as everything else. For a single rig (no head
camera) that resolves to:

- **Left, largest**: the `/world` 3D trajectory view.
- **Right**: the wrist camera, then the two tactile pads in their own grid.
- **Bottom**: scalars in tabs by unit — `gripper.pos`, `tcp pose`, `imu` (when
  `--robot.enable_imu=true`), plus a catch-all `all` tab. Metres, unit-length rotation
  components and accelerations share no y-axis, so one merged plot would be unreadable.

With `--robot.enable_tracker=false` there is no 3D view, and the wrist + tactile views take
the whole top half.

The tactile pads you see are the recorded `rectify` stream (`tactile_{left,right}`) —
`tactile_display_output_types` defaults to the recorded type, so screen and dataset show
the same image. (Before the 2026-08 silicone change the pads were the amplified
`difference` view, because contact barely showed on the old gel.) The viewer is laid out from the robot's `display_features` rather than
`observation_features`, which is the same schema **unless** a display-only tactile view is
configured (`--robot.tactile_display_output_types='["difference"]'`); then each sensor's
recorded stream is swapped in place for its display one, `log_rerun_data` is fed the
matching subset of the observation (`select_display_observation`), and the recorded stream
never reaches Rerun. The tile count is unchanged either way.

### 3D trajectory

In the `/world` view the gripper is drawn as a labelled ellipsoid + axis triad at its live
Pico4 pose (`tcp.*`), trailing a breadcrumb of where it has been — mirroring the SDK's
[`rerun_dual_with_tracker.py`](../../../third_party/taccap-gripper/python/examples/rerun_dual_with_tracker.py)
example. Our pose is already in the gravity-aligned world frame, so the scene is
`FLU` — X forward, Y left, Z up (the example shows the raw Pico `LEFT_HAND_Y_UP` frame).

On by default; `--show_trajectory=false` drops **that view only** (the rest of the layout
still applies), and it auto-skips when `--robot.enable_tracker=false` (no pose to draw).
Implemented in [`visualization.py`](visualization.py) and shared by both teleoperate and
record — same flags on `lerobot-record`.

## Standalone smoke test

Verifies the robot stack independently of `lerobot-record`. Devices are
auto-discovered; pass `--side` only when both grippers are connected. The station
label the config requires is `--id` here, defaulted to `taccap_0`:

```bash
# Gripper + tactile + wrist, all auto-discovered (pick a side if both present):
python -m lerobot.robots.taccap_gripper.taccap_gripper_example --side left

# Cameras + gripper only (no wrist camera):
python -m lerobot.robots.taccap_gripper.taccap_gripper_example --side left --no-wrist

# + Pico4 tracker (pose). --tracker-sn is optional — omit it to auto-discover
# by the second-to-last-digit side rule:
python -m lerobot.robots.taccap_gripper.taccap_gripper_example \
    --side left --tracker --tracker-sn PC2310MLL3200496G

# + IMU channels, and a different jaw-open calibration:
python -m lerobot.robots.taccap_gripper.taccap_gripper_example \
    --side left --imu --open-rad 1.7 --frames 30
```

## End-to-end recording

`taccap_gripper` runs without a teleoperator (`RecordConfig.__post_init__`
allows `teleop=None` for it). Recording is handled by the dedicated
`self_driven_record_loop` in `lerobot_record.py` (the device is routed there
via `SELF_DRIVEN_RECORD_ROBOTS`). Each recorded row uses **shifted-frame**
pairing: the observation from step _t-1_ is paired with the pose at step _t_
(Pico4 pose + normalised `gripper.pos`) as the action, so the action leads
its observation by one step — a real "move-to-next" target rather than the
degenerate same-frame pose. One frame is dropped per episode (the first
sample has no predecessor). The between-episode reset phase is a passive
wait: reposition the device, no teleop needed. **No `--teleop.*` flags.**

Devices are **auto-discovered by serial rule** — no gripper/tactile/camera serials
are listed. With a single gripper connected it is picked up automatically; when both
are connected, set `--robot.side=left|right`:

```bash
lerobot-record \
    --robot.type=taccap_gripper \
    --robot.id=0 \
    --robot.side=right \
    --dataset.repo_id=<your_org>/<your_dataset> \
    --dataset.num_episodes=1 \
    --dataset.episode_time_s=10 \
    --dataset.single_task='Pick up the object'
```

With the Pico4 tracker powered on, 6-DoF pose is recorded automatically — the tracker
is auto-discovered and matched to this unit's side by its serial's second-to-last digit
(odd → left, even → right). Add `--robot.enable_tracker=false` to record tactile +
gripper only.

To bypass the side rule — e.g. a tracker whose serial does not follow it, or when PC-service
enumeration is flaky — pin the serial directly with `--robot.tracker_serial=<SN>`. A pinned
serial is used **verbatim**: no enumeration, no rule check (a typo surfaces as a device-not-found
at connect). Leave it unset (default) to keep auto-discovery.

- **Tactile** → obs keys `tactile_left` / `tactile_right`; landscape `(400,700,3)` uint8
  (width/height auto-derive — don't hard-code). Each sensor has **two destinations**,
  which by default read the same view:
  - **recorded** — `--robot.tactile_output_types`, default `rectify` (exactly one type).
    The unsubtracted image; this is the only tactile key in `observation_features`, so
    it is the only one that lands in the dataset.
  - **displayed** — `--robot.tactile_display_output_types`, default `rectify` as well,
    i.e. Rerun shows the recorded stream and the sensor is read once. (`'[]'` means the
    same thing.)

  Set the display type to something else and it becomes a second output on the same
  read, published as `tactile_{left,right}_<type>` and fed to Rerun _instead of_ the
  recorded stream (`display_features`), so the tile count never changes. The one worth
  knowing is `'["difference"]'` (SDK `OutputType.AugDifference`), which amplifies
  deformation against the rest baseline. That **used to be the default**: on the gel
  the rig shipped with, raw `rectify` showed too little deformation to read contact
  from live. The silicone was changed in 2026-08 and `rectify` now shows contact
  directly, so the viewer no longer trades away showing what is actually recorded.
  `difference` is deliberately kept out of
  `observation_features` and never recorded: its baseline is taken at sensor init, so a
  finger loaded at connect would have that pressure subtracted out of the whole run —
  keep the fingers **unloaded** at connect for a readable live view.
  `--robot.tactile_diff_gain` (default `1.0`, sensors ship at `1.5`) is the linear gain on
  that difference — inert unless you ask for it: 1.0 cuts per-pixel temporal noise from
  ~1.77 to ~1.18 grey levels and stops the image clipping, but scales signal down with
  it — raise it if light contact becomes invisible, set it to `None` to keep whatever
  the sensor was flashed with.
  Tune `--robot.tactile_fps`; `--robot.expected_tactiles_per_side` validates the count.
  `--robot.enable_tactile=false` takes the sensors out of the run altogether — a way to
  bisect a USB bandwidth problem (see the troubleshooting section), not a way to record.
  The two sensors are paired to this unit's gripper by **USB hub**; `left`/`right` finger
  comes from the GSPS serial's **last digit** (odd → `left`, even → `right`, 单左双右).

- **Wrist** → obs key `wrist_cam`; `--robot.enable_wrist_camera=false` skips. Tune
  `--robot.wrist_camera_width/_height/_fps`. `--robot.wrist_undistort=true` (off by
  default) rectifies the fisheye **before the frame is recorded** — see below.

### Wrist fisheye undistortion (`--robot.wrist_undistort`, off by default)

The wrist lens is a 190° fisheye and what gets recorded is, by default, the raw
frame. Turning this on rectifies it with the intrinsics stored in that gripper's
own flash (`Cmd 0x2B`, command set V2.0+), read through the SDK's
`Calibration::resolve_fisheye()`.

**It changes what lands in the dataset, and the change is invisible.** A rectified
`wrist_cam` and a raw one have the same shape and dtype, so datasets recorded with
and without it are not interchangeable and nothing downstream can tell them apart.
That is why `meta/hardware.json` records, per unit, whether it was applied and
which intrinsics were used — and why flipping the flag part-way through a dataset
opens a new epoch.

Three things worth knowing before turning it on:

- **An uncalibrated unit does not fail.** The SDK's shared reference intrinsics
  stand in and `connect()` warns: every unit carries the same lens on the same
  sensor, so reference numbers beat raw fisheye. They are **approximate** — lens
  placement varies per assembly, so the principal point drifts. Anything that
  measures in pixels off these frames wants this unit's own calibration:
  `python third_party/taccap-gripper/python/examples/fisheye_cal.py set-fisheye`.
  The manifest records `"calibration": "reference"` when this happened, because a
  warning at connect scrolls past and the dataset is what remains.
- **640x480 only.** The firmware record holds the 8 intrinsic/distortion floats
  and no image size, so any other resolution would mean guessing a scale factor
  and rectifying wrongly with nothing in the frames to show it. Combining
  `--robot.wrist_undistort=true` with another `--robot.wrist_camera_width/_height`
  fails at **CLI-parse time**, before any device is touched.
- **`--robot.wrist_undistort_balance`** (0..1, default 0) trades field of view
  against black border: 0 keeps the calibrated focal length (also the PC
  calibration tool's default), 1 shortens it to 0.70x for the widest view. Only
  fx/fy move, so the view does not drift as the knob turns.

Note this is _not_ the SDK's own `Config::undistort_wrist`. That one only applies
when the SDK owns the wrist UVC device, which it never does here — the cameras
come from the LeRobot camera framework. `resolve_fisheye()` exists precisely for
callers in that position, so both paths make the same read-and-fall-back
decision instead of drifting apart.

- **Head** → obs keys `left_head` / `right_head`, off by default;
  `--robot.enable_head_camera=true` streams the Pico headset's stereo camera as **one key
  per eye**. `--robot.head_camera_width/_height` accept `640x480` (default), `1024x768` or
  `1280x960`, and must match the headset app's Resolution setting;
  `--robot.head_camera_eyes=left` (or `right`) records a single eye. It shares the
  XenseVR SDK connection with the tracker, so the headset app must be streaming. There is
  **one headset**, so this is the same view the bimanual robot records — running two
  single-arm processes does not give two independent head views. Details:
  [`bi_taccap_gripper`](../bi_taccap_gripper/README.md).
- **Role**: `--robot.role=follower` binds the follower units (default `leader`).

### Streaming video encoding & encoder warmup

Video keys (tactile + wrist) are encoded **in real time during capture** rather than
written as PNGs and encoded at episode end, so `save_episode()` is near-instant. This is
on by default (`--dataset.streaming_encoding=true`); pair it with
`--dataset.encoder_threads=2` and optionally `--dataset.vcodec=auto` for hardware
encoding. One `_CameraEncoderThread` runs per camera, fed raw frames through a bounded
queue (`--dataset.encoder_queue_maxsize`, ~1 s of frames); if the encoder can't keep up
the oldest frames are dropped with a warning rather than stalling the loop.

**Encoder warmup.** Opening the PyAV container + codec context is expensive (~25 ms), and
doing it lazily on the episode's first frame made that frame badly overrun the `fps`
budget. To avoid this, `dataset.prepare_episode_recording()` is called once before each
episode's record loop: it starts the encoder threads and opens their codec contexts up
front (using each video key's declared `(H, W, C)` shape), blocking until every encoder
reports ready. By the time the first frame is recorded the encoders are hot, so the first
`add_frame()` no longer pays the init cost. The lazy first-frame start remains as a
defensive fallback for callers that don't pre-warm.

### `--robot.id` (required) and the hardware manifest

Two different things, and they answer two different questions.

**`--robot.id` is the station label**, one per rig (a bimanual rig is one rig,
one id). It names the _seat_, not the hardware in it, so it stays put when a
gripper is swapped. It reaches the log prefix, the calibration filename,
`str(robot)` and the manifest below, but **not a dataset column** —
`LeRobotDataset.create()` is handed `robot_type` and nothing else. It is still
binding: a dataset records the station it was recorded in and refuses to be
resumed from another one (below).

**Pass a number.** `--robot.id=0` is stored as `taccap_0` here and as
`bi_taccap_0` on a bimanual rig: a bare number is expanded against
`--robot.type`, minus its `_gripper` suffix, because the label names a station
and a station is not a gripper. Typing the prefix by hand only repeated what
`--robot.type` already said, and got it wrong often enough to matter —
`--robot.type=bi_taccap_gripper --robot.id=taccap_0` parses fine and then
labels a bimanual rig as a single one.

Anything that is not all digits is taken verbatim, so an existing
`--robot.id=taccap_0` keeps working — along with the calibration file named
after it — and a rig can still be named after a room.

**It is required**, unlike upstream's optional `RobotConfig.id`. Both TacCap
configs put it through `validate_robot_id()` in `__post_init__`, so a missing or
blank id fails at CLI-parse time — before any device is touched — instead of a
rig spinning up and recording anonymously. That `None` default is also why
terminal output used to read `None TaccapGripper`. Identity itself still lives
in the serials below, not in this string. The smoke test takes `--id` and
defaults it to `taccap_0`.

**The hardware manifest is the identity.** `lerobot-record` writes
`meta/hardware.json` into the dataset right after `robot.connect()`:

```json
{
  "robot_type": "bi_taccap_gripper",
  "robot_id": "bi_taccap_0",
  "epochs": [
    {
      "from_episode": 0,
      "to_episode": null,
      "recorded_at": "2026-08-22T16:03:09+08:00",
      "robot_id": "bi_taccap_0",
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
          ],
          "wrist_undistort": { "applied": false }
        }
      ]
    }
  ]
}
```

`epochs` is a list because a rig can be swapped part-way through a dataset: the
open epoch is closed at the current episode count and a new one starts, so every
episode keeps pointing at the devices that produced it. `to_episode` is exclusive
and `null` on the epoch still being recorded.

**`robot_id` is not one of those swaps — it is a wall.** One dataset belongs to
one station, and `lerobot-record --resume` on a rig whose `--robot.id` disagrees
with the manifest fails:

```text
ValueError: This dataset was recorded on station taccap_0 but this run is
--robot.id=taccap_1; refusing to resume it (…/meta/hardware.json). One dataset is
one station: a task recorded across two rigs mixes their calibration, timing and
mounting into episodes nothing downstream can separate. Resume on the original
station, or record this run into a dataset of its own (--dataset.repo_id=…).
```

Identical `units` on both sides do not make it one rig — the label names the
_seat_, and the seat is where the mounting, the lighting and the operator are.
The check runs twice: once in `lerobot-record` right after the resumed dataset is
loaded, **before `robot.connect()`**, so a wrong rig is turned away before every
gripper, sensor and tracker spins up; and once inside `write_hardware_manifest`,
which is the choke point every writer goes through. Because the label is now a
property of the dataset rather than of an epoch, it is also written at the top
level beside `robot_type`, and repeated on each epoch so readers of older files
keep working (`manifest_robot_ids` reads both).

A dataset recorded before `--robot.id` was required carries no label, and that is
not a mismatch: the file cannot say the station changed, and refusing would
strand real datasets. Same reading as an open epoch — _"nothing here says it
changed"_ is not _"it didn't"_.

- `side` is which gripper; `finger` is which sensor on it. Both are called
  left/right and they are **independent** — 单左双右 is applied once to the
  gripper's own serial and again to each tactile's. Each sensor therefore also
  carries the `observation_key` it feeds, so a dataset column traces back to a
  physical sensor without re-deriving the naming rule.
- `gripper_sn` is the **firmware** SN (`Cmd::GetSn`, read over the wire at
  connect), never the CH343 `mcu_serial` — that one identifies the USB-serial
  adapter and changes when the adapter does.
- The single-arm robot writes the same shape with one entry in `units`, so
  anything reading these datasets needs one code path, not two.
- A side whose gripper is off records `"gripper_sn": null` rather than being
  omitted; `enable_tactile=false` gives it an empty `tactile_sensors`.
- It is a file of its own, **not** a key in `meta/info.json`: that schema is
  upstream's and a fork-local key in it would collide on the next v5.x sync.
- Tactile runtime bundles are no longer written (older datasets may still show a
  `runtime` key per sensor; it is ignored). The derived channels are rebuilt from
  the stream's own first `rectify` frame plus the serial.
- `wrist_undistort` says whether that unit's wrist frames were rectified and from
  whose intrinsics (`"unit"` or `"reference"`). Present whenever the unit has a
  wrist camera, including as `{"applied": false}`.
- Resuming a dataset on _different_ hardware **closes the open epoch and appends a
  new one**. It used to keep the original file and warn, but that warning only
  reached the log: nothing on disk said the rig had changed, while the manifest
  went on attributing post-swap episodes to the old devices. Only a `robot_type`
  mismatch is still keep-and-warn — single vs bimanual is a different dataset,
  not a swap.

Trackers are deliberately not in the manifest, and neither is the wrist camera's
_identity_: they are mounted accessories, while the gripper + its two tactiles are
the unit whose serials the data is about. `wrist_undistort` is not an exception to
that — it records how the recorded frames were **processed**, which is invisible in
the frames themselves. `hardware_manifest_unit()` in `common.py` is where all of
this is decided.

## What gets recorded per frame

Keys are **unprefixed** — unlike [`bi_taccap_gripper`](../bi_taccap_gripper/README.md),
which prefixes everything `left_` / `right_`. A single-arm dataset is therefore not a
column subset of a bimanual one.

| Key                             | When                  | Source                                                 | Shape / type                                         |
| ------------------------------- | --------------------- | ------------------------------------------------------ | ---------------------------------------------------- |
| `tcp.x`, `tcp.y`, `tcp.z`       | `enable_tracker`      | Pico4 tracker → EE                                     | float (m)                                            |
| `tcp.r1`..`tcp.r6`              | `enable_tracker`      | 6-D rotation of EE                                     | float                                                |
| `gripper.pos`                   | `enable_gripper`      | TacCap encoder, normalised                             | float ∈ [0, 1]                                       |
| `imu.accel.{x,y,z}`             | `enable_imu`          | TacCap IMU                                             | float (m/s²)                                         |
| `imu.gyro.{x,y,z}`              | `enable_imu`          | TacCap IMU                                             | float (rad/s)                                        |
| `imu.mag.{x,y,z}`               | `enable_imu`          | TacCap IMU                                             | float (µT)                                           |
| `tactile_left`, `tactile_right` | auto-discovered       | Xense sensor on that finger, recorded view (`rectify`) | uint8 (H, W, 3), landscape — currently (400, 700, 3) |
| `wrist_cam`                     | `enable_wrist_camera` | wrist UVC via `cameras/`                               | uint8 (H, W, 3)                                      |
| `left_head`, `right_head`       | `enable_head_camera`  | Pico headset camera, one key per eye                   | uint8 (H, W, 3) — default (480, 640, 3)              |
| `head_camera.x/y/z/r1..r6`      | `enable_head_camera`  | headset pose, same world frame as `tcp.*`              | float                                                |

The flags are all `--robot.*` (e.g. `--robot.enable_imu=true`); the tracker and gripper
ones are on by default, `enable_imu` is off. Disabling one **removes** its keys from the
schema rather than zero-filling them, so `observation.state` is 10-D by default
(9 pose + 1 jaw), 19-D with the IMU on, and a further 9 wider with the head camera on.

Tactile cameras contribute their **recorded** view only (`rectify` by default). Any
display-only key `get_observation()` also returns — `tactile_{left,right}_difference`
when `--robot.tactile_display_output_types` asks for it — is absent from
`observation_features`, so `build_dataset_frame` never sees it and nothing is written
for it.

`action_features` is the `tcp.*` + `gripper.pos` subset — images are observation-only.

The 6-D rotation convention matches `vive_tracker`:
`r1..r3` is the first column of the rotation matrix, `r4..r6` is the
second column.

## Files in this package

- `taccap_gripper.py` — the `Robot` subclass. `get_observation()` and
  `get_action()` both surface pose + gripper + optional IMU + cameras.
- `config_taccap_gripper.py` — `RobotConfig` dataclass.
- `common.py` — everything the single and bimanual robots share, including the
  hardware-manifest helpers (`hardware_manifest_unit` / `build_hardware_manifest`
  / `write_hardware_manifest`).
- `taccap_gripper_example.py` — standalone smoke test (above).
- `check_tracker.py` — sanity-check the Pico4 tracker (read-only; it calibrates nothing).

Encoder zero calibration lives in the SDK itself
(`third_party/taccap-gripper/python/examples/calibrate.py`) — we no
longer ship a duplicate.

The Pico4 tracker reader is shared with future devices and lives at
`src/lerobot/teleoperators/pico4/tracker.py`.

The integration point in the record script is
`src/lerobot/scripts/lerobot_record.py`:

- `RecordConfig.__post_init__` allows `teleop=None` for the self-driven
  robots (`SELF_DRIVEN_RECORD_ROBOTS`).
- The dispatch in `record()` routes `taccap_gripper` / `bi_taccap_gripper`
  to the dedicated `self_driven_record_loop` (shifted-frame pairing).
