#!/usr/bin/env python
"""Production UI helper for TacCap serial discovery.

This script intentionally lives under production_ui and only calls the existing
LeRobot/Xense discovery helpers. It prints one JSON object to stdout so the UI
can refresh device SNs without blocking the main window.
"""

from __future__ import annotations

import argparse
import contextlib
import ctypes
import importlib.util
import json
import os
import subprocess
import sys
import time
import traceback
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if SRC_ROOT.exists() and str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

SIDES = ("left", "right")
FINGERS = ("left", "right")
SCAN_MARKER = "__PRODUCTION_UI_SCAN_JSON__"
TRACKER_MARKER = "__PRODUCTION_UI_TRACKER_JSON__"
PXREA_FULL_MASK = 0xFFFFFFFF
PXREA_DEVICE_FIND = 1 << 4
PXREA_DEVICE_STATE_JSON = 1 << 25


class PXREADevStateJson(ctypes.Structure):
    _fields_ = [
        ("devID", ctypes.c_char * 32),
        ("stateJson", ctypes.c_char * 16352),
    ]


def endpoint_info(endpoint: Any) -> dict[str, str]:
    return {
        "firmware_sn": str(getattr(endpoint, "firmware_sn", "") or ""),
        "mcu_serial": str(getattr(endpoint, "mcu_serial", "") or ""),
        "mcu_device": str(getattr(endpoint, "mcu_device", "") or ""),
        "side": str(getattr(getattr(endpoint, "side", None), "name", "") or ""),
        "role": str(getattr(getattr(endpoint, "role", None), "name", "") or ""),
    }


def tactile_cell(serial: str = "", state: str = "missing", message: str = "") -> dict[str, str]:
    return {
        "serial": str(serial or ""),
        "state": str(state or "missing"),
        "message": str(message or ""),
    }


def error_payload(exc: BaseException) -> dict[str, str]:
    return {
        "type": exc.__class__.__name__,
        "message": str(exc),
        "traceback": traceback.format_exc(limit=4),
    }


def load_serial_discovery() -> Any:
    module_path = SRC_ROOT / "lerobot/robots/taccap_gripper/serial_discovery.py"
    if not module_path.exists():
        raise FileNotFoundError(f"serial_discovery.py not found: {module_path}")
    spec = importlib.util.spec_from_file_location("production_ui_taccap_serial_discovery", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load serial_discovery.py from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_marked_json(text: str, marker: str) -> dict[str, Any]:
    index = text.rfind(marker)
    if index < 0:
        raise ValueError(f"marker {marker!r} not found")
    payload = text[index + len(marker) :].strip()
    obj, _ = json.JSONDecoder().raw_decode(payload)
    return obj


def unique_nonempty(items: list[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for item in items:
        value = str(item or "").strip()
        if value and value not in seen:
            seen.add(value)
            result.append(value)
    return result


def pxrea_sdk_library_candidates() -> list[Path]:
    repo_root = Path(__file__).resolve().parents[1]
    candidates = [
        repo_root / "third_party/XenseVR-PC-Service/RoboticsService/SDK/linux/64/libPXREARobotSDK.so",
        repo_root / "third_party/XenseVR-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so",
        repo_root / "src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/lib/libPXREARobotSDK.so",
    ]
    env_path = os.environ.get("PXREA_ROBOT_SDK_LIB")
    if env_path:
        candidates.insert(0, Path(env_path).expanduser())
    return [path for path in candidates if path.exists()]


def decode_c_string(value: bytes | ctypes.Array[Any]) -> str:
    if isinstance(value, bytes):
        raw = value
    else:
        raw = bytes(value)
    return raw.split(b"\0", 1)[0].decode(errors="replace").strip()


def parse_motion_tracker_serials(state_json: str) -> list[str]:
    try:
        outer = json.loads(state_json)
    except Exception:
        return []
    if outer.get("functionName") != "Tracking":
        return []

    value = outer.get("value")
    if isinstance(value, str):
        try:
            value = json.loads(value.replace("\\", ""))
        except Exception:
            try:
                value = json.loads(value)
            except Exception:
                return []
    if not isinstance(value, dict):
        return []

    motion = value.get("Motion")
    if not isinstance(motion, dict):
        return []
    joints = motion.get("joints")
    if not isinstance(joints, list):
        return []
    return unique_nonempty([str(joint.get("sn", "")) for joint in joints if isinstance(joint, dict)])


def scan_trackers_pxrea_sdk(timeout_s: float) -> tuple[list[str], dict[str, str] | None]:
    if timeout_s <= 0:
        return [], None

    if os.environ.get("PRODUCTION_UI_PXREA_CHILD") != "1":
        code = (
            "import json, os, sys, traceback\n"
            f"sys.path.insert(0, {str(REPO_ROOT)!r})\n"
            "from production_ui.device_scan import TRACKER_MARKER, scan_trackers_pxrea_sdk\n"
            "os.environ['PRODUCTION_UI_PXREA_CHILD'] = '1'\n"
            "try:\n"
            "    serials, error = scan_trackers_pxrea_sdk(float(sys.argv[1]))\n"
            "    print(TRACKER_MARKER + json.dumps({'serials': serials, 'error': error}, ensure_ascii=False), flush=True)\n"
            "except Exception as exc:\n"
            "    print(TRACKER_MARKER + json.dumps({'serials': [], 'error': {'type': exc.__class__.__name__, 'message': str(exc), 'traceback': traceback.format_exc(limit=4)}}, ensure_ascii=False), flush=True)\n"
        )
        try:
            completed = subprocess.run(
                [sys.executable, "-c", code, str(timeout_s)],
                check=False,
                capture_output=True,
                text=True,
                timeout=max(timeout_s + 1.0, 2.0),
            )
        except Exception as exc:
            return [], error_payload(exc)

        combined = f"{completed.stdout}\n{completed.stderr}"
        try:
            payload = parse_marked_json(combined, TRACKER_MARKER)
        except Exception as exc:
            return [], {
                "type": exc.__class__.__name__,
                "message": f"{exc}; PXREA child return code={completed.returncode}",
                "traceback": combined[-2000:],
            }
        serials = [str(item) for item in payload.get("serials", [])]
        error = payload.get("error")
        if isinstance(error, dict):
            return serials, error
        if completed.returncode not in (0, None) and not serials:
            return [], {
                "type": "PXREASDKProcessError",
                "message": f"PXREA child return code={completed.returncode}",
                "traceback": combined[-2000:],
            }
        return serials, None

    candidates = pxrea_sdk_library_candidates()
    if not candidates:
        return [], {
            "type": "PXREASDKLibraryNotFound",
            "message": "libPXREARobotSDK.so not found in bundled XenseVR PC-Service paths.",
            "traceback": "",
        }

    callback_refs: list[Any] = []
    errors: list[str] = []
    for lib_path in candidates:
        found_devices: list[str] = []
        motion_serials: list[str] = []
        try:
            sdk = ctypes.CDLL(str(lib_path))
            callback_type = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p)

            def on_pxrea_callback(_context: Any, event_type: int, _status: int, user_data: Any) -> None:
                try:
                    if event_type == PXREA_DEVICE_FIND and user_data:
                        found_devices.append(ctypes.cast(user_data, ctypes.c_char_p).value.decode(errors="replace"))
                    elif event_type == PXREA_DEVICE_STATE_JSON and user_data:
                        state = ctypes.cast(user_data, ctypes.POINTER(PXREADevStateJson)).contents
                        motion_serials.extend(parse_motion_tracker_serials(decode_c_string(state.stateJson)))
                except Exception:
                    pass

            callback = callback_type(on_pxrea_callback)
            callback_refs.append(callback)

            sdk.PXREAInit.argtypes = [ctypes.c_void_p, callback_type, ctypes.c_uint]
            sdk.PXREAInit.restype = ctypes.c_int
            sdk.PXREADeinit.argtypes = []
            sdk.PXREADeinit.restype = ctypes.c_int

            rc = sdk.PXREAInit(None, callback, PXREA_FULL_MASK)
            if rc != 0:
                errors.append(f"{lib_path}: PXREAInit returned {rc}")
                continue
            try:
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline:
                    serials = unique_nonempty(motion_serials)
                    if serials:
                        return serials, None
                    time.sleep(0.05)
            finally:
                with contextlib.suppress(Exception):
                    sdk.PXREADeinit()

            serials = unique_nonempty(motion_serials)
            if serials:
                return serials, None
            errors.append(f"{lib_path}: no Motion.joints[].sn reported; device find={unique_nonempty(found_devices)}")
        except Exception as exc:
            errors.append(f"{lib_path}: {exc}")

    return [], {
        "type": "PXREASDKTrackerScanFailed",
        "message": " | ".join(errors[-3:]) if errors else "PXREA SDK tracker scan returned no serials.",
        "traceback": "",
    }


def scan_trackers_subprocess(timeout_s: float) -> tuple[list[str], dict[str, str] | None]:
    if timeout_s <= 0:
        return [], None

    code = f"""
import contextlib
import json
import sys
import time
import traceback

sys.path.insert(0, {str(SRC_ROOT)!r})
marker = {TRACKER_MARKER!r}
try:
    import xensevr_pc_service_sdk as xrt
    with contextlib.redirect_stdout(sys.stderr):
        xrt.init()
        deadline = time.monotonic() + float(sys.argv[1])
        serials = []
        while time.monotonic() < deadline:
            if xrt.num_motion_data_available() > 0:
                raw = xrt.get_motion_tracker_serial_numbers()
                serials = [
                    (item.decode() if isinstance(item, bytes) else str(item)).strip()
                    for item in raw
                ]
                serials = [item for item in serials if item]
                if serials:
                    break
            time.sleep(0.05)
    print(marker + json.dumps({{"serials": serials}}, ensure_ascii=False), flush=True)
except Exception as exc:
    print(marker + json.dumps({{
        "serials": [],
        "error": {{
            "type": exc.__class__.__name__,
            "message": str(exc),
            "traceback": traceback.format_exc(limit=4),
        }},
    }}, ensure_ascii=False), flush=True)
"""
    try:
        completed = subprocess.run(
            [sys.executable, "-c", code, str(timeout_s)],
            check=False,
            capture_output=True,
            text=True,
            timeout=max(timeout_s + 5.0, 6.0),
        )
    except Exception as exc:
        return [], error_payload(exc)

    combined = f"{completed.stdout}\n{completed.stderr}"
    try:
        payload = parse_marked_json(combined, TRACKER_MARKER)
    except Exception as exc:
        return [], {
            "type": exc.__class__.__name__,
            "message": f"{exc}; tracker scan return code={completed.returncode}",
            "traceback": combined[-2000:],
        }

    serials = [str(item) for item in payload.get("serials", [])]
    error = payload.get("error")
    if isinstance(error, dict):
        return serials, error
    if completed.returncode not in (0, None) and not serials:
        return [], {
            "type": "TrackerScanProcessError",
            "message": f"tracker scan return code={completed.returncode}",
            "traceback": combined[-2000:],
        }
    return serials, None


def scan_trackers(timeout_s: float) -> tuple[list[str], dict[str, str] | None]:
    serials, pxrea_error = scan_trackers_pxrea_sdk(timeout_s)
    if serials:
        return serials, None

    fallback_serials, fallback_error = scan_trackers_subprocess(timeout_s)
    if fallback_serials:
        return fallback_serials, None

    if fallback_error and pxrea_error:
        fallback_error = dict(fallback_error)
        fallback_error["message"] = (
            f"{fallback_error.get('message', '')} | PXREA direct scan: {pxrea_error.get('message', '')}"
        )
        return fallback_serials, fallback_error
    return fallback_serials, fallback_error or pxrea_error


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--role", default="leader")
    parser.add_argument("--tracker-timeout", default="2.0")
    args = parser.parse_args()

    result: dict[str, Any] = {
        "role": args.role,
        "sides": {
            side: {
                "gripper": {},
                "tactiles": {finger: "" for finger in FINGERS},
                "tactile_status": {finger: tactile_cell() for finger in FINGERS},
                "wrist_camera": "",
                "tracker": "",
            }
            for side in SIDES
        },
        "raw": {"trackers": []},
        "errors": {},
    }

    try:
        disco = load_serial_discovery()
        result["role"] = disco.normalize_role(args.role)
    except Exception as exc:
        result["errors"]["serial_discovery_import"] = error_payload(exc)
        print(SCAN_MARKER + json.dumps(result, ensure_ascii=False))
        return 0

    with contextlib.redirect_stdout(sys.stderr):
        try:
            grippers = disco.discover_grippers(result["role"])
            for side, endpoint in grippers.items():
                if side in result["sides"]:
                    result["sides"][side]["gripper"] = endpoint_info(endpoint)
        except Exception as exc:
            result["errors"]["grippers"] = error_payload(exc)

        try:
            tactile_diag = disco.diagnose_tactiles_by_hub(result["role"])
            for side in SIDES:
                for finger in FINGERS:
                    cell = tactile_diag.get("sides", {}).get(side, {}).get(finger, {})
                    serial = str(cell.get("serial", "") or "")
                    result["sides"][side]["tactiles"][finger] = serial
                    result["sides"][side]["tactile_status"][finger] = tactile_cell(
                        serial=serial,
                        state=str(cell.get("state", "") or "missing"),
                        message=str(cell.get("message", "") or ""),
                    )
            tactile_warnings = []
            for side in SIDES:
                for finger in FINGERS:
                    cell = result["sides"][side]["tactile_status"][finger]
                    if cell.get("state") != "ok":
                        tactile_warnings.append(f"{side}.{finger}: {cell.get('message', '')}")
            for item in tactile_diag.get("unknown", []):
                tactile_warnings.append(str(item.get("message") or item))
            for message in tactile_diag.get("errors", []):
                tactile_warnings.append(str(message))
            if tactile_warnings:
                result["errors"]["tactiles"] = {
                    "type": "TactileSerialCheckWarning",
                    "message": " | ".join(tactile_warnings),
                    "traceback": "",
                }
        except Exception as exc:
            result["errors"]["tactiles"] = error_payload(exc)

        try:
            cameras = disco.discover_wrist_cameras(result["role"])
            for side, serial in cameras.items():
                if side in result["sides"]:
                    result["sides"][side]["wrist_camera"] = str(serial or "")
        except Exception as exc:
            result["errors"]["wrist_cameras"] = error_payload(exc)

        try:
            serials, tracker_error = scan_trackers(float(args.tracker_timeout))
            result["raw"]["trackers"] = serials
            tracker_warnings = []
            trackers_by_side = {side: [] for side in SIDES}
            for serial in serials:
                try:
                    side = disco.pico_tracker_side(serial)
                except Exception as exc:
                    tracker_warnings.append(str(exc))
                    continue
                if side in trackers_by_side:
                    trackers_by_side[side].append(str(serial))
            for side, side_serials in trackers_by_side.items():
                if side_serials:
                    result["sides"][side]["tracker"] = side_serials[0]
                if len(side_serials) > 1:
                    parity = "odd" if side == "left" else "even"
                    tracker_warnings.append(
                        f"Multiple Pico4 trackers map to {side} "
                        f"(2nd-to-last digit {parity}): {side_serials}."
                    )
            if tracker_error:
                if tracker_warnings:
                    tracker_error["message"] = (
                        f"{tracker_error.get('message', '')} | " + " | ".join(tracker_warnings)
                    )
                result["errors"]["trackers"] = tracker_error
            elif tracker_warnings:
                result["errors"]["trackers"] = {
                    "type": "TrackerSerialCheckWarning",
                    "message": " | ".join(tracker_warnings),
                    "traceback": "",
                }
        except Exception as exc:
            result["errors"]["trackers"] = error_payload(exc)

    print(SCAN_MARKER + json.dumps(result, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
