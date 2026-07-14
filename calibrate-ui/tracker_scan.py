#!/usr/bin/env python
"""Pico tracker serial discovery for the standalone calibration UI.

This mirrors the production UI strategy: query the PXREA C SDK first because it
can report Motion.joints[].sn in environments where the Python xrt wrapper does
not surface serials yet, then fall back to xensevr_pc_service_sdk in a child
process.
"""

from __future__ import annotations

import contextlib
import ctypes
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

TRACKER_MARKER = "__CALIBRATE_UI_TRACKER_JSON__"
PXREA_FULL_MASK = 0xFFFFFFFF
PXREA_DEVICE_FIND = 1 << 4
PXREA_DEVICE_STATE_JSON = 1 << 25


class PXREADevStateJson(ctypes.Structure):
    _fields_ = [
        ("devID", ctypes.c_char * 32),
        ("stateJson", ctypes.c_char * 16352),
    ]


def error_payload(exc: BaseException) -> dict[str, str]:
    return {
        "type": exc.__class__.__name__,
        "message": str(exc),
        "traceback": traceback.format_exc(limit=4),
    }


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
    candidates = [
        REPO_ROOT / "third_party/XenseVR-PC-Service/RoboticsService/SDK/linux/64/libPXREARobotSDK.so",
        REPO_ROOT / "third_party/XenseVR-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so",
        REPO_ROOT / "src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/lib/libPXREARobotSDK.so",
    ]
    env_path = os.environ.get("PXREA_ROBOT_SDK_LIB")
    if env_path:
        candidates.insert(0, Path(env_path).expanduser())
    return [path for path in candidates if path.exists()]


def decode_c_string(value: bytes | ctypes.Array[Any]) -> str:
    raw = value if isinstance(value, bytes) else bytes(value)
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

    # Isolate the C SDK lifecycle in a child process. This avoids contaminating
    # the Qt process if PXREAInit/PXREADeinit leaves process-global state behind.
    if os.environ.get("CALIBRATE_UI_PXREA_CHILD") != "1":
        code = (
            "import json, os, sys, traceback\n"
            f"sys.path.insert(0, {str(Path(__file__).resolve().parent)!r})\n"
            "from tracker_scan import TRACKER_MARKER, scan_trackers_pxrea_sdk\n"
            "os.environ['CALIBRATE_UI_PXREA_CHILD'] = '1'\n"
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


def scan_trackers_xrt_subprocess(timeout_s: float) -> tuple[list[str], dict[str, str] | None]:
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

    fallback_serials, fallback_error = scan_trackers_xrt_subprocess(timeout_s)
    if fallback_serials:
        return fallback_serials, None

    if fallback_error and pxrea_error:
        fallback_error = dict(fallback_error)
        fallback_error["message"] = (
            f"{fallback_error.get('message', '')} | PXREA direct scan: {pxrea_error.get('message', '')}"
        )
        return fallback_serials, fallback_error
    return fallback_serials, fallback_error or pxrea_error
