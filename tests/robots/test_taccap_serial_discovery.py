from __future__ import annotations

from lerobot.robots.taccap_gripper import serial_discovery as disco


def test_diagnose_tactiles_by_hub_marks_duplicate_finger(monkeypatch):
    paths = [
        "/dev/v4l/by-id/usb-Xense_GSPS01A24Z0001-video-index0",
        "/dev/v4l/by-id/usb-Xense_GSPS01A24Z0003-video-index0",
    ]

    monkeypatch.setattr(disco.glob, "glob", lambda pattern: paths)
    monkeypatch.setattr(
        disco,
        "_gripper_hub_sides",
        lambda role: {"0:6": ("left", "TCGU01A24Z0001m")},
    )
    monkeypatch.setattr(disco, "_device_hub", lambda path, bypath_dir: "0:6")

    result = disco.diagnose_tactiles_by_hub("leader")

    left_finger = result["sides"]["left"]["left"]
    assert left_finger["state"] == "error"
    assert left_finger["serial"] == "GSPS01A24Z0001, GSPS01A24Z0003"
    assert "Two tactile sensors" in left_finger["message"]
    assert result["sides"]["left"]["right"]["state"] == "missing"


def test_diagnose_tactiles_by_hub_keeps_unassigned_sensor(monkeypatch):
    paths = ["/dev/v4l/by-id/usb-Xense_GSPS01A24Z0002-video-index0"]

    monkeypatch.setattr(disco.glob, "glob", lambda pattern: paths)
    monkeypatch.setattr(
        disco,
        "_gripper_hub_sides",
        lambda role: {"0:6": ("left", "TCGU01A24Z0001m")},
    )
    monkeypatch.setattr(disco, "_device_hub", lambda path, bypath_dir: "0:7")

    result = disco.diagnose_tactiles_by_hub("leader")

    assert result["unknown"][0]["serial"] == "GSPS01A24Z0002"
    assert result["unknown"][0]["state"] == "error"
    assert "no matching leader gripper" in result["unknown"][0]["message"]
