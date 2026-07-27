#!/usr/bin/env python
"""Standalone TacCap tracker-to-EE calibration UI.

Run from the repository root:

    python calibrate-ui/calibrate_ui.py

The tool performs fixed-point pivot calibration per side. Hold the gripper EE
contact point on one fixed point in space, rotate the tracker through different
orientations, capture samples, then solve for tracker_to_ee_pos.
"""

from __future__ import annotations

import ctypes
import json
import os
import sys
import sysconfig
import time
from collections import deque
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from calibration_math import (
    IDENTITY_QUAT_WXYZ,
    PivotResult,
    SOLIDWORKS_TRACKER_TO_EE_POS_M,
    SOLIDWORKS_TRACKER_TO_EE_QUAT_WXYZ,
    SOLIDWORKS_TRACKER_TO_EE_RPY_DEG,
    calibrated_ee_transform_world,
    cli_vector,
    estimated_ee_point_world,
    format_vector,
    normalize_quat_wxyz,
    rpy_degrees_to_quat_wxyz,
    raw_pico_pose_wxyz_to_world_matrix,
    solve_pivot,
)
from lerobot.utils.robot_utils import matrix_to_pose7d


def _prepare_qt_runtime() -> list[Path]:
    """Set Qt plugin paths before importing PySide6."""
    purelib = Path(sysconfig.get_paths().get("purelib", ""))
    plugin_roots = [
        purelib / "PySide6" / "Qt" / "plugins",
        Path(sys.prefix) / "lib" / "qt6" / "plugins",
    ]
    plugin_roots = [p for p in plugin_roots if p.exists()]
    if plugin_roots:
        os.environ.setdefault("QT_PLUGIN_PATH", str(plugin_roots[0]))
        platforms = plugin_roots[0] / "platforms"
        if platforms.exists():
            os.environ.setdefault("QT_QPA_PLATFORM_PLUGIN_PATH", str(platforms))
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

    conda_lib = Path(sys.prefix) / "lib"
    for soname in (
        "libxcb-cursor.so.0",
        "libxcb-icccm.so.4",
        "libxcb-image.so.0",
        "libxcb-keysyms.so.1",
        "libxcb-render-util.so.0",
        "libxkbcommon-x11.so.0",
    ):
        lib = conda_lib / soname
        if lib.exists():
            try:
                ctypes.CDLL(str(lib), mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
    return plugin_roots


QT_PLUGIN_ROOTS = _prepare_qt_runtime()

try:
    from PySide6.QtCore import QTimer, Qt
    from PySide6.QtWidgets import (
        QApplication,
        QAbstractItemView,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFileDialog,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QHeaderView,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPlainTextEdit,
        QPushButton,
        QScrollArea,
        QSpinBox,
        QTableWidget,
        QTableWidgetItem,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    print(
        "PySide6 is required for calibrate-ui/calibrate_ui.py.\n"
        "Install it with: python -m pip install -r production_ui/requirements.txt",
        file=sys.stderr,
    )
    raise SystemExit(1)

for plugin_root in QT_PLUGIN_ROOTS:
    QApplication.addLibraryPath(str(plugin_root))


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "tracker_ee_calibration.json"
SIDES = ("left", "right")
SIDE_COLORS = {
    "left": (255, 80, 80),
    "right": (80, 160, 255),
}
ORIENTATION_MODES = (
    ("solidworks", "SolidWorks EE 坐标系"),
    ("tracker_aligned", "与 tracker 一致"),
    ("custom_rpy", "自定义 RPY"),
)


def _ensure_python_bin_on_path() -> None:
    """Make console scripts installed next to the active Python visible.

    Rerun's Python API starts the viewer by looking up the ``rerun`` executable
    on PATH. The production launchers often execute a conda-env Python directly
    while PATH still points at the base env, so prepend sys.executable's bin dir.
    """
    python_bin_dir = str(Path(sys.executable).resolve().parent)
    path = os.environ.get("PATH", "")
    parts = [part for part in path.split(os.pathsep) if part]
    if python_bin_dir not in parts:
        os.environ["PATH"] = python_bin_dir + (os.pathsep + path if path else "")


@dataclass
class SideState:
    side: str
    reader: Any | None = None
    serial: str = ""
    last_world_from_tracker: np.ndarray | None = None
    samples: list[np.ndarray] = field(default_factory=list)
    sample_times: list[str] = field(default_factory=list)
    result: PivotResult | None = None
    trail: deque[list[float]] = field(default_factory=lambda: deque(maxlen=600))
    ee_trail: deque[list[float]] = field(default_factory=lambda: deque(maxlen=600))
    static_logged: bool = False
    ee_static_logged: bool = False
    result_source: str = ""

    @property
    def connected(self) -> bool:
        return self.reader is not None


class CalibrationWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("TacCap Tracker EE Calibration")
        self.setMinimumSize(1120, 760)

        self.states: dict[str, SideState] = {side: SideState(side) for side in SIDES}
        self._rr = None
        self._rerun_started = False
        self._frame_index = 0

        self._build_ui()
        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._poll_trackers)
        self._timer.start()

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        outer = QWidget()
        outer_layout = QVBoxLayout(outer)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)
        self.setCentralWidget(outer)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        outer_layout.addWidget(scroll)

        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)
        scroll.setWidget(root)

        title = QLabel("左右夹爪 EE / 夹持点标定")
        title.setObjectName("Title")
        layout.addWidget(title)

        hint = QLabel(
            "把夹爪夹持端固定顶在同一个空间点上，改变 tracker 姿态并采样。"
            "四点是最低要求，建议每侧采 8-12 个姿态。"
        )
        hint.setObjectName("Hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_sampling_group())
        layout.addWidget(self._build_orientation_group())
        layout.addWidget(self._build_results_group())
        layout.addWidget(self._build_pose_monitor_group())

        self.sample_table = QTableWidget(0, 8)
        self.sample_table.setHorizontalHeaderLabels(
            ["侧别", "#", "tracker x", "tracker y", "tracker z", "residual mm", "time", "serial"]
        )
        self.sample_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.sample_table.setMinimumHeight(180)
        layout.addWidget(self.sample_table)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)
        self.log.setMinimumHeight(130)
        layout.addWidget(self.log)

        self.setStyleSheet(self._stylesheet())
        self._refresh_orientation_labels()
        self._refresh_result_labels()
        self._refresh_pose_labels()

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Tracker 连接")
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        self.left_serial = QLineEdit()
        self.right_serial = QLineEdit()
        self.timeout_s = QLineEdit("10.0")
        self.timeout_s.setMaximumWidth(80)
        self.rerun_check = QCheckBox("启动 Rerun 三维显示")
        self.rerun_check.setChecked(True)

        grid.addWidget(QLabel("Left tracker SN"), 0, 0)
        grid.addWidget(self.left_serial, 0, 1)
        grid.addWidget(QLabel("Right tracker SN"), 0, 2)
        grid.addWidget(self.right_serial, 0, 3)
        grid.addWidget(QLabel("超时 s"), 0, 4)
        grid.addWidget(self.timeout_s, 0, 5)

        scan = QPushButton("扫描 tracker")
        scan.clicked.connect(self.scan_trackers)
        connect_left = QPushButton("连接左")
        connect_left.clicked.connect(lambda: self.connect_sides(("left",)))
        connect_right = QPushButton("连接右")
        connect_right.clicked.connect(lambda: self.connect_sides(("right",)))
        connect_both = QPushButton("连接左右")
        connect_both.setObjectName("PrimaryButton")
        connect_both.clicked.connect(lambda: self.connect_sides(SIDES))
        disconnect = QPushButton("断开")
        disconnect.clicked.connect(self.disconnect_all)
        reset_viewer = QPushButton("刷新 Rerun 场景")
        reset_viewer.clicked.connect(self.reset_rerun)

        buttons = QWidget()
        row = QHBoxLayout(buttons)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        for button in (scan, connect_left, connect_right, connect_both, disconnect, reset_viewer):
            row.addWidget(button)
        row.addWidget(self.rerun_check)
        row.addStretch(1)
        grid.addWidget(buttons, 1, 0, 1, 6)
        return group

    def _build_sampling_group(self) -> QGroupBox:
        group = QGroupBox("采样与求解")
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        self.side_combo = QComboBox()
        self.side_combo.addItems(["left", "right"])
        self.min_samples = QSpinBox()
        self.min_samples.setRange(4, 30)
        self.min_samples.setValue(4)
        self.output_path = QLineEdit(str(DEFAULT_OUTPUT))

        capture = QPushButton("记录当前位姿")
        capture.setObjectName("PrimaryButton")
        capture.clicked.connect(self.capture_sample)
        compute_side = QPushButton("求解当前侧并刷新三维")
        compute_side.clicked.connect(self.compute_selected_side)
        compute_both = QPushButton("求解左右并刷新三维")
        compute_both.clicked.connect(self.compute_both_sides)
        refresh_calibrated = QPushButton("重启 Rerun 显示标定 EE")
        refresh_calibrated.clicked.connect(self.refresh_calibrated_rerun)
        clear_side = QPushButton("清空当前侧")
        clear_side.clicked.connect(self.clear_selected_side)
        save = QPushButton("保存结果 JSON")
        save.clicked.connect(self.save_results)
        browse = QPushButton("浏览")
        browse.clicked.connect(self.browse_output)

        grid.addWidget(QLabel("当前侧"), 0, 0)
        grid.addWidget(self.side_combo, 0, 1)
        grid.addWidget(QLabel("最少样本"), 0, 2)
        grid.addWidget(self.min_samples, 0, 3)
        grid.addWidget(capture, 0, 4)
        grid.addWidget(compute_side, 0, 5)
        grid.addWidget(compute_both, 0, 6)
        grid.addWidget(clear_side, 0, 7)

        grid.addWidget(QLabel("输出文件"), 1, 0)
        grid.addWidget(self.output_path, 1, 1, 1, 5)
        grid.addWidget(browse, 1, 6)
        grid.addWidget(save, 1, 7)
        grid.addWidget(QLabel("三维显示"), 2, 0)
        grid.addWidget(refresh_calibrated, 2, 1, 1, 3)
        return group

    def _build_orientation_group(self) -> QGroupBox:
        group = QGroupBox("EE 坐标系方向")
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        self.orientation_mode: dict[str, QComboBox] = {}
        self.orientation_rpy: dict[str, tuple[QDoubleSpinBox, QDoubleSpinBox, QDoubleSpinBox]] = {}
        self.orientation_quat_labels: dict[str, QLabel] = {}

        headers = ("侧别", "方向模式", "Roll X", "Pitch Y", "Yaw Z", "当前外参", "直接显示")
        for col, text in enumerate(headers):
            grid.addWidget(QLabel(text), 0, col)

        for row, side in enumerate(SIDES, start=1):
            side_title = QLabel(side.upper())
            side_title.setObjectName("SideTitle")
            mode = QComboBox()
            for mode_id, label in ORIENTATION_MODES:
                mode.addItem(label, mode_id)
            mode.setCurrentIndex(0)

            spins: list[QDoubleSpinBox] = []
            for value in SOLIDWORKS_TRACKER_TO_EE_RPY_DEG[side]:
                spin = QDoubleSpinBox()
                spin.setRange(-360.0, 360.0)
                spin.setDecimals(3)
                spin.setSingleStep(1.0)
                spin.setSuffix(" deg")
                spin.setKeyboardTracking(False)
                spin.setValue(float(value))
                spins.append(spin)

            quat_label = QLabel("")
            quat_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            quat_label.setWordWrap(True)
            apply_sw = QPushButton("应用 SW 全外参")
            apply_sw.clicked.connect(lambda _checked=False, s=side: self.apply_solidworks_transform(s))

            self.orientation_mode[side] = mode
            self.orientation_rpy[side] = (spins[0], spins[1], spins[2])
            self.orientation_quat_labels[side] = quat_label

            grid.addWidget(side_title, row, 0)
            grid.addWidget(mode, row, 1)
            grid.addWidget(spins[0], row, 2)
            grid.addWidget(spins[1], row, 3)
            grid.addWidget(spins[2], row, 4)
            grid.addWidget(quat_label, row, 5)
            grid.addWidget(apply_sw, row, 6)

            mode.currentIndexChanged.connect(lambda _idx, s=side: self._orientation_settings_changed(s))
            for spin in spins:
                spin.valueChanged.connect(lambda _value, s=side: self._orientation_settings_changed(s))
            self._set_orientation_inputs_enabled(side)

        apply_both = QPushButton("应用左右 SW 全外参")
        apply_both.setObjectName("PrimaryButton")
        apply_both.clicked.connect(self.apply_solidworks_transforms)
        grid.addWidget(apply_both, len(SIDES) + 1, 0, 1, 2)

        note = QLabel(
            "Pivot 样本只求 EE 原点；这里选择 EE 坐标轴方向。"
            "也可直接应用 SolidWorks 测出的完整 ^Tracker T_EE。"
            "自定义 RPY 约定：R = Rz(yaw) * Ry(pitch) * Rx(roll)。"
        )
        note.setObjectName("Hint")
        note.setWordWrap(True)
        grid.addWidget(note, len(SIDES) + 2, 0, 1, 7)
        return group

    def _build_results_group(self) -> QGroupBox:
        group = QGroupBox("标定结果")
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        self.result_labels: dict[str, QLabel] = {}
        for row, side in enumerate(SIDES):
            side_title = QLabel(side.upper())
            side_title.setObjectName("SideTitle")
            label = QLabel("未求解")
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            label.setWordWrap(True)
            self.result_labels[side] = label
            grid.addWidget(side_title, row, 0)
            grid.addWidget(label, row, 1)
        note = QLabel(
            "注意：固定点 pivot 标定只能求 tracker_to_ee_pos；"
            "tracker_to_ee_quat 来自上方选择的 EE 坐标系方向。"
        )
        note.setObjectName("Hint")
        note.setWordWrap(True)
        grid.addWidget(note, 2, 0, 1, 2)
        return group

    def _build_pose_monitor_group(self) -> QGroupBox:
        group = QGroupBox("实时位姿 / 坐标链路")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(6)

        rows: list[tuple[str, str]] = [
            ("pico_world", "PICO world"),
            ("left_tracker", "left world_tracker"),
            ("left_ee", "left world_calibrated_ee"),
            ("left_rel", "left tracker_to_ee"),
            ("right_tracker", "right world_tracker"),
            ("right_ee", "right world_calibrated_ee"),
            ("right_rel", "right tracker_to_ee"),
        ]
        self.pose_row_by_key = {key: row for row, (key, _name) in enumerate(rows)}
        self.pose_items: dict[tuple[str, int], QTableWidgetItem] = {}
        self.pose_table = QTableWidget(len(rows), 5)
        self.pose_table.setObjectName("PoseTable")
        self.pose_table.setHorizontalHeaderLabels(["坐标链路", "状态 / SN", "位置 xyz (m)", "四元数 wxyz", "说明"])
        self.pose_table.verticalHeader().setVisible(False)
        self.pose_table.verticalHeader().setDefaultSectionSize(30)
        self.pose_table.setAlternatingRowColors(True)
        self.pose_table.setWordWrap(False)
        self.pose_table.setTextElideMode(Qt.ElideNone)
        self.pose_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.pose_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.pose_table.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.pose_table.setMinimumHeight(252)
        self.pose_table.setMaximumHeight(280)
        header = self.pose_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)

        for row, (key, name) in enumerate(rows):
            for col in range(5):
                item = QTableWidgetItem("")
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                item.setTextAlignment(Qt.AlignVCenter | Qt.AlignLeft)
                self.pose_table.setItem(row, col, item)
                self.pose_items[(key, col)] = item
            self.pose_items[(key, 0)].setText(name)

        layout.addWidget(self.pose_table)
        return group

    # ------------------------------------------------------------------ Tracker lifecycle

    def scan_trackers(self) -> None:
        try:
            from lerobot.robots.taccap_gripper import serial_discovery as disco
            from tracker_scan import scan_trackers

            timeout = self._timeout()
            serials, error = scan_trackers(timeout)
            if error and not serials:
                raise RuntimeError(f"{error.get('type', 'TrackerScanError')}: {error.get('message', '')}")
            if error:
                self._append_log(f"[scan warning] {error.get('type', '')}: {error.get('message', '')}")
            self._append_log(f"[scan] discovered: {serials}")
            for serial in serials:
                try:
                    side = disco.pico_tracker_side(serial)
                except Exception as exc:
                    self._append_log(f"[scan] skip malformed tracker SN {serial!r}: {exc}")
                    continue
                if side == "left" and not self.left_serial.text().strip():
                    self.left_serial.setText(serial)
                elif side == "right" and not self.right_serial.text().strip():
                    self.right_serial.setText(serial)
        except Exception as exc:
            QMessageBox.critical(self, "扫描失败", str(exc))
            self._append_log(f"[scan error] {exc}")

    def connect_sides(self, sides: tuple[str, ...]) -> None:
        self._ensure_rerun()
        try:
            from lerobot.teleoperators.pico4.tracker import Pico4TrackerReader
        except Exception as exc:
            QMessageBox.critical(self, "导入失败", str(exc))
            return

        for side in sides:
            state = self.states[side]
            if state.connected:
                self._append_log(f"[{side}] already connected")
                continue
            serial = self._serial_for_side(side)
            if not serial:
                self._append_log(f"[{side}] no serial specified; Pico reader will use index 0")
                serial_arg = None
            else:
                serial_arg = serial

            try:
                QApplication.setOverrideCursor(Qt.WaitCursor)
                reader = Pico4TrackerReader(
                    tracker_sn=serial_arg,
                    tracker_to_ee_pos=(0.0, 0.0, 0.0),
                    tracker_to_ee_quat=IDENTITY_QUAT_WXYZ,
                    device_wait_timeout=self._timeout(),
                    pico_to_world=False,
                    logger_name=f"calibrate-{side}",
                )
                reader.connect()
                state.reader = reader
                state.serial = str(getattr(reader, "_resolved_sn", None) or serial)
                state.trail.clear()
                state.static_logged = False
                self._append_log(f"[{side}] connected tracker SN={state.serial or '<index 0>'}")
            except Exception as exc:
                QMessageBox.critical(self, f"{side} 连接失败", str(exc))
                self._append_log(f"[{side} connect error] {exc}")
            finally:
                QApplication.restoreOverrideCursor()
        self._refresh_pose_labels()

    def disconnect_all(self) -> None:
        for side, state in self.states.items():
            if state.reader is None:
                continue
            try:
                state.reader.disconnect()
            except Exception as exc:
                self._append_log(f"[{side} disconnect warning] {exc}")
            state.reader = None
            state.last_world_from_tracker = None
            self._append_log(f"[{side}] disconnected")
        self._refresh_pose_labels()

    # ------------------------------------------------------------------ Sampling and solve

    def capture_sample(self) -> None:
        side = self.side_combo.currentText()
        state = self.states[side]
        if state.last_world_from_tracker is None:
            QMessageBox.warning(self, "无法采样", f"{side} tracker 还没有有效位姿。")
            return
        state.samples.append(state.last_world_from_tracker.copy())
        state.sample_times.append(datetime.now().strftime("%H:%M:%S"))
        state.result = None
        state.result_source = ""
        state.ee_trail.clear()
        state.ee_static_logged = False
        index = len(state.samples)
        self._append_log(f"[{side}] captured sample #{index}")
        self._log_sample_pose(side, index, state.samples[-1])
        self._refresh_table()
        self._refresh_result_labels()
        self._refresh_pose_labels()

    def compute_selected_side(self) -> None:
        self.compute_side(self.side_combo.currentText())

    def compute_both_sides(self) -> None:
        solved = False
        for side in SIDES:
            if self.states[side].samples:
                solved = self.compute_side(side, restart_rerun=False) or solved
        if solved:
            self._restart_rerun_with_current_state("left/right calibration solve")

    def compute_side(self, side: str, restart_rerun: bool = True) -> bool:
        state = self.states[side]
        try:
            result = solve_pivot(
                state.samples,
                min_samples=int(self.min_samples.value()),
                tracker_to_ee_quat=self._orientation_quat(side),
            )
        except Exception as exc:
            QMessageBox.warning(self, f"{side} 求解失败", str(exc))
            self._append_log(f"[{side} solve error] {exc}")
            return False
        state.result = result
        state.result_source = "pivot"
        state.ee_trail.clear()
        state.ee_static_logged = False
        self._append_log(
            f"[{side}] solved tracker_to_ee_pos={format_vector(result.tracker_to_ee_pos)} "
            f"tracker_to_ee_quat={format_vector(result.tracker_to_ee_quat)} "
            f"rmse={result.rmse_m * 1000.0:.2f}mm max={result.max_error_m * 1000.0:.2f}mm "
            f"rank={result.rank} cond={result.condition:.1f}"
        )
        self._log_result(side)
        self._refresh_table()
        self._refresh_result_labels()
        self._refresh_pose_labels()
        if restart_rerun:
            self._restart_rerun_with_current_state(f"{side} calibration solve")
        return True

    def apply_solidworks_transforms(self) -> None:
        for side in SIDES:
            self.apply_solidworks_transform(side, restart_rerun=False)
        self._restart_rerun_with_current_state("SolidWorks full tracker_to_ee transform")

    def apply_solidworks_transform(self, side: str, restart_rerun: bool = True) -> None:
        state = self.states[side]
        self._set_orientation_mode(side, "solidworks")
        result = PivotResult(
            tracker_to_ee_pos=np.asarray(SOLIDWORKS_TRACKER_TO_EE_POS_M[side], dtype=np.float64),
            tracker_to_ee_quat=normalize_quat_wxyz(SOLIDWORKS_TRACKER_TO_EE_QUAT_WXYZ[side]),
            fixed_point_world=np.full(3, np.nan, dtype=np.float64),
            residuals_m=np.asarray([], dtype=np.float64),
            rmse_m=float("nan"),
            mean_error_m=float("nan"),
            max_error_m=float("nan"),
            rank=0,
            condition=float("nan"),
        )
        state.result = result
        state.result_source = "solidworks_full_matrix"
        state.ee_trail.clear()
        state.ee_static_logged = False
        self._append_log(
            f"[{side}] applied SolidWorks ^Tracker T_EE "
            f"pos={format_vector(result.tracker_to_ee_pos)} quat={format_vector(result.tracker_to_ee_quat)}"
        )
        self._refresh_orientation_labels()
        self._refresh_table()
        self._refresh_result_labels()
        self._refresh_pose_labels()
        if restart_rerun:
            self._restart_rerun_with_current_state(f"{side} SolidWorks full tracker_to_ee transform")

    def clear_selected_side(self) -> None:
        side = self.side_combo.currentText()
        state = self.states[side]
        state.samples.clear()
        state.sample_times.clear()
        state.result = None
        state.result_source = ""
        state.trail.clear()
        state.ee_trail.clear()
        state.ee_static_logged = False
        self._append_log(f"[{side}] cleared samples")
        self._refresh_table()
        self._refresh_result_labels()
        self._refresh_pose_labels()

    # ------------------------------------------------------------------ Save

    def browse_output(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "保存标定结果", self.output_path.text(), "JSON (*.json)")
        if path:
            self.output_path.setText(path)

    def save_results(self) -> None:
        payload = {
            "type": "taccap_tracker_ee_calibration",
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "units": "meters",
            "notes": (
                "Results may come from fixed-point pivot samples or a direct "
                "SolidWorks ^Tracker T_EE transform. Pivot estimates "
                "tracker_to_ee_pos only; tracker_to_ee_quat comes from the "
                "selected EE orientation mode."
            ),
            "sides": {},
            "robot_args": {
                "bi_taccap_gripper": [],
                "taccap_gripper_left": [],
                "taccap_gripper_right": [],
            },
        }

        solved = False
        for side in SIDES:
            state = self.states[side]
            result = state.result
            if result is None:
                continue
            solved = True
            orientation_mode, orientation_label = self._orientation_mode(side)
            orientation_rpy_deg = self._orientation_rpy_deg(side)
            source = state.result_source or "pivot"
            fixed_point_world = (
                result.fixed_point_world.tolist()
                if np.all(np.isfinite(result.fixed_point_world))
                else None
            )
            rmse_m = float(result.rmse_m) if np.isfinite(result.rmse_m) else None
            mean_error_m = float(result.mean_error_m) if np.isfinite(result.mean_error_m) else None
            max_error_m = float(result.max_error_m) if np.isfinite(result.max_error_m) else None
            condition = float(result.condition) if np.isfinite(result.condition) else None
            side_payload = {
                "tracker_serial": state.serial,
                "calibration_source": source,
                "tracker_to_ee_pos": result.tracker_to_ee_pos.tolist(),
                "tracker_to_ee_quat": result.tracker_to_ee_quat.tolist(),
                "orientation_mode": orientation_mode,
                "orientation_label": orientation_label,
                "orientation_rpy_deg": list(orientation_rpy_deg),
                "fixed_point_world": fixed_point_world,
                "samples": len(state.samples),
                "rmse_m": rmse_m,
                "mean_error_m": mean_error_m,
                "max_error_m": max_error_m,
                "rank": result.rank,
                "condition": condition,
            }
            payload["sides"][side] = side_payload

            pos = cli_vector(result.tracker_to_ee_pos)
            quat = cli_vector(result.tracker_to_ee_quat)
            payload["robot_args"]["bi_taccap_gripper"].extend(
                [
                    f"--robot.{side}_tracker_to_ee_pos={pos}",
                    f"--robot.{side}_tracker_to_ee_quat={quat}",
                ]
            )
            payload["robot_args"][f"taccap_gripper_{side}"].extend(
                [
                    f"--robot.side={side}",
                    f"--robot.tracker_to_ee_pos={pos}",
                    f"--robot.tracker_to_ee_quat={quat}",
                ]
            )

        if not solved:
            QMessageBox.warning(self, "没有结果", "请先至少求解一侧标定结果。")
            return

        path = Path(self.output_path.text()).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
        except Exception as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return
        self._append_log(f"[save] wrote {path}")
        QMessageBox.information(self, "保存完成", f"已保存:\n{path}")

    # ------------------------------------------------------------------ Polling and Rerun

    def _poll_trackers(self) -> None:
        self._frame_index += 1
        pose_updated = False
        for side, state in self.states.items():
            if state.reader is None:
                continue
            try:
                raw = state.reader.get_pose_raw()
            except Exception as exc:
                self._append_log(f"[{side} poll warning] {exc}")
                continue
            if raw is None:
                continue
            try:
                world_from_tracker = raw_pico_pose_wxyz_to_world_matrix(raw)
            except Exception as exc:
                self._append_log(f"[{side} transform warning] {exc}")
                continue
            state.last_world_from_tracker = world_from_tracker
            self._log_tracker_pose(side, world_from_tracker)
            pose_updated = True
        if pose_updated:
            self._refresh_pose_labels()

    def _ensure_rerun(self) -> None:
        if self._rerun_started or not self.rerun_check.isChecked():
            return
        try:
            import rerun as rr

            _ensure_python_bin_on_path()
            self._rr = rr
            os.environ.setdefault("RERUN_FLUSH_NUM_BYTES", "8000")
            rr.init("taccap_tracker_ee_calibration")
            rr.spawn(memory_limit=os.getenv("LEROBOT_RERUN_MEMORY_LIMIT", "10%"))
            self._rerun_started = True
            self._log_world_static()
            self._append_log("[rerun] viewer started")
        except Exception as exc:
            self._rerun_started = False
            self._rr = None
            QMessageBox.warning(self, "Rerun 启动失败", str(exc))
            self._append_log(f"[rerun error] {exc}")

    def reset_rerun(self) -> None:
        self._restart_rerun_with_current_state("manual reset")

    def refresh_calibrated_rerun(self) -> None:
        if not any(state.result is not None for state in self.states.values()):
            QMessageBox.information(self, "没有标定结果", "请先求解至少一侧，再显示标定后的 EE 坐标系。")
            return
        self.rerun_check.setChecked(True)
        self._restart_rerun_with_current_state("manual calibrated EE refresh")

    def _restart_rerun_with_current_state(self, reason: str) -> None:
        if not self.rerun_check.isChecked():
            return
        if self._rr is not None:
            try:
                self._rr.rerun_shutdown()
            except Exception:
                pass
        self._rr = None
        self._rerun_started = False
        for state in self.states.values():
            state.static_logged = False
            state.ee_static_logged = False
            state.trail.clear()
            state.ee_trail.clear()
        self._ensure_rerun()
        if not self._rerun_started:
            return
        for side, state in self.states.items():
            for index, sample in enumerate(state.samples, start=1):
                self._log_sample_pose(side, index, sample)
            if state.result is not None:
                self._log_result(side)
            if state.last_world_from_tracker is not None:
                self._log_tracker_pose(side, state.last_world_from_tracker)
        self._append_log(f"[rerun] restarted after {reason}; calibrated EE frames are visible")

    def _log_world_static(self) -> None:
        rr = self._rr
        if rr is None:
            return
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log(
            "world/pico_world",
            rr.Transform3D(translation=[0.0, 0.0, 0.0], quaternion=rr.Quaternion(xyzw=[0.0, 0.0, 0.0, 1.0])),
            static=True,
        )
        rr.log(
            "world/pico_world/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[0.30, 0, 0], [0, 0.30, 0], [0, 0, 0.30]],
                colors=[[255, 50, 50], [50, 220, 50], [50, 80, 255]],
                radii=0.008,
            ),
            static=True,
        )
        rr.log(
            "world/pico_world/origin",
            rr.Points3D([[0.0, 0.0, 0.36]], labels=["PICO WORLD"], colors=[[230, 230, 230]], radii=0.006),
            static=True,
        )

    def _log_static_once(self, side: str) -> None:
        rr = self._rr
        state = self.states[side]
        if rr is None or state.static_logged:
            return
        color = SIDE_COLORS[side]
        ent = f"world/{side}/tracker"
        rr.log(
            f"{ent}/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[0.12, 0, 0], [0, 0.12, 0], [0, 0, 0.12]],
                colors=[[255, 60, 60], [60, 255, 60], [60, 80, 255]],
                radii=0.004,
            ),
            static=True,
        )
        rr.log(
            f"{ent}/origin",
            rr.Points3D([[0, 0, 0]], labels=[f"{side} tracker"], colors=[color], radii=0.01),
            static=True,
        )
        state.static_logged = True

    def _log_ee_static_once(self, side: str) -> None:
        rr = self._rr
        state = self.states[side]
        result = state.result
        if rr is None or state.ee_static_logged or result is None:
            return
        color = SIDE_COLORS[side]
        ent = f"world/{side}/calibrated_ee"
        rr.log(
            f"{ent}/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[0.10, 0, 0], [0, 0.10, 0], [0, 0, 0.10]],
                colors=[[255, 60, 60], [60, 255, 60], [60, 80, 255]],
                radii=0.004,
            ),
            static=True,
        )
        rr.log(
            f"{ent}/origin",
            rr.Points3D([[0, 0, 0]], labels=[f"{side} calibrated EE"], colors=[color], radii=0.012),
            static=True,
        )
        tracker_to_ee = f"world/{side}/tracker/tracker_to_ee"
        quat = result.tracker_to_ee_quat
        xyzw = [float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])]
        offset = [float(v) for v in result.tracker_to_ee_pos]
        rr.log(
            tracker_to_ee,
            rr.Transform3D(translation=offset, quaternion=rr.Quaternion(xyzw=xyzw)),
            static=True,
        )
        rr.log(
            f"{tracker_to_ee}/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[0.075, 0, 0], [0, 0.075, 0], [0, 0, 0.075]],
                colors=[[255, 60, 60], [60, 255, 60], [60, 80, 255]],
                radii=0.003,
            ),
            static=True,
        )
        rr.log(
            f"{tracker_to_ee}/origin",
            rr.Points3D([[0, 0, 0]], labels=[f"{side} T_tracker_ee"], colors=[color], radii=0.01),
            static=True,
        )
        rr.log(
            f"world/{side}/tracker/tracker_to_ee_link",
            rr.LineStrips3D([[[0.0, 0.0, 0.0], offset]], colors=[color], radii=0.004),
            static=True,
        )
        state.ee_static_logged = True

    def _log_tracker_pose(self, side: str, world_from_tracker: np.ndarray) -> None:
        rr = self._rr
        if rr is None or not self._rerun_started:
            return
        self._log_static_once(side)
        pose = matrix_to_pose7d(world_from_tracker, output_format="wxyz")
        xyzw = [float(pose[4]), float(pose[5]), float(pose[6]), float(pose[3])]
        rr.set_time_sequence("frame", self._frame_index)
        rr.log(
            f"world/{side}/tracker",
            rr.Transform3D(
                translation=[float(pose[0]), float(pose[1]), float(pose[2])],
                quaternion=rr.Quaternion(xyzw=xyzw),
            ),
        )

        state = self.states[side]
        state.trail.append([float(pose[0]), float(pose[1]), float(pose[2])])
        if len(state.trail) >= 2:
            rr.log(
                f"world/{side}/tracker_trail",
                rr.LineStrips3D([list(state.trail)], colors=[SIDE_COLORS[side]], radii=0.003),
            )
        if state.result is not None:
            self._log_ee_pose(side, world_from_tracker)

    def _log_ee_pose(self, side: str, world_from_tracker: np.ndarray) -> None:
        rr = self._rr
        state = self.states[side]
        result = state.result
        if rr is None or not self._rerun_started or result is None:
            return
        self._log_ee_static_once(side)
        world_from_ee = calibrated_ee_transform_world(
            world_from_tracker,
            result.tracker_to_ee_pos,
            result.tracker_to_ee_quat,
        )
        pose = matrix_to_pose7d(world_from_ee, output_format="wxyz")
        xyzw = [float(pose[4]), float(pose[5]), float(pose[6]), float(pose[3])]
        rr.log(
            f"world/{side}/calibrated_ee",
            rr.Transform3D(
                translation=[float(pose[0]), float(pose[1]), float(pose[2])],
                quaternion=rr.Quaternion(xyzw=xyzw),
            ),
        )
        state.ee_trail.append([float(pose[0]), float(pose[1]), float(pose[2])])
        if len(state.ee_trail) >= 2:
            rr.log(
                f"world/{side}/calibrated_ee_trail",
                rr.LineStrips3D([list(state.ee_trail)], colors=[SIDE_COLORS[side]], radii=0.003),
            )

    def _log_sample_pose(self, side: str, index: int, world_from_tracker: np.ndarray) -> None:
        rr = self._rr
        if rr is None or not self._rerun_started:
            return
        pose = matrix_to_pose7d(world_from_tracker, output_format="wxyz")
        xyzw = [float(pose[4]), float(pose[5]), float(pose[6]), float(pose[3])]
        ent = f"world/{side}/samples/sample_{index:02d}"
        rr.log(
            ent,
            rr.Transform3D(
                translation=[float(pose[0]), float(pose[1]), float(pose[2])],
                quaternion=rr.Quaternion(xyzw=xyzw),
            ),
        )
        rr.log(
            f"{ent}/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[0.08, 0, 0], [0, 0.08, 0], [0, 0, 0.08]],
                colors=[[255, 80, 80], [80, 255, 80], [80, 80, 255]],
                radii=0.003,
            ),
            static=True,
        )
        rr.log(
            f"{ent}/label",
            rr.Points3D([[0, 0, 0.10]], labels=[f"{side} #{index}"], colors=[SIDE_COLORS[side]], radii=0.004),
            static=True,
        )

    def _log_result(self, side: str) -> None:
        rr = self._rr
        state = self.states[side]
        result = state.result
        if rr is None or not self._rerun_started or result is None:
            return
        if not np.all(np.isfinite(result.fixed_point_world)):
            return
        rr.log(
            f"world/{side}/fixed_point",
            rr.Points3D(
                [result.fixed_point_world.tolist()],
                labels=[f"{side} fixed point"],
                colors=[SIDE_COLORS[side]],
                radii=0.018,
            ),
        )
        strips = []
        for sample in state.samples:
            predicted = estimated_ee_point_world(sample, result.tracker_to_ee_pos)
            strips.append([predicted.tolist(), result.fixed_point_world.tolist()])
        if strips:
            rr.log(
                f"world/{side}/residuals",
                rr.LineStrips3D(strips, colors=[SIDE_COLORS[side]], radii=0.002),
            )

    # ------------------------------------------------------------------ UI updates

    def _orientation_mode(self, side: str) -> tuple[str, str]:
        combo = self.orientation_mode[side]
        mode = str(combo.currentData() or "solidworks")
        return mode, combo.currentText()

    def _set_orientation_mode(self, side: str, mode_id: str) -> None:
        combo = self.orientation_mode[side]
        for index in range(combo.count()):
            if combo.itemData(index) == mode_id:
                combo.setCurrentIndex(index)
                break
        if mode_id == "solidworks":
            for spin, value in zip(self.orientation_rpy[side], SOLIDWORKS_TRACKER_TO_EE_RPY_DEG[side], strict=True):
                blocked = spin.blockSignals(True)
                spin.setValue(float(value))
                spin.blockSignals(blocked)
        self._set_orientation_inputs_enabled(side)

    def _orientation_rpy_deg(self, side: str) -> tuple[float, float, float]:
        mode, _label = self._orientation_mode(side)
        if mode == "tracker_aligned":
            return (0.0, 0.0, 0.0)
        if mode == "solidworks":
            return tuple(float(v) for v in SOLIDWORKS_TRACKER_TO_EE_RPY_DEG[side])
        roll, pitch, yaw = self.orientation_rpy[side]
        return (float(roll.value()), float(pitch.value()), float(yaw.value()))

    def _orientation_quat(self, side: str) -> np.ndarray:
        mode, _label = self._orientation_mode(side)
        if mode == "tracker_aligned":
            return np.asarray(IDENTITY_QUAT_WXYZ, dtype=np.float64)
        if mode == "solidworks":
            return normalize_quat_wxyz(SOLIDWORKS_TRACKER_TO_EE_QUAT_WXYZ[side])
        roll_deg, pitch_deg, yaw_deg = self._orientation_rpy_deg(side)
        return rpy_degrees_to_quat_wxyz(roll_deg, pitch_deg, yaw_deg)

    def _set_orientation_inputs_enabled(self, side: str) -> None:
        mode, _label = self._orientation_mode(side)
        enabled = mode == "custom_rpy"
        for spin in self.orientation_rpy[side]:
            spin.setEnabled(enabled)

    def _orientation_settings_changed(self, side: str) -> None:
        if not hasattr(self, "orientation_mode"):
            return
        self._set_orientation_inputs_enabled(side)
        state = self.states[side]
        if state.result is not None:
            state.result = replace(state.result, tracker_to_ee_quat=self._orientation_quat(side))
            mode, _label = self._orientation_mode(side)
            if state.result_source == "solidworks_full_matrix" and mode != "solidworks":
                state.result_source = "solidworks_position_selected_orientation"
            elif state.result_source == "solidworks_position_selected_orientation" and mode == "solidworks":
                state.result_source = "solidworks_full_matrix"
            state.ee_static_logged = False
            state.ee_trail.clear()
            if state.last_world_from_tracker is not None and self._rerun_started:
                self._log_ee_pose(side, state.last_world_from_tracker)
                self._append_log(
                    f"[{side}] updated EE orientation: "
                    f"{self._orientation_mode(side)[1]} quat={format_vector(state.result.tracker_to_ee_quat)}"
                )
        self._refresh_orientation_labels()
        self._refresh_result_labels()
        self._refresh_pose_labels()

    def _refresh_orientation_labels(self) -> None:
        if not hasattr(self, "orientation_quat_labels"):
            return
        for side in SIDES:
            self._set_orientation_inputs_enabled(side)
            quat = self._orientation_quat(side)
            rpy_deg = self._orientation_rpy_deg(side)
            self.orientation_quat_labels[side].setText(
                f"{format_vector(quat, precision=6)}  "
                f"RPY(deg)={format_vector(rpy_deg, precision=3)}"
            )

    def _refresh_table(self) -> None:
        rows = sum(len(state.samples) for state in self.states.values())
        self.sample_table.setRowCount(rows)
        row = 0
        for side in SIDES:
            state = self.states[side]
            residuals = (
                state.result.residuals_m
                if state.result is not None and state.result.residuals_m.shape == (len(state.samples),)
                else None
            )
            for index, sample in enumerate(state.samples, start=1):
                pos = sample[:3, 3]
                residual_mm = "" if residuals is None else f"{residuals[index - 1] * 1000.0:.2f}"
                values = [
                    side,
                    str(index),
                    f"{pos[0]:+.4f}",
                    f"{pos[1]:+.4f}",
                    f"{pos[2]:+.4f}",
                    residual_mm,
                    state.sample_times[index - 1] if index - 1 < len(state.sample_times) else "",
                    state.serial,
                ]
                for col, value in enumerate(values):
                    item = QTableWidgetItem(value)
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                    self.sample_table.setItem(row, col, item)
                row += 1

    def _refresh_result_labels(self) -> None:
        for side in SIDES:
            state = self.states[side]
            result = state.result
            if result is None:
                self.result_labels[side].setText(f"样本 {len(state.samples)} 个；未求解")
                continue
            _mode, orientation_label = self._orientation_mode(side)
            source = state.result_source or "pivot"
            rmse_text = "n/a" if not np.isfinite(result.rmse_m) else f"{result.rmse_m * 1000.0:.2f}mm"
            max_text = "n/a" if not np.isfinite(result.max_error_m) else f"{result.max_error_m * 1000.0:.2f}mm"
            cond_text = "n/a" if not np.isfinite(result.condition) else f"{result.condition:.1f}"
            text = (
                f"source={source}  "
                f"samples={len(state.samples)}  "
                f"tracker_to_ee_pos={format_vector(result.tracker_to_ee_pos)}  "
                f"tracker_to_ee_quat={format_vector(result.tracker_to_ee_quat)}  "
                f"orientation={orientation_label}  "
                f"rmse={rmse_text}  "
                f"max={max_text}  "
                f"rank={result.rank}  cond={cond_text}"
            )
            self.result_labels[side].setText(text)

    def _refresh_pose_labels(self) -> None:
        if not hasattr(self, "pose_items"):
            return
        self._set_pose_row(
            "pico_world",
            status="固定参考系",
            pos="[0.0000, 0.0000, 0.0000]",
            quat="[1.00000, 0.00000, 0.00000, 0.00000]",
            note="PICO remap world",
        )
        for side in SIDES:
            state = self.states[side]
            tracker_key = f"{side}_tracker"
            ee_key = f"{side}_ee"
            rel_key = f"{side}_rel"

            if state.last_world_from_tracker is None:
                status = "未连接" if state.reader is None else "等待 tracker 位姿"
                note = f"SN={state.serial}" if state.serial else ""
                self._set_pose_row(tracker_key, status=status, note=note)
            else:
                pos, quat = self._format_pose_matrix_parts(state.last_world_from_tracker)
                status = f"SN={state.serial or '<index 0>'}" if state.serial or state.reader is not None else "实时"
                self._set_pose_row(tracker_key, status=status, pos=pos, quat=quat, note="PICO tracker")

            result = state.result
            if result is None:
                self._set_pose_row(ee_key, status="未求解")
                self._set_pose_row(rel_key, status="未求解")
                continue

            self._set_pose_row(
                rel_key,
                status="静态外参",
                pos=format_vector(result.tracker_to_ee_pos, precision=6),
                quat=format_vector(result.tracker_to_ee_quat, precision=5),
                note=self._orientation_mode(side)[1],
            )
            if state.last_world_from_tracker is None:
                self._set_pose_row(ee_key, status="等待 tracker 位姿")
            else:
                world_from_ee = calibrated_ee_transform_world(
                    state.last_world_from_tracker,
                    result.tracker_to_ee_pos,
                    result.tracker_to_ee_quat,
                )
                pos, quat = self._format_pose_matrix_parts(world_from_ee)
                self._set_pose_row(
                    ee_key,
                    status="实时计算",
                    pos=pos,
                    quat=quat,
                    note="T_world_tracker * T_tracker_ee",
                )

    # ------------------------------------------------------------------ Helpers

    def _set_pose_row(self, key: str, status: str = "", pos: str = "", quat: str = "", note: str = "") -> None:
        values = ("", status, pos, quat, note)
        for col, value in enumerate(values):
            if col == 0:
                continue
            self.pose_items[(key, col)].setText(value)

    def _format_pose_matrix_parts(self, transform: np.ndarray) -> tuple[str, str]:
        pose = matrix_to_pose7d(transform, output_format="wxyz")
        return format_vector(pose[:3], precision=4), format_vector(pose[3:], precision=5)

    def _serial_for_side(self, side: str) -> str:
        return (self.left_serial if side == "left" else self.right_serial).text().strip()

    def _timeout(self) -> float:
        try:
            return max(0.5, float(self.timeout_s.text().strip() or "10.0"))
        except ValueError:
            return 10.0

    def _append_log(self, text: str) -> None:
        self.log.appendPlainText(f"{time.strftime('%H:%M:%S')} {text}")

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.disconnect_all()
        if self._rr is not None:
            try:
                self._rr.rerun_shutdown()
            except Exception:
                pass
        event.accept()

    def _stylesheet(self) -> str:
        return """
        QWidget {
            font-size: 13px;
            color: #172033;
            background: #f6f8fb;
        }
        QLabel#Title {
            font-size: 20px;
            font-weight: 700;
        }
        QLabel#Hint {
            color: #52606d;
        }
        QLabel#SideTitle {
            font-weight: 700;
        }
        QTableWidget#PoseTable {
            background: #ffffff;
            alternate-background-color: #f8fafc;
            border: 1px solid #d7e0ea;
            border-radius: 5px;
            gridline-color: #e2e8f0;
            font-size: 12px;
        }
        QTableWidget#PoseTable::item {
            padding: 4px 8px;
            color: #243247;
            font-family: "DejaVu Sans Mono", "Consolas", monospace;
        }
        QGroupBox {
            border: 1px solid #d7e0ea;
            border-radius: 6px;
            margin-top: 8px;
            padding-top: 8px;
            background: #ffffff;
            font-weight: 700;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 0 5px;
            left: 8px;
            background: #ffffff;
        }
        QLineEdit, QComboBox, QPlainTextEdit, QSpinBox, QDoubleSpinBox, QTableWidget {
            background: #ffffff;
            border: 1px solid #cbd5e1;
            border-radius: 5px;
            padding: 3px 6px;
            selection-background-color: #2563eb;
        }
        QPushButton {
            background: #ffffff;
            border: 1px solid #cbd5e1;
            border-radius: 5px;
            padding: 4px 10px;
            font-weight: 600;
        }
        QPushButton:hover {
            background: #eef2f7;
        }
        QPushButton#PrimaryButton {
            background: #2563eb;
            color: #ffffff;
            border-color: #2563eb;
        }
        QPushButton#PrimaryButton:hover {
            background: #1d4ed8;
        }
        QHeaderView::section {
            background: #e7edf5;
            padding: 4px;
            border: 1px solid #cbd5e1;
            font-weight: 700;
        }
        """


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("TacCap Tracker EE Calibration")
    window = CalibrationWindow()
    window.show()
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
