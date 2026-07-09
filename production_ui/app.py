#!/usr/bin/env python
"""
PySide6 production UI for TacCap data collection.

It wraps the command flows from src/lerobot/scripts/client_commands.md and
integrates dataset checking plus push_dataset_to_hub.
"""

from __future__ import annotations

import json
import os
import shlex
import sys
import sysconfig
import ctypes
from pathlib import Path
from typing import Any


def _prepare_qt_runtime() -> list[Path]:
    """Set Qt plugin paths before importing PySide6.

    Some conda/pip mixed environments do not expose PySide6's Qt plugin path to
    Qt, which produces: "Could not find the Qt platform plugin 'xcb' in ''".
    PySide6 wheels can also miss libxcb-cursor at the system level; conda often
    has it under $CONDA_PREFIX/lib, so preload it when available.
    """
    purelib = Path(sysconfig.get_paths().get("purelib", ""))
    frozen_root = Path(getattr(sys, "_MEIPASS", "")) if getattr(sys, "frozen", False) else None
    executable_root = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else None
    plugin_roots = [
        purelib / "PySide6" / "Qt" / "plugins",
        Path(sys.prefix) / "lib" / "qt6" / "plugins",
    ]
    for root in (frozen_root, executable_root):
        if root is not None:
            plugin_roots.extend(
                [
                    root / "PySide6" / "Qt" / "plugins",
                    root / "_internal" / "PySide6" / "Qt" / "plugins",
                ]
            )
    plugin_roots = [p for p in plugin_roots if p.exists()]

    if plugin_roots:
        # Prefer the PySide6 wheel's own Qt plugin tree. It matches the PySide6
        # Qt libraries bundled with the wheel.
        os.environ.setdefault("QT_PLUGIN_PATH", str(plugin_roots[0]))
        platforms = plugin_roots[0] / "platforms"
        if platforms.exists():
            os.environ.setdefault("QT_QPA_PLATFORM_PLUGIN_PATH", str(platforms))

    # Force the normal desktop backend unless the caller explicitly requested
    # wayland/offscreen/minimal/etc.
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

    # Make the common missing xcb helper libraries available to dlopen. This is
    # useful when PySide6 is installed by pip inside a conda env whose lib/
    # contains the xcb dependency set.
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
    from PySide6.QtCore import QCoreApplication, QProcess, QProcessEnvironment, Qt
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QFileDialog,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPlainTextEdit,
        QPushButton,
        QSizePolicy,
        QSpacerItem,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    print(
        "PySide6 is required for production_ui/app.py.\n"
        "Install it in the active environment with:\n"
        "  python -m pip install -r production_ui/requirements.txt",
        file=sys.stderr,
    )
    raise SystemExit(1)

for plugin_root in QT_PLUGIN_ROOTS:
    QCoreApplication.addLibraryPath(str(plugin_root))


APP_DIR = Path(__file__).resolve().parent


def _is_frozen_app() -> bool:
    return bool(getattr(sys, "frozen", False))


def _frozen_executable() -> Path | None:
    if not _is_frozen_app():
        return None
    return Path(sys.executable).expanduser().resolve()


def _is_frozen_executable_path(path: Path) -> bool:
    frozen_executable = _frozen_executable()
    if frozen_executable is None:
        return False
    try:
        return path.expanduser().resolve() == frozen_executable
    except OSError:
        return False


def _default_repo_root() -> Path:
    candidates = [APP_DIR.parent, Path.cwd()]
    if _is_frozen_app():
        exe_dir = Path(sys.executable).resolve().parent
        candidates.extend([exe_dir, *exe_dir.parents])

    for candidate in candidates:
        if (candidate / "src/lerobot").exists() and (candidate / "production_ui").exists():
            return candidate
    return APP_DIR.parent


DEFAULT_REPO_ROOT = _default_repo_root()
DEFAULT_DATASET_ROOT = Path.home() / ".cache/huggingface/lerobot"
PYTHON_ENV_CURRENT = "Current Python"
PYTHON_ENV_CUSTOM = "Custom path"
PREFERRED_ENV_NAMES = ("xense-taccap", "lerobot-xense")
LANGUAGE_ZH = "zh"
LANGUAGE_EN = "en"
LANGUAGE_LABELS = {LANGUAGE_ZH: "中文", LANGUAGE_EN: "English"}
LANGUAGE_BY_LABEL = {label: code for code, label in LANGUAGE_LABELS.items()}

TEXT: dict[str, dict[str, str]] = {
    "window_title": {"zh": "XENSE-TACCAP", "en": "XENSE-TACCAP"},
    "app_title": {"zh": "XENSE-TACCAP", "en": "XENSE-TACCAP"},
    "language": {"zh": "语言", "en": "Language"},
    "environment": {"zh": "环境", "en": "Environment"},
    "repo_root": {"zh": "仓库路径", "en": "Repo root"},
    "python_env": {"zh": "Python 环境", "en": "Python env"},
    "python": {"zh": "Python", "en": "Python"},
    "tab_display": {"zh": "显示", "en": "Display"},
    "tab_record": {"zh": "正式采集", "en": "Record"},
    "tab_check": {"zh": "数据检查", "en": "Check"},
    "tab_push": {"zh": "上传 Hub", "en": "Upload Hub"},
    "tab_config": {"zh": "配置", "en": "Config"},
    "robot": {"zh": "机器人", "en": "Robot"},
    "robot_type": {"zh": "机器人类型", "en": "Robot type"},
    "display_smoke": {"zh": "显示 / 冒烟测试", "en": "Display / Smoke Test"},
    "fps": {"zh": "帧率", "en": "FPS"},
    "tracker_timeout": {"zh": "Tracker 超时", "en": "Tracker timeout"},
    "display_hint": {"zh": "teleoperate 显示预览，不写数据集。", "en": "teleoperate preview only; no dataset is written."},
    "run_display": {"zh": "运行显示", "en": "Run Display"},
    "scan_devices": {"zh": "扫描设备", "en": "Scan Devices"},
    "display_action_hint": {"zh": "显示预览，不写数据", "en": "Preview display without writing data"},
    "device_status": {"zh": "设备 SN 状态", "en": "Device SN Status"},
    "device_side": {"zh": "侧别", "en": "Side"},
    "device_gripper_fw": {"zh": "夹爪 FW", "en": "Gripper FW"},
    "device_mcu": {"zh": "MCU", "en": "MCU"},
    "device_tactile_l": {"zh": "左触觉", "en": "Tactile L"},
    "device_tactile_r": {"zh": "右触觉", "en": "Tactile R"},
    "device_wrist_cam": {"zh": "腕部相机", "en": "Wrist Cam"},
    "device_tracker": {"zh": "Tracker", "en": "Tracker"},
    "left_gripper": {"zh": "左爪", "en": "Left Gripper"},
    "right_gripper": {"zh": "右爪", "en": "Right Gripper"},
    "device_item_gripper_fw": {"zh": "夹爪固件", "en": "Gripper FW"},
    "device_item_mcu": {"zh": "MCU 序列号", "en": "MCU Serial"},
    "device_item_tactile_left": {"zh": "左指触觉", "en": "Left Finger Tactile"},
    "device_item_tactile_right": {"zh": "右指触觉", "en": "Right Finger Tactile"},
    "device_item_wrist_camera": {"zh": "腕部相机", "en": "Wrist Camera"},
    "device_item_tracker": {"zh": "Tracker", "en": "Tracker"},
    "side_left": {"zh": "左", "en": "left"},
    "side_right": {"zh": "右", "en": "right"},
    "scan_not_run": {"zh": "未扫描", "en": "Not scanned"},
    "device_pending": {"zh": "待扫描", "en": "Pending scan"},
    "device_scanning": {"zh": "扫描中", "en": "Scanning"},
    "dataset": {"zh": "数据集", "en": "Dataset"},
    "dataset_repo_id": {"zh": "数据集 repo_id", "en": "Dataset repo_id"},
    "task": {"zh": "任务", "en": "Task"},
    "episodes": {"zh": "集数", "en": "Episodes"},
    "dataset_fps": {"zh": "数据集帧率", "en": "Dataset FPS"},
    "episode_seconds": {"zh": "单集秒数", "en": "Episode seconds"},
    "reset_seconds": {"zh": "复位秒数", "en": "Reset seconds"},
    "dataset_root": {"zh": "数据集根目录", "en": "Dataset root"},
    "encoding_save": {"zh": "编码与保存", "en": "Encoding and Save"},
    "encoder_threads": {"zh": "编码线程", "en": "Encoder threads"},
    "encoder_queue": {"zh": "编码队列", "en": "Encoder queue"},
    "video_codec": {"zh": "视频编码", "en": "Video codec"},
    "record_extra_args": {"zh": "采集额外参数", "en": "Record extra args"},
    "run_record": {"zh": "运行采集", "en": "Run Record"},
    "record_action_hint": {"zh": "正式采集，写数据集", "en": "Record episodes and write dataset"},
    "dataset_check": {"zh": "数据集完整性检查", "en": "Dataset Integrity Check"},
    "root": {"zh": "根目录", "en": "Root"},
    "episode_index": {"zh": "Episode 索引", "en": "Episode index"},
    "episode_index_hint": {"zh": "Episode 索引支持：留空、'0 1 2' 或 '0,1,2'。", "en": "Episode index accepts: blank, '0 1 2', or '0,1,2'."},
    "run_check": {"zh": "运行检查", "en": "Run Check"},
    "check_action_hint": {"zh": "检查本地数据集", "en": "Check the local dataset"},
    "hub_target": {"zh": "Hub 目标", "en": "Hub Target"},
    "hub_repo_id": {"zh": "Hub repo_id", "en": "Hub repo_id"},
    "dataset_path": {"zh": "数据集路径", "en": "Dataset path"},
    "branch": {"zh": "分支", "en": "Branch"},
    "upload_options": {"zh": "上传选项", "en": "Upload Options"},
    "tags": {"zh": "标签", "en": "Tags"},
    "license": {"zh": "许可证", "en": "License"},
    "run_push": {"zh": "运行上传", "en": "Run Push"},
    "push_action_hint": {"zh": "上传前先 login", "en": "Log in before uploading"},
    "ui_config": {"zh": "UI 配置", "en": "UI Configuration"},
    "config_intro": {"zh": "配置可保存为 JSON，方便按工位、任务、操作者复用；也可加载 ui_config.example.json 后调整。", "en": "Save configuration as JSON for stations, tasks, and operators; you can also load ui_config.example.json and adjust it."},
    "load_config": {"zh": "加载配置", "en": "Load Config"},
    "save_config": {"zh": "保存配置", "en": "Save Config"},
    "save_config_as": {"zh": "另存配置...", "en": "Save Config As..."},
    "command_preview": {"zh": "命令预览", "en": "Command Preview"},
    "command_preview_empty": {"zh": "选择参数后自动生成命令。", "en": "A command is generated automatically after choosing parameters."},
    "command_preview_error": {"zh": "命令参数错误: {error}", "en": "Command argument error: {error}"},
    "browse": {"zh": "浏览", "en": "Browse"},
    "check_include_side": {"zh": "使用侧别", "en": "Use side"},
    "check_include_role": {"zh": "使用角色", "en": "Use role"},
    "check_enable_tracker": {"zh": "启用 Tracker", "en": "Tracker on"},
    "check_display": {"zh": "显示", "en": "Display"},
    "check_trajectory": {"zh": "轨迹", "en": "Trajectory"},
    "check_streaming": {"zh": "流式编码", "en": "Streaming"},
    "check_auto_push": {"zh": "自动上传", "en": "Auto push"},
    "check_private": {"zh": "私有", "en": "Private"},
    "check_resume": {"zh": "续采", "en": "Resume"},
    "check_no_videos": {"zh": "不上传视频", "en": "No videos"},
    "check_large_folder": {"zh": "大文件夹", "en": "Large folder"},
    "check_no_version_tag": {"zh": "不打版本标签", "en": "No version tag"},
    "python_env_tooltip": {"zh": "选择采集命令使用的 Python 环境；默认使用 mamba 环境 xense-taccap。", "en": "Choose the Python environment for collection commands; xense-taccap is the default mamba environment."},
    "include_side_tooltip": {"zh": "taccap_gripper 单手模式需要指定 left/right；bi_taccap_gripper 不使用 side。", "en": "Single-hand taccap_gripper requires left/right; bi_taccap_gripper does not use side."},
    "side_tooltip": {"zh": "单手模式选择 left 或 right。", "en": "Choose left or right for single-hand mode."},
    "load_config_dialog": {"zh": "加载 UI 配置", "en": "Load UI config"},
    "save_config_dialog": {"zh": "保存 UI 配置", "en": "Save UI config"},
    "json_filter": {"zh": "JSON 文件 (*.json);;所有文件 (*)", "en": "JSON files (*.json);;All files (*)"},
    "load_config_failed": {"zh": "加载配置失败", "en": "Load config failed"},
    "save_config_failed": {"zh": "保存配置失败", "en": "Save config failed"},
    "choose_directory": {"zh": "选择目录", "en": "Choose directory"},
    "choose_file": {"zh": "选择文件", "en": "Choose file"},
    "scan_running_title": {"zh": "扫描运行中", "en": "Scan running"},
    "scan_running_text": {"zh": "设备扫描正在运行。", "en": "Device scan is already running."},
    "language_switch_blocked": {"zh": "设备扫描运行中，结束后再切换语言。", "en": "A device scan is running. Switch language after it finishes."},
    "scan_scanning": {"zh": "扫描中...", "en": "Scanning..."},
    "scan_parse_failed": {"zh": "扫描失败: {error}", "en": "Scan failed: {error}"},
    "scan_process_error": {"zh": "扫描进程错误: {error}", "en": "Scan process error: {error}"},
    "scan_done_warnings": {"zh": "扫描完成，有告警: ", "en": "Scan complete with warnings: "},
    "scan_done_exit": {"zh": "扫描完成，退出码 {code}", "en": "Scan complete, exit code {code}"},
    "python_not_found": {"zh": "Python 不存在", "en": "Python not found"},
    "python_not_found_text": {"zh": "当前选择的 Python 不存在:\n{python}\n\n请在顶部 Python 环境选择已安装环境，或用浏览指定解释器。", "en": "The selected Python does not exist:\n{python}\n\nChoose an installed environment in Python env, or use Browse to select an interpreter."},
    "python_invalid": {"zh": "Python 选择错误", "en": "Invalid Python"},
    "python_frozen_self_text": {"zh": "当前选择的是 production_ui 打包程序，不是 Python 解释器:\n{python}\n\n请在顶部 Python 环境选择 xense-taccap，或用浏览指定真实的 python 文件。", "en": "The selected path is the packaged production_ui executable, not a Python interpreter:\n{python}\n\nChoose xense-taccap in Python env, or browse to a real python executable."},
    "invalid_repo_root": {"zh": "仓库路径无效", "en": "Invalid repo root"},
    "invalid_repo_root_text": {"zh": "仓库路径不存在:\n{repo_root}", "en": "Repo root does not exist:\n{repo_root}"},
    "terminal_launch_failed": {"zh": "终端启动失败", "en": "Terminal launch failed"},
    "terminal_not_found": {"zh": "没有找到可用终端。请安装 gnome-terminal、konsole、xfce4-terminal、xterm 或 x-terminal-emulator。", "en": "No supported terminal was found. Install gnome-terminal, konsole, xfce4-terminal, xterm, or x-terminal-emulator."},
    "terminal_press_enter": {"zh": "按 Enter 关闭此终端...", "en": "Press Enter to close this terminal..."},
    "stop_terminating": {"zh": "[stop] 正在终止设备扫描...", "en": "[stop] terminating device scan..."},
    "stop_killing": {"zh": "[stop] 扫描进程未退出，正在 kill。", "en": "[stop] scan process did not exit; killing it."},
}


def _unique_paths(paths: list[Path]) -> list[Path]:
    result: list[Path] = []
    seen: set[str] = set()
    for path in paths:
        key = str(path)
        if key not in seen:
            seen.add(key)
            result.append(path)
    return result


def _conda_env_dirs() -> list[Path]:
    candidates: list[Path] = []
    prefixes = [Path(sys.prefix)]
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        prefixes.append(Path(conda_prefix))

    for prefix in prefixes:
        if prefix.parent.name == "envs":
            candidates.append(prefix.parent)
        candidates.append(prefix / "envs")

    conda_exe = os.environ.get("CONDA_EXE")
    if conda_exe:
        candidates.append(Path(conda_exe).resolve().parent.parent / "envs")

    home = Path.home()
    for root_name in ("miniforge3", "miniconda3", "mambaforge", "anaconda3"):
        candidates.append(home / root_name / "envs")

    return _unique_paths(candidates)


def _python_for_named_env(env_name: str) -> str:
    env_dirs = _conda_env_dirs()
    for env_dir in env_dirs:
        python = env_dir / env_name / "bin" / "python"
        if python.exists():
            return str(python)
    if env_dirs:
        return str(env_dirs[0] / env_name / "bin" / "python")
    return ""


def _python_env_paths() -> dict[str, str]:
    paths = {}
    if not _is_frozen_app():
        paths[PYTHON_ENV_CURRENT] = sys.executable
    for env_name in PREFERRED_ENV_NAMES:
        paths[env_name] = _python_for_named_env(env_name)
    paths[PYTHON_ENV_CUSTOM] = ""
    return paths


def _default_python_env() -> str:
    paths = _python_env_paths()
    for env_name in PREFERRED_ENV_NAMES:
        python = paths.get(env_name, "")
        if python and Path(python).exists():
            return env_name
    if not _is_frozen_app():
        return PYTHON_ENV_CURRENT
    return PYTHON_ENV_CUSTOM


def _default_python_executable() -> str:
    paths = _python_env_paths()
    python = paths.get(_default_python_env(), "")
    if python:
        return python
    return "" if _is_frozen_app() else sys.executable


DEFAULTS: dict[str, Any] = {
    "repo_root": str(DEFAULT_REPO_ROOT),
    "python_env": _default_python_env(),
    "python_executable": _default_python_executable(),
    "robot_type": "bi_taccap_gripper",
    "side": "left",
    "include_side": False,
    "role": "leader",
    "include_role": False,
    "enable_tracker": False,
    "teleop_display_data": True,
    "teleop_show_trajectory": False,
    "record_display_data": True,
    "record_show_trajectory": False,
    "teleop_fps": "30",
    "repo_id": "Xense/taccap_smoke_test",
    "single_task": "Pick up the cube",
    "num_episodes": "2",
    "dataset_fps": "30",
    "episode_time_s": "10",
    "reset_time_s": "5",
    "dataset_root": "",
    "streaming_encoding": True,
    "encoder_threads": "2",
    "encoder_queue_maxsize": "30",
    "vcodec": "auto",
    "record_push_to_hub": False,
    "private": False,
    "resume": False,
    "record_extra_args": "",
    "check_repo_id": "Xense/taccap_smoke_test",
    "check_root": str(DEFAULT_DATASET_ROOT),
    "check_episode_index": "",
    "push_repo_id": "Xense/taccap_smoke_test",
    "push_dataset_path": str(DEFAULT_DATASET_ROOT / "Xense/taccap_smoke_test"),
    "push_branch": "",
    "push_tags": "",
    "push_license": "apache-2.0",
    "push_private": False,
    "push_no_videos": False,
    "push_upload_large_folder": True,
    "push_no_tag_version": False,
    "scan_tracker_timeout": "2.0",
}


def bool_arg(value: bool) -> str:
    return "true" if value else "false"


def shell_join(cmd: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in cmd)


def split_extra_args(raw: str) -> list[str]:
    raw = raw.strip()
    return shlex.split(raw) if raw else []


class ProductionWindow(QMainWindow):
    DEVICE_VALUE_WIDTH_SAMPLE = "PC2310MLL3200496G"
    DEVICE_VALUE_FIXED_WIDTH = 170
    WINDOW_SIZE = (980, 535)
    DISPLAY_TABS_HEIGHT = 310
    COMPACT_TABS_HEIGHT = 215
    DISPLAY_COMMAND_PREVIEW_CARD_HEIGHT = 88
    DISPLAY_COMMAND_PREVIEW_HEIGHT = 54
    LARGE_COMMAND_PREVIEW_CARD_HEIGHT = 183
    LARGE_COMMAND_PREVIEW_HEIGHT = 149

    COMPACT_LINE_FIELDS = {
        "teleop_fps",
        "num_episodes",
        "dataset_fps",
        "episode_time_s",
        "reset_time_s",
        "encoder_threads",
        "encoder_queue_maxsize",
        "scan_tracker_timeout",
    }
    MEDIUM_LINE_FIELDS = {
        "check_episode_index",
        "push_branch",
        "push_license",
        "vcodec",
    }
    COMPACT_COMBO_FIELDS = {"side", "role"}

    def __init__(self) -> None:
        super().__init__()
        self.language = LANGUAGE_ZH
        self.setWindowTitle(self._t("window_title"))
        self.setFixedSize(*self.WINDOW_SIZE)

        self.fields: dict[str, QWidget] = {}
        self.device_labels: dict[str, QLabel] = {}
        self.python_env_paths = _python_env_paths()
        self.config_path = APP_DIR / "ui_config.json"
        self.scan_process: QProcess | None = None
        self._applying_config = False

        self._build_ui()
        self._apply_config(DEFAULTS)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _t(self, key: str) -> str:
        values = TEXT.get(key, {})
        return values.get(self.language, values.get(LANGUAGE_EN, key))

    def on_language_changed(self, label: str) -> None:
        language = LANGUAGE_BY_LABEL.get(label, LANGUAGE_ZH)
        if language == self.language:
            return
        if self.scan_process is not None and self.scan_process.state() != QProcess.NotRunning:
            self.language_combo.blockSignals(True)
            self.language_combo.setCurrentText(LANGUAGE_LABELS[self.language])
            self.language_combo.blockSignals(False)
            QMessageBox.information(self, self._t("scan_running_title"), self._t("language_switch_blocked"))
            return

        config = self._config_dict()
        device_values = {name: self._scanned_device_value(name) for name in self.device_labels}
        current_tab_index = self.tabs.currentIndex() if hasattr(self, "tabs") else 0

        self.language = language
        self._build_ui()
        if hasattr(self, "tabs") and 0 <= current_tab_index < self.tabs.count():
            self.tabs.setCurrentIndex(current_tab_index)
        for name, text in device_values.items():
            label = self.device_labels.get(name)
            if label is not None and text:
                self._set_device_label(label, text, "ok")
        self._apply_config(config)

    def on_tab_changed(self, _index: int) -> None:
        self._apply_preview_geometry()
        self.refresh_command_preview()

    def _apply_preview_geometry(self) -> None:
        if not hasattr(self, "tabs") or not hasattr(self, "command_preview_card"):
            return
        display_tab = self.tabs.currentIndex() == 0
        tabs_height = self.DISPLAY_TABS_HEIGHT if display_tab else self.COMPACT_TABS_HEIGHT
        preview_card_height = (
            self.DISPLAY_COMMAND_PREVIEW_CARD_HEIGHT if display_tab else self.LARGE_COMMAND_PREVIEW_CARD_HEIGHT
        )
        preview_height = self.DISPLAY_COMMAND_PREVIEW_HEIGHT if display_tab else self.LARGE_COMMAND_PREVIEW_HEIGHT

        self.tabs.setFixedHeight(tabs_height)
        self.command_preview_card.setFixedHeight(preview_card_height)
        self.command_preview.setFixedHeight(preview_height)

    def _build_ui(self) -> None:
        self.fields = {}
        self.device_labels = {}
        self.python_env_paths = _python_env_paths()
        self.setWindowTitle(self._t("window_title"))
        self.setStyleSheet(self._stylesheet())

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(4)
        self.setCentralWidget(root)

        root_layout.addWidget(self._build_header())
        root_layout.addWidget(self._build_robot_panel())

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.setElideMode(Qt.ElideNone)
        self.tabs.tabBar().setExpanding(True)
        self.tabs.tabBar().setUsesScrollButtons(False)
        self.tabs.addTab(self._build_display_page(), self._t("tab_display"))
        self.tabs.addTab(self._build_record_page(), self._t("tab_record"))
        self.tabs.addTab(self._build_check_page(), self._t("tab_check"))
        self.tabs.addTab(self._build_push_page(), self._t("tab_push"))
        self.tabs.addTab(self._build_config_page(), self._t("tab_config"))
        self.tabs.setFixedHeight(self.DISPLAY_TABS_HEIGHT)
        self.tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        root_layout.addWidget(self.tabs, 0)
        root_layout.addWidget(self._build_output_panel(), 0)
        self.tabs.currentChanged.connect(self.on_tab_changed)
        self._connect_auto_preview_signals()
        self._apply_preview_geometry()
        root_layout.addStretch(1)

    def _build_header(self) -> QWidget:
        card = self._card(self._t("environment"))
        layout = QGridLayout(card)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(3)
        layout.setColumnMinimumWidth(0, 68)
        layout.setColumnMinimumWidth(2, 50)
        layout.setColumnMinimumWidth(3, 56)
        layout.setColumnMinimumWidth(4, 118)
        layout.setColumnMinimumWidth(5, 50)
        layout.setColumnMinimumWidth(7, 54)
        layout.setColumnStretch(1, 5)
        layout.setColumnStretch(6, 5)

        layout.addWidget(self._label(self._t("repo_root")), 0, 0)
        self._line_edit("repo_root")
        layout.addWidget(self.fields["repo_root"], 0, 1)
        layout.addWidget(self._browse_button("repo_root", directory=True), 0, 2)

        layout.addWidget(self._label(self._t("language")), 0, 3)
        self.language_combo = QComboBox()
        self.language_combo.addItems([LANGUAGE_LABELS[LANGUAGE_ZH], LANGUAGE_LABELS[LANGUAGE_EN]])
        self.language_combo.setCurrentText(LANGUAGE_LABELS.get(self.language, LANGUAGE_LABELS[LANGUAGE_ZH]))
        self.language_combo.setMinimumHeight(24)
        self.language_combo.setMaximumWidth(96)
        self.language_combo.currentTextChanged.connect(self.on_language_changed)
        layout.addWidget(self.language_combo, 0, 4)

        layout.addWidget(self._label(self._t("python_env")), 1, 0)
        self._combo("python_env", list(self.python_env_paths.keys()))
        env_combo = self.fields["python_env"]
        if isinstance(env_combo, QComboBox):
            env_combo.currentTextChanged.connect(self.on_python_env_changed)
        layout.addWidget(env_combo, 1, 1)

        layout.addWidget(self._label(self._t("python")), 1, 3)
        self._line_edit("python_executable")
        python_field = self.fields["python_executable"]
        if isinstance(python_field, QLineEdit):
            python_field.editingFinished.connect(self._sync_python_env_from_path)
        layout.addWidget(self.fields["python_executable"], 1, 4, 1, 3)
        layout.addWidget(self._browse_button("python_executable", directory=False), 1, 7)
        return card

    def _build_robot_panel(self) -> QWidget:
        robot = self._card(self._t("robot"))
        grid = QGridLayout(robot)
        grid.setContentsMargins(8, 3, 8, 3)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(0)
        grid.setColumnMinimumWidth(0, 72)
        grid.setColumnMinimumWidth(2, 68)
        grid.setColumnMinimumWidth(4, 68)
        grid.setColumnMinimumWidth(6, 78)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        grid.setColumnStretch(5, 1)
        grid.setColumnStretch(7, 1)

        for col in (1, 3, 5, 7):
            grid.setColumnMinimumWidth(col, 130)

        self._combo("robot_type", ["taccap_gripper", "bi_taccap_gripper"])
        self._combo("side", ["left", "right", ""])
        self._combo("role", ["leader", "follower"])
        robot_type = self.fields["robot_type"]
        if isinstance(robot_type, QComboBox):
            robot_type.currentTextChanged.connect(self._sync_robot_type_options)

        grid.addWidget(self._label(self._t("robot_type")), 0, 0)
        grid.addWidget(self.fields["robot_type"], 0, 1)
        grid.addWidget(self._check("include_side"), 0, 2)
        grid.addWidget(self.fields["side"], 0, 3)
        grid.addWidget(self._check("include_role"), 0, 4)
        grid.addWidget(self.fields["role"], 0, 5)
        grid.addWidget(self._check("enable_tracker"), 0, 6, 1, 2)
        return robot

    def _build_display_page(self) -> QWidget:
        page = QWidget()
        page.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QGridLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(6)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 2)

        display = self._group(self._t("display_smoke"))
        display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid = self._grid(display)
        grid.setContentsMargins(8, 10, 8, 8)
        grid.setVerticalSpacing(5)
        grid.setColumnMinimumWidth(0, 70 if self.language == LANGUAGE_ZH else 110)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 0)
        self._line_edit("teleop_fps")
        self._line_edit("scan_tracker_timeout")
        for field_name in ("teleop_fps", "scan_tracker_timeout"):
            self.fields[field_name].setMaximumWidth(16777215)
            self.fields[field_name].setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        grid.addWidget(self._label(self._t("fps")), 0, 0)
        grid.addWidget(self.fields["teleop_fps"], 0, 1)
        grid.addWidget(self._label(self._t("tracker_timeout")), 1, 0)
        grid.addWidget(self.fields["scan_tracker_timeout"], 1, 1)
        self._add_checks(grid, 2, ["teleop_display_data", "teleop_show_trajectory"])
        hint = QLabel(self._t("display_hint"))
        hint.setObjectName("Hint")
        grid.addWidget(hint, 3, 0, 1, 2)

        layout.addWidget(display, 0, 0)
        layout.addWidget(self._build_device_panel(), 0, 1)
        layout.addWidget(
            self._action_bar(
                [
                    (self._t("run_display"), lambda: self.run(self.build_teleoperate_command()), "primary"),
                    (self._t("scan_devices"), self.scan_devices),
                ],
                self._t("display_action_hint"),
            ),
            1,
            0,
            1,
            2,
        )
        self._pin_page_top(layout, 2, 2)
        return page

    def _build_device_panel(self) -> QWidget:
        group = self._group(self._t("device_status"))
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid = QGridLayout(group)
        grid.setContentsMargins(6, 8, 6, 6)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setColumnMinimumWidth(0, 0)
        grid.setColumnMinimumWidth(1, 0)

        for col, side in enumerate(("left", "right")):
            grid.addWidget(self._build_device_side_panel(side), 0, col)

        self.scan_status = QLabel(self._t("scan_not_run"))
        self.scan_status.setObjectName("Hint")
        grid.addWidget(self.scan_status, 1, 0, 1, 2)
        return group

    def _build_device_side_panel(self, side: str) -> QWidget:
        panel = QFrame()
        panel.setObjectName("DeviceSidePanel")
        panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        layout = QGridLayout(panel)
        layout.setContentsMargins(6, 5, 6, 5)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(3)
        item_name_width = self._device_item_name_width()
        value_width = self._device_value_width(panel)
        layout.setColumnMinimumWidth(0, item_name_width)
        layout.setColumnMinimumWidth(1, value_width)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        title = QLabel(self._t("left_gripper") if side == "left" else self._t("right_gripper"))
        title.setObjectName("DeviceSideTitle")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title, 0, 0, 1, 2)

        rows = [
            ("device_item_gripper_fw", "gripper_fw"),
            ("device_item_mcu", "mcu_serial"),
            ("device_item_tactile_left", "tactile_left"),
            ("device_item_tactile_right", "tactile_right"),
            ("device_item_wrist_camera", "wrist_camera"),
            ("device_item_tracker", "tracker"),
        ]
        for row in range(1, 7):
            layout.setRowMinimumHeight(row, 22)

        for row, (text_key, value_key) in enumerate(rows, start=1):
            name = QLabel(self._t(text_key))
            name.setObjectName("DeviceItemName")
            name.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            name.setFixedWidth(item_name_width)
            value = QLabel("-")
            value.setObjectName("DeviceValue")
            value.setTextFormat(Qt.PlainText)
            value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setMinimumHeight(22)
            value.setMinimumWidth(value_width)
            value.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self._set_device_label(value, self._t("device_pending"), "pending")
            self.device_labels[f"{side}_{value_key}"] = value
            layout.addWidget(name, row, 0)
            layout.addWidget(value, row, 1)
        return panel

    def _build_record_page(self) -> QWidget:
        page = QWidget()
        page.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QGridLayout(page)
        layout.setContentsMargins(4, 3, 4, 2)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

        dataset = self._group(self._t("dataset"))
        dataset.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid = self._grid(dataset)
        for name in (
            "repo_id",
            "single_task",
            "num_episodes",
            "dataset_fps",
            "episode_time_s",
            "reset_time_s",
            "dataset_root",
        ):
            self._line_edit(name)
        self._add_row(grid, 0, self._t("dataset_repo_id"), "repo_id", self._t("task"), "single_task")
        self._add_row(grid, 1, self._t("episodes"), "num_episodes", self._t("dataset_fps"), "dataset_fps")
        self._add_row(grid, 2, self._t("episode_seconds"), "episode_time_s", self._t("reset_seconds"), "reset_time_s")
        self._add_path_row(grid, 3, self._t("dataset_root"), "dataset_root")

        encoding = self._group(self._t("encoding_save"))
        encoding.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid = self._grid(encoding)
        self._line_edit("encoder_threads")
        self._line_edit("encoder_queue_maxsize")
        self._line_edit("vcodec")
        self._line_edit("record_extra_args")
        self._add_checks(grid, 0, ["record_display_data", "record_show_trajectory"])
        self._add_checks(
            grid,
            1,
            ["streaming_encoding", "record_push_to_hub", "private", "resume"],
        )
        self._add_row(grid, 2, self._t("encoder_threads"), "encoder_threads", self._t("encoder_queue"), "encoder_queue_maxsize")
        self._add_row(grid, 3, self._t("video_codec"), "vcodec", self._t("record_extra_args"), "record_extra_args")

        layout.addWidget(dataset, 0, 0)
        layout.addWidget(encoding, 0, 1)
        layout.addWidget(
            self._action_bar(
                [
                    (self._t("run_record"), lambda: self.run(self.build_record_command()), "primary"),
                ],
                self._t("record_action_hint"),
            ),
            1,
            0,
            1,
            2,
        )
        return page

    def _build_check_page(self) -> QWidget:
        page = QWidget()
        page.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QGridLayout(page)
        layout.setContentsMargins(4, 3, 4, 2)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)
        layout.setColumnStretch(0, 1)

        group = self._group(self._t("dataset_check"))
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid = self._grid(group)
        self._line_edit("check_repo_id")
        self._line_edit("check_root")
        self._line_edit("check_episode_index")
        self._add_single_row(grid, 0, self._t("dataset_repo_id"), "check_repo_id")
        self._add_path_row(grid, 1, self._t("root"), "check_root")
        self._add_single_row(grid, 2, self._t("episode_index"), "check_episode_index")
        hint = QLabel(self._t("episode_index_hint"))
        hint.setObjectName("Hint")
        grid.addWidget(hint, 3, 1, 1, 3)

        layout.addWidget(group, 0, 0)
        layout.addWidget(
            self._action_bar(
                [
                    (self._t("run_check"), lambda: self.run(self.build_check_command()), "primary"),
                ],
                self._t("check_action_hint"),
            ),
            1,
            0,
        )
        return page

    def _build_push_page(self) -> QWidget:
        page = QWidget()
        page.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QGridLayout(page)
        layout.setContentsMargins(4, 3, 4, 2)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)
        layout.setColumnStretch(0, 3)
        layout.setColumnStretch(1, 2)

        target = self._group(self._t("hub_target"))
        target.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid = self._grid(target)
        self._line_edit("push_repo_id")
        self._line_edit("push_dataset_path")
        self._line_edit("push_branch")
        self._add_single_row(grid, 0, self._t("hub_repo_id"), "push_repo_id")
        self._add_path_row(grid, 1, self._t("dataset_path"), "push_dataset_path")
        self._add_single_row(grid, 2, self._t("branch"), "push_branch")

        options = self._group(self._t("upload_options"))
        options.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid = self._grid(options)
        self._line_edit("push_tags")
        self._line_edit("push_license")
        self._add_row(grid, 0, self._t("tags"), "push_tags", self._t("license"), "push_license")
        self._add_checks(
            grid,
            1,
            ["push_private", "push_no_videos", "push_upload_large_folder", "push_no_tag_version"],
        )

        layout.addWidget(target, 0, 0)
        layout.addWidget(options, 0, 1)
        layout.addWidget(
            self._action_bar(
                [
                    (self._t("run_push"), lambda: self.run(self.build_push_command()), "primary"),
                ],
                self._t("push_action_hint"),
            ),
            1,
            0,
            1,
            2,
        )
        return page

    def _build_config_page(self) -> QWidget:
        page = QWidget()
        page.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QGridLayout(page)
        layout.setContentsMargins(4, 3, 4, 2)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)
        layout.setColumnStretch(0, 1)

        group = self._group(self._t("ui_config"))
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        group_layout = QVBoxLayout(group)
        group_layout.setContentsMargins(8, 9, 8, 7)
        group_layout.setSpacing(6)
        intro = QLabel(self._t("config_intro"))
        intro.setObjectName("Hint")
        group_layout.addWidget(intro)

        row = QHBoxLayout()
        row.setSpacing(6)
        load = QPushButton(self._t("load_config"))
        load.clicked.connect(self.load_config)
        save = QPushButton(self._t("save_config"))
        save.clicked.connect(self.save_config)
        save_as = QPushButton(self._t("save_config_as"))
        save_as.clicked.connect(self.save_config_as)
        for button in (load, save, save_as):
            button.setMinimumHeight(24)
        row.addWidget(load)
        row.addWidget(save)
        row.addWidget(save_as)
        row.addStretch(1)
        group_layout.addLayout(row)

        self.config_path_edit = QLineEdit(str(self.config_path))
        self.config_path_edit.setMinimumHeight(28)
        group_layout.addWidget(self.config_path_edit)

        layout.addWidget(group, 0, 0)
        return page

    def _build_output_panel(self) -> QWidget:
        self.command_preview_card = self._card(self._t("command_preview"))
        self.command_preview_card.setFixedHeight(self.DISPLAY_COMMAND_PREVIEW_CARD_HEIGHT)
        self.command_preview_card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        preview_layout = QVBoxLayout(self.command_preview_card)
        preview_layout.setContentsMargins(7, 4, 7, 4)
        preview_layout.setSpacing(3)
        preview_title = QLabel(self._t("command_preview"))
        preview_title.setObjectName("CardTitle")
        preview_layout.addWidget(preview_title)
        self.command_preview = QPlainTextEdit()
        self.command_preview.setReadOnly(True)
        self.command_preview.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.command_preview.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.command_preview.setFixedHeight(self.DISPLAY_COMMAND_PREVIEW_HEIGHT)
        self.command_preview.setPlainText(self._t("command_preview_empty"))
        preview_layout.addWidget(self.command_preview)
        return self.command_preview_card

    # ------------------------------------------------------------------
    # Small UI helpers
    # ------------------------------------------------------------------

    def _card(self, title: str | None = None) -> QFrame:
        frame = QFrame()
        frame.setObjectName("Card")
        frame.setFrameShape(QFrame.NoFrame)
        if title:
            frame.setProperty("title", title)
        return frame

    def _group(self, title: str) -> QGroupBox:
        group = QGroupBox(title)
        group.setObjectName("Group")
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        return group

    def _grid(self, parent: QWidget) -> QGridLayout:
        grid = QGridLayout(parent)
        grid.setContentsMargins(8, 9, 8, 7)
        grid.setHorizontalSpacing(7)
        grid.setVerticalSpacing(5)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        return grid

    def _label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("FieldLabel")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        label.setMinimumWidth(56)
        return label

    def _device_item_name_width(self) -> int:
        return 70 if self.language == LANGUAGE_ZH else 110

    def _device_value_width(self, widget: QWidget) -> int:
        metrics = widget.fontMetrics()
        return max(self.DEVICE_VALUE_FIXED_WIDTH, metrics.horizontalAdvance(self.DEVICE_VALUE_WIDTH_SAMPLE) + 16)

    def _line_edit(self, name: str) -> QLineEdit:
        widget = QLineEdit()
        widget.setMinimumHeight(24)
        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        if name in self.COMPACT_LINE_FIELDS:
            widget.setMaximumWidth(92)
            widget.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        elif name in self.MEDIUM_LINE_FIELDS:
            widget.setMaximumWidth(180)
            widget.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.fields[name] = widget
        return widget

    def _combo(self, name: str, values: list[str]) -> QComboBox:
        widget = QComboBox()
        widget.addItems(values)
        widget.setMinimumHeight(24)
        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        if name in self.COMPACT_COMBO_FIELDS:
            widget.setMaximumWidth(130)
            widget.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        elif name == "robot_type":
            widget.setMaximumWidth(230)
        elif name == "python_env":
            widget.setMaximumWidth(150)
            widget.setToolTip(self._t("python_env_tooltip"))
        self.fields[name] = widget
        return widget

    def _check(self, name: str, label: str | None = None) -> QCheckBox:
        widget = QCheckBox(label or self._checkbox_label(name))
        widget.setMinimumHeight(24)
        self.fields[name] = widget
        return widget

    def _checkbox_label(self, name: str) -> str:
        labels = {
            "include_side": self._t("check_include_side"),
            "include_role": self._t("check_include_role"),
            "enable_tracker": self._t("check_enable_tracker"),
            "teleop_display_data": self._t("check_display"),
            "teleop_show_trajectory": self._t("check_trajectory"),
            "record_display_data": self._t("check_display"),
            "record_show_trajectory": self._t("check_trajectory"),
            "streaming_encoding": self._t("check_streaming"),
            "record_push_to_hub": self._t("check_auto_push"),
            "private": self._t("check_private"),
            "resume": self._t("check_resume"),
            "push_private": self._t("check_private"),
            "push_no_videos": self._t("check_no_videos"),
            "push_upload_large_folder": self._t("check_large_folder"),
            "push_no_tag_version": self._t("check_no_version_tag"),
        }
        return labels.get(name, name)

    def _add_row(
        self,
        grid: QGridLayout,
        row: int,
        label_a: str,
        field_a: str,
        label_b: str,
        field_b: str,
    ) -> None:
        grid.addWidget(self._label(label_a), row, 0)
        grid.addWidget(self.fields[field_a], row, 1)
        grid.addWidget(self._label(label_b), row, 2)
        grid.addWidget(self.fields[field_b], row, 3)

    def _add_single_row(self, grid: QGridLayout, row: int, label: str, field: str) -> None:
        grid.addWidget(self._label(label), row, 0)
        grid.addWidget(self.fields[field], row, 1, 1, 3)

    def _add_path_row(self, grid: QGridLayout, row: int, label: str, field: str) -> None:
        grid.addWidget(self._label(label), row, 0)
        holder = QWidget()
        layout = QHBoxLayout(holder)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self.fields[field], 1)
        layout.addWidget(self._browse_button(field, directory=True))
        grid.addWidget(holder, row, 1, 1, 3)

    def _add_checks(self, grid: QGridLayout, row: int, names: list[str]) -> None:
        holder = QWidget()
        layout = QHBoxLayout(holder)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        for name in names:
            layout.addWidget(self._check(name))
        layout.addStretch(1)
        grid.addWidget(holder, row, 0, 1, 4)

    def _pin_page_top(self, layout: QGridLayout, spacer_row: int, column_span: int) -> None:
        layout.setRowStretch(spacer_row, 1)
        layout.addItem(
            QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding),
            spacer_row,
            0,
            1,
            column_span,
        )

    def _browse_button(self, field: str, directory: bool) -> QPushButton:
        button = QPushButton(self._t("browse"))
        button.setMaximumWidth(62)
        button.setMinimumHeight(24)
        button.clicked.connect(lambda: self.browse(field, directory=directory))
        return button

    def _action_bar(self, actions: list[tuple], hint: str) -> QWidget:
        bar = QFrame()
        bar.setObjectName("ActionBar")
        bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(7, 4, 7, 4)
        layout.setSpacing(6)
        for action in actions:
            text, callback = action[0], action[1]
            kind = action[2] if len(action) > 2 else ""
            button = QPushButton(text)
            button.setMinimumHeight(24)
            if kind == "primary":
                button.setObjectName("PrimaryButton")
            button.clicked.connect(callback)
            layout.addWidget(button)
        layout.addStretch(1)
        hint_label = QLabel(hint)
        hint_label.setObjectName("Hint")
        hint_label.setWordWrap(False)
        hint_label.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        layout.addWidget(hint_label)
        return bar

    def _connect_auto_preview_signals(self) -> None:
        for widget in self.fields.values():
            if isinstance(widget, QLineEdit):
                widget.textChanged.connect(lambda _text: self.refresh_command_preview())
            elif isinstance(widget, QComboBox):
                widget.currentTextChanged.connect(lambda _text: self.refresh_command_preview())
            elif isinstance(widget, QCheckBox):
                widget.toggled.connect(lambda _checked: self.refresh_command_preview())

    # ------------------------------------------------------------------
    # Config and field access
    # ------------------------------------------------------------------

    def value(self, name: str) -> str:
        widget = self.fields.get(name)
        if widget is None:
            return ""
        if isinstance(widget, QLineEdit):
            return widget.text().strip()
        if isinstance(widget, QComboBox):
            return widget.currentText().strip()
        if isinstance(widget, QCheckBox):
            return str(widget.isChecked())
        return ""

    def bool_value(self, name: str) -> bool:
        widget = self.fields.get(name)
        if widget is None:
            return False
        return bool(widget.isChecked()) if isinstance(widget, QCheckBox) else False

    def _config_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for name, widget in sorted(self.fields.items()):
            if isinstance(widget, QLineEdit):
                result[name] = widget.text()
            elif isinstance(widget, QComboBox):
                result[name] = widget.currentText()
            elif isinstance(widget, QCheckBox):
                result[name] = widget.isChecked()
        return result

    def _apply_config(self, config: dict[str, Any]) -> None:
        self._applying_config = True
        try:
            if "display_data" in config:
                config = {
                    "teleop_display_data": config["display_data"],
                    "record_display_data": config["display_data"],
                    **config,
                }
            if "show_trajectory" in config:
                config = {
                    "teleop_show_trajectory": config["show_trajectory"],
                    "record_show_trajectory": config["show_trajectory"],
                    **config,
                }
            for name, value in config.items():
                widget = self.fields.get(name)
                if widget is None:
                    continue
                if isinstance(widget, QLineEdit):
                    widget.setText("" if value is None else str(value))
                elif isinstance(widget, QComboBox):
                    text = "" if value is None else str(value)
                    idx = widget.findText(text)
                    if idx >= 0:
                        widget.setCurrentIndex(idx)
                elif isinstance(widget, QCheckBox):
                    if isinstance(value, str):
                        widget.setChecked(value.lower() in {"1", "true", "yes", "y", "on"})
                    else:
                        widget.setChecked(bool(value))
            self._sync_robot_type_options(self.value("robot_type"))
            self._repair_frozen_python_selection()
            self._sync_python_env_from_path()
        finally:
            self._applying_config = False
        self.refresh_command_preview()

    def on_python_env_changed(self, env_name: str) -> None:
        if env_name == PYTHON_ENV_CUSTOM:
            return
        python = self.python_env_paths.get(env_name, "")
        field = self.fields.get("python_executable")
        if python and isinstance(field, QLineEdit):
            field.setText(python)

    def _sync_python_env_from_path(self) -> None:
        combo = self.fields.get("python_env")
        python_field = self.fields.get("python_executable")
        if not isinstance(combo, QComboBox) or not isinstance(python_field, QLineEdit):
            return
        current_path = python_field.text().strip()
        ordered_env_names = [*PREFERRED_ENV_NAMES, PYTHON_ENV_CURRENT]
        for env_name in ordered_env_names:
            python = self.python_env_paths.get(env_name, "")
            if not python:
                continue
            if current_path == python:
                idx = combo.findText(env_name)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
                return
        if current_path:
            idx = combo.findText(PYTHON_ENV_CUSTOM)
            if idx >= 0:
                combo.setCurrentIndex(idx)

    def _repair_frozen_python_selection(self) -> None:
        if not _is_frozen_app():
            return
        python_field = self.fields.get("python_executable")
        if not isinstance(python_field, QLineEdit):
            return
        current_path = python_field.text().strip()
        if not current_path or not _is_frozen_executable_path(Path(current_path)):
            return

        fallback_env = _default_python_env()
        fallback_python = _default_python_executable()
        combo = self.fields.get("python_env")
        if fallback_python:
            python_field.setText(fallback_python)
        else:
            python_field.clear()
            fallback_env = PYTHON_ENV_CUSTOM
        if isinstance(combo, QComboBox):
            idx = combo.findText(fallback_env)
            if idx >= 0:
                combo.setCurrentIndex(idx)

    def _sync_robot_type_options(self, robot_type: str) -> None:
        include_side = self.fields.get("include_side")
        side = self.fields.get("side")
        single_mode = robot_type == "taccap_gripper"

        if isinstance(include_side, QCheckBox):
            include_side.setEnabled(single_mode)
            include_side.setChecked(single_mode)
            include_side.setToolTip(
                self._t("include_side_tooltip")
            )

        if isinstance(side, QComboBox):
            side.setEnabled(single_mode)
            side.setToolTip(self._t("side_tooltip"))
            if single_mode and not side.currentText().strip():
                index = side.findText("left")
                if index >= 0:
                    side.setCurrentIndex(index)

    def load_config(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            self._t("load_config_dialog"),
            str(APP_DIR),
            self._t("json_filter"),
        )
        if not path:
            return
        try:
            self._apply_config(json.loads(Path(path).read_text()))
            self.config_path = Path(path)
            self.config_path_edit.setText(path)
            self.append_log(f"[config] loaded {path}")
        except Exception as exc:
            QMessageBox.critical(self, self._t("load_config_failed"), str(exc))

    def save_config(self) -> None:
        path = Path(self.config_path_edit.text()).expanduser()
        if not path:
            self.save_config_as()
            return
        try:
            path.write_text(json.dumps(self._config_dict(), indent=2, ensure_ascii=False) + "\n")
            self.config_path = path
            self.append_log(f"[config] saved {path}")
        except Exception as exc:
            QMessageBox.critical(self, self._t("save_config_failed"), str(exc))

    def save_config_as(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            self._t("save_config_dialog"),
            str(APP_DIR / "ui_config.json"),
            self._t("json_filter"),
        )
        if not path:
            return
        self.config_path_edit.setText(path)
        self.save_config()

    def browse(self, field: str, directory: bool) -> None:
        current = self.value(field)
        if directory:
            path = QFileDialog.getExistingDirectory(self, self._t("choose_directory"), current or str(DEFAULT_REPO_ROOT))
        else:
            path, _ = QFileDialog.getOpenFileName(self, self._t("choose_file"), current or str(DEFAULT_REPO_ROOT))
        if path and isinstance(self.fields[field], QLineEdit):
            self.fields[field].setText(path)
            if field == "python_executable":
                combo = self.fields.get("python_env")
                if isinstance(combo, QComboBox):
                    idx = combo.findText(PYTHON_ENV_CUSTOM)
                    if idx >= 0:
                        combo.setCurrentIndex(idx)

    def scan_devices(self) -> None:
        if self.scan_process is not None and self.scan_process.state() != QProcess.NotRunning:
            QMessageBox.information(self, self._t("scan_running_title"), self._t("scan_running_text"))
            return

        repo_root = Path(self.value("repo_root")).expanduser().resolve()
        if not self._validate_python_executable():
            return
        cmd = self.python_module_command("production_ui.device_scan")
        role = self.value("role") if self.bool_value("include_role") else "leader"
        cmd.extend(["--role", role, "--tracker-timeout", self.value("scan_tracker_timeout") or "2.0"])

        self.scan_status.setText(self._t("scan_scanning"))
        self._set_all_device_labels(self._t("device_scanning"), "pending")
        self.refresh_command_preview()
        self.append_log(f"[scan] {shell_join(cmd)}")

        process = QProcess(self)
        process.setWorkingDirectory(str(repo_root))
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.setProcessEnvironment(self._process_environment(repo_root))
        process.readyReadStandardOutput.connect(self._read_scan_output)
        process.finished.connect(self._scan_finished)
        process.errorOccurred.connect(self._scan_error)
        self.scan_process = process
        self._scan_buffer = ""
        process.start(cmd[0], cmd[1:])

    def _read_scan_output(self) -> None:
        if self.scan_process is None:
            return
        data = bytes(self.scan_process.readAllStandardOutput()).decode(errors="replace")
        if data:
            self._scan_buffer += data

    def _scan_finished(self, exit_code: int, _exit_status: QProcess.ExitStatus) -> None:
        text = getattr(self, "_scan_buffer", "").strip()
        marker = "__PRODUCTION_UI_SCAN_JSON__"
        marker_index = text.rfind(marker)
        if marker_index >= 0:
            text = text[marker_index + len(marker) :].strip()
        try:
            result = json.loads(text)
        except Exception as exc:
            self.scan_status.setText(self._t("scan_parse_failed").format(error=exc))
            self._set_all_device_labels(self._t("device_pending"), "pending")
            self.refresh_command_preview()
            self.append_log(f"[scan] parse failed: {exc}\n{text}")
            return
        self._render_scan_result(result, exit_code)

    def _scan_error(self, error: QProcess.ProcessError) -> None:
        self.scan_status.setText(self._t("scan_process_error").format(error=error.name))
        self._set_all_device_labels(self._t("device_pending"), "pending")
        self.refresh_command_preview()

    def _render_scan_result(self, result: dict[str, Any], exit_code: int) -> None:
        for side in ("left", "right"):
            info = result.get("sides", {}).get(side, {})
            gripper = info.get("gripper") or {}
            tactiles = info.get("tactiles") or {}
            tactile_status = info.get("tactile_status") or {}
            values = {
                "gripper_fw": gripper.get("firmware_sn") or "-",
                "mcu_serial": gripper.get("mcu_serial") or "-",
                "tactile_left": tactiles.get("left") or "-",
                "tactile_right": tactiles.get("right") or "-",
                "wrist_camera": info.get("wrist_camera") or "-",
                "tracker": info.get("tracker") or "-",
            }
            for key, value in values.items():
                label = self.device_labels.get(f"{side}_{key}")
                if label is not None:
                    state = "ok" if value and value != "-" else "missing"
                    tooltip = ""
                    if key in {"tactile_left", "tactile_right"}:
                        finger = "left" if key == "tactile_left" else "right"
                        cell = tactile_status.get(finger) or {}
                        state = str(cell.get("state") or state)
                        tooltip = str(cell.get("message") or "")
                    self._set_device_label(label, str(value), state, tooltip)

        errors = result.get("errors") or {}
        if errors:
            parts = [f"{name}: {detail.get('message', '')}" for name, detail in errors.items()]
            self.scan_status.setText(self._t("scan_done_warnings") + " | ".join(parts))
            self.append_log("[scan warning] " + " | ".join(parts))
        else:
            self.scan_status.setText(self._t("scan_done_exit").format(code=exit_code))
        self.refresh_command_preview()

    def _set_device_label(self, label: QLabel, text: str, state: str, tooltip: str = "") -> None:
        label.setText(text)
        label.setProperty("state", state)
        label.setToolTip(tooltip or (text if text and text not in {"-", self._t("device_pending"), self._t("device_scanning")} else ""))
        label.style().unpolish(label)
        label.style().polish(label)
        label.update()

    def _set_all_device_labels(self, text: str, state: str) -> None:
        for label in self.device_labels.values():
            self._set_device_label(label, text, state)

    def _scanned_device_value(self, key: str) -> str:
        label = self.device_labels.get(key)
        if label is None:
            return ""
        value = label.text().strip()
        ignored = {"", "-", self._t("device_pending"), self._t("device_scanning")}
        return "" if value in ignored else value

    def _scanned_tracker_serial(self, side: str) -> str:
        return self._scanned_device_value(f"{side}_tracker")

    # ------------------------------------------------------------------
    # Command builders
    # ------------------------------------------------------------------

    def python_module_command(self, module: str) -> list[str]:
        python = self.value("python_executable") or _default_python_executable()
        return [python, "-m", module]

    def _validate_python_executable(self) -> bool:
        python_text = self.value("python_executable") or _default_python_executable()
        python = Path(python_text).expanduser()
        if python.exists() and _is_frozen_executable_path(python):
            QMessageBox.critical(
                self,
                self._t("python_invalid"),
                self._t("python_frozen_self_text").format(python=python),
            )
            return False
        if python_text and python.exists():
            return True
        QMessageBox.critical(
            self,
            self._t("python_not_found"),
            self._t("python_not_found_text").format(python=python_text or "<empty>"),
        )
        return False

    def selected_python_bin_dir(self) -> Path:
        return Path(self.value("python_executable") or _default_python_executable()).expanduser().resolve().parent

    def append_robot_args(self, cmd: list[str]) -> None:
        robot_type = self.value("robot_type")
        cmd.append(f"--robot.type={robot_type}")
        if self.bool_value("include_role") and self.value("role"):
            cmd.append(f"--robot.role={self.value('role')}")
        if robot_type == "taccap_gripper" and self.bool_value("include_side") and self.value("side"):
            cmd.append(f"--robot.side={self.value('side')}")

        if not self.bool_value("enable_tracker"):
            cmd.append("--robot.enable_tracker=false")
        elif robot_type == "taccap_gripper":
            tracker_serial = self._scanned_tracker_serial(self.value("side") or "left")
            if tracker_serial:
                cmd.append(f"--robot.tracker_serial={tracker_serial}")
        else:
            left_tracker_serial = self._scanned_tracker_serial("left")
            right_tracker_serial = self._scanned_tracker_serial("right")
            if left_tracker_serial:
                cmd.append(f"--robot.left_tracker_serial={left_tracker_serial}")
            if right_tracker_serial:
                cmd.append(f"--robot.right_tracker_serial={right_tracker_serial}")

    def build_teleoperate_command(self) -> list[str]:
        cmd = self.python_module_command("lerobot.scripts.lerobot_teleoperate")
        self.append_robot_args(cmd)
        cmd.append(f"--fps={self.value('teleop_fps')}")
        if self.bool_value("teleop_display_data"):
            cmd.append("--display_data=true")
        if self.bool_value("enable_tracker") and self.bool_value("teleop_show_trajectory"):
            cmd.append("--show_trajectory=true")
        return cmd

    def build_record_command(self) -> list[str]:
        cmd = self.python_module_command("lerobot.scripts.lerobot_record")
        self.append_robot_args(cmd)
        cmd.extend(
            [
                f"--dataset.repo_id={self.value('repo_id')}",
                f"--dataset.single_task={self.value('single_task')}",
                f"--dataset.num_episodes={self.value('num_episodes')}",
                f"--dataset.fps={self.value('dataset_fps')}",
                f"--dataset.episode_time_s={self.value('episode_time_s')}",
                f"--dataset.reset_time_s={self.value('reset_time_s')}",
                f"--dataset.streaming_encoding={bool_arg(self.bool_value('streaming_encoding'))}",
                f"--dataset.push_to_hub={bool_arg(self.bool_value('record_push_to_hub'))}",
            ]
        )
        if self.bool_value("private"):
            cmd.append("--dataset.private=true")
        if self.bool_value("record_display_data"):
            cmd.append("--display_data=true")
        if self.bool_value("enable_tracker") and self.bool_value("record_show_trajectory"):
            cmd.append("--show_trajectory=true")
        if self.value("dataset_root"):
            cmd.append(f"--dataset.root={self.value('dataset_root')}")
        if self.value("encoder_threads"):
            cmd.append(f"--dataset.encoder_threads={self.value('encoder_threads')}")
        if self.value("encoder_queue_maxsize"):
            cmd.append(f"--dataset.encoder_queue_maxsize={self.value('encoder_queue_maxsize')}")
        if self.value("vcodec"):
            cmd.append(f"--dataset.vcodec={self.value('vcodec')}")
        if self.bool_value("resume"):
            cmd.append("--resume=true")
        cmd.extend(split_extra_args(self.value("record_extra_args")))
        return cmd

    def build_check_command(self) -> list[str]:
        cmd = self.python_module_command("lerobot.scripts.lerobot_check_dataset")
        cmd.extend(["--repo-id", self.value("check_repo_id")])
        if self.value("check_root"):
            cmd.extend(["--root", self.value("check_root")])
        indices = [item for item in self.value("check_episode_index").replace(",", " ").split() if item]
        if indices:
            cmd.append("--episode-index")
            cmd.extend(indices)
        return cmd

    def build_push_command(self) -> list[str]:
        cmd = self.python_module_command("lerobot.scripts.push_dataset_to_hub")
        cmd.extend(["--repo-id", self.value("push_repo_id"), "--dataset-path", self.value("push_dataset_path")])
        if self.value("push_branch"):
            cmd.extend(["--branch", self.value("push_branch")])
        tags = self.value("push_tags").replace(",", " ").split()
        if tags:
            cmd.append("--tags")
            cmd.extend(tags)
        if self.value("push_license"):
            cmd.extend(["--license", self.value("push_license")])
        if self.bool_value("push_private"):
            cmd.append("--private")
        if self.bool_value("push_no_videos"):
            cmd.append("--no-videos")
        if self.bool_value("push_upload_large_folder"):
            cmd.append("--upload-large-folder")
        if self.bool_value("push_no_tag_version"):
            cmd.append("--no-tag-version")
        return cmd

    # ------------------------------------------------------------------
    # Process handling
    # ------------------------------------------------------------------

    def preview(self, cmd: list[str]) -> None:
        self.command_preview.setPlainText(shell_join(cmd))

    def refresh_command_preview(self) -> None:
        if self._applying_config or not hasattr(self, "command_preview"):
            return
        try:
            cmd = self.command_for_current_tab()
        except Exception as exc:
            self.command_preview.setPlainText(self._t("command_preview_error").format(error=exc))
            return
        if cmd is None:
            self.command_preview.setPlainText(self._t("command_preview_empty"))
            return
        self.preview(cmd)

    def command_for_current_tab(self) -> list[str] | None:
        if not hasattr(self, "tabs"):
            return None
        builders = {
            0: self.build_teleoperate_command,
            1: self.build_record_command,
            2: self.build_check_command,
            3: self.build_push_command,
        }
        builder = builders.get(self.tabs.currentIndex())
        return builder() if builder is not None else None

    def run(self, cmd: list[str]) -> None:
        repo_root = Path(self.value("repo_root")).expanduser().resolve()
        if not repo_root.exists():
            QMessageBox.critical(self, self._t("invalid_repo_root"), self._t("invalid_repo_root_text").format(repo_root=repo_root))
            return
        if not self._validate_python_executable():
            return

        self.preview(cmd)
        launched = self.launch_external_terminal(cmd, repo_root)
        if launched:
            self.append_log(f"[terminal] launched: {shell_join(cmd)}")
            self.append_log(f"[cwd] {repo_root}")
        else:
            QMessageBox.critical(
                self,
                self._t("terminal_launch_failed"),
                self._t("terminal_not_found"),
            )

    def _process_environment(self, repo_root: Path) -> QProcessEnvironment:
        env = QProcessEnvironment.systemEnvironment()
        src = repo_root / "src"
        existing = env.value("PYTHONPATH")
        existing_path = env.value("PATH")
        python_bin = str(self.selected_python_bin_dir())
        env.insert("PYTHONPATH", str(src) if not existing else f"{src}{os.pathsep}{existing}")
        env.insert("PATH", python_bin if not existing_path else f"{python_bin}{os.pathsep}{existing_path}")
        env.insert("PYTHONUNBUFFERED", "1")
        return env

    def launch_external_terminal(self, cmd: list[str], repo_root: Path) -> bool:
        command = shell_join(cmd)
        src = repo_root / "src"
        python_bin = self.selected_python_bin_dir()
        close_prompt = shlex.quote(self._t("terminal_press_enter"))
        script = (
            f"cd {shlex.quote(str(repo_root))}; "
            f"export PATH={shlex.quote(str(python_bin))}${{PATH:+:$PATH}}; "
            f"export PYTHONPATH={shlex.quote(str(src))}${{PYTHONPATH:+:$PYTHONPATH}}; "
            "export PYTHONUNBUFFERED=1; "
            f"echo '$ {command}'; "
            f"{command}; "
            "status=$?; echo; echo \"[exit] return code $status\"; "
            f"echo {close_prompt}; read _"
        )
        terminal_commands = [
            ["gnome-terminal", "--", "bash", "-lc", script],
            ["konsole", "-e", "bash", "-lc", script],
            ["xfce4-terminal", "--command", f"bash -lc {shlex.quote(script)}"],
            ["xterm", "-e", "bash", "-lc", script],
            ["x-terminal-emulator", "-e", "bash", "-lc", script],
        ]
        for terminal_cmd in terminal_commands:
            if QProcess.startDetached(terminal_cmd[0], terminal_cmd[1:], str(repo_root)):
                return True
        return False

    def stop_process(self) -> None:
        if self.scan_process is None or self.scan_process.state() == QProcess.NotRunning:
            return
        self.append_log(self._t("stop_terminating"))
        self.scan_process.terminate()
        if not self.scan_process.waitForFinished(2500):
            self.append_log(self._t("stop_killing"))
            self.scan_process.kill()

    def append_log(self, text: str) -> None:
        return

    # ------------------------------------------------------------------
    # Styling
    # ------------------------------------------------------------------

    def _stylesheet(self) -> str:
        return """
        QMainWindow, QWidget {
            background: #f5f7fb;
            color: #1f2937;
            font-size: 12px;
        }
        QFrame#Card {
            background: #ffffff;
            border: 1px solid #dfe5ee;
            border-radius: 6px;
        }
        QLabel#CardTitle {
            font-size: 12px;
            font-weight: 700;
            color: #0f172a;
        }
        QLabel#Hint {
            color: #64748b;
        }
        QLabel#FieldLabel {
            color: #334155;
            font-weight: 600;
            min-height: 22px;
        }

        QFrame#DeviceSidePanel {
            background: #f8fafc;
            border: 1px solid #e2e8f0;
            border-radius: 5px;
        }
        QLabel#DeviceSideTitle {
            color: #0f172a;
            font-weight: 800;
            min-height: 20px;
        }
        QLabel#DeviceItemName {
            color: #475569;
            font-weight: 600;
            font-size: 11px;
            min-height: 20px;
        }
        QLabel#DeviceValue {
            color: #111827;
            background: #f8fafc;
            border: 1px solid #e2e8f0;
            border-radius: 4px;
            padding: 1px 5px;
            font-size: 11px;
            font-family: "DejaVu Sans Mono", "Noto Sans Mono", monospace;
        }
        QLabel#DeviceValue[state="pending"] {
            color: #475569;
            background: #f8fafc;
            border-color: #dbe4ee;
        }
        QLabel#DeviceValue[state="ok"] {
            color: #14532d;
            background: #f0fdf4;
            border-color: #86efac;
        }
        QLabel#DeviceValue[state="missing"] {
            color: #92400e;
            background: #fffbeb;
            border-color: #fbbf24;
            font-weight: 700;
        }
        QLabel#DeviceValue[state="error"] {
            color: #991b1b;
            background: #fef2f2;
            border-color: #ef4444;
            font-weight: 700;
        }
        QGroupBox#Group {
            background: #ffffff;
            border: 1px solid #dfe5ee;
            border-radius: 6px;
            margin-top: 8px;
            font-weight: 700;
            color: #0f172a;
        }
        QGroupBox#Group::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 0 5px;
            left: 8px;
            background: #f5f7fb;
        }
        QFrame#ActionBar {
            background: #f8fafc;
            border: 1px solid #dbe4ee;
            border-radius: 6px;
        }
        QLineEdit, QComboBox, QPlainTextEdit {
            background: #ffffff;
            border: 1px solid #cfd8e3;
            border-radius: 5px;
            padding: 2px 6px;
            selection-background-color: #2563eb;
        }
        QLineEdit:focus, QComboBox:focus, QPlainTextEdit:focus {
            border: 1px solid #2563eb;
        }
        QPlainTextEdit {
            font-family: "DejaVu Sans Mono", "Noto Sans Mono", monospace;
            font-size: 12px;
        }
        QPushButton {
            background: #ffffff;
            border: 1px solid #cfd8e3;
            border-radius: 5px;
            padding: 2px 8px;
            color: #1f2937;
            font-weight: 600;
            min-height: 20px;
        }
        QPushButton:hover {
            background: #f1f5f9;
            border-color: #94a3b8;
        }
        QPushButton#PrimaryButton {
            background: #2563eb;
            color: #ffffff;
            border-color: #2563eb;
            padding-left: 12px;
            padding-right: 12px;
        }
        QPushButton#PrimaryButton:hover {
            background: #1d4ed8;
        }
        QTabWidget::pane {
            border: 1px solid #dfe5ee;
            border-radius: 6px;
            background: #f5f7fb;
            top: -1px;
        }
        QTabBar::tab {
            background: #edf2f8;
            color: #475569;
            padding: 4px 15px;
            border-top-left-radius: 6px;
            border-top-right-radius: 6px;
            margin-right: 2px;
        }
        QTabBar::tab:selected {
            background: #ffffff;
            color: #111827;
            font-weight: 700;
        }
        """


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("TacCap Production UI")
    window = ProductionWindow()
    window.show()
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
