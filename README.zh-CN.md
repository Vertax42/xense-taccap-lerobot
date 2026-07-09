# 项目概览

语言：[中文](README.zh-CN.md) | [English](README.md)

本仓库是 XenseRobotics 基于
[`lerobot`](https://github.com/huggingface/lerobot) 的分支，用于 Xense
多模态触觉数据采集系统。当前分支聚焦单一设备：
**TacCap-Gripper**，其中 **TacCap** 表示 *Tactile Capture*，即用于触觉数据
采集的手持式 **UMI** 主端夹爪。

本分支跟随上游 **lerobot v5.1**，并裁剪到 TacCap-Gripper 单手、双手形态及其
**Pico4** 遥操作器/追踪器，同时叠加 Xense 触觉相机支持。设备级使用说明见
[`src/lerobot/robots/taccap_gripper/README.md`](src/lerobot/robots/taccap_gripper/README.md)。
通用 lerobot 用法，包括数据集、策略和训练脚本，请参考
[上游 README](https://github.com/huggingface/lerobot#readme)。

## 安装

测试环境为 Ubuntu 22.04、NVIDIA driver >= 570.144。推荐使用
[`Mamba`](https://github.com/conda-forge/miniforge?tab=readme-ov-file#install)，
它比普通 conda 更适合 robostack-staging 频道中的 ROS Humble 和 SOEM 依赖。
v5.1 固定使用 **Python 3.12** 和 **PyTorch >= 2.2**，CUDA 版本为 12.8。

### 一键初始化

在仓库根目录运行：

```bash
bash scripts/bootstrap.sh --install-miniforge
```

该命令会串起子模块初始化、Miniforge/mamba 环境创建、`setup_env.sh --install`
和导入验证。如果已经安装 Miniforge/mamba，`--install-miniforge` 是安全的；
如果你希望自己管理 conda，可以去掉该参数。

常用变体：

```bash
# 初始化并运行本地 pytest
bash scripts/bootstrap.sh --test

# 通过 make 调用同一入口
make bootstrap
make bootstrap-test
```

如果需要按 fork -> 修改 -> 测试 -> merge request 的流程工作，提交本地修改后
可以使用 GitHub CLI (`gh`)：

```bash
# 创建/同步 fork，推送当前分支，并向 main 打开 PR
bash scripts/bootstrap.sh --all --branch <your-branch-name>
```

脚本只有在显式传入 `--fork`、`--push`、`--pr` 或 `--all` 时才会执行远端
fork、push 和 PR 操作。完整参数见：

```bash
bash scripts/bootstrap.sh --help
```

### 手动安装

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

### 环境配置

**Step 1:** 克隆仓库并初始化全部子模块：

```bash
git clone \
  --recurse-submodules \
  https://github.com/Vertax42/xense-taccap-lerobot.git
cd xense-taccap-lerobot
```

如果已经克隆但没有拉取子模块，手动执行：

```bash
git submodule update --init --recursive --progress
```

本仓库通过 `third_party/` 子模块管理硬件 SDK 依赖：

| 子模块 | 安装包 |
| ------ | ------ |
| `third_party/taccap-gripper` | `xense.taccap`，TacCap UMI 触觉夹爪 SDK |
| `third_party/XenseVR-PC-Service` | `xensevr_pc_service_sdk`，Pico4 遥操作/追踪器 |
| `third_party/XenseVR-RobotVision-PC` | ZED-M 到 Pico4 的立体透视服务，单独构建 |

`xensesdk` 不是子模块，也没有内置在仓库里；它是约 90 MB 的二进制 wheel，
包含 patched `libxense_c.so` flash reader。请通过外部方式获取 wheel，并放到
`~/Downloads/` 或仓库 `dist/` 目录，也可以显式指定：

```bash
export XENSESDK_WHEEL=/path/to/xensesdk-*-cp312-*-linux_x86_64.whl
```

`setup_env.sh --install` 会自动解析并安装它。后续 `xensesdk` 2.x 发布到 PyPI 后，
该步骤会退化为普通的 `pip install xensesdk`。

**XenseVR PC Service daemon** 是 Pico4 遥操作/追踪器通信所需的服务，同样作为
约 100 MB 的 Debian 包单独发布，安装路径为 `/opt/apps/roboticsservice`。
`setup_env.sh --install` 会自动安装它：优先使用 `$XENSEVR_DEB`、仓库 `dist/`
或 `~/Downloads/` 中的本地包，否则从
[`v0.1.0 release`](https://github.com/Vertax42/XenseVR-PC-Service/releases/tag/v0.1.0)
下载匹配架构的 asset，也可以用 `$XENSEVR_DEB_URL` 覆盖下载地址。安装时会执行
`sudo dpkg -i`，同版本会跳过。手动启动方式为：

```bash
/opt/apps/roboticsservice/runService.sh
```

**Step 2:** 创建并激活 mamba 环境：

```bash
bash ./setup_env.sh --mamba lerobot-xense
mamba activate lerobot-xense
```

`conda_environment.yaml` 中默认环境名为 `lerobot-xense`。你可以向 `--mamba`
传入其他名称，但 README 和 openpi 项目默认假设使用 `lerobot-xense`。

**Step 3:** 安装 LeRobot-Xense 和全部硬件 SDK 绑定：

```bash
bash ./setup_env.sh --install
```

该步骤会完成：

- 按 `conda_environment.yaml` 更新 conda 环境
- 从 `pyproject.toml` 安装主包
- 从外部解析到的 wheel 安装 `xensesdk`
- 从 `.deb` 安装 XenseVR PC Service daemon，并构建 `third_party` SDK 包：
  `xensevr_pc_service_sdk` 和 `xense.taccap`

**Step 4:** 验证安装：

```bash
python -c 'import xensevr_pc_service_sdk; print("xensevr_pc_service_sdk OK ->", xensevr_pc_service_sdk.__file__)'
python -c 'import xensesdk; print("xensesdk OK ->", xensesdk.__file__)'
python -c 'import xense.taccap; print("xense.taccap OK ->", xense.taccap.__file__)'
```

**Step 5:** FFmpeg / video 说明。v5.1 不再通过 conda 固定 `ffmpeg`，因为
robostack 的 ICU pin 会和新版 ffmpeg 冲突。视频编码/解码由
`setup_env.sh --install` 安装的 `torchcodec` 和 `av` wheels 处理。如果需要带
`libsvtav1` 的系统 ffmpeg，请单独通过 apt 或 upstream static build 安装。

```bash
# 可选：验证 torchcodec wheel 可以加载
python -c 'import torchcodec; print("torchcodec OK ->", torchcodec.__version__)'
```

**Step 6:** TacCap-Gripper 串口权限。夹爪 MCU 会枚举为 `/dev/ttyACM*`，
所属组为 `dialout`。如果用户不在 `dialout` 组内，SDK 可以列出设备，但无法
打开串口读取 firmware SN，导致 `scan_grippers()` 返回 `role=Unknown`、
`firmware_sn` 为空，`connect()` 可能报错：

```text
RuntimeError: No leader gripper discovered for the left side.
```

一次性把当前用户加入 `dialout`，然后重新登录或刷新当前 shell 组权限：

```bash
sudo usermod -aG dialout "$USER"
# 退出并重新登录，或当前 shell 使用 `newgrp dialout`，然后重新插拔设备
```

验证夹爪可完整读取；`role` 必须为 `Leader` 或 `Follower`，`firmware_sn`
必须非空：

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side.name, g.role.name, repr(g.firmware_sn))"
```

如果修复权限后 `firmware_sn` 仍为空，说明设备 SN 没有烧录，或固件版本低于
V1.6；这是设备/固件问题，不是主机权限问题。

**Step 7:** 避免 ModemManager 占用夹爪串口。夹爪 MCU 是 CH343 USB-serial
设备，USB VID/PID 为 `1a86:55d2`，会枚举为 CDC-ACM 端口。Ubuntu/GNOME 默认
带的 **ModemManager** 会在每次热插拔时探测新端口并短暂占用，导致立即
`connect()` 时可能报错：

```text
IoError: SerialBus: open(/dev/serial/by-id/usb-1a86_USB_Dual_Serial_..-if02): Device or resource busy
```

临时规避方法是重新插拔后等待约 3 秒。永久修复方法是添加 udev rule，让
ModemManager 忽略该设备：

```bash
sudo tee /etc/udev/rules.d/99-taccap-ignore-modemmanager.rules >/dev/null <<'EOF'
# TacCap-Gripper MCUs are CH343 USB-serial (1a86:55d2) - keep ModemManager off them
ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

验证：

```bash
udevadm info -q property -n /dev/ttyACM0 | grep ID_MM_DEVICE_IGNORE   # -> ID_MM_DEVICE_IGNORE=1
mmcli -L                                                               # grippers no longer listed
```

如需回滚，删除该 rule 文件并重新加载 udev。专用机器人主机如果没有蜂窝 modem，
也可以直接执行：

```bash
sudo systemctl disable --now ModemManager
```

## Robotics Service 开机自启动

本节只负责把已经安装好的 XenseVR PC Service daemon 注册成开机自启动的
`systemd` 服务。daemon 本体来自原项目的 XenseVR-PC-Service `.deb` 包，必须先
单独安装；自启动脚本不内置、也不下载这个二进制包。

通过项目安装脚本一键安装 `.deb`。这是推荐路径：`setup_env.sh --install` 会优先
从 `$XENSEVR_DEB`、仓库 `dist/` 或 `~/Downloads/` 查找本地 `.deb`；如果没有找到，
就从 release 下载匹配架构的包，并通过 `dpkg` 安装。

```bash
bash ./setup_env.sh --install
```

如果 `.deb` 下载或安装失败，`setup_env.sh --install` 会直接失败，避免遗漏 daemon。
只有在明确想跳过 daemon、单独安装 Python bindings 时，才设置 `XENSEVR_SKIP_DEB=1`。

也可以从原项目 release 下载匹配架构的 `.deb`，然后手动安装：

```bash
# amd64 或 arm64，取决于 `dpkg --print-architecture`
sudo dpkg -i ~/Downloads/XenseVR-PC-Service_0.1.0_amd64.deb
sudo apt-get install -f -y
```

Release 页面：
[`XenseVR-PC-Service v0.1.0`](https://github.com/Vertax42/XenseVR-PC-Service/releases/tag/v0.1.0)。
如果 `.deb` 不在 `~/Downloads/` 或仓库 `dist/` 目录，可以用下面方式指定给安装脚本：

```bash
XENSEVR_DEB=/path/to/XenseVR-PC-Service_0.1.0_amd64.deb bash ./setup_env.sh --install
```

安装后应存在：

```bash
/opt/apps/roboticsservice/runService.sh
/opt/apps/roboticsservice/RoboticsServiceProcess
```

确认 `.deb` 已安装后，再注册开机自启动：

```bash
scripts/roboticsservice_autostart.sh install
```

该命令会写入 `/etc/systemd/system/roboticsservice.service`，设置开机启用，
并立即启动服务。默认运行用户为脚本检测到的登录用户；如需指定用户：

```bash
scripts/roboticsservice_autostart.sh install --user <username>
```

如果安装前已经手动执行过 `/opt/apps/roboticsservice/runService.sh`，需要先停止
已有进程。也可以让安装脚本停止已有的 `RoboticsServiceProcess`，并把生命周期
交给 `systemd`：

```bash
scripts/roboticsservice_autostart.sh install --stop-existing
```

常用服务命令：

```bash
scripts/roboticsservice_autostart.sh start
scripts/roboticsservice_autostart.sh stop
scripts/roboticsservice_autostart.sh restart
scripts/roboticsservice_autostart.sh status
scripts/roboticsservice_autostart.sh logs
scripts/roboticsservice_autostart.sh uninstall
```

查看详细状态：

```bash
scripts/roboticsservice_autostart.sh status
```

用于自动化验收或部署检查的简洁检查命令：

```bash
scripts/roboticsservice_autostart.sh check
```

`check` 会检查：

- `/etc/systemd/system/roboticsservice.service` 是否存在
- `roboticsservice.service` 是否已设置为开机自启动
- `roboticsservice.service` 当前是否为 active
- `systemd` 记录的主进程 PID 是否仍在运行

如果 `check` 报告服务一直处于 `activating`，并且日志里反复出现 `release mode`
然后 `Deactivated successfully`，通常说明已经有一个手动启动的实例在运行。检查：

```bash
pgrep -af '[R]oboticsServiceProcess'
```

停止手动进程，或重新执行 `install --stop-existing`，让 `systemd` 接管服务生命周期。

查看日志：

```bash
scripts/roboticsservice_autostart.sh logs
```

生成的 unit 不直接调用 `/opt/apps/roboticsservice/runService.sh`，因为该脚本
会把 `RoboticsServiceProcess` 放到后台。安装脚本会写入
`/usr/local/bin/roboticsservice-systemd-start.sh`，并生成带有
`PIDFile=/run/roboticsservice/roboticsservice.pid` 的 `Type=forking` unit，
这样 `systemd` 可以追踪实际服务进程。wrapper 使用和 `runService.sh` 一致的
运行时路径：

- `LD_LIBRARY_PATH=/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib:/opt/apps/roboticsservice/SDK/x64`
- `QT_PLUGIN_PATH=/opt/apps/roboticsservice/plugins/`
- `QT_QML_PATH=/opt/apps/roboticsservice/qml/`

## LeRobotDataset 格式

`LeRobotDataset` 可以直接从 Hugging Face Hub 仓库或本地目录加载，例如：

```python
dataset = LeRobotDataset("lerobot/aloha_static_coffee")
```

它可以像普通 Hugging Face / PyTorch 数据集一样索引。`LeRobotDataset` 的一个
特点是可以通过 `delta_timestamps` 按时间关系取多帧，而不是只取当前索引帧。
例如：

```python
delta_timestamps = {"observation.image": [-1, -0.5, -0.2, 0]}
```

这会取当前帧以及当前帧之前 1 秒、0.5 秒、0.2 秒的图像。更完整说明见英文
[`README.md`](README.md) 和上游示例。

底层格式主要由以下部分组成：

- `hf_dataset`：基于 Hugging Face datasets 的 Arrow/parquet 数据
- `videos`：以 mp4 保存的视频数据
- `meta`：json/jsonl 格式的元数据、episode 信息、统计信息和任务信息

本地数据集可以通过 `root` 参数指定路径；未指定时默认使用
`~/.cache/huggingface/lerobot`。

## 引用

如果使用本代码库，请引用原始 LeRobot 项目：

```bibtex
@misc{cadene2024lerobot,
    author = {Cadene, Remi and Alibert, Simon and Soare, Alexander and Gallouedec, Quentin and Zouitine, Adil and Palma, Steven and Kooijmans, Pepijn and Aractingi, Michel and Shukor, Mustafa and Aubakirova, Dana and Russi, Martino and Capuano, Francesco and Pascal, Caroline and Choghari, Jade and Moss, Jess and Wolf, Thomas},
    title = {LeRobot: State-of-the-art Machine Learning for Real-World Robotics in Pytorch},
    howpublished = "\url{https://github.com/huggingface/lerobot}",
    year = {2024}
}
```

如果专门使用本分支 LeRobot-Xense，也请引用：

```bibtex
@misc{xense-taccap-lerobot,
    author = {XenseRobotics Team},
    title = {LeRobot-Xense: LeRobot with Xense Tactile Robotics Support},
    howpublished = "\url{https://github.com/Vertax42/xense-taccap-lerobot}",
    year = {2026}
}
```
