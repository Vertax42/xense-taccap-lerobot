# LeRobot-Xense Docker 使用说明

该镜像包含项目的完整 `xense-taccap` Conda 环境、CUDA 12.8 用户态库、
LeRobot-Xense、XenseSDK、TacCap-Gripper SDK、Pico4 Python 绑定、Insight SDK，
并在容器启动时默认启动 XenseVR PC Service。

## 1. 宿主机要求

- Ubuntu 22.04/24.04，`linux/amd64`
- NVIDIA 驱动 `>= 570.144`
- Docker Engine + Docker Compose 插件
- NVIDIA Container Toolkit；`docker run --rm --gpus all ubuntu:22.04 nvidia-smi`
  应能看到显卡
- USB/串口/HID/CAN 设备仍需在**宿主机**完成 README 中的 udev 规则，容器不能替宿主机管理热插拔规则

Compose 使用 `privileged: true`、host 网络和 host IPC，以支持运行中热插拔的
Xense/Insight 相机、TacCap 串口、Pico4 和 CAN。请只在可信的机器人主机上运行。

## 2. 初始化源码并构建

镜像会编译三个硬件 SDK，因此构建前必须拉取 git submodule：

```bash
git submodule update --init --recursive --progress
docker compose build
```

首次构建需要下载 Conda/CUDA/Python 依赖并编译原生模块，耗时较长，镜像也会比较大。
后续未修改 `conda_environment.yaml` 时会复用最耗时的依赖层。
Dockerfile 已包含 apt/curl 自动重试、CUDA 12.8 构建期覆盖和 flexible channel
priority，不再需要使用临时 `sed` 命令修改构建过程。

## 3. 启动与验证

进入容器：

```bash
docker compose run --rm xense-taccap
```

验证软件环境和 GPU：

```bash
python -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
python -c 'import xensesdk; print("xensesdk ->", xensesdk.__file__)'
python -c 'import xense.taccap; print("taccap ->", xense.taccap.__file__)'
python -c 'import xensevr_pc_service_sdk; print("pico4 ->", xensevr_pc_service_sdk.__file__)'
pyinsight-check-env --hidraw
```

发现相机和串口：

```bash
lerobot-find-cameras
lerobot-find-port
```

直接执行一次性命令也可以：

```bash
docker compose run --rm xense-taccap lerobot-info
```

## 4. 数据、缓存与 GUI

LeRobot 数据根目录 `HF_LEROBOT_HOME` 已设为 `/data/lerobot`，Hugging Face
和 Torch 缓存也使用 Docker volume，删除容器不会丢失：

```text
/data                         -> lerobot-data
/root/.xensesdk               -> xensesdk-cache（按传感器序列号缓存配置）
/root/.cache/huggingface      -> huggingface-cache
/root/.cache/torch            -> torch-cache
```

Compose 会透传宿主机 `/dev` 和 `/run/udev`，使程序能够读取
`/dev/v4l/by-id`、`/dev/v4l/by-path` 和 `/dev/serial/by-path` 完成 USB Hub
自动配对。XenseSDK 的配置缓存会跨 `--rm` 临时容器保留，避免每次启动重新读取
传感器 flash 并触发 USB 重新枚举。

查看或备份数据可使用：

```bash
docker compose run --rm xense-taccap bash -lc 'ls -la /data'
```

Compose 已透传 `DISPLAY` 和 X11 socket。宿主机若拒绝窗口连接，可临时授权本机 root：

```bash
xhost +si:localuser:root
# 使用完后撤销
xhost -si:localuser:root
```

镜像已包含 Rerun 所需的 XKB、Vulkan 和 XDG 运行库，并默认设置
`WGPU_BACKEND=vulkan`。可用 `vulkaninfo --summary` 检查 NVIDIA Vulkan 是否正常。

无 Pico4、只处理数据时，可关闭随容器启动的服务：

```bash
START_XENSEVR_SERVICE=0 docker compose run --rm xense-taccap
```

服务日志默认位于容器内 `/tmp/xensevr-service.log`。

## 5. 常见问题

- `Missing git submodules`：在仓库根目录执行
  `git submodule update --init --recursive` 后重新构建。
- `could not select device driver ... gpu`：安装/配置 NVIDIA Container Toolkit，
  然后重启 Docker daemon。
- 容器能看到 `/dev/ttyACM*` 但设备 busy：按顶层 README 配置宿主机
  ModemManager udev 规则，重新插拔设备。
- GUI 不显示：检查 `echo "$DISPLAY"`、`/tmp/.X11-unix` 和上述 `xhost` 授权。
- Rerun 报 Vulkan adapter 错误：先确认 `nvidia-smi` 和
  `vulkaninfo --summary` 均能在容器中识别 NVIDIA GPU。
- 找不到 GSPS，但宿主机存在：确认容器通过 Compose 启动，并检查
  `ls /dev/v4l/by-id/*GSPS*`；必要时重新插拔 USB Hub 后执行
  `sudo udevadm settle --timeout=20`。
- 构建需要离线/定制的 vendor wheel 或 `.deb`：先按 `setup_env.sh` 支持的
  `XENSESDK_WHEEL` / `XENSEVR_DEB` 方式将安装物纳入构建上下文，再定制 Dockerfile；
  默认镜像从项目规定的公开发布地址下载。
