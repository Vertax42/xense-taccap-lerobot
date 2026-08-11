# LeRobot-Xense Docker 使用说明

该镜像包含项目的完整 `xense-taccap` Conda 环境、CUDA 12.8 用户态库、
LeRobot-Xense、XenseSDK、TacCap-Gripper SDK、Pico4 Python 绑定，
并在容器启动时默认启动 XenseVR PC Service。

**镜像从 GHCR 拉取，不需要自己构建，也不需要登录。** 本文只讲怎么把环境跑起来。
构建镜像、发布新版本、离线交付这些维护者的事，见
[`MAINTAINING.md`](MAINTAINING.md)。

## 0. 镜像里有什么

当前发布版本 **0.0.5**（`ghcr.io/vertax42/xense-taccap-lerobot:0.0.5`，`latest` 指向同一镜像）。

| 组成                      | 版本 / 来源                                                        |
| ------------------------- | ------------------------------------------------------------------ |
| Conda 环境                | `xense-taccap`，Python 3.12                                        |
| CUDA 用户态               | 12.8（`CONDA_OVERRIDE_CUDA=12.8`）                                 |
| XenseVR PC Service daemon | `.deb` **v0.2.1**，装在 `/opt/apps/roboticsservice`                |
| `xensevr_pc_service_sdk`  | 仓库内 pybind 编译；链接的 C SDK 取自上面那个 `.deb`，版本号也随它 |
| `xense.taccap`            | 由 `third_party/taccap-gripper` 在镜像内从源码编译                 |
| `xensesdk`                | PyPI 预编译 wheel                                                  |

### 0.0.4 → 0.0.5 的变化

**镜像内容与 0.0.4 相同** —— conda 环境、各个 SDK、daemon（仍是 v0.2.1）都没变。
变的是安装方式，所以已经装好 0.0.4 并且跑得好好的机器，没有必须升级的理由。

- **安装改为从 GHCR 在线拉取。** 干净 clone 之后 `docker compose pull` 直接可用。
  以前不写 `.env` 会撞 `pull access denied ... may require 'docker login'`，
  而那句提示是误导——包是公开的，拉取从不需要登录。
- **不再需要 tar 交付包。** 断网客户机仍可离线安装，用法见 `MAINTAINING.md`。

对**已有命令行用法**没有影响：`lerobot-record` 等一概照旧。

### 0.0.3 → 0.0.4 的变化

对**已有使用方式**有影响的只有一条：

- **`--robot.id` 现在接受纯数字**，并按 `--robot.type` 展开：`--robot.id=0` 在单臂上存为
  `taccap_0`、在双臂上存为 `bi_taccap_0`。**旧写法不受影响** —— 非纯数字一律原样保留，
  所以 `--robot.id=taccap_0` 和按它命名的标定文件都照常工作。

其余是内部变化，不改变命令行：

- daemon 升到 v0.2.1（修掉了 Pico 相机每帧刷屏、把调用方控制台冲垮的问题）
- `--dataset.vcodec=auto` 现在会真正打开一次编码器再判定，无 GPU 的机器上会正确
  回退到 `libsvtav1`，而不是选中 nvenc 后在第一帧崩掉

## 1. 宿主机要求

- Ubuntu 22.04/24.04，`linux/amd64`
- NVIDIA 驱动 `>= 570.144`
- Docker Engine + Docker Compose 插件
- NVIDIA Container Toolkit；`docker run --rm --gpus all ubuntu:22.04 nvidia-smi`
  应能看到显卡
- USB/串口/HID/CAN 设备仍需在**宿主机**完成 udev 规则，容器不能替宿主机管理热插拔规则

其中 Docker、NVIDIA Container Toolkit 和 udev 规则都由下一节的 `install_customer.sh`
自动装好，这里列出只是为了说明这台机器最终需要具备什么。

Compose 使用 `privileged: true`、host 网络和 host IPC，以支持运行中热插拔的
Xense 触觉/手腕相机、TacCap 串口、Pico4 和 CAN。请只在可信的机器人主机上运行。

## 2. 安装（在线拉取）

```bash
git clone https://github.com/Vertax42/xense-taccap-lerobot.git
cd xense-taccap-lerobot
./docker/install_customer.sh
```

**不需要 `git submodule update`** —— submodule 只在自己构建镜像时才用得上，
拉现成镜像用不到。

脚本会自动完成：

1. 检查 Ubuntu/Debian amd64 和宿主机 NVIDIA 驱动。
2. 缺少时安装 Docker Engine、Buildx 和 Compose 插件。
3. 缺少时安装 NVIDIA Container Toolkit，并配置 Docker GPU runtime/CDI。
4. 安装 TacCap 宿主机 udev 规则（CH343 的 ModemManager 屏蔽规则）。
5. 从 GHCR 拉取镜像并运行 PyTorch CUDA 冒烟测试。

需要本机 HTTP 代理时：

```bash
XENSE_PROXY_URL=http://127.0.0.1:7897 ./docker/install_customer.sh
```

脚本不会自动安装或升级宿主机 NVIDIA 驱动，因为驱动安装涉及显卡型号、
Secure Boot 和系统重启。若 `nvidia-smi` 不可用或驱动低于 `570.144`，脚本会停止并
提示先处理驱动。

### 安装完成后的宿主机设置

`install_customer.sh` 已经执行 Docker 服务启用和用户组配置。为了确保当前用户
立即获得 Docker 权限，可在宿主机执行以下命令：

```bash
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
newgrp docker
docker images
```

`newgrp docker` 会为当前终端启动一个应用了新用户组的子 Shell；也可以注销并重新
登录。这里仅将**当前用户**加入 `docker` 组，不会自动授权所有本地用户。请注意，
`docker` 组成员拥有接近 root 的系统控制权限，只应加入可信用户。

如果需要在容器内显示 Rerun 等 X11 图形窗口，还要由宿主机当前图形桌面用户执行：

```bash
xhost +si:localuser:root
```

使用完成后可以撤销授权：

```bash
xhost -si:localuser:root
```

## 3. 录数据前请 pin 版本

默认拉的是 `latest`，它是**浮动**的：下一次发布会把同一个 tag 指到新镜像上。正式采集
之前，在仓库根目录的 `.env` 里钉死版本：

```dotenv
LEROBOT_IMAGE_TAG=0.0.5
```

`.env` 已在 `.gitignore` 里，不会被提交。改完确认 compose 解析成了你要的那个：

```bash
docker compose config --images
```

### 想在拉之前看远端有什么

```bash
docker manifest inspect ghcr.io/vertax42/xense-taccap-lerobot:0.0.5
```

`manifest inspect` 查的是 registry，**不需要先下载 21 GB**。

### 想确认手上跑的是哪一个

```bash
docker image inspect --format '{{index .RepoDigests 0}}' \
    ghcr.io/vertax42/xense-taccap-lerobot:0.0.5
```

注意 `image inspect` **只看本地已有的镜像**。没拉过这个 tag 就会报
`No such image`，那不是出错，是还没拉——先 `docker compose pull`。

发布时 `0.0.5` 和 `latest` 指向同一镜像，两者 digest 应当一致。

## 4. 启动与验证

进入容器：

```bash
docker compose run --rm xense-taccap
```

验证软件环境和 GPU：

```bash
python -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
python -c 'import xensesdk; print("xensesdk ->", xensesdk.__file__)'
python -c 'import xense.taccap; print("taccap ->", xense.taccap.__file__)'
python -c 'import importlib.metadata as M; print("pico4 ->", M.version("xensevr_pc_service_sdk"))'
dpkg-query -W -f='daemon -> ${Version}\n' xensevr-pc-service
```

后两行应当**打印同一个版本号**（0.0.5 镜像里是 `0.2.1`）。这不是巧合：pico4 绑定链接的
C SDK 就是从那个 `.deb` 里取的，它的包版本也由 `dpkg-query` 推导。两者不一致，说明镜像
是半新不旧的构建，不要拿它录数据。

发现相机和串口：

```bash
lerobot-find-cameras
lerobot-info
```

直接执行一次性命令也可以：

```bash
docker compose run --rm xense-taccap lerobot-info
```

## 5. 数据、缓存与 GUI

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

## 6. 升级到新版本

改 `.env` 里的 `LEROBOT_IMAGE_TAG`，然后：

```bash
docker compose pull
```

21 GB 里绝大部分是 conda 层和 SDK 层，版本迭代时只会拉变动的那几层，不是重新
搬一遍整包。

## 7. 常见问题

- `No such image: ghcr.io/...`：`docker image inspect` 只查本地。这个 tag 还没拉过，
  先 `docker compose pull`；只想看远端用 `docker manifest inspect`。
- `pull access denied ... may require 'docker login'`：**不是权限问题**。GHCR 上的包是
  公开的，拉取从不需要登录。这条报错说明镜像名被解析成了 registry 上不存在的名字 ——
  用 `docker compose config --images` 看看实际解析出来的是什么，并检查 `.env` 里的
  `LEROBOT_IMAGE` 有没有被改掉。
- `could not select device driver ... gpu`：安装/配置 NVIDIA Container Toolkit，
  然后重启 Docker daemon。
- 容器能看到 `/dev/ttyACM*` 但设备 busy：按顶层 README 配置宿主机
  ModemManager udev 规则，重新插拔设备。
- GUI 不显示：检查 `echo "$DISPLAY"`、`/tmp/.X11-unix` 和上述 `xhost` 授权。
- Rerun 报 Vulkan adapter 错误：先确认 `nvidia-smi` 和
  `vulkaninfo --summary` 均能在容器中识别 NVIDIA GPU。
- `torch.cuda.is_available()` 是 `False`，但 `nvidia-smi` 正常：宿主机的 CUDA 状态坏了，
  与容器无关。笔记本挂起/恢复后常见。先在**宿主机**上确认
  `python -c 'import ctypes; print(ctypes.CDLL("libcuda.so.1").cuInit(0))'` 是否返回 0；
  非 0 时用 `sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm` 重载，或重启。
- 找不到 GSPS，但宿主机存在：确认容器通过 Compose 启动，并检查
  `ls /dev/v4l/by-id/*GSPS*`；必要时重新插拔 USB Hub 后执行
  `sudo udevadm settle --timeout=20`。
