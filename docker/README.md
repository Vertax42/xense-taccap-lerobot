# LeRobot-Xense Docker 使用说明

该镜像包含项目的完整 `xense-taccap` Conda 环境、CUDA 12.8 用户态库、
LeRobot-Xense、XenseSDK、TacCap-Gripper SDK、Pico4 Python 绑定，
并在容器启动时默认启动 XenseVR PC Service。

## 0. 镜像里有什么

当前发布版本 **0.0.4**（`ghcr.io/vertax42/xense-taccap-lerobot:0.0.4`，`latest` 指向同一镜像）。

| 组成                      | 版本 / 来源                                                        |
| ------------------------- | ------------------------------------------------------------------ |
| Conda 环境                | `xense-taccap`，Python 3.12                                        |
| CUDA 用户态               | 12.8（`CONDA_OVERRIDE_CUDA=12.8`）                                 |
| XenseVR PC Service daemon | `.deb` **v0.2.1**，装在 `/opt/apps/roboticsservice`                |
| `xensevr_pc_service_sdk`  | 仓库内 pybind 编译；链接的 C SDK 取自上面那个 `.deb`，版本号也随它 |
| `xense.taccap`            | 由 `third_party/taccap-gripper` submodule 在镜像内从源码编译       |
| `xensesdk`                | PyPI 预编译 wheel                                                  |

镜像的 tag 由 `pyproject.toml` 的 `version = "0.5.1+xtac.<版本>"` 推导（Docker tag 不能含
`+`，故只取 `+xtac.` 之后的部分）。每次发布同时打一个 `sha-<commit>` tag，可以精确追溯
镜像是从哪个源码提交构建的。

### 0.0.3 → 0.0.4 的变化

对**已有使用方式**有影响的只有一条：

- **`--robot.id` 现在接受纯数字**，并按 `--robot.type` 展开：`--robot.id=0` 在单臂上存为
  `taccap_0`、在双臂上存为 `bi_taccap_0`。**旧写法不受影响** —— 非纯数字一律原样保留，
  所以 `--robot.id=taccap_0` 和按它命名的标定文件都照常工作。

其余是内部变化，不改变命令行：

- daemon 升到 v0.2.1（修掉了 Pico 相机每帧刷屏、把调用方控制台冲垮的问题）
- 镜像不再需要 `XenseVR-PC-Service` submodule：pico4 的 C SDK 直接取自 `.deb`，
  构建时少一次 cmake + 静态 gRPC 链接
- `--dataset.vcodec=auto` 现在会真正打开一次编码器再判定，无 GPU 的机器上会正确
  回退到 `libsvtav1`，而不是选中 nvenc 后在第一帧崩掉

## 1. 宿主机要求

- Ubuntu 22.04/24.04，`linux/amd64`
- NVIDIA 驱动 `>= 570.144`
- Docker Engine + Docker Compose 插件
- NVIDIA Container Toolkit；`docker run --rm --gpus all ubuntu:22.04 nvidia-smi`
  应能看到显卡
- USB/串口/HID/CAN 设备仍需在**宿主机**完成 README 中的 udev 规则，容器不能替宿主机管理热插拔规则

Compose 使用 `privileged: true`、host 网络和 host IPC，以支持运行中热插拔的
Xense 触觉/手腕相机、TacCap 串口、Pico4 和 CAN。请只在可信的机器人主机上运行。

## 2. 客户交付与新机器一键安装

开发机在镜像构建、验证完成后执行：

```bash
export LEROBOT_IMAGE_TAG=0.0.4
./docker/package_customer_delivery.sh
```

脚本会在 `dist/customer/` 下生成一个完整交付目录，包含：

- `xense-taccap-lerobot-<版本>-linux-amd64.tar`
- `SHA256SUMS`
- `compose.yaml` 和 `.env`
- 客户侧 `install_customer.sh`
- Docker 中文说明

将整个目录复制到客户的新机器，然后使用普通用户执行：

```bash
cd xense-taccap-lerobot-0.0.4-linux-amd64
./install_customer.sh
```

客户脚本会自动完成：

1. 检查 Ubuntu/Debian amd64 和宿主机 NVIDIA 驱动。
2. 缺少时安装 Docker Engine、Buildx 和 Compose 插件。
3. 缺少时安装 NVIDIA Container Toolkit，并配置 Docker GPU runtime/CDI。
4. 安装 TacCap 宿主机 udev 规则（CH343 的 ModemManager 屏蔽规则）。
5. 校验 SHA256、导入镜像并运行 PyTorch CUDA 冒烟测试。

需要本机 HTTP 代理时：

```bash
XENSE_PROXY_URL=http://127.0.0.1:7897 ./install_customer.sh
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

## 3. 从 GHCR 拉取镜像（在线交付）

镜像同时发布在 GitHub Container Registry：

```text
ghcr.io/vertax42/xense-taccap-lerobot
```

该包是 **public** 的，拉取不需要登录 —— 与仓库本身一致：镜像里的 XenseSDK 来自公开
PyPI，XenseVR-PC-Service 的 `.deb` 来自公开 GitHub release，taccap-gripper submodule
也是公开仓库，没有一样是靠镜像才拿得到的。只有**推送**需要凭据。

第 2 节的 tar 交付方式**继续保留**：客户机完全离线时仍然只能走 tar。GHCR 的价值在
后续升级 —— 21 GB 里绝大部分是 conda 层和 SDK 层，版本迭代时客户只需要拉变动的
几层，而不是重新搬一遍整包。

### 客户侧拉取

在交付目录（或仓库根目录）的 `.env` 里指向 GHCR：

```dotenv
LEROBOT_IMAGE=ghcr.io/vertax42/xense-taccap-lerobot
LEROBOT_IMAGE_TAG=0.0.4
```

`compose.yaml` 默认仍是本地构建的 `xense-taccap-lerobot`，只有设置了
`LEROBOT_IMAGE` 才改为从 GHCR 拉取。之后照常：

```bash
docker compose pull
docker compose run --rm xense-taccap
```

`latest` 是浮动的，出问题时先确认手上这个到底是哪一个镜像：

```bash
docker image inspect --format '{{index .RepoDigests 0}}' \
    ghcr.io/vertax42/xense-taccap-lerobot:0.0.4
```

发布时 `0.0.4` 和 `latest` 指向同一镜像，两者 digest 应当一致。想在**拉之前**看远端的，
用 `docker manifest inspect <image>:<tag>`，不必先下载 21 GB。

### 维护者侧推送

推荐走 GitHub Actions（在托管 runner 上构建并推送，不占用本机上行带宽）：

- 仓库 Actions 页面手动触发 **Docker Publish**（`workflow_dispatch`），
  tag 留空则取 `pyproject.toml` 里 `+xtac.` 之后的版本号；
- 或推一个 `v*` git tag 自动触发。

需要推送**本机已经构建并验证过**的镜像时，用脚本：

```bash
export GHCR_TOKEN=<classic PAT，需要 write:packages>
./docker/push_ghcr.sh 0.0.4
```

不传参数时同样从 `pyproject.toml` 推导 tag；默认连带推 `latest`，用 `--no-latest`
关闭。镜像很大，脚本内置了推送重试 —— `docker push` 按层续传，重试不会从头再来。

## 4. 初始化源码并构建

构建前必须拉取 git submodule —— `third_party/taccap-gripper` 会在镜像里从源码编译：

```bash
git submodule update --init --recursive --progress
docker compose build
```

> 只剩这一个 submodule。Pico4 的 `xensevr_pc_service_sdk` 仍然会编译（那是仓库内的
> pybind），但它链接的 C SDK 直接取自安装好的 `xensevr-pc-service` `.deb`，不再需要
> 克隆 `XenseVR-PC-Service`；`xensesdk` 则是 PyPI 上的预编译 wheel。

首次构建需要下载 Conda/CUDA/Python 依赖并编译原生模块，耗时较长，镜像也会比较大。
后续未修改 `conda_environment.yaml` 时会复用最耗时的依赖层。
Dockerfile 已包含 apt/curl 自动重试、CUDA 12.8 构建期覆盖和 flexible channel
priority，不再需要使用临时 `sed` 命令修改构建过程。

## 5. 启动与验证

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

后两行应当**打印同一个版本号**（0.0.4 镜像里是 `0.2.1`）。这不是巧合：pico4 绑定链接的
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

## 6. 数据、缓存与 GUI

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

## 7. 常见问题

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
- `docker push` 报 `denied`：包是公开的，拉取不需要登录，但推送要 —— 确认已
  `docker login ghcr.io`，且 PAT 带 `write:packages`。
- 构建需要离线/定制的 vendor wheel 或 `.deb`：先按 `setup_env.sh` 支持的
  `XENSESDK_WHEEL` / `XENSEVR_DEB` 方式将安装物纳入构建上下文，再定制 Dockerfile；
  默认镜像从项目规定的公开发布地址下载。
