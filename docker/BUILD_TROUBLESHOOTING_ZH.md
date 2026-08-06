# Docker 构建与硬件联调问题记录

记录日期：2026-08-05
最终镜像：`xense-taccap-lerobot:latest`
镜像内容大小约 10 GB，本地磁盘占用约 30 GB。

> 当前状态：本文记录的 apt/curl 重试、CUDA 求解、Rerun/Vulkan 依赖、
> `/dev` 透传和 Xense 缓存持久化已经正式写入 Dockerfile 与 Compose。
> 下文的 `sed` 构建命令仅用于保留历史排查过程，新构建直接运行
> `docker compose build --progress=plain`。

## 1. 镜像构建

构建前初始化硬件 SDK：

```bash
git submodule update --init --recursive --progress
```

### Docker Hub 超时

终端代理不会自动传给 Docker daemon。临时设置：

```bash
export HTTP_PROXY=http://127.0.0.1:7897
export HTTPS_PROXY=http://127.0.0.1:7897
export NO_PROXY=localhost,127.0.0.1,::1,192.168.0.0/16,10.0.0.0/8,172.16.0.0/12

sudo systemctl set-environment \
  HTTP_PROXY="$HTTP_PROXY" \
  HTTPS_PROXY="$HTTPS_PROXY" \
  NO_PROXY="$NO_PROXY"
sudo systemctl restart docker
```

### GitHub、apt 和 Conda 问题

实际遇到：

- GitHub 下载 Miniforge 时出现 HTTP/2 `PROTOCOL_ERROR`。
- Ubuntu apt 经过代理时返回 `502 Bad Gateway`。
- Docker 构建阶段无法自动检测 CUDA，且 strict channel priority 导致 CUDA 12.8 求解失败。
- Conda channel 索引 `repodata.json.zst` 下载超时，导致大量正常依赖被误报为 `does not exist`。

新版 Dockerfile 已将 Conda/Mamba 网络读取超时提高到 300 秒，单连接最多
重试 10 次，并为整个环境创建增加最多 5 次重试。出现大量
`does not exist` 前，应先检查日志中是否存在 `repodata.json.zst` 超时；如果有，
通常是索引下载不完整，而不是真正的版本冲突。

当时使用以下命令临时构建（仅作历史记录）：

```bash
export HTTPS_PROXY=http://127.0.0.1:7897

sed \
  -e 's/apt-get update/apt-get -o Acquire::Retries=10 -o Acquire::http::Timeout=60 update/' \
  -e 's/apt-get install -y/apt-get -o Acquire::Retries=10 -o Acquire::http::Timeout=60 install -y/' \
  -e 's/RUN curl -fsSL/RUN curl --http1.1 --retry 10 --retry-delay 5 --retry-all-errors -fsSL/' \
  -e 's|RUN mamba env create|RUN CONDA_OVERRIDE_CUDA=12.8 mamba env create --channel-priority flexible|' \
  -e 's|RUN bash ./setup_env.sh --install|RUN conda config --system --set channel_priority flexible \&\& CONDA_OVERRIDE_CUDA=12.8 bash ./setup_env.sh --install|' \
  docker/Dockerfile.user |
docker build \
  --network host \
  --build-arg HTTP_PROXY= \
  --build-arg http_proxy= \
  --build-arg ALL_PROXY= \
  --build-arg all_proxy= \
  --build-arg HTTPS_PROXY="$HTTPS_PROXY" \
  --build-arg https_proxy="$HTTPS_PROXY" \
  --progress=plain \
  -f - \
  -t xense-taccap-lerobot:latest \
  .
```

构建完成后确认：

```bash
docker images xense-taccap-lerobot
```

## 2. 容器启动与设备透传

项目依赖 `/dev/v4l/by-id`、`/dev/v4l/by-path` 和
`/dev/serial/by-path` 做硬件自动配对，因此需要透传完整 `/dev`。

同时为 XenseSDK 配置独立持久化缓存，避免每次临时容器都重新读取传感器 flash：

```bash
docker volume create xense-taccap-xensesdk-cache

docker compose run --rm --remove-orphans \
  -v /dev:/dev \
  -v /run/udev:/run/udev:ro \
  -v xense-taccap-xensesdk-cache:/root/.xensesdk \
  xense-taccap
```

宿主机应先确认四个 GSPS 设备存在：

```bash
ls -l /dev/v4l/by-id/ | grep GSPS
```

如果设备在读取 flash 后掉线，重新插拔 USB Hub并等待 udev：

```bash
sudo udevadm trigger --subsystem-match=video4linux
sudo udevadm settle --timeout=20
```

## 3. Rerun 图形界面

宿主机授权容器 root 访问 X11：

```bash
xhost +SI:localuser:root
```

镜像需要包含以下运行包：

```bash
apt-get update
apt-get install -y \
  libxkbcommon-x11-0 \
  xdg-utils \
  libvulkan1 \
  usbutils
dpkg --configure -a
```

容器内设置：

```bash
export XDG_RUNTIME_DIR=/tmp/xdg-runtime
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
export WGPU_BACKEND=vulkan
```

## 4. Pico4 启动顺序

1. 打开并配对两个 Pico4 tracker。
2. 启动头显中的 Unity VR Client，并保持前台运行。
3. 启动 Docker 容器，由 entrypoint 启动 XenseVR PC Service。
4. 正式录制期间不要重启 Unity Client，否则坐标原点会改变。

## 5. 最终验证命令

```bash
lerobot-teleoperate \
  --robot.type=bi_taccap_gripper \
  --fps=30 \
  --display_data=true \
  --robot.enable_tracker=true
```

最终验证通过：两个夹爪、两个 Pico4 tracker、四个触觉传感器、两个腕部相机、CUDA 和 Rerun 均可正常工作。

> 该记录写于夹爪固件还是 `1.1.0` 的时候：当时不支持 encoder-max 校准，程序
> 临时回退到 `gripper_open_rad=1.7`。两台 leader 现已升级到 `1.2.1` 并完成
> encoder-max 校准，`gripper.pos` 走的是固件里的实测行程，不再走那个回退。
> 如果连接日志里仍出现 `Firmware encoder-max calibration unavailable …`，
> 说明该台没校准过，按 `taccap_gripper/README.md` 的"硬件启动流程"补一次。
