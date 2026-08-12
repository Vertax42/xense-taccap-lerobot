# LeRobot-Xense Docker

预装 `xense-taccap` Conda 环境、CUDA 12.8、LeRobot-Xense、XenseSDK、
TacCap-Gripper SDK 和 Pico4 绑定的镜像。**从 GHCR 拉取，不需要自己构建，也不需要登录。**

构建镜像、发布新版本、离线交付见 [`MAINTAINING.md`](MAINTAINING.md)。

## 快速开始

**逐条敲,不要整块粘贴** —— `newgrp` 会开一个子 shell，整块粘贴时它后面的命令会被吃掉。

```bash
git clone https://github.com/Vertax42/xense-taccap-lerobot.git
cd xense-taccap-lerobot
./docker/install_customer.sh          # 装 Docker / NVIDIA Toolkit / udev 规则，并拉镜像
```

装完在**宿主机**上继续，这两条不能跳过：

```bash
newgrp docker                         # 让 docker 组权限在当前终端生效
xhost +si:localuser:root              # 要显示 Rerun 等窗口
```

```bash
docker compose run --rm xense-taccap  # 进容器
```

脚本已经把你加进 `docker` 组了，但**当前终端不会自动生效** —— 不执行 `newgrp docker`
就直接进容器会得到 `permission denied`。**注销后重新登录是更干净的做法**:`newgrp` 之后
你在该终端新建的文件属组会变成 `docker`，重新登录则没有这个副作用。

容器里环境已激活，直接用：

```bash
lerobot-info
lerobot-find-cameras
```

`install_customer.sh` 需要几分钟到几十分钟（镜像约 21 GB）。如果本机要走代理就加
`XENSE_PROXY_URL=http://127.0.0.1:7897`。

## 宿主机要求

Ubuntu 22.04/24.04、`linux/amd64`、**NVIDIA 驱动 ≥ 570.144**。

Docker、NVIDIA Container Toolkit 和 TacCap 的 udev 规则都由 `install_customer.sh`
装好；**驱动要你自己先装**（涉及显卡型号、Secure Boot 和重启，脚本不碰）。

用户权限和 X11 授权见上面的快速开始。`xhost` 的授权用完可以撤销：

```bash
xhost -si:localuser:root
```

> Compose 使用 `privileged: true` + host 网络/IPC，以支持热插拔的触觉相机、串口、
> Pico4 和 CAN。请只在可信的机器人主机上运行。

## 录数据前请 pin 版本

默认拉的是 `latest`，它是**浮动**的 —— 下次发布会把它指向新镜像。正式采集前在仓库根目录
的 `.env` 里钉死（`.env` 不会被提交）：

```dotenv
LEROBOT_IMAGE_TAG=0.0.5
```

改完用 `docker compose config --images` 确认解析结果。

## 常用操作

| 想做什么                 | 命令                                                                    |
| ------------------------ | ----------------------------------------------------------------------- |
| 跑一次性命令             | `docker compose run --rm xense-taccap lerobot-info`                     |
| 升级镜像                 | 改 `.env` 的 tag，再 `docker compose pull`                              |
| 确认解析到哪个镜像       | `docker compose config --images`                                        |
| 看远端有什么（不下载）   | `docker manifest inspect ghcr.io/vertax42/xense-taccap-lerobot:0.0.5`   |
| 确认本地跑的是哪个       | `docker image inspect --format '{{index .RepoDigests 0}}' <镜像>:<tag>` |
| 不用 Pico4，关掉随启服务 | `START_XENSEVR_SERVICE=0 docker compose run --rm xense-taccap`          |
| 查看数据                 | `docker compose run --rm xense-taccap bash -lc 'ls -la /data'`          |

## 数据存在哪

**录的数据集和 HF 下载缓存不是同一个地方**，别混：

| 存什么                     | 环境变量          | 容器内路径                 | Docker 卷           |
| -------------------------- | ----------------- | -------------------------- | ------------------- |
| **你录的数据集**           | `HF_LEROBOT_HOME` | `/data/lerobot`            | `lerobot-data`      |
| HF Hub 缓存（模型/数据集） | `HF_HOME`         | `/root/.cache/huggingface` | `huggingface-cache` |
| Torch 权重缓存             | `TORCH_HOME`      | `/root/.cache/torch`       | `torch-cache`       |
| 触觉传感器配置缓存         | —                 | `/root/.xensesdk`          | `xensesdk-cache`    |

环境变量在 `docker/Dockerfile.user` 里设，卷映射在 `compose.yaml` 里。用 Docker
**具名卷**而不是仓库目录，所以 `docker compose run --rm` 每次删容器都不会丢数据。

宿主机上的实际位置是 `/var/lib/docker/volumes/xense-taccap-lerobot_<卷名>/_data`，
属 root，直接 `ls` 要 sudo。查数据走容器更省事：

```bash
docker compose run --rm xense-taccap bash -lc 'ls -la /data/lerobot'
```

导出到宿主机：

```bash
mkdir -p export
docker compose run --rm --no-deps \
    --entrypoint /bin/bash \
    --user "$(id -u):$(id -g)" \
    -v "$PWD/export:/export" \
    xense-taccap \
    -lc 'cp -a /data/lerobot /export/'
```

三个参数都是必需的，少一个就不成立：

- **`--entrypoint /bin/bash`** —— 绕开 `lerobot-entrypoint`。它会 `chmod 0700`
  运行目录，非 root 身份做不到，报
  `chmod: changing permissions of '/tmp/xdg-runtime': Operation not permitted`。
  顺带也免得为了拷个文件把 XenseVR 服务拉起来。
- **`--user`** —— 否则容器以 root 写文件，导出的东西你自己删不掉也改不了。
- **`mkdir -p export` 先建目录** —— Docker 自动创建的挂载点归 root，非 root 进去写会
  `Permission denied`。

已经导出成 root 属主的，用容器改回来（宿主机上 `chown` 要 sudo）：

```bash
docker compose run --rm --no-deps --entrypoint /bin/bash \
    -v "$PWD:/host" xense-taccap \
    -lc "chown -R $(id -u):$(id -g) /host/export"
```

> **不要用 `docker volume prune`。** 它删的是"没有容器在用"的卷，而你的数据卷平时正是
> 这个状态 —— 那条命令会把录好的数据一起删掉。清理镜像用 `docker image prune`，清理
> 构建缓存用 `docker builder prune`，这两个都不碰卷。

想把数据直接存到宿主机某个目录（比如挂了块大盘），改 `compose.yaml` 把
`lerobot-data:/data` 换成 `/your/path:/data`。

## 验证镜像

怀疑镜像有问题时，在容器里跑：

```bash
python -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
python -c 'import importlib.metadata as M; print("pico4 ->", M.version("xensevr_pc_service_sdk"))'
dpkg-query -W -f='daemon -> ${Version}\n' xensevr-pc-service
```

**后两行必须打印同一个版本号**（0.0.5 里是 `0.2.1`）。pico4 绑定链接的 C SDK 就取自那个
`.deb`，两者不一致说明镜像是半新不旧的构建，不要拿它录数据。

## 版本

当前 **0.0.5**（`latest` 指向同一镜像）。只列影响使用方式的变化：

- **0.0.5** — 安装改为从 GHCR 在线拉取，不再需要 tar 交付包。镜像内容与 0.0.4 相同，
  已装好 0.0.4 的机器没有必须升级的理由。
- **0.0.4** — `--robot.id` 接受纯数字并按 `--robot.type` 展开（`--robot.id=0` → 单臂
  `taccap_0`、双臂 `bi_taccap_0`）。旧写法不受影响。`--dataset.vcodec=auto` 改为真正
  打开一次编码器再判定，无 GPU 的机器会正确回退到 `libsvtav1`。

## 常见问题

| 现象                                         | 处理                                                                                                                                                                                                                                                                            |
| -------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `No such image: ghcr.io/...`                 | `image inspect` 只查本地。先 `docker compose pull`；只想看远端用 `manifest inspect`                                                                                                                                                                                             |
| `mamba activate` 报 `Shell not initialized`  | 环境本来就是激活的，不用 activate。要手动切环境先 `eval "$(mamba shell hook --shell bash)"`                                                                                                                                                                                     |
| `pull access denied ... 'docker login'`      | 不是权限问题（包是公开的）。用 `docker compose config --images` 看镜像名被解析成了什么，检查 `.env` 的 `LEROBOT_IMAGE`                                                                                                                                                          |
| `Unknown runtime specified nvidia`           | NVIDIA runtime 没在 Docker 里注册。`sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`，再用 `docker info --format '{{json .Runtimes}}'` 确认列出了 `nvidia`                                                                                  |
| `could not select device driver ... gpu`     | 没装 NVIDIA Container Toolkit,或装完没重启 Docker daemon                                                                                                                                                                                                                        |
| `/dev/ttyACM*` 存在但 busy                   | 按顶层 README 配置宿主机 ModemManager udev 规则，重新插拔                                                                                                                                                                                                                       |
| GUI 不显示                                   | 检查 `$DISPLAY`、`/tmp/.X11-unix` 和 `xhost +si:localuser:root`                                                                                                                                                                                                                 |
| Rerun 报 `Failed to create surface`          | 容器没拿到 NVIDIA 的 Vulkan ICD。容器里跑 `vulkaninfo --summary` 看有没有 NVIDIA 设备，没有就看下一行                                                                                                                                                                           |
| 容器内 `vulkaninfo` 报 `INCOMPATIBLE_DRIVER` | CUDA 正常但图形能力没注入。确认 `docker info --format '{{json .Runtimes}}'` 列出了 `nvidia`；没有就 `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`。**别把 compose 的 `runtime: nvidia` 改回 `gpus: all`** —— 后者只申请 compute+utility |
| `cuda: False` 但 `nvidia-smi` 正常           | 宿主机 CUDA 状态坏了（挂起/恢复后常见），与容器无关。`sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm`，或重启                                                                                                                                                                |
| 找不到 GSPS 但宿主机能看到                   | 确认经 Compose 启动，检查 `ls /dev/v4l/by-id/*GSPS*`，必要时重插 USB Hub 后 `sudo udevadm settle --timeout=20`                                                                                                                                                                  |
| `groups: cannot find name for group ID <n>`  | 无害。NVIDIA runtime 注入了宿主机的 `render` 组 GID，容器里没有同名组而已                                                                                                                                                                                                       |
