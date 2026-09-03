# LeRobot-Xense Docker

预装 `xense-taccap` Conda 环境、CUDA 12.8、LeRobot-Xense、XenseSDK、
TacCap-Gripper SDK 和 Pico4 绑定的镜像。**从 GHCR 拉取，不需要自己构建，也不需要登录。**

构建镜像、发布新版本、离线交付见 [`MAINTAINING.md`](MAINTAINING.md)。

## 快速开始

**逐条敲,不要整块粘贴** —— `newgrp` 会开一个子 shell，整块粘贴时它后面的命令会被吃掉。

```bash
git clone https://github.com/XenseRobotics-AI/xense-taccap-lerobot.git
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
LEROBOT_IMAGE_TAG=0.0.8
```

改完用 `docker compose config --images` 确认解析结果。

## 常用操作

| 想做什么                 | 命令                                                                    |
| ------------------------ | ----------------------------------------------------------------------- |
| 跑一次性命令             | `docker compose run --rm xense-taccap lerobot-info`                     |
| 升级镜像                 | 改 `.env` 的 tag，再 `docker compose pull`                              |
| 确认解析到哪个镜像       | `docker compose config --images`                                        |
| 看远端有什么（不下载）   | `docker manifest inspect ghcr.io/xenserobotics-ai/xense-taccap-lerobot:0.0.8`   |
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

### 想直接在宿主机访问数据（推荐给要频繁看/删数据的机器）

在 `.env` 里把数据目录指到宿主机，数据集就直接落在你能打开的地方，不用每次拷出来：

```dotenv
LEROBOT_DATA_DIR=/home/你的用户名/.cache/huggingface/docker_data
```

带 `/` 的值是 bind mount，不带的当具名卷 —— 不设就还是默认的 `lerobot-data`。改完
`docker compose config` 确认一下，然后正常录制，数据会出现在
`<那个目录>/lerobot/`。

**但录制仍以 root 运行，所以文件属主是 root。** 录完把属主交还给自己：

```bash
docker compose run --rm --no-deps --entrypoint /bin/bash --user 0:0 \
    xense-taccap -lc "chown -R $(id -u):$(id -g) /data"
```

或者直接在宿主机 `sudo chown -R "$(id -u):$(id -g)" ~/.cache/huggingface/docker_data`。
用 `ls -ln` 检查（显示数字 uid/gid，比 `ls -l` 直观）：

```bash
ls -ln ~/.cache/huggingface/docker_data/lerobot
```

宿主机上的实际位置是 `/var/lib/docker/volumes/xense-taccap-lerobot_<卷名>/_data`，
属 root，直接 `ls` 要 sudo。查数据走容器更省事：

```bash
docker compose run --rm xense-taccap bash -lc 'ls -la /data/lerobot'
```

导出到宿主机。**以 root 读、拷完再交还属主** —— 这是唯一对已录数据成立的写法：

```bash
mkdir -p export
docker compose run --rm --no-deps \
    --entrypoint /bin/bash \
    -v "$PWD/export:/export" \
    xense-taccap \
    -lc "cp -a /data/lerobot /export/ && chown -R $(id -u):$(id -g) /export"
```

为什么不能直接用 `--user` 以自己的身份拷:

- **0.0.5 及更早的镜像录出来的视频是 `-rw------- root`**。`NamedTemporaryFile` 建的临时
  文件是 `0600`，`shutil.move` 保留权限，于是整份数据集只有 root 读得了。非 root 拷贝会
  在每个视频上报 `cp: cannot open '.../file-000.mp4' for reading: Permission denied`
  ——而元数据是正常 `0644`，所以**只有 `.mp4` 失败**，很像是个别文件坏了。这个已在源码里
  修掉（视频落盘后 `chmod 0644`），但要等下次发镜像才生效。
- **`--entrypoint /bin/bash` 仍然必需** —— `lerobot-entrypoint` 会 `chmod 0700` 运行目录，
  也会顺带启动 XenseVR 服务，拷个文件没必要。
- **先 `mkdir -p export`** —— Docker 自动创建的挂载点归 root。

`cp -a` 会连权限一起带过去，所以导出的视频仍是 `0600`（只是属主已经是你）。想让它们对同组
或其他用户可读，在上面那条末尾再加一段：

```bash
    && chmod -R u+rwX,go+rX /export
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

**后两行必须打印同一个版本号**（0.0.8 里是 `0.2.1`，与 0.0.7 相同）。pico4 绑定链接的 C SDK 就取自那个
`.deb`，两者不一致说明镜像是半新不旧的构建，不要拿它录数据。

## 版本

当前 **0.0.8**（`latest` 指向同一镜像）。只列影响使用方式的变化：

- **0.0.8** — **建议所有机器升级，没有 N 卡的机器必须升级**：
  - **软件编码（`libsvtav1`）的录制会话不再每条 episode 多占约 1.3 GB 内存。** 没有
    NVIDIA 显卡时 `--dataset.vcodec=auto` 落到 `libsvtav1`，此前每条 episode 结束后编码
    线程释放的内存留在 glibc 的线程 arena 里不还给系统，实测六条 5 秒的 episode 从
    2.5 GB 涨到 8.8 GB，16 GB 的机器一小时内进 swap —— 从录制循环看就是一直报
    `[slow_frame] ... overrun=`。现在每条 episode 存盘后归还（同样六条稳定在 2.6 GB）。
  - **数据修正：所有图像/视频特征的 `std` 此前恒为 0.0**（uint8 平方溢出，streaming 与
    非 streaming 两条路径都受影响；mean/min/max/分位数正确）。**已有数据集若下游按
    `std` 归一化，需要重算 stats。**
  - **出问题直接把 `~/xenselogs/session_<时间戳>.log` 发回来即可定位。** 日志新增
    `[session]`（机器、核数、GPU、编码器、相机配置）、`[slow_frame]` 的分相耗时
    （obs / build / add / display，加各设备分解）、每条 episode 一行 `[loop_summary]`
    （实际帧率对标称、超时次数、各阶段 p99、进程 CPU/内存/线程数）和每路
    `[encoder_summary]`（编码耗时、队列高水位、丢帧）。每个 episode 只在屏幕上打前 5 条
    `[slow_frame]`，其余落文件，每 5 秒汇总一条，不会再刷屏。
  - **夹爪编码器改为固件 100 Hz 推流**（IMU 开启时一同推流），录制循环不再每帧等一次
    串口往返。`--robot.gripper_stream_hz=0` 回到旧的逐帧读取。仅 leader 生效。
  - **`--display_data=true` 不再造成 overrun。** Rerun 日志搬到独立线程，viewer 跟不上时
    丢显示帧而不阻塞采集，丢帧数在退出时汇报一次。
  - **重录（左箭头）时 reset 阶段不再被跳过。** 以前重录只拿到 0 秒 reset，数采员听到
    "复位环境"去摆放物品的动作整段录进了下一条。
  - **`--robot.id` 必填，且一个数据集只属于一个工位**：`--resume` 时若与
    `meta/hardware.json` 里记录的不符直接报错。硬件清单改为按 episode 区间分段
    （epochs），中途换设备会开新段而不是静默错标。
  - 4 核以下的机器上实际帧率可能略低于标称（`sleep` 在高负载下晚醒，不算超时），
    `[loop_summary]` 会把实际 fps 打出来；数据集时间戳仍按标称帧率写。
  - Pico 头显立体轮询 120 → 60 Hz，退出时汇报丢帧数（非零才打印）。
  - 编码线程每帧少做约 2 ms 无用统计与一次 PIL 往返，8 路合计约省 0.6 核。
- **0.0.7** — **建议所有机器升级**，一条崩溃修复加一轮日志整改：
  - **录制不再因为一次误按方向键而崩掉。** 两条 episode 之间有大约 2 秒没人读键盘事件
    （存盘 + 编码器预热）。在这个空档里按下的方向右键会一直挂着，导致下一条 episode
    一帧未录就退出、reset 照常跑满，最后在存盘时抛
    `ValueError: You must add one or several frames` 把整个采集会话打死 —— 在按键之后
    两分多钟，中间还夹着一段看起来完全正常的 reset，现场几乎不可能把两件事联系起来。
  - **日志安静了，而且全都落盘。** 以前每存一条 episode 都会刷一屏
    `[mov,mp4,...] Auto-inserting h264_mp4toannexb`（那句本该压掉它的设置是无效的），
    另有每秒 180 条、约每分钟 1 MiB 的逐帧 `read took` 只进文件不进屏幕。现在两者都没了。
    同时上游 lerobot / xensesdk / libav 的日志统一成一种格式，并和我们自己的日志一起进
    `~/xenselogs/session_<时间戳>.log` —— 以前它们从不落盘。控制台等级可用
    `XENSE_LOG_LEVEL` 调。
  - **按键操作现在带时间戳**（`Right arrow pressed -> ...`）。注意键盘监听是**全局**的：
    任何窗口里按方向右键都会结束当前 episode，包括 Rerun 窗口（它的时间轴就用左右键
    翻帧）。数采员报"我没按它自己就退了"时，先看这行的时间戳。
  - **新增两个数据质量信号。** 相机后台采集卡顿时会告警，并说明大约多少帧录进去的是
    重复图像；每条 episode 结束会报一行 `[stale_frames] ...`，给出重复帧数、次数和最长
    连续段。两者只在真正录制期间统计。
  - 从爪固件 `tc-gu-01-slave.bin` 更新到 1.1.6（运动安全包络、I2t 降额、可配温度墙）。
    主爪镜像仍是 1.2.2 —— 机位上如果还跑着 1.2.0，升级是独立于本镜像的运维动作，走
    `python/examples/ota_update.py`，**刷完必须断电重启**。
  - xensesdk 2.1.1 → 2.1.2。
- **0.0.6** — **建议所有机器升级**，三条都是踩过的坑：
  - **录制不再因为语音提示崩溃。** `--play_sounds` 默认开，而镜像里没有 `spd-say`，
    于是第一集刚开始就 `FileNotFoundError`，清理时又抛一次，最后 `Aborted (core dumped)`。
    现在 TTS 失败只警告不中断。**升级后不再需要 `--play_sounds=false`。**
    注意**容器里仍然听不到声音** —— 镜像装了 `speech-dispatcher`，但没有语音合成器
    模块，`spd-say` 会以非零退出，然后被安全忽略。补上合成器反而更糟：容器里没有可用的
    音频输出，`spd-say --wait` 会**无限挂住**而不是失败，所以这条 blocking 调用现在带
    10 秒上限。想真正听到提示音，请在宿主机上跑录制，或自己映射音频服务。
  - **录出来的视频不再是 `-rw------- root`**，改为 `0644`，别的用户/账号读得了。
  - **`LEROBOT_DATA_DIR`** 可以把数据集直接落到宿主机目录，见上面「想直接在宿主机访问
    数据」。这条不需要新镜像，改 `.env` 即可。
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
