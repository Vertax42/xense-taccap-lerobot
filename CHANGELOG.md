# Changelog

本文件的条目原来写在文档站的「版本与支持」页,记录的是需要升级到哪个 commit 之后才有的行为变化(原文形如"某功能需要 `<commit>` 之后的版本"、"`0.0.x` 镜像起包含")。
版本按 Docker 镜像 tag 划分(`ghcr.io/xenserobotics-ai/xense-taccap-lerobot:<tag>`,与 `pyproject.toml` 里 `0.5.1+xtac.<tag>` 同号),每条归属按仓库 `v0.0.x` tag 之间的提交范围核对;每条末尾的短哈希是引入该变化的提交。
格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/),新的在前,分类只用 Added / Changed / Removed / Fixed。

## [Unreleased]

### Changed

- 仓库、子模块与 GHCR 镜像路径改到 `XenseRobotics-AI` 组织(`573b820b`;取自 git log,版本页未记录)。

## [0.0.8] - 2026-09-02

版本页没有记录这一版,以下按 `v0.0.7..v0.0.8` 之间的提交整理。

### Added

- 数据集与工位绑定:`--robot.id` 写进数据集,`--resume` 续录时校验工位号一致(`dc0861be`)。
- 夹爪编码器 / IMU 改为固件推流,并接通 `_last_obs_timing` 探针(`24657e41`)。
- 录制 session 日志自足:`[session]` / `[slow_frame]` 分相 / `[loop_summary]`(`1e8ac099`)。
- TacVerse 数据集卡片模板及其资源(`f8836dfc`、`3d506521`)。

### Changed

- Rerun 日志移出录制循环,消除 `[slow_frame]` overrun(`160f4141`)。
- Pico 立体轮询 120 Hz 改为 60 Hz,并把丢帧计数接出来(`d3c763cc`)。
- 视频聚合先规划布局再逐文件单次写入(`a872b5ed`)。
- `taccap-gripper` 子模块升到 `3d44440`,URL 统一用 SSH 与正确大小写(`c15729bd`、`31efb9b6`)。

### Removed

- 从未被消费的 `go_start` 事件(`e0a41e84`)。

### Fixed

- 重录时 reset 阶段被跳过,复位动作被录进下一条(`c3b70237`)。
- 图像 / 视频特征的 std 恒为 0;uint8 统计改走精确 bincount(`1ade9315`)。
- 每个 episode 后 `malloc_trim` 回收 SVT 内存;编码线程去掉 PIL 往返(`ef005f9a`)。

## [0.0.7] - 2026-08-27

### Added

- 触觉传感器的 runtime bundle 随数据集落盘到 `meta/runtimes/<SN>-<北京时间>.bin`(一次采集会话一份,每枚约 841 KB),epoch 里的每个传感器指向自己那一份,从 `rectify` 流重建 depth / force / difference 只靠数据集本身即可;没有 `meta/runtimes/` 的老数据集应跳过重建,拿错 bundle 不会报错但结果是错的;参与计算的时间用 epoch 里的 `recorded_at`(`dbbdd08e`)。
- 腕相机鱼眼矫正:新增 `--robot.wrist_undistort`(默认关),开启后 `wrist_cam` 在落盘前按夹爪 flash 里的内参矫正,没标定的夹爪回退到 SDK 参考内参并告警;`meta/hardware.json` 每个 unit 多一项 `wrist_undistort`,录到一半改设置会另起一个 epoch,老数据集没有这一项等同于原始鱼眼(`e5b4445a`)。
- `libusb-dev` 列为独立的安装项,`setup_env.sh` 在缺 `libusb-0.1.so.4` 时告警(只告警不中断);缺它或版本落后时相机连不上且报错不提 libusb,内核更新后需 `sudo apt install -y --only-upgrade libusb-dev libusb-1.0-0`(`9a4d696b`)。

### Changed

- `meta/hardware.json` 从扁平的 `units` 改为 `epochs` 数组,每段记 `from_episode` / `to_episode`(左闭右开)、`recorded_at`、`robot_id`、`role`、`units`;续录时换了夹爪或传感器,旧段在当前集数处封口、新段接上。旧版本只打一条告警并把换后的每一集记在旧设备名下。读取端把老文件当成一个开口 epoch(`from_episode` 0、`to_episode` `null`)(`052eb354`)。
- 头显相机默认分辨率改为 640x480,与头显 APP 的出厂默认一致;`1024x768` / `1280x960` 仍可用。此前采集端只认后两档,头显停在默认 640 时 `--robot.enable_head_camera=true` 会因首帧尺寸不符连不上,`--robot.head_camera_width=640` 也会被判错(`4b5f5cea`)。
- SDK 子模块升到 0.1.9(`3dac16a`),`firmware/` 附带固件镜像 leader 1.2.2 / follower 1.1.5(本地工具链编译,`manifest.json` 的 `build` 为 `local`),修掉命令通道活锁、日志阻塞实时任务、上电越界写三个主从共用代码里的缺陷;刷完必须断电重插(`e1cebea0`)。随后子模块 pin 前移到 `a3382db`(`d1b9e79a`)。

### Removed

- `usbutils` 从推荐安装列表移除,`lsusb -t` 那半段排查说明一并去掉(`9a4d696b`)。

### Fixed

- 语音播报的阻塞调用加 10 秒上限,容器里自行装了语音合成器也不会把录制卡在收尾那一步(`d1ad7140`)。

## [0.0.6] - 2026-08-12

镜像里采集程序的行为、数据格式和三个 SDK 与 0.0.5 相同;升级不改变已录数据,也不需要重新标定。

### Added

- `.env` 里设 `LEROBOT_DATA_DIR` 可把数据集直接写进宿主机目录,不用再从容器里拷出来;不设时仍用具名卷 `lerobot-data`(`89239c71`)。

### Changed

- `compose.yaml` 从 `gpus: all` 改为 `runtime: nvidia`:此前容器能跑 CUDA,但 NVIDIA 的 Vulkan ICD 不会被注入,Rerun 窗口起不来(`Failed to create surface`);改后要求 NVIDIA runtime 已注册进 Docker,否则报 `Unknown runtime specified nvidia`(`d1ed6d46`,修正于 `93beb2aa`)。

### Removed

- `lerobot-teleoperate` 的 `--dryrun`,它只打印一句话、从不生效;脚本里带着它现在会报未知参数(`d46fcf66`)。

### Fixed

- 语音播报失败不再中断录制:此前镜像里没有 `spd-say`,第一集播报就抛异常、收尾再抛一次,进程直接崩掉,容器里等于录不了;修好后只告警一次,不必再带 `--play_sounds=false`。镜像装了 `speech-dispatcher` 但刻意不装语音合成器,播报被安全跳过(`94597ba2`)。
- 录出来的视频对非 root 可读:此前 `.mp4` 落盘为 `0600 root`(拼接用临时文件的权限在移动时被保留),非 root 导出时只有 `.mp4` 报 `Permission denied`;权限取决于录数据时的镜像,老数据仍需以 root 拷出再 `chown`(`dac15f74`)。
- 容器里 `mamba activate` 不再提示 `Shell not initialized`(`bebb7f7c`)。

## [0.0.5] - 2026-08-11

只有安装方式变了,镜像里的采集程序和三个 SDK 与 0.0.4 相同;已装好 0.0.4 的机器没有必须升级的理由,采集进行到一半时不要动。

### Changed

- Docker 改为默认从 GHCR 拉取:`compose.yaml` 默认镜像为 `ghcr.io/xenserobotics-ai/xense-taccap-lerobot`,`.env` 只需写 tag 一行;`.tar` 离线包仍支持但不再是默认路径。旧版本默认镜像是本机构建的名字,要拉 GHCR 必须同时写 `LEROBOT_IMAGE` 和 `LEROBOT_IMAGE_TAG`,只改 tag 不生效(`854d4cdf`)。

## [0.0.4] - 2026-08-10

### Added

- Docker 交付路径:交付目录、`install_customer.sh`、`compose.yaml`(`9387ef05`)。
- Pico4 头显相机(视频帧经 XenseVR PC Service v0.2.0 转发),`--robot.enable_head_camera` 在单夹爪上可用;Rerun 的 `/world` 3D 视图不再画 TRACKER 坐标系和虚线(PR #9,`ffc94d53`)。
- `--robot.id` 变成必填,漏了在解析命令行时就退出;录制时向数据集写 `meta/hardware.json`(夹爪固件 SN + 触觉 SN),旧版本的数据集没有这个文件,说不出自己是哪套硬件采的(`e8146c4e`)。
- `--robot.id` 可以只填数字,按 `--robot.type` 自动补前缀(`0` → `taccap_0` / `bi_taccap_0`);写全的形式新旧版本都接受(`04812536`)。
- `setup_env.sh --install` 先检查 `build-essential` / `cmake` / `pkg-config` / `git` / `curl` 在不在,缺了直接停下并打印该敲的 apt 命令;此前缺包要到编译阶段才报出来(`2892929a`)。
- 新增 `--display_image_every_n`(`f491cae5`)。
- SDK 子模块升到 0.1.7,`firmware/` 随仓库附带已发布的固件镜像(leader 1.2.0 / follower 1.1.0,命令集 V2.1);刷写与刷后校验需 SDK ≥ 0.1.7(`373ff74b`)。

### Changed

- 未做行程标定的主夹爪被直接拒绝连接,不再需要自己判断标没标过;旧版本没有这道拦截,要自己确认 `gripper.pos` 张到底能到 `1.0` 再开录(`4fb5b79b`)。
- `head_camera.*` 头显位姿从"仅观测"改为同时也是动作;`--display_compressed_images` 默认值从 `true` 改为 `false`,开 `--display_data=true` 时不再默认走 JPEG 压缩(`f491cae5`)。
- Pico4 的 C SDK(`libPXREARobotSDK.so`)改从已安装的 `.deb`(`/opt/apps/roboticsservice/SDK`)里取,`.deb` 基线提到 v0.2.1;递归克隆从约 33 MiB 降到约 1.6 MiB,安装不再链接 gRPC 静态库,`--install` 也不再现编译这个库(`42b44066`)。
- `--dataset.vcodec=auto` 通过真开一次编码会话判断有没有硬件编码器,没有 NVIDIA 驱动的主机自动落到 `libsvtav1`;旧版本要显式写 `--dataset.vcodec=libsvtav1`(`3b9d2deb`)。
- 子模块 URL 从 `git@github.com:` 改为 `https://`,没有 GitHub SSH key 的机器也能拉;老 clone 的 `.git/config` 还记着旧 URL,需 `git submodule sync --recursive` 一次(`ffc94d53` 之后;文档提交 `ac70e683`)。
- SDK 子模块升到 `83314c8`,附带固件 leader 1.2.1 / follower 1.1.1,无协议变化(`fc9e9b93`)。

### Removed

- Insight 头显相机链路,由 Pico4 头显相机取代(`ffc94d53`)。
- `third_party/XenseVR-PC-Service` 子模块(`42b44066`)。

## [0.0.3] - 2026-08-01

版本页从这一版之后才开始按 commit 记录变更,此处无条目。

[Unreleased]: https://github.com/XenseRobotics-AI/xense-taccap-lerobot/compare/v0.0.8...HEAD
[0.0.8]: https://github.com/XenseRobotics-AI/xense-taccap-lerobot/compare/v0.0.7...v0.0.8
[0.0.7]: https://github.com/XenseRobotics-AI/xense-taccap-lerobot/compare/v0.0.6...v0.0.7
[0.0.6]: https://github.com/XenseRobotics-AI/xense-taccap-lerobot/compare/v0.0.5...v0.0.6
[0.0.5]: https://github.com/XenseRobotics-AI/xense-taccap-lerobot/compare/68bd7b2c...v0.0.5
[0.0.4]: https://github.com/XenseRobotics-AI/xense-taccap-lerobot/compare/fc2621f7...68bd7b2c
[0.0.3]: https://github.com/XenseRobotics-AI/xense-taccap-lerobot/commit/fc2621f7
