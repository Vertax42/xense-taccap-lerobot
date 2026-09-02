# LeRobot-Xense Docker 维护者说明

面向构建和发布镜像的人。**使用镜像不需要看这份文档**，用法见
[`README.md`](README.md)。

## 1. 镜像 tag 从哪来

镜像的 tag 由 `pyproject.toml` 的 `version = "0.5.1+xtac.<版本>"` 推导（Docker tag
不能含 `+`，故只取 `+xtac.` 之后的部分）。`.github/workflows/docker-publish.yml` 和
`docker/push_ghcr.sh` 用同一段 `sed` 推导，改动时要同步两处。

每次发布同时打一个 `sha-<commit>` tag，可以精确追溯镜像是从哪个源码提交构建的。

## 2. 本地构建

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

`docker compose build` 打出来的镜像名和 compose 的默认值一致，也就是
`ghcr.io/vertax42/xense-taccap-lerobot:latest` —— 和拉下来的镜像同名。想构建成别的
名字或版本，用 `LEROBOT_IMAGE` / `LEROBOT_IMAGE_TAG` 覆盖。

**注意 tag：`docker compose build` 默认打 `latest`。** 要构建一个具体版本，构建时就得
带上 tag，否则后面按版本号找镜像的脚本会找不到：

```bash
LEROBOT_IMAGE_TAG=0.0.5 docker compose build
```

## 3. 发布

**正常发布路径是推一个 `v*` git tag**，由 `.github/workflows/docker-publish.yml`
在托管 runner 上构建并推送，不占用本机上行带宽：

```bash
# 1. 改 pyproject.toml 的 version，并更新 README 的版本号与变更说明
#    例如 version = "0.5.1+xtac.0.0.8"
# 2. 合并到 main
# 3. 打 tag
git tag -a v0.0.8 -m 'release 0.0.8'
git push origin v0.0.8
```

tag 名去掉前导 `v` 就是镜像 tag。workflow 同时推 `latest` 和 `sha-<commit>`。
构建约 13–15 分钟（registry 构建缓存生效时）。

也可以在仓库 Actions 页面手动触发 **Docker Publish**（`workflow_dispatch`），
tag 留空则取 `pyproject.toml` 里 `+xtac.` 之后的版本号，`push_latest` 可关掉。

发布后确认三件事：

```bash
R=ghcr.io/vertax42/xense-taccap-lerobot
docker manifest inspect $R:0.0.8 | grep -m1 digest   # 新 tag 存在
docker manifest inspect $R:latest | grep -m1 digest  # 与上面一致
```

`latest` 是浮动的，发布会把它移到新镜像上；**旧的版本号 tag 不受影响**。

> **不要重复发布同一个版本号。** workflow 会用新构建覆盖掉那个 tag，digest 变了但
> 版本号没变，任何 pin 了它的机器下次 pull 会静默换到另一个镜像。要重发就升版本号。

镜像包是 **public** 的，拉取不需要登录 —— 与仓库本身一致：镜像里的 XenseSDK 来自公开
PyPI，XenseVR-PC-Service 的 `.deb` 来自公开 GitHub release，taccap-gripper submodule
也是公开仓库，没有一样是靠镜像才拿得到的。只有**推送**需要凭据。

## 4. 应急手段

以下两条都不是常规路径，也不进客户文档。

### 4.1 手动推送本机镜像

workflow 挂了，或者要发布的就是刚在本机实机验证过的那个镜像时：

```bash
export GHCR_TOKEN=<classic PAT，需要 write:packages>
./docker/push_ghcr.sh 0.0.5
```

不传参数时同样从 `pyproject.toml` 推导 tag；默认连带推 `latest`，用 `--no-latest`
关闭。镜像很大，脚本内置了推送重试 —— `docker push` 按层续传，重试不会从头再来。

### 4.2 离线 tar 交付

只在客户机**完全无法访问 ghcr.io** 时使用。开发机上：

```bash
LEROBOT_IMAGE_TAG=0.0.5 docker compose build
LEROBOT_IMAGE_TAG=0.0.5 ./docker/package_customer_delivery.sh
```

两条命令的 `LEROBOT_IMAGE_TAG` 必须一致，原因见第 2 节末尾。

脚本会在 `dist/customer/` 下生成一个交付目录，包含 tar、`SHA256SUMS`、
`compose.yaml`、`.env` 和 `install_customer.sh`。把整个目录复制到客户机后：

```bash
cd xense-taccap-lerobot-0.0.5-linux-amd64
./install_customer.sh
```

`install_customer.sh` 看到同目录下有 tar 就自动走离线装载（校验 SHA256 后
`docker load`），宿主机准备步骤与在线路径完全相同。目录里有多个 tar 时它会拒绝猜测，
要求把目标 tar 作为参数传入。

## 5. 维护者侧常见问题

- `Missing git submodules`：在仓库根目录执行
  `git submodule update --init --recursive` 后重新构建。
- 打包/推送脚本报 `Docker image not found: ...:0.0.5`：`docker compose build` 打的 tag 是
  `latest`。用 `LEROBOT_IMAGE_TAG=0.0.5 docker compose build` 重新构建，或先
  `docker tag` 到目标 tag。
- `docker push` 报 `denied`：包是公开的，拉取不需要登录，但推送要 —— 确认已
  `docker login ghcr.io`，且 PAT 带 `write:packages`。
- 构建需要离线/定制的 vendor wheel 或 `.deb`：先按 `setup_env.sh` 支持的
  `XENSESDK_WHEEL` / `XENSEVR_DEB` 方式将安装物纳入构建上下文，再定制 Dockerfile；
  默认镜像从项目规定的公开发布地址下载。
