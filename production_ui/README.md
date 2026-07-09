# Xense TacCap Production UI

这是一个独立的产线 UI 程序，封装了 `src/lerobot/scripts/client_commands.md`
里的显示/采集命令，并集成了数据集检查和 `push_dataset_to_hub` 上传能力。

## 运行

首次使用前安装 UI 依赖：

```bash
cd /path/to/xense-taccap-lerobot
mamba activate lerobot-taccap
python -m pip install -r production_ui/requirements.txt
```

启动 UI：

```bash
python production_ui/app.py
```

UI 使用 PySide6；采集相关依赖建议安装在 `lerobot-taccap` 环境。
旧电脑如果仍然使用 `lerobot-xense`，可以继续兼容，不需要马上重装。

如果启动时报：

```text
Could not find the Qt platform plugin "xcb"
```

先确认使用的是环境内 Python：

```bash
which python
python -c "import PySide6; print(PySide6.__file__)"
```

本 UI 会在启动前自动设置 Qt plugin path。若仍失败，通常是系统缺少
`libxcb-cursor.so.0`，安装系统库后重试：

```bash
sudo apt install libxcb-cursor0
```

## 功能

- 显示 / 冒烟测试：生成并运行 `lerobot_teleoperate`，只看实时画面和轨迹，不写数据。
- 正式采集：生成并运行 `lerobot_record`，写入 LeRobotDataset。
- 数据检查：生成并运行 `lerobot_check_dataset`。
- 上传 Hub：生成并运行 `push_dataset_to_hub`，支持 private、no videos、
  upload large folder、branch、tags、license 等参数。
- 配置：所有 UI 参数可保存为 JSON，也可从 JSON 加载复用。

运行命令会打开独立终端窗口，完整日志在终端里查看；UI 下方只保留启动状态、
扫描结果和错误摘要。

外部终端优先使用 `gnome-terminal`，也会尝试 `konsole`、`xfce4-terminal`、
`xterm` 和 `x-terminal-emulator`。运行中的采集/显示命令请在新终端里用
`Ctrl+C` 停止。

## 示例：双手显示命令如何配置

命令：

```bash
lerobot-teleoperate \
    --robot.type=bi_taccap_gripper \
    --fps=30 \
    --display_data=true \
    --robot.enable_tracker=false
```

在 UI 中这样填：

1. 顶部 `Robot` 区：
   - `Robot type`: `bi_taccap_gripper`
   - `Tracker on`: 取消勾选
   - `Use role`: 不勾选；不勾选时不会生成 `--robot.role=leader`
   - `Use side`: 双手设备不勾选；选择 `taccap_gripper` 单手模式时 UI 会自动勾选并要求选择 `left/right`
2. 打开 `显示 / 冒烟测试` 页签：
   - `FPS`: `30`
   - `Display data`: 勾选
   - `Trajectory`: 不勾选；不勾选时不会生成 `--show_trajectory=false`
3. 点击 `Scan Devices` 可读取左右夹爪、视触觉、腕部相机、Tracker SN。
4. 底部 `命令预览` 会随参数自动更新。
5. 点击 `Run Display` 运行显示，日志会在新终端窗口中输出。

采集请切到 `正式采集` 页签，填写 dataset repo_id、task、episode 数等，再点击
`Run Record`。显示和采集是两套入口，不要混用。

## 配置文件

示例配置在：

```text
production_ui/ui_config.example.json
```

建议每条产线或每个任务复制一份，例如：

```text
production_ui/line1_pick_cube.json
```

然后在 UI 的“配置”页加载、调整、保存。

## 说明

UI 默认使用当前 Python 解释器，并自动把仓库 `src/` 加到子进程
`PYTHONPATH`。顶部 `Python env` 下拉框可直接选择：

- `lerobot-taccap`：新命名环境，优先推荐。
- `lerobot-xense`：旧命名环境，用于兼容已有产线电脑。
- `Current Python`：使用启动 UI 的当前 Python。
- `Custom path`：手动指定 Python 解释器路径。

运行采集或上传前，请先确认：

- 已选择包含采集依赖的 Python 环境。
- `xensesdk`、`xense.taccap`、`xensevr_pc_service_sdk` 可 import。
- 需要上传 Hub 时已经执行 `huggingface-cli login`。
