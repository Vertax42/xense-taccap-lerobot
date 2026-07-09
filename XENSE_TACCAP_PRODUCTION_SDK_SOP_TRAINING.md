# Xense TacCap LeRobot 产线检测 SOP

适用仓库：`xense-taccap-lerobot`

目标：产线只做最小检测：环境、SDK、gripper 串口、实时显示。本文不做采集，不做 Pico 检测。

参考依据：顶层 `README.md` 的安装验证、串口权限、ModemManager 检查，以及 `src/lerobot/robots/bi_taccap_gripper/README.md` 的 live visualization 命令。

## 1. 进入环境

进入项目目录：

```bash
cd ~/xense-taccap-lerobot
```

激活环境。项目 README 默认环境名是 `lerobot-xense`：

```bash
mamba activate lerobot-xense
```

如果产线机器环境名是 `xense-taccap`，改用：

```bash
mamba activate xense-taccap
```

确认当前环境：

```bash
echo "${CONDA_DEFAULT_ENV:-NO_CONDA_ENV}"
```

合格：输出 `lerobot-xense` 或 `xense-taccap`。

## 2. SDK 检查

按 README 的安装验证检查核心 SDK：

```bash
python -c 'import xensesdk; print("xensesdk OK ->", xensesdk.__file__)'
python -c 'import xense.taccap; print("xense.taccap OK ->", xense.taccap.__file__)'
```

合格：两条命令都输出 `OK`，无 traceback。

## 3. Gripper 串口检查

按 README 的 gripper 可读性检查：

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side.name, g.role.name, repr(g.firmware_sn))"
```

双手合格：

- 能看到 `Left` 和 `Right`。
- `role` 不是 `Unknown`。
- `firmware_sn` 不是空字符串。

## 4. ModemManager 检查

如果 gripper 报 `Device or resource busy`，按 README 检查：

```bash
udevadm info -q property -n /dev/ttyACM0 | grep ID_MM_DEVICE_IGNORE
```

合格输出：

```text
ID_MM_DEVICE_IGNORE=1
```

如果没有该输出，交给工程师按 README 补充 TacCap 忽略 ModemManager 的 udev 规则。

## 5. 实时显示检测

按 `bi_taccap_gripper` README 的 live visualization 命令检测。产线固定关闭 tracker：

```bash
lerobot-teleoperate \
    --robot.type=bi_taccap_gripper \
    --fps=30 \
    --display_data=true \
    --robot.enable_tracker=false
```

合格：

- 显示窗口能打开。
- 双手 gripper 自动发现。
- 4 个触觉画面正常显示。
- 2 个腕部相机画面正常显示。
- 夹爪开合时数值有变化。
- 连续运行 60 秒无崩溃。

结束检测按 `Ctrl+C`。

## 6. 最终结论

全部通过才判定 `PASS`：

| 项目 | 结果 |
| --- | --- |
| 环境激活 | PASS / FAIL |
| SDK 检查 | PASS / FAIL |
| Gripper 串口检查 | PASS / FAIL |
| ModemManager 检查 | PASS / FAIL / N/A |
| 实时显示 60 秒 | PASS / FAIL |

任一项失败，先排除后重新完整检测。
