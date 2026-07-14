# TacCap Tracker EE Calibration UI

独立的左右夹爪 EE / 夹持点标定工具。所有新增标定实现都放在本目录，不依赖
`production_ui` 主采集界面。

## 启动

```bash
cd /path/to/xense-taccap-lerobot
./calibrate-ui/xense-taccap-calibrate-ui
```

也可以直接指定解释器：

```bash
XENSE_TACCAP_PYTHON=/path/to/python ./calibrate-ui/xense-taccap-calibrate-ui
```

启动脚本会自动把所选 Python 的 `bin/` 目录加入 `PATH`，这样 Rerun SDK
调用 `rr.spawn()` 时能找到同一环境里的 `rerun` viewer。

## 标定流程

1. 打开 Pico4 headset、启动 Unity VR Client、启动 XenseVR PC Service。
2. 点击 `扫描 tracker`，确认左右 tracker SN 被填入。
   扫描逻辑与 `production_ui` 一致：优先用 PXREA C SDK 读取
   `Motion.joints[].sn`，失败后再 fallback 到 `xensevr_pc_service_sdk`。
3. 点击 `连接左右`。默认会打开 Rerun 三维窗口。
4. 选择 `left` 或 `right`。
5. 将对应夹爪的夹持端 / EE 接触点顶在同一个固定空间点上。
6. 改变 tracker 姿态并点击 `记录当前位姿`。每侧至少 4 个样本，建议 8-12 个，姿态变化要足够大。
7. 点击 `求解当前侧并刷新三维` 或 `求解左右并刷新三维`。求解成功后 Rerun 会自动重启并重放当前样本/结果。
8. 检查 RMSE / max residual。残差过大时清空当前侧重采。
9. 如需手动重放三维结果，点击 `重启 Rerun 显示标定 EE`。
10. 点击 `保存结果 JSON`。

窗口里的 `实时位姿 / 坐标链路` 会同步显示：

- `PICO world`: 固定世界坐标系。
- `{left,right} world_tracker`: tracker 在世界坐标系下的实时位姿。
- `{left,right} world_calibrated_ee`: 标定后 EE 在世界坐标系下的实时位姿。
- `{left,right} tracker_to_ee`: tracker 到 EE 的静态刚体外参。

## Rerun 视图

工具会显示：

- `/world/pico_world`: Pico remap 后的世界坐标系位姿，固定为标定参考系。
- `/world/{left,right}/tracker`: tracker 自身实时位姿。
- `/world/{left,right}/tracker/tracker_to_ee`: 标定出的 tracker 到 EE 的静态相对位姿。
- `/world/{left,right}/tracker/tracker_to_ee_link`: tracker 原点到 EE 原点的刚性连接线。
- `/world/{left,right}/calibrated_ee`: 使用 `T_world_tracker * T_tracker_ee` 实时显示的 EE 位姿。
- `/world/{left,right}/calibrated_ee_trail`: 标定后 EE 原点轨迹。
- `/world/{left,right}/tracker_trail`: tracker 原点轨迹。
- `/world/{left,right}/samples/*`: 已记录的不同 tracker 位姿。
- `/world/{left,right}/fixed_point`: pivot 求出的固定空间点。
- `/world/{left,right}/residuals`: 每个样本的 EE 点到固定点误差线。

## 结果含义

固定点 pivot 标定求的是：

```text
t_world_tracker_i + R_world_tracker_i @ p_tracker_ee = p_world_fixed
```

输出的 `tracker_to_ee_pos` 就是 `p_tracker_ee`，单位为米。

注意：这个方法只能求 EE / 夹持点的位置，不能单靠一个固定点求出完整 TCP 朝向。
因此 `tracker_to_ee_quat` 当前保存为 identity：

```text
[1.0, 0.0, 0.0, 0.0]
```

如果后续需要完整 6D TCP 外参，需要增加方向夹具或已知姿态约束。

## 输出文件

默认保存到：

```text
calibrate-ui/tracker_ee_calibration.json
```

JSON 中包含每侧结果和可用于 LeRobot 命令的 `robot_args`，例如双手：

```text
--robot.left_tracker_to_ee_pos=[...]
--robot.left_tracker_to_ee_quat=[1,0,0,0]
--robot.right_tracker_to_ee_pos=[...]
--robot.right_tracker_to_ee_quat=[1,0,0,0]
```
