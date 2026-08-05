# Pico 相机接入说明

本文档记录 Pico 端相机画面接入 PC 侧的改动点、数据格式、构建安装注意事项，以及 Python 调用方式。当前目标是用 Pico 相机数据替代原来的 PC 侧相机输入路径；相机画面只需要给 PC 使用，不再回传给头显。

## 数据链路

整体链路如下：

```text
Pico / UE 应用
  -> TCP 自定义二进制帧 cmd=0x30
  -> XenseVR-PC-Service TCP 接收
  -> DeviceManagement 转成 PXREAServerSendCustomMessage
  -> gRPC feedback: deviceCustomMessage
  -> PXREARobotSDK callback: PXREADeviceCustomMessage
  -> xensevr_pc_service_sdk(pybind) 缓存最新左右眼 JPEG
  -> Python 侧读取/解码/显示/喂给上层应用
```

PC 侧不新增相机回传到头显的路径。相机帧只作为 `PXREADeviceCustomMessage` 到达 Python SDK 调用方。

## Pico 相机 TCP 帧格式

Pico 端沿用 XenseVR-PC-Service 已有 TCP 包格式：

```text
0x3F + cmd + payload_length + payload + timestamp + 0xA5
```

其中相机帧命令为：

```text
cmd = 0x30
```

payload 内部格式：

| 字段          | 类型   | 说明                                   |
| ------------- | ------ | -------------------------------------- |
| deviceIdLen   | uint8  | 设备 ID 字符串长度                     |
| deviceId      | bytes  | 设备 ID 字符串                         |
| eyeIndex      | uint8  | `0=left`, `1=right`                    |
| width         | uint16 | JPEG 原始宽度                          |
| height        | uint16 | JPEG 原始高度                          |
| frameSequence | uint32 | Pico 端递增帧序号                      |
| timestampNs   | uint64 | Pico 端时间戳，纳秒                    |
| jpegBytes     | bytes  | JPEG 图像数据，必须以 `0xFF 0xD8` 开头 |

当前 C++ 解析端使用 `memcpy` 读取非对齐数值，按当前 Pico/PC 小端平台解释。

## 代码改动点

### 1. XenseVR-PC-Service 接收 `0x30`

路径：

```text
third_party/XenseVR-PC-Service/RoboticsService/Business/Business_global.h
third_party/XenseVR-PC-Service/RoboticsService/Business/DeviceManage/devicemanagement.cpp
```

改动内容：

- 新增 `TCP_CLIENT_MSG_VIDEO_FRAME_WITH_TIMESTAMP 0x30`。
- `DeviceManagement::ReplySDKClient()` 收到 `0x30` 后，不解析 payload，直接按原始二进制转发为 `PXREAServerSendCustomMessage`。
- gRPC 层原有 `PXREAServerSendCustomMessage -> deviceCustomMessage` 路径继续复用。

这样可以尽量不改上层应用，也不新增专用相机 gRPC proto。

### 2. xensevr_pc_service_sdk(pybind) 缓存 Pico 相机帧

路径：

```text
src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/bindings/py_bindings.cpp
```

改动内容：

- 新增 `PicoCameraFrame` 结构体。
- 新增左右眼最新帧缓存：`std::array<PicoCameraFrame, 2> PicoCameraFrames`。
- 在 `PXREADeviceCustomMessage` callback 中解析 Pico payload。
- 只保留每只眼最新一帧 JPEG，不缓存历史帧。
- 新增 Python 接口：

```python
has_pico_camera_frame(eye_index: int) -> bool
get_pico_camera_frame_metadata(eye_index: int) -> dict
get_pico_camera_frame_jpeg(eye_index: int) -> bytes
get_pico_camera_frame(eye_index: int) -> dict
get_left_pico_camera_frame() -> dict
get_right_pico_camera_frame() -> dict
```

其中 `eye_index` 定义：

```text
0 = left eye
1 = right eye
```

## 最小 Python 调用

```python
import xensevr_pc_service_sdk as xrt

xrt.init()
try:
    # 0=left, 1=right
    if xrt.has_pico_camera_frame(0):
        meta = xrt.get_pico_camera_frame_metadata(0)
        jpeg = xrt.get_pico_camera_frame_jpeg(0)
        print(meta)
        print(len(jpeg))
finally:
    xrt.close()
```

`get_pico_camera_frame_metadata()` 返回示例：

```python
{
    "device_id": "...",
    "eye_index": 0,
    "width": 1280,
    "height": 720,
    "frame_sequence": 123,
    "timestamp_ns": 1234567890,
    "jpeg_size": 100000,
}
```

## 解码成 OpenCV / NumPy 图像

Python 接口返回的是 JPEG bytes，上层需要自行解码：

```python
import cv2
import numpy as np
import xensevr_pc_service_sdk as xrt


def read_pico_eye_bgr(eye_index: int):
    if not xrt.has_pico_camera_frame(eye_index):
        return None, None

    meta = xrt.get_pico_camera_frame_metadata(eye_index)
    jpeg = bytes(xrt.get_pico_camera_frame_jpeg(eye_index))
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    image_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    # 释放临时引用，避免循环里积累大对象
    del arr
    del jpeg

    return meta, image_bgr


xrt.init()
try:
    meta, image_bgr = read_pico_eye_bgr(0)
    if image_bgr is not None:
        print(meta, image_bgr.shape)
finally:
    xrt.close()
```

如果上层希望使用 RGB：

```python
image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
```

## 轮询左右眼最新帧

推荐用 `frame_sequence` 去重：同一个序号不要重复解码。这样可以降低 CPU/内存压力。

```python
import time
import cv2
import numpy as np
import xensevr_pc_service_sdk as xrt

last_seq = {0: None, 1: None}


def poll_eye(eye_index: int):
    if not xrt.has_pico_camera_frame(eye_index):
        return None

    meta = xrt.get_pico_camera_frame_metadata(eye_index)
    seq = int(meta["frame_sequence"])
    if last_seq[eye_index] == seq:
        return None

    jpeg = bytes(xrt.get_pico_camera_frame_jpeg(eye_index))
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    del arr
    del jpeg

    last_seq[eye_index] = seq
    return meta, image


xrt.init()
try:
    while True:
        for eye in (0, 1):
            result = poll_eye(eye)
            if result is None:
                continue
            meta, image = result
            if image is not None:
                print("eye", eye, "seq", meta["frame_sequence"], "shape", image.shape)
                # 在这里把 image 交给上层应用；不要 append 到 list 里长期保存。
                del image
        time.sleep(0.01)
finally:
    xrt.close()
```

## 显示到桌面

如果只是验证左右眼画面，可参考 `xense-taccap-lerobot/doc/pico_camera_integration.md` 的 Tkinter 显示方式。避免使用 `cv2.imshow()` 的原因是部分 conda/OpenCV wheel 会依赖 Qt `xcb` 插件，插件缺失时会报：

```text
Could not find the Qt platform plugin "xcb"
```

Tkinter 显示模式只保留每只眼当前 `PhotoImage` 引用，新帧覆盖旧帧，避免显存/内存持续上涨。

## 内存使用约束

当前设计只做“最新帧缓存”：

- C++ pybind 层：每只眼一个 `std::vector<uint8_t>` 保存最新 JPEG；新帧覆盖旧帧。
- Python 调用层：每次从 pybind 拿到的是一份 bytes 拷贝；循环中应尽快解码并释放临时对象。
- 上层应用不要把每一帧 `jpeg` / `np.ndarray` 追加进 list 或队列长期保存。
- 显示窗口只应保存当前帧引用；新帧到来时覆盖旧帧引用。

推荐实践：

```python
# 好：只保留当前帧
latest_image = image

# 不推荐：无限缓存所有帧
all_frames.append(image)
```

## 常见问题

### `AttributeError: module 'xensevr_pc_service_sdk' has no attribute 'has_pico_camera_frame'`

说明当前 Python 环境加载的是旧版 pybind。检查路径：

```bash
python - <<'PY'
import xensevr_pc_service_sdk as xrt
print(xrt.__file__)
print(hasattr(xrt, "has_pico_camera_frame"))
PY
```

解决方式：重新构建并安装 `src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind`。

### 一直 `waiting for Pico camera frame`

可能原因：

- Pico 端未连接 PC Service。
- Pico 端未发送 `cmd=0x30` 相机帧。
- PC Service 未使用包含 `0x30` 转发改动的版本。
- Python 程序未先调用 `xrt.init()`。

### JPEG 解码返回 `None`

可能原因：

- payload 格式和本文档不一致。
- JPEG bytes 不完整或不是以 `0xFF 0xD8` 开头。
- 读取到了非相机 custom message；当前解析函数会过滤掉不符合 Pico 相机 payload 的消息。

## 后续接入上层应用建议

为了尽量少改上层应用，可以把 Pico 相机封装成一个类似普通 camera 的 `read()` 接口：

```python
class PicoCamera:
    def __init__(self, eye_index: int):
        self.eye_index = eye_index
        self.last_seq = None

    def read(self):
        if not xrt.has_pico_camera_frame(self.eye_index):
            return None
        meta = xrt.get_pico_camera_frame_metadata(self.eye_index)
        if self.last_seq == meta["frame_sequence"]:
            return None
        jpeg = bytes(xrt.get_pico_camera_frame_jpeg(self.eye_index))
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        del arr
        del jpeg
        self.last_seq = meta["frame_sequence"]
        return image
```

然后让原来读取 ZED/Insight 图像的上层逻辑改为读取该 wrapper 的 `read()` 输出即可。
