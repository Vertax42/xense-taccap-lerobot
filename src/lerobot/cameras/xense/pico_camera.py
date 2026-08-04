"""Standalone tkinter viewer for the Pico headset camera. Debug tool only.

This is **not** the recording path and not a lerobot ``Camera`` — it is a
script with its own ``__main__``, it is not registered in the camera factory,
and it shows each eye in its own panel without pairing them. For recording,
see ``lerobot.cameras.pico.PicoCamera``, which pairs the eyes, merges them and
plugs into ``--robot.enable_head_camera``.

Useful for answering "is the headset streaming at all, and at what size?"
without starting a whole recording session.

SDK surface used here:
    xrt.has_pico_camera_frame(eye_index)
    xrt.get_pico_camera_frame_metadata(eye_index)
    xrt.get_pico_camera_frame_jpeg(eye_index)
    xrt.get_left_pico_camera_frame() / get_right_pico_camera_frame()
"""

import argparse
import gc
import sys
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

REQUIRED_XRT_APIS = (
    "has_pico_camera_frame",
    "get_pico_camera_frame_metadata",
    "get_pico_camera_frame_jpeg",
)


def load_xensevr_sdk():
    import xensevr_pc_service_sdk as module

    missing = [name for name in REQUIRED_XRT_APIS if not hasattr(module, name)]
    if missing:
        raise RuntimeError(
            "xensevr_pc_service_sdk 缺少 Pico 相机接口: "
            f"{', '.join(missing)}\n"
            f"当前加载模块: {getattr(module, '__file__', '<unknown>')}\n"
            "请先把带 Pico 相机接口的 xensevr-pc-service-pybind 安装到当前 Python 环境。"
        )
    return module


xrt = load_xensevr_sdk()


EYES = {
    0: "left",
    1: "right",
}


def clear_screen():
    print("\033[2J\033[H", end="")


def resize_for_display(image, max_width):
    if max_width <= 0 or image.shape[1] <= max_width:
        return image

    scale = max_width / float(image.shape[1])
    height = max(1, int(image.shape[0] * scale))
    return cv2.resize(image, (max_width, height), interpolation=cv2.INTER_AREA)


def decode_eye_frame(eye_index, last_sequences, max_width):
    eye_name = EYES[eye_index]

    if not xrt.has_pico_camera_frame(eye_index):
        return None, f"[{eye_name:5}] waiting for Pico camera frame..."

    metadata = xrt.get_pico_camera_frame_metadata(eye_index)
    sequence = int(metadata["frame_sequence"])
    if last_sequences.get(eye_index) == sequence:
        return None, (
            f"[{eye_name:5}] unchanged seq={sequence} "
            f"size={metadata['width']}x{metadata['height']}"
        )

    jpeg_bytes = bytes(xrt.get_pico_camera_frame_jpeg(eye_index))
    jpeg_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    bgr_image = cv2.imdecode(jpeg_array, cv2.IMREAD_COLOR)

    del jpeg_array
    del jpeg_bytes

    if bgr_image is None:
        return None, f"[{eye_name:5}] failed to decode JPEG seq={sequence}"

    display_image = resize_for_display(bgr_image, max_width)
    if display_image is not bgr_image:
        del bgr_image

    rgb_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
    del display_image

    pil_image = Image.fromarray(rgb_image)
    del rgb_image

    last_sequences[eye_index] = sequence
    status = (
        f"[{eye_name:5}] displayed "
        f"device={metadata['device_id']} "
        f"eye={metadata['eye_index']} "
        f"size={metadata['width']}x{metadata['height']} "
        f"seq={sequence} "
        f"ts_ns={metadata['timestamp_ns']} "
        f"jpeg={metadata['jpeg_size']} bytes"
    )
    return pil_image, status


class CameraViewer:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.interval_ms = max(1, int((1.0 / args.hz if args.hz > 0 else 0.03) * 1000))
        self.last_sequences = {}
        self.displayed_once = set()
        self.loop_count = 0
        self.last_log_time = 0.0
        self.running = True
        self.sdk_initialized = False
        self.photo_refs = {}
        self.image_labels = {}
        self.status_vars = {}

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.stop)

    def _build_ui(self):
        self.root.title("Pico Camera Desktop Viewer")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)

        for eye_index, eye_name in EYES.items():
            frame = ttk.Frame(self.root, padding=8)
            frame.grid(row=0, column=eye_index, sticky="nsew")
            frame.rowconfigure(1, weight=1)
            frame.columnconfigure(0, weight=1)

            title = ttk.Label(frame, text=f"{eye_name.capitalize()} eye", anchor="center")
            title.grid(row=0, column=0, sticky="ew")

            image_label = ttk.Label(frame, text="waiting for frame", anchor="center")
            image_label.grid(row=1, column=0, sticky="nsew")
            self.image_labels[eye_index] = image_label

            status_var = tk.StringVar(value="waiting for Pico camera frame...")
            status = ttk.Label(frame, textvariable=status_var, anchor="center")
            status.grid(row=2, column=0, sticky="ew")
            self.status_vars[eye_index] = status_var

    def start(self):
        print("Initializing XenseVR PC Service SDK...")
        xrt.init()
        self.sdk_initialized = True
        print("SDK initialized. Close the window or press Ctrl+C to exit.")
        if self.args.once:
            print("--once enabled: exits after one left frame and one right frame are displayed.")
        self.root.after(0, self.update_once)

    def update_once(self):
        if not self.running:
            return

        statuses = []
        for eye_index in EYES:
            pil_image, status = decode_eye_frame(
                eye_index, self.last_sequences, self.args.max_width
            )
            statuses.append(status)
            self.status_vars[eye_index].set(status)

            if pil_image is not None:
                photo = ImageTk.PhotoImage(pil_image)
                del pil_image
                self.image_labels[eye_index].configure(image=photo, text="")
                self.photo_refs[eye_index] = photo
                self.displayed_once.add(eye_index)

        now = time.monotonic()
        if self.args.verbose or now - self.last_log_time >= self.args.log_interval:
            if not self.args.no_clear:
                clear_screen()
            print("=" * 100)
            print("Pico camera desktop viewer")
            print("Tkinter displays only one current PhotoImage per eye; old frame refs are overwritten.")
            print("=" * 100)
            for status in statuses:
                print(status)
            print("=" * 100)
            sys.stdout.flush()
            self.last_log_time = now

        if self.args.once and self.displayed_once == set(EYES.keys()):
            self.root.after(max(1, self.args.once_hold_ms), self.stop)
            return

        self.loop_count += 1
        if self.args.gc_every > 0 and self.loop_count % self.args.gc_every == 0:
            gc.collect()

        self.root.after(self.interval_ms, self.update_once)

    def stop(self):
        self.running = False
        self.photo_refs.clear()
        gc.collect()
        if self.sdk_initialized:
            print("Closing SDK...")
            xrt.close()
            self.sdk_initialized = False
            print("SDK closed.")
        self.root.destroy()


def run_camera_viewer(args):
    root = tk.Tk()
    viewer = CameraViewer(root, args)
    try:
        viewer.start()
        root.mainloop()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        if viewer.running:
            viewer.stop()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Display the latest Pico left/right camera frames on the desktop."
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=30.0,
        help="Polling/display refresh rate. Duplicate frame sequences are not decoded again.",
    )
    parser.add_argument(
        "--max-width",
        type=int,
        default=1280,
        help="Downscale each displayed image to this width. Use 0 to show native size.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Display one left frame and one right frame, then exit after --once-hold-ms.",
    )
    parser.add_argument(
        "--once-hold-ms",
        type=int,
        default=1000,
        help="When --once is set, keep the windows visible for this many milliseconds before exit.",
    )
    parser.add_argument(
        "--gc-every",
        type=int,
        default=120,
        help="Run Python garbage collection every N loops. Use 0 to disable periodic GC.",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=1.0,
        help="Seconds between terminal status refreshes.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print terminal status every loop.",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not clear the terminal between status refreshes.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_camera_viewer(parse_args())
