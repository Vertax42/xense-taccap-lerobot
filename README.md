# 🎯 Project Overview

🤗 This repository is a fork of [`lerobot`](https://github.com/huggingface/lerobot)
by XenseRobotics, used for Xense's multimodal tactile data acquisition system.
This branch tracks **upstream lerobot v5.1**, slimmed to the **TacCap-Gripper**
(single + bimanual) handheld UMI device and the **Pico4** teleoperator/tracker,
with Xense tactile cameras layered on top. See
[`src/lerobot/robots/taccap_gripper/README.md`](src/lerobot/robots/taccap_gripper/README.md)
for device-specific usage. For generic lerobot usage (datasets, policies,
training scripts) refer to the
[upstream README](https://github.com/huggingface/lerobot#readme).

## 🔧 Installation

Tested on Ubuntu 22.04, NVIDIA driver ≥ 570.144. Use
[`Mamba`](https://github.com/conda-forge/miniforge?tab=readme-ov-file#install)
(strongly recommended over plain conda — it's much faster on the
robostack-staging channel that ships ROS Humble + SOEM). v5.1 pins
**Python 3.12** and **PyTorch ≥ 2.2** with CUDA 12.8.

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

### 📦 Environment Setup

**Step 1:** 📂 Clone the repository with all submodules:

```bash
git clone \
  --recurse-submodules \
  https://github.com/Vertax42/lerobot-xense.git
cd lerobot-xense
```

> If you already cloned without submodules, initialize them manually:

> ```bash
> git submodule update --init --recursive --progress
> ```

This repository uses `third_party/` git submodules to manage hardware SDK dependencies:

| Submodule | Installed package |
|-----------|-------------------|
| `third_party/taccap-gripper` | `xense.taccap` (TacCap UMI tactile gripper SDK) |
| `third_party/XenseVR-PC-Service` | `xensevr_pc_service_sdk` (Pico4 teleop/tracker) |
| `third_party/XenseVR-RobotVision-PC` | ZED-M → Pico4 stereo passthrough (built separately) |

> Submodules default to their **GitHub** mirrors. Each entry in `.gitmodules`
> notes its internal **GitLab** mirror as an (optional) comment. To pull a
> submodule from GitLab instead, override its URL locally without editing the
> tracked file:
>
> ```bash
> git config submodule.third_party/taccap-gripper.url \
>   git@192.168.1.61:physical-ai/grippersdk/taccap-gripper.git
> git submodule sync third_party/taccap-gripper
> git submodule update --init --recursive third_party/taccap-gripper
> ```

> `xensesdk` is **not** a submodule — it is installed from the vendored wheel
> `dist/xensesdk-2.0.0-cp312-cp312-linux_x86_64.whl` (which bundles the patched
> `libxense_c.so` flash reader).

**Step 2:** 🐍 Create and activate the mamba environment:

```bash
bash ./setup_env.sh --mamba lerobot-xense
mamba activate lerobot-xense
```

> The default env name baked into `conda_environment.yaml` is
> `lerobot-xense`. You can pass a different name to `--mamba`,
> but the rest of this README and the openpi project assume
> `lerobot-xense`.

**Step 3:** 📦 Install LeRobot-Xense and all hardware SDK bindings:

```bash
bash ./setup_env.sh --install
```

This step will:

- Update the conda environment from `conda_environment.yaml`
- Install the main package from `pyproject.toml`
- Install `xensesdk` from the vendored wheel (`dist/xensesdk-2.0.0-cp312-cp312-linux_x86_64.whl`)
- Build and install the `third_party` SDK packages: `xensevr_pc_service_sdk` (Pico4) and `xense.taccap` (TacCap UMI gripper)

**Step 4:** ✅ Verify the installation:

```bash
python -c 'import xensevr_pc_service_sdk; print("xensevr_pc_service_sdk OK ->", xensevr_pc_service_sdk.__file__)'
python -c 'import xensesdk; print("xensesdk OK ->", xensesdk.__file__)'
python -c 'import xense.taccap; print("xense.taccap OK ->", xense.taccap.__file__)'
```

**Step 5:** 📌 **Note on FFmpeg / video:** v5.1 no longer pins `ffmpeg`
through conda (the robostack ICU pin conflicted with newer ffmpeg
builds). Video encoding/decoding is handled by `torchcodec` + `av`
wheels installed via `setup_env.sh --install`. If you need a system
ffmpeg with `libsvtav1`, install it separately (apt or upstream
static build):

```bash
# Optional: verify torchcodec wheel is loadable
python -c 'import torchcodec; print("torchcodec OK ->", torchcodec.__version__)'
```

**Step 6:** 🔌 **Serial-port permissions (required for the TacCap-Gripper).**
The gripper's MCU enumerates as `/dev/ttyACM*`, owned by the `dialout` group.
If your user is **not** in `dialout`, the SDK can list the devices but cannot
open the serial port to read the firmware SN — so `scan_grippers()` returns
`role=Unknown` / empty `firmware_sn`, and `connect()` fails with e.g.:

```
RuntimeError: No leader gripper discovered for the left side.
```

(Under the hood the open fails with
`IoError: SerialBus: open(/dev/serial/by-id/...): Permission denied`.)

Add your user to the `dialout` group **once**, then start a fresh session so
the group membership takes effect:

```bash
sudo usermod -aG dialout "$USER"
# log out and back in (or `newgrp dialout` for the current shell), then replug
```

Verify the gripper is fully readable — `role` must be `Leader`/`Follower`
(not `Unknown`) and `firmware_sn` non-empty:

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side.name, g.role.name, repr(g.firmware_sn))"
```

> If `firmware_sn` is still empty *after* fixing permissions, the device's SN
> was never burned (or its firmware is < V1.6) — that is a device/firmware
> issue, not a host one.

**Step 7:** 🔌 **Keep ModemManager off the gripper serial (one-time host setup).**
The gripper MCU is a CH343 USB-serial (`1a86:55d2`) that enumerates as a CDC-ACM
port. On every hot-plug, **ModemManager** (the cellular-modem service shipped by
default on Ubuntu/GNOME) probes the fresh port with AT commands and holds it open
for a few seconds — so a `connect()` in that window fails with:

```
IoError: SerialBus: open(/dev/serial/by-id/usb-1a86_USB_Dual_Serial_..-if02): Device or resource busy
```

Classic symptom: the **first** launch works (the port has settled), but unplug →
move to another USB port → relaunch immediately is **busy**. This is **not** a
tactile/camera/bandwidth problem. (`brltty`, the braille driver, grabs `1a86`
devices the same way if it is installed.) Quick workaround: wait ~3 s after
replug. Permanent fix — a udev rule telling ModemManager to ignore these devices
(it keeps managing real modems):

```bash
sudo tee /etc/udev/rules.d/99-taccap-ignore-modemmanager.rules >/dev/null <<'EOF'
# TacCap-Gripper MCUs are CH343 USB-serial (1a86:55d2) — keep ModemManager off them
ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify the gripper serial is now flagged ignored:

```bash
udevadm info -q property -n /dev/ttyACM0 | grep ID_MM_DEVICE_IGNORE   # -> ID_MM_DEVICE_IGNORE=1
mmcli -L                                                               # grippers no longer listed
```

Revert by deleting the rule file and reloading. (Alternatively, on a dedicated
robot PC with no cellular modem: `sudo systemctl disable --now ModemManager`.)

## 🔑 The `LeRobotDataset` format

A dataset in `LeRobotDataset` format is very simple to use. It can be loaded from a repository on the Hugging Face hub or a local folder simply with e.g. `dataset = LeRobotDataset("lerobot/aloha_static_coffee")` and can be indexed into like any Hugging Face and PyTorch dataset. For instance `dataset[0]` will retrieve a single temporal frame from the dataset containing observation(s) and an action as PyTorch tensors ready to be fed to a model.

A specificity of `LeRobotDataset` is that, rather than retrieving a single frame by its index, we can retrieve several frames based on their temporal relationship with the indexed frame, by setting `delta_timestamps` to a list of relative times with respect to the indexed frame. For example, with `delta_timestamps = {"observation.image": [-1, -0.5, -0.2, 0]}` one can retrieve, for a given index, 4 frames: 3 "previous" frames 1 second, 0.5 seconds, and 0.2 seconds before the indexed frame, and the indexed frame itself (corresponding to the 0 entry). See example [1_load_lerobot_dataset.py](https://github.com/huggingface/lerobot/blob/main/examples/dataset/load_lerobot_dataset.py) for more details on `delta_timestamps`.

Under the hood, the `LeRobotDataset` format makes use of several ways to serialize data which can be useful to understand if you plan to work more closely with this format. We tried to make a flexible yet simple dataset format that would cover most type of features and specificities present in reinforcement learning and robotics, in simulation and in real-world, with a focus on cameras and robot states but easily extended to other types of sensory inputs as long as they can be represented by a tensor.

Here are the important details and internal structure organization of a typical `LeRobotDataset` instantiated with `dataset = LeRobotDataset("lerobot/aloha_static_coffee")`. The exact features will change from dataset to dataset but not the main aspects:

```
dataset attributes:
  ├ hf_dataset: a Hugging Face dataset (backed by Arrow/parquet). Typical features example:
  │  ├ observation.images.cam_high (VideoFrame):
  │  │   VideoFrame = {'path': path to a mp4 video, 'timestamp' (float32): timestamp in the video}
  │  ├ observation.state (list of float32): position of an arm joints (for instance)
  │  ... (more observations)
  │  ├ action (list of float32): goal position of an arm joints (for instance)
  │  ├ episode_index (int64): index of the episode for this sample
  │  ├ frame_index (int64): index of the frame for this sample in the episode ; starts at 0 for each episode
  │  ├ timestamp (float32): timestamp in the episode
  │  ├ next.done (bool): indicates the end of an episode ; True for the last frame in each episode
  │  └ index (int64): general index in the whole dataset
  ├ meta: a LeRobotDatasetMetadata object containing:
  │  ├ info: a dictionary of metadata on the dataset
  │  │  ├ codebase_version (str): this is to keep track of the codebase version the dataset was created with
  │  │  ├ fps (int): frame per second the dataset is recorded/synchronized to
  │  │  ├ features (dict): all features contained in the dataset with their shapes and types
  │  │  ├ total_episodes (int): total number of episodes in the dataset
  │  │  ├ total_frames (int): total number of frames in the dataset
  │  │  ├ robot_type (str): robot type used for recording
  │  │  ├ data_path (str): formattable string for the parquet files
  │  │  └ video_path (str): formattable string for the video files (if using videos)
  │  ├ episodes: a DataFrame containing episode metadata with columns:
  │  │  ├ episode_index (int): index of the episode
  │  │  ├ tasks (list): list of tasks for this episode
  │  │  ├ length (int): number of frames in this episode
  │  │  ├ dataset_from_index (int): start index of this episode in the dataset
  │  │  └ dataset_to_index (int): end index of this episode in the dataset
  │  ├ stats: a dictionary of statistics (max, mean, min, std) for each feature in the dataset, for instance
  │  │  ├ observation.images.front_cam: {'max': tensor with same number of dimensions (e.g. `(c, 1, 1)` for images, `(c,)` for states), etc.}
  │  │  └ ...
  │  └ tasks: a DataFrame containing task information with task names as index and task_index as values
  ├ root (Path): local directory where the dataset is stored
  ├ image_transforms (Callable): optional image transformations to apply to visual modalities
  └ delta_timestamps (dict): optional delta timestamps for temporal queries
```

A `LeRobotDataset` is serialised using several widespread file formats for each of its parts, namely:

- hf_dataset stored using Hugging Face datasets library serialization to parquet
- videos are stored in mp4 format to save space
- metadata are stored in plain json/jsonl files

Dataset can be uploaded/downloaded from the HuggingFace hub seamlessly. To work on a local dataset, you can specify its location with the `root` argument if it's not in the default `~/.cache/huggingface/lerobot` location.

## Citation

If you use this codebase, please cite the original LeRobot project:

```bibtex
@misc{cadene2024lerobot,
    author = {Cadene, Remi and Alibert, Simon and Soare, Alexander and Gallouedec, Quentin and Zouitine, Adil and Palma, Steven and Kooijmans, Pepijn and Aractingi, Michel and Shukor, Mustafa and Aubakirova, Dana and Russi, Martino and Capuano, Francesco and Pascal, Caroline and Choghari, Jade and Moss, Jess and Wolf, Thomas},
    title = {LeRobot: State-of-the-art Machine Learning for Real-World Robotics in Pytorch},
    howpublished = "\url{https://github.com/huggingface/lerobot}",
    year = {2024}
}
```

If you use this fork (LeRobot-Xense) specifically, please also cite:

```bibtex
@misc{vertax2026lerobotxense,
    author = {vertax42 and Xense Robotics Team},
    title = {LeRobot-Xense: LeRobot with Xense Tactile Robotics Support},
    howpublished = "\url{https://github.com/Vertax42/lerobot-xense}",
    year = {2026}
}
```
