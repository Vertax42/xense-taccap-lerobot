# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Records a dataset from a self-driven TacCap-Gripper (single or bimanual).

These handheld rigs are their own teleoperator: ``robot.get_observation()``
yields both the observation and (its pose + gripper subset) the demonstrated
action, so no separate ``--teleop`` is required. Frames are paired shifted by
one step (action[t] with obs[t-1]).

Example (bimanual TacCap-Gripper):

```shell
lerobot-record \
    --robot.type=bi_taccap_gripper \
    --robot.id=taccap_0 \
    --dataset.repo_id=<my_username>/<my_dataset_name> \
    --dataset.num_episodes=50 \
    --dataset.single_task="Pick up the cube" \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --display_data=true
```
"""

import contextlib
import time
import traceback
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat

# Load TacCap native libs before cv2/Pillow/torchvision. Those packages may
# preload vendored JPEG/TIFF libraries that conflict with the conda OpenCV libs
# used by xense.taccap.
with contextlib.suppress(ImportError):  # SDK absent: the TacCap robots fail later with a clear error
    from xense.taccap import (
        FollowerGripper as _TaccapFollowerGripper,  # noqa: F401
        LeaderGripper as _TaccapLeaderGripper,  # noqa: F401
    )

from lerobot.cameras import (  # noqa: F401
    CameraConfig,  # noqa: F401
)
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.cameras.zmq.configuration_zmq import ZMQCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts, hw_to_dataset_features
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_taccap_gripper,
    make_robot_from_config,
    taccap_gripper,
)
from lerobot.robots.taccap_gripper.common import check_dataset_station, write_hardware_manifest
from lerobot.robots.taccap_gripper.visualization import TaccapTrajectoryViz
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_pico4,
    make_teleoperator_from_config,
    pico4,
)
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    is_headless,
    refresh_listener_events,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import busy_wait, get_logger, set_capture_recording
from lerobot.utils.utils import (
    init_logging,
    log_say,
)
from lerobot.utils.visualization_utils import (
    RerunLogSink,
    init_rerun,
    log_rerun_data,
    select_display_observation,
)

logger = get_logger("lerobot_record")


def _format_slow_frame_obs_suffix(robot: Robot | None) -> str:
    if robot is None:
        return ""

    timing = getattr(robot, "_last_obs_timing", None)
    if not isinstance(timing, dict):
        return ""

    parts: list[str] = []
    total_ms = timing.get("total_ms")
    if isinstance(total_ms, (int, float)):
        parts.append(f"obs={float(total_ms):.1f}ms")

    arm_items = [
        (key[:-3], float(value))
        for key, value in timing.items()
        if key.endswith("_arm_ms") and isinstance(value, (int, float))
    ]
    if arm_items:
        parts.append(f"arms={sum(value for _, value in arm_items):.1f}ms")

    grip_items = [
        (key[:-3], float(value))
        for key, value in timing.items()
        if key.endswith("_grip_ms") and isinstance(value, (int, float))
    ]
    if grip_items:
        parts.append(f"grips={sum(value for _, value in grip_items):.1f}ms")

    cameras_ms = timing.get("cameras_ms")
    if isinstance(cameras_ms, (int, float)):
        parts.append(f"cams={float(cameras_ms):.1f}ms")

    cam_items = [
        (key[4:-4], float(value))
        for key, value in timing.items()
        if (key.startswith("cam[") and key.endswith("]_ms") and isinstance(value, (int, float)))
    ]
    cam_items.sort(key=lambda item: item[1], reverse=True)

    obs_part_items = arm_items + grip_items + cam_items
    obs_part_items.sort(key=lambda item: item[1], reverse=True)
    if obs_part_items:
        visible_obs_items = [item for item in obs_part_items if item[1] >= 0.1]
        if not visible_obs_items:
            visible_obs_items = obs_part_items
        top_parts = ", ".join(f"{name}={value:.1f}ms" for name, value in visible_obs_items[:4])
        parts.append(f"top_obs={top_parts}")

    return f" | {' '.join(parts)}" if parts else ""


def _record_loop_sleep(
    start_loop_t: float,
    fps: int,
    start_episode_t: float,
    robot: Robot | None = None,
) -> None:
    if fps <= 0:
        return

    budget_s = 1.0 / fps
    dt_s = time.perf_counter() - start_loop_t
    remaining_s = budget_s - dt_s
    if remaining_s > 0:
        busy_wait(remaining_s)
        return

    episode_t_s = time.perf_counter() - start_episode_t
    robot_name = (
        getattr(robot, "name", None) or getattr(type(robot), "__name__", "record") if robot is not None else "record"
    )
    logger.warn(
        f"[slow_frame] robot={robot_name} t={episode_t_s:.3f}s "
        f"loop={dt_s * 1e3:.1f}ms budget={budget_s * 1e3:.1f}ms "
        f"overrun={(-remaining_s) * 1e3:.1f}ms"
        f"{_format_slow_frame_obs_suffix(robot)}"
    )


# Robots that are their own teleoperator: ``get_observation()`` yields the
# observation, which already contains the demonstrated pose + gripper, with no
# separate teleop device (``teleop=None``). Handheld data-collection units like
# the TacCap-Gripper. Recording routes to ``self_driven_record_loop``, which
# logs the device's own demonstrated state (the ``action_features`` subset of
# the observation) as the action — shifted-frame, so action[t] pairs with
# obs[t-1] — instead of a separate ``teleop.get_action()``.
SELF_DRIVEN_RECORD_ROBOTS = frozenset({"taccap_gripper", "bi_taccap_gripper"})


@dataclass
class DatasetRecordConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # A short but accurate description of the task performed during the recording (e.g. "Pick the Lego block and drop it in the box on the right.")
    single_task: str
    # Root directory where the dataset will be stored (e.g. 'dataset/path'). If None, defaults to $HF_LEROBOT_HOME/repo_id.
    root: str | Path | None = None
    # Limit the frames per second.
    fps: int = 30
    # Number of seconds for data recording for each episode.
    episode_time_s: int | float = 60
    # Number of seconds for resetting the environment after each episode.
    reset_time_s: int | float = 60
    # Number of episodes to record.
    num_episodes: int = 50
    # Encode frames in the dataset into video
    video: bool = True
    # Upload dataset to Hugging Face hub.
    push_to_hub: bool = True
    # Upload on private repository on the Hugging Face hub.
    private: bool = False
    # Add tags to your dataset on the hub.
    tags: list[str] | None = None
    # Number of subprocesses handling the saving of frames as PNG. Set to 0 to use threads only;
    # set to ≥1 to use subprocesses, each using threads to write images. The best number of processes
    # and threads depends on your system. We recommend 4 threads per camera with 0 processes.
    # If fps is unstable, adjust the thread count. If still unstable, try using 1 or more subprocesses.
    num_image_writer_processes: int = 0
    # Number of threads writing the frames as png images on disk, per camera.
    # Too many threads might cause unstable teleoperation fps due to main thread being blocked.
    # Not enough threads might cause low camera fps.
    num_image_writer_threads_per_camera: int = 4
    # Number of episodes to record before batch encoding videos
    # Set to 1 for immediate encoding (default behavior), or higher for batched encoding
    video_encoding_batch_size: int = 1
    # Video codec for encoding videos. Options: 'h264', 'hevc', 'libsvtav1', 'auto',
    # or hardware-specific: 'h264_videotoolbox', 'h264_nvenc', 'h264_vaapi', 'h264_qsv'.
    # Use 'auto' to auto-detect the best available hardware encoder.
    vcodec: str = "auto"
    # Enable streaming video encoding: encode frames in real-time during capture instead
    # of writing PNG images first. Makes save_episode() near-instant. More info in the documentation: https://huggingface.co/docs/lerobot/streaming_video_encoding
    streaming_encoding: bool = True
    # Maximum number of frames to buffer per camera when using streaming encoding.
    # ~1s buffer at 30fps. Provides backpressure if the encoder can't keep up.
    encoder_queue_maxsize: int = 30
    # Number of threads per encoder instance. None = auto (codec default).
    # Lower values reduce CPU usage, maps to 'lp' (via svtav1-params) for libsvtav1 and 'threads' for h264/hevc..
    encoder_threads: int | None = None
    # Rename map for the observation to override the image and state keys
    rename_map: dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        if self.single_task is None:
            raise ValueError("You need to provide a task as argument in `single_task`.")


@dataclass
class RecordConfig:
    robot: RobotConfig
    dataset: DatasetRecordConfig
    # Teleoperator used to control the robot
    teleop: TeleoperatorConfig | None = None
    # Display all cameras on screen
    display_data: bool = False
    # Display data on a remote Rerun server
    display_ip: str | None = None
    # Port of the remote Rerun server
    display_port: int | None = None
    # JPEG-encode images before sending them to Rerun. Off by default: on a
    # bimanual rig with a head camera the encode measures ~15 ms per frame versus
    # ~3 ms without. Since RerunLogSink moved this onto its own thread it no
    # longer costs the record loop anything — it costs the *viewer* frames, which
    # the sink drops when the encode cannot keep up with 30 fps. Turn it on when
    # the viewer is on another machine (--display_ip), where the bandwidth it
    # saves is worth more than the display frames it drops.
    display_compressed_images: bool = False
    # Log camera images to Rerun every N-th frame (1 = every frame). Scalars are
    # always logged at full rate, so raising this thins the camera tiles without
    # making the tcp.* / gripper.pos plots sparse. Also off-loop; raise it when
    # the sink reports dropping a lot of frames and you would rather choose which
    # ones go than have it choose.
    display_image_every_n: int = 1
    # Overlay the 3D pose + breadcrumb trajectory view in Rerun (when display_data
    # is on and the device emits tcp.* poses). Auto-skips if enable_tracker=false.
    show_trajectory: bool = True
    # Use vocal synthesis to read events.
    play_sounds: bool = True
    # Resume recording on an existing dataset.
    resume: bool = False

    def __post_init__(self):
        if self.display_image_every_n < 1:
            raise ValueError(f"display_image_every_n must be >= 1 (1 = every frame), got {self.display_image_every_n}.")
        # Robots that act as both observation source and action source
        # (i.e. "self-driven" data-collection devices like handheld grippers)
        # do not require a separate teleoperator (e.g. ``taccap_gripper``).
        if self.teleop is None and self.robot.type not in SELF_DRIVEN_RECORD_ROBOTS:
            raise ValueError("A teleoperator configuration is required to control the robot.")


@safe_stop_image_writer
def self_driven_record_loop(
    robot: Robot,
    events: dict,
    fps: int,
    dataset: LeRobotDataset | None = None,
    control_time_s: int | None = None,
    single_task: str | None = None,
    display_data: bool = False,
    rerun_sink: RerunLogSink | None = None,
    display_features: dict | None = None,
    display_compressed_images: bool = False,
    display_image_every_n: int = 1,
):
    """Record loop for self-driven handheld devices (e.g. TacCap-Gripper).

    These robots have no separate teleoperator: ``robot.get_observation()``
    yields the observation, whose ``action_features`` subset (pose + gripper)
    is also the demonstrated action. ``send_action`` is a no-op — nothing is
    ever commanded; we just sample at ``fps`` and log.

    **Shifted-frame pairing (错帧).** Each recorded row pairs the *previous*
    observation with the *current* pose as the action — ``(obs[t-1],
    action=pose[t])`` — so the action leads its observation by one step and is
    a genuine "where to move next" target. Recording ``(obs[t], pose[t])`` in
    the same frame would make the action identical to the proprioception
    already inside the observation (a degenerate "stay put" target). The
    first iteration has no previous frame and is intentionally not recorded,
    so an episode of N samples yields N-1 frames.

    With ``dataset=None`` (the between-episode reset phase) the loop is a
    passive wait: it still honours the keyboard/stop events but reads no
    hardware and records nothing, so the operator can reposition the device.

    ``display_features`` (the robot's viewer-facing schema, if it has one) only
    narrows what reaches Rerun — the dataset always gets the full observation.
    On the TacCap grippers that is what would send a display-only tactile view
    (the amplified ``difference``) to the screen while the recorded ``rectify``
    image goes to disk; by default both are ``rectify``, so screen and disk agree.
    """
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    timestamp = 0
    start_episode_t = time.perf_counter()
    prev_observation_frame = None  # shifted-frame: pair obs[t-1] with pose[t]
    loop_iteration = -1

    while timestamp < control_time_s:
        loop_iteration += 1
        start_loop_t = time.perf_counter()
        refresh_listener_events(events)

        # `exit_early` is the *only* break condition, as in upstream's
        # `record_loop`, and the loop that breaks on it always consumes it.
        # `stop_recording` and `rerecord_episode` are intent flags owned by the
        # caller; this loop neither breaks on them nor clears them.
        #
        # Every keypress that should end this loop sets `exit_early` too
        # (`control_utils.on_press`: left also sets `rerecord_episode`, ESC also
        # sets `stop_recording`), so one condition is enough — and checking the
        # intent flags instead is actively wrong. Breaking on `rerecord_episode`
        # left `exit_early` unconsumed, and the reset phase is this same loop with
        # `dataset=None`, so it broke in its own first iteration: the retake got
        # 0s of reset while "Reset the environment" was still playing, and the
        # operator put the scene back *into* the next take. Measured 2.35s from
        # that announcement to "Recording episode 56". See CLAUDE.md →
        # "The event contract".
        if events["exit_early"]:
            events["exit_early"] = False
            logger.info("Exit early requested, exiting record loop early")
            break

        # Reset phase (dataset=None) is a passive wait: skip hardware reads
        # unless we need them for the Rerun display.
        observation = None
        action = None
        if dataset is not None or display_data:
            observation = robot.get_observation()

            # Graceful degradation on mid-episode hardware loss (e.g. a wrist
            # camera hot-unplug): the robot returns fallback frames instead of
            # raising, and flags device_lost. Stop the whole session cleanly so
            # the in-progress episode is saved rather than crashing the loop.
            if getattr(robot, "device_lost", False):
                logger.error("Device lost mid-recording; stopping to save recorded data.")
                events["stop_recording"] = True
                break
            # Self-driven device: the demonstrated action is the pose + gripper
            # subset of this same observation sample (images excluded — we
            # iterate action_features, not the full obs). Single hardware read.
            action = {k: observation[k] for k in robot.action_features if k in observation}

        if dataset is not None:
            current_observation_frame = build_dataset_frame(dataset.features, observation, prefix=OBS_STR)
            # Shifted-frame (错帧): the current pose (action[t]) is paired with
            # the PREVIOUS observation so the action leads obs by one step.
            # The first sample has no predecessor and is skipped.
            if prev_observation_frame is not None:
                action_frame = build_dataset_frame(dataset.features, action, prefix=ACTION)
                frame = {
                    **prev_observation_frame,
                    **action_frame,
                    "task": single_task,
                }
                dataset.add_frame(frame)
            prev_observation_frame = current_observation_frame

        if display_data and observation is not None:
            display_observation = select_display_observation(observation, display_features)
            # Counts loop iterations, not recorded frames, so the viewer keeps
            # a steady image cadence through the reset phase too.
            log_images = display_image_every_n <= 1 or loop_iteration % display_image_every_n == 0
            if rerun_sink is not None:
                # Hands the frame to a worker thread and returns. This is what
                # keeps the viewer out of the frame budget — see RerunLogSink.
                rerun_sink.submit(
                    display_observation,
                    action or {},
                    compress_images=display_compressed_images,
                    log_images=log_images,
                )
            else:
                # No sink (a direct caller that built none): log inline, as
                # before. Costs the loop ~3 ms/frame on a bimanual rig.
                log_rerun_data(
                    observation=display_observation,
                    action=action or {},
                    compress_images=display_compressed_images,
                    log_images=log_images,
                )

        _record_loop_sleep(
            start_loop_t=start_loop_t,
            fps=fps,
            start_episode_t=start_episode_t,
            robot=robot,
        )

        timestamp = time.perf_counter() - start_episode_t


@parser.wrap()
def record(cfg: RecordConfig) -> LeRobotDataset:
    init_logging()
    logger.info(pformat(asdict(cfg)))
    if cfg.display_data:
        init_rerun(session_name="recording", ip=cfg.display_ip, port=cfg.display_port)

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop) if cfg.teleop is not None else None

    dataset_features = combine_feature_dicts(
        hw_to_dataset_features(robot.action_features, prefix=ACTION, use_video=cfg.dataset.video),
        hw_to_dataset_features(robot.observation_features, prefix=OBS_STR, use_video=cfg.dataset.video),
    )

    dataset = None
    listener = None
    rerun_sink = None

    try:
        if cfg.resume:
            # Resume must operate on the local dataset; never fall back to a remote copy.
            dataset = LeRobotDataset(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
                local_files_only=True,
            )

            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer(
                    num_processes=cfg.dataset.num_image_writer_processes,
                    num_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                )
            sanity_check_dataset_robot_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
            # One dataset, one station. Checked here — before connect(), off
            # ``--robot.id`` alone — so a rig that cannot take this dataset says
            # so now instead of after every gripper, sensor and tracker has been
            # brought up. ``write_hardware_manifest`` re-checks below; this is
            # the early exit, not the enforcement.
            if hasattr(type(robot), "hardware_manifest"):
                check_dataset_station(dataset.root, robot.id)
        else:
            # Create empty dataset or load existing saved episodes
            sanity_check_dataset_name(cfg.dataset.repo_id)
            dataset = LeRobotDataset.create(
                cfg.dataset.repo_id,
                cfg.dataset.fps,
                root=cfg.dataset.root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=cfg.dataset.video,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
            )

        # Self-driven robots (TacCap-Gripper) have no teleoperator; connect the
        # optional one only for symmetric teardown.
        robot.connect()
        if teleop is not None:
            teleop.connect()

        # Which physical devices produced this dataset. It has to happen here:
        # the gripper firmware SNs are read over the wire at connect(), and the
        # dataset itself only stores ``robot_type`` — neither ``robot.id`` nor a
        # serial reaches meta/info.json, so without this the episodes are not
        # traceable to a rig at all. Robots that expose no manifest (anything but
        # TacCap) are skipped.
        # (asked of the class, not the instance: ``getattr(robot, ...)`` with a
        # default would also swallow an AttributeError raised *inside* the
        # property, turning a bug into a silently missing manifest.)
        # ``num_episodes`` is where this run starts writing, and it is what
        # bounds the hardware epochs: on a resume after a device swap it closes
        # the epoch the earlier episodes belong to and opens the one for these.
        if hasattr(type(robot), "hardware_manifest"):
            write_hardware_manifest(
                dataset.root,
                robot.hardware_manifest,
                logger,
                episode_index=dataset.num_episodes,
                # Each tactile sensor's runtime bundle goes in with it, so the
                # derived channels can be rebuilt from this dataset alone later.
                runtimes=getattr(robot, "tactile_runtimes", None) or None,
            )

        # 3D pose + trajectory overlay (no-op if the device emits no tcp.* poses).
        # The viewer is laid out from display_features when the robot has one —
        # observation_features, except that a tactile sensor configured with a
        # display-only view (difference) shows that in place of the recorded one.
        # Same thing by default, which displays the recorded rectify stream.
        display_features = getattr(robot, "display_features", None)
        traj_viz = None
        if cfg.display_data:
            traj_viz = TaccapTrajectoryViz(
                display_features or robot.observation_features,
                show_trajectory=cfg.show_trajectory,
            )
            if traj_viz.active:
                traj_viz.setup()
            else:
                traj_viz = None
            # Everything the viewer needs now happens on this worker's thread,
            # not the record loop's. The blueprint above stays on this thread:
            # it is sent once, before the loop.
            rerun_sink = RerunLogSink(traj_viz=traj_viz)

        listener, events = init_keyboard_listener(teleop=teleop)

        # A camera's background capture can stall without blocking the record
        # loop — async_read() hands back the cached frame — so the cost shows up
        # as duplicated frames in the dataset, and only while a dataset is
        # actually being written. Default the reporting window closed and open it
        # per episode below: encoder warm-up, the reset phase and the save/encode
        # gap all stall captures (8-stream nvenc starves the tactile threads for
        # ~0.3-0.9s) but record nothing, so warning there is noise nobody can act
        # on. Measured: a session's first stalls fire during
        # `[streaming_encoder] warming up`, entirely before "Recording episode 0".
        set_capture_recording(False)

        if not cfg.dataset.streaming_encoding:
            logger.info(
                "Streaming encoding is disabled. If you have capable hardware, consider enabling it for way faster episode saving. --dataset.streaming_encoding=true --dataset.encoder_threads=2 # --dataset.vcodec=auto. More info in the documentation: https://huggingface.co/docs/lerobot/streaming_video_encoding"
            )

        with VideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
                # Warm up the streaming encoder before the loop so the first frame
                # doesn't overrun on encoder/codec initialization.
                dataset.prepare_episode_recording()

                # Discard any exit request that arrived while nothing was
                # reading events. The keyboard listener runs for the whole
                # session, but only the record/reset loops consume these flags —
                # between the reset loop ending and the next episode starting
                # there is a gap (save_episode + the encoder warm-up above) with
                # no consumer, ~2s here with streaming encoding and much longer
                # without it.
                #
                # A right arrow in that gap — the natural double-press when the
                # reset happens to time out on its own just as the operator
                # reaches for the key — is still set when this episode's first
                # iteration checks it. The episode ends after zero frames, the
                # reset then runs its full duration (the flag was consumed by
                # that break), and `save_episode()` below hits
                # `validate_episode_buffer`, which raises on an empty buffer:
                # "You must add one or several frames with `add_frame`". That
                # kills the session, mid-collection, minutes after the press.
                #
                # `rerecord_episode` has a related hole — the record loop breaks
                # on it without clearing it — but that one is self-correcting: it
                # also short-circuits the reset, so the empty buffer reaches
                # `clear_episode_buffer()` instead of `save_episode()` and the
                # episode is simply retried. Cleared here anyway so the operator
                # does not see a "Re-record episode" for an episode that never
                # started.
                #
                # `stop_recording` is deliberately NOT cleared: that one should
                # survive the gap, and the `while` above has already acted on it.
                #
                # Upstream lerobot and the sister repo `lerobot-xense` both carry
                # this hole; these two lines are a deliberate divergence.
                events["exit_early"] = False
                events["rerecord_episode"] = False

                # Non-blocking, matching `lerobot-xense`: the phrase plays over
                # the take's first second rather than in front of it. Waiting for
                # it (`blocking=True`) opens a ~1s window in which nothing is
                # recorded but the operator, hearing the announcement end, has
                # already started moving — it loses the beginning of the
                # demonstration, and puts speech-dispatcher's latency inside the
                # record loop. A second of lead-in at the head of an episode costs
                # nothing.
                log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)

                # Fresh breadcrumb trail per episode so trajectories don't bleed
                # across takes.
                if rerun_sink is not None:
                    rerun_sink.reset_trajectory()

                set_capture_recording(True)
                try:
                    self_driven_record_loop(
                        robot=robot,
                        events=events,
                        fps=cfg.dataset.fps,
                        dataset=dataset,
                        control_time_s=cfg.dataset.episode_time_s,
                        single_task=cfg.dataset.single_task,
                        display_data=cfg.display_data,
                        rerun_sink=rerun_sink,
                        display_features=display_features,
                        display_compressed_images=cfg.display_compressed_images,
                        display_image_every_n=cfg.display_image_every_n,
                    )
                finally:
                    set_capture_recording(False)

                # How much of what just went to disk is duplicated frames. The
                # guard counts this by frame-object identity while reading (see
                # CameraReadGuard.stale_frame_report), so it is exact and costs
                # nothing: a long run is a capture stall, many one-frame runs are
                # the beat between the sensor's capture loop and this one, both
                # free-running at the same nominal rate.
                # INFO, not WARNING, on purpose: some duplication is expected
                # (the beat between two unsynchronised 30 Hz loops) and warning
                # on it every episode is the noise this whole change removed.
                # Promote to a warning once a rig's baseline is known and there
                # is a defensible threshold — there is not one yet.
                stale_report = getattr(robot, "stale_frame_report", None)
                if callable(stale_report):
                    for line in stale_report():
                        logger.info(f"[stale_frames] episode {dataset.num_episodes}{line}")

                # Execute a few seconds without recording to give time to manually reset the environment
                # Skip reset for the last episode to be recorded
                #
                # `or events["rerecord_episode"]` — a retake of the *last* episode
                # still gets reset time — only became reachable once the loop
                # stopped breaking on that flag. It used to enter the reset and
                # leave again in the same millisecond.
                if not events["stop_recording"] and (
                    (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
                ):
                    log_say("Reset the environment", cfg.play_sounds)
                    # Passive wait (dataset=None): no teleop, nothing to reset on a
                    # handheld — just let the operator reposition.
                    self_driven_record_loop(
                        robot=robot,
                        events=events,
                        fps=cfg.dataset.fps,
                        control_time_s=cfg.dataset.reset_time_s,
                        single_task=cfg.dataset.single_task,
                        display_data=cfg.display_data,
                        rerun_sink=rerun_sink,
                        display_features=display_features,
                        display_compressed_images=cfg.display_compressed_images,
                        display_image_every_n=cfg.display_image_every_n,
                    )

                if events["rerecord_episode"]:
                    log_say("Re-record episode", cfg.play_sounds)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue

                dataset.save_episode()
                recorded_episodes += 1
    finally:
        log_say("Stop recording", cfg.play_sounds, blocking=True)

        # Before the devices go: the worker still holds the last frames they
        # produced, and its drop count is only meaningful once it has stopped.
        if rerun_sink is not None:
            rerun_sink.close()

        if dataset:
            dataset.finalize()

        # Each device gets its own guard, and it catches BaseException: an
        # unguarded throw here — or an impatient second Ctrl+C — skipped every
        # device after it, and a skipped device keeps holding its /dev node, so
        # the next run fails to open hardware that is physically fine.
        for name, device in (("robot", robot), ("teleop", teleop)):
            if device is None:
                continue
            try:
                if device.is_connected:
                    device.disconnect()
            except BaseException as e:
                logger.error(f"Error disconnecting {name}: {e}\n{traceback.format_exc()}")

        if not is_headless() and listener:
            listener.stop()

        if cfg.dataset.push_to_hub:
            dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)

        log_say("Exiting", cfg.play_sounds)
    return dataset


def main():
    register_third_party_plugins()
    try:
        record()
    except KeyboardInterrupt:
        # record()'s finally block has already finalized the dataset, disconnected
        # devices and pushed to hub before the exception propagates here, so there
        # is nothing left to clean up — just suppress the noisy traceback.
        logger.info("Recording interrupted by user (Ctrl+C). Cleanup already done.")


if __name__ == "__main__":
    main()
