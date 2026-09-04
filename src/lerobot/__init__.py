#!/usr/bin/env python

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
Lists the robots, teleoperators, cameras and ported datasets this build exposes,
so they can be inspected without importing their (heavy, hardware-bound)
dependencies.

Example:
    ```python
        import lerobot
        print(lerobot.available_robots)
        print(lerobot.available_teleoperators)
        print(lerobot.available_cameras)
        print(lerobot.available_datasets)
    ```

NOTE: the simulation-environment and policy registries upstream carries here
(``available_envs``, ``available_tasks_per_env``, ``available_datasets_per_env``,
``env_task_pairs``, ``env_dataset_pairs``) are gone: this fork ships neither
``lerobot.envs`` nor ``lerobot.policies``, so they described things that could
not be constructed. ``available_policies`` is kept as an empty list because it
is part of the documented surface.
"""

from lerobot.__version__ import __version__  # noqa: F401

available_real_world_datasets = [
    "lerobot/aloha_mobile_cabinet",
    "lerobot/aloha_mobile_chair",
    "lerobot/aloha_mobile_elevator",
    "lerobot/aloha_mobile_shrimp",
    "lerobot/aloha_mobile_wash_pan",
    "lerobot/aloha_mobile_wipe_wine",
    "lerobot/aloha_static_battery",
    "lerobot/aloha_static_candy",
    "lerobot/aloha_static_coffee",
    "lerobot/aloha_static_coffee_new",
    "lerobot/aloha_static_cups_open",
    "lerobot/aloha_static_fork_pick_up",
    "lerobot/aloha_static_pingpong_test",
    "lerobot/aloha_static_pro_pencil",
    "lerobot/aloha_static_screw_driver",
    "lerobot/aloha_static_tape",
    "lerobot/aloha_static_thread_velcro",
    "lerobot/aloha_static_towel",
    "lerobot/aloha_static_vinh_cup",
    "lerobot/aloha_static_vinh_cup_left",
    "lerobot/aloha_static_ziploc_slide",
    "lerobot/umi_cup_in_the_wild",
    "lerobot/unitreeh1_fold_clothes",
    "lerobot/unitreeh1_rearrange_objects",
    "lerobot/unitreeh1_two_robot_greeting",
    "lerobot/unitreeh1_warehouse",
    "lerobot/nyu_rot_dataset",
    "lerobot/utokyo_saytap",
    "lerobot/imperialcollege_sawyer_wrist_cam",
    "lerobot/utokyo_xarm_bimanual",
    "lerobot/tokyo_u_lsmo",
    "lerobot/utokyo_pr2_opening_fridge",
    "lerobot/cmu_franka_exploration_dataset",
    "lerobot/cmu_stretch",
    "lerobot/asu_table_top",
    "lerobot/utokyo_pr2_tabletop_manipulation",
    "lerobot/utokyo_xarm_pick_and_place",
    "lerobot/ucsd_kitchen_dataset",
    "lerobot/austin_buds_dataset",
    "lerobot/dlr_sara_grid_clamp",
    "lerobot/conq_hose_manipulation",
    "lerobot/columbia_cairlab_pusht_real",
    "lerobot/dlr_sara_pour",
    "lerobot/dlr_edan_shared_control",
    "lerobot/ucsd_pick_and_place_dataset",
    "lerobot/berkeley_cable_routing",
    "lerobot/nyu_franka_play_dataset",
    "lerobot/austin_sirius_dataset",
    "lerobot/cmu_play_fusion",
    "lerobot/berkeley_gnm_sac_son",
    "lerobot/nyu_door_opening_surprising_effectiveness",
    "lerobot/berkeley_fanuc_manipulation",
    "lerobot/jaco_play",
    "lerobot/viola",
    "lerobot/kaist_nonprehensile",
    "lerobot/berkeley_mvp",
    "lerobot/uiuc_d3field",
    "lerobot/berkeley_gnm_recon",
    "lerobot/austin_sailor_dataset",
    "lerobot/utaustin_mutex",
    "lerobot/roboturk",
    "lerobot/stanford_hydra_dataset",
    "lerobot/berkeley_autolab_ur5",
    "lerobot/stanford_robocook",
    "lerobot/toto",
    "lerobot/fmb",
    "lerobot/droid_100",
    "lerobot/berkeley_rpt",
    "lerobot/stanford_kuka_multimodal_dataset",
    "lerobot/iamlab_cmu_pickup_insert",
    "lerobot/taco_play",
    "lerobot/berkeley_gnm_cory_hall",
    "lerobot/usc_cloth_sim",
]

available_datasets = sorted(set(available_real_world_datasets))

# Policies are not bundled in this build.
available_policies: list[str] = []
available_policies_per_env: dict[str, list[str]] = {}

# lists all available robots from `lerobot/robots`
available_robots = [
    "taccap_gripper",
    "bi_taccap_gripper",
    "xtac_umi_g1",
]

# lists all available cameras from `lerobot/cameras`
# These are the ``type`` strings registered by each CameraConfig subclass, i.e.
# what ``make_cameras_from_configs`` dispatches on — ``pico`` and ``zmq`` used to
# be missing here while being perfectly constructible.
available_cameras = [
    "opencv",
    "intelrealsense",
    "xense",
    "pico",
    "zmq",
]

# lists all available teleoperators from `lerobot/teleoperators`
available_teleoperators = [
    "pico4",
    "bi_pico4",
]

# lists all available motors from `lerobot/motors`
# The buses are still vendored from upstream, but this fork drives no motorised
# arm — the TacCap jaw is read-only over its own SDK, not through a MotorsBus.
available_motors = [
    "dynamixel",
    "feetech",
    "damiao",
    "robstride",
]
