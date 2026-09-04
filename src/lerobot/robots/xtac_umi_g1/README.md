# xtac_umi_g1

The XTac-UMI-G1 rig: the bimanual [`bi_taccap_gripper`](../bi_taccap_gripper/) plus the
Pico headset, always recording it.

```bash
lerobot-teleoperate --robot.type=xtac_umi_g1 --robot.id=0 --fps=30 --display_data=true

lerobot-record --robot.type=xtac_umi_g1 --robot.id=0 \
    --dataset.repo_id=<org>/<name> --dataset.single_task='...'
```

## What it records

Everything `bi_taccap_gripper` records, plus:

| Key                        | Meaning                                                          |
| -------------------------- | ---------------------------------------------------------------- |
| `left_head` / `right_head` | headset camera, one key per **eye** (not per arm)                |
| `head_camera.x/y/z`        | headset position, same world frame as `{side}_tcp.*`             |
| `head_camera.r1..r6`       | headset orientation, first two rotation-matrix columns           |

So **29 state dimensions** (20 + 9) and **8 camera keys** (6 + 2), against 20 and 6 for
`bi_taccap_gripper`. The head pose is an action as well as an observation — where the
operator looked while demonstrating is part of the demonstration.

Headset knobs (`--robot.head_camera_eyes`, `head_camera_width` / `height` / `fps`,
`head_camera_pair_max_skew_ms`) are inherited unchanged; see the
[bimanual README](../bi_taccap_gripper/README.md) for what they accept.

## Why this is a type and not a flag

`enable_head_camera` used to be an ordinary config field on `bi_taccap_gripper`. It could
change what was recorded, but never what the recording *claimed* to be: a dataset's
`robot_type` comes from `robot.name`, a class attribute bound to the draccus registry key,
so `lerobot-record` wrote `bi_taccap_gripper` no matter what the flag said.

A head-enabled run therefore produced 29 state dimensions and 8 cameras under a label
meaning 20 and 6, with nothing at record time able to notice. Twelve datasets on disk were
mislabelled that way before the mismatch was caught downstream, by a viewer that checks a
dataset's shape against its declared robot type.

With the headset in the type, `--robot.type=` decides the shape and the label together and
they cannot disagree. `enable_head_camera` is still a field — the robot code branches on
it in five places, and the single-arm `taccap_gripper` still exposes it — but on this type
and on `bi_taccap_gripper` a value contradicting the class is rejected at config time,
naming the type to switch to.

Removing the head has to move the label back, or the same mismatch reappears from the other
direction. `convert_8_to_6_cameras` relabels `xtac_umi_g1` → `bi_taccap_gripper`, and
`modify_features` does the same when the head image keys are among those removed.

## `--robot.id`

A bare number expands to `xtac_umi_g1_<n>`, not `bi_taccap_<n>`. The station label is kept
distinct on purpose: `meta/hardware.json` keys provenance on `robot_id`, and two robot
types answering to one station id would leave a run's own manifest unable to say which rig
recorded it.
