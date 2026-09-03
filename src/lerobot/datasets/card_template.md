---
# For reference on dataset card metadata, see the spec: https://github.com/huggingface/hub-docs/blob/main/datasetcard.md?plain=1
# Doc / guide: https://huggingface.co/docs/hub/datasets-cards
# prettier-ignore
{{card_data}}
---

This dataset was created using [LeRobot](https://github.com/huggingface/lerobot).

<img src="assets/footer.png" width="100%" alt="Xense Robotics"/>

<p align="center"><em>Xense Robotics dataset overview: representative frames from
TacVerse's bimanual visuo-tactile demonstrations.</em></p>

<img src="assets/teaser.png" width="100%" alt="TacVerse collection and processing pipeline"/>

<p align="center"><em>TacVerse is collected with XTac-UMI-G1 handheld grippers,
processed by TacFlow-Engine, and released as standard LeRobot datasets.</em></p>

{% if repo_id is defined and repo_id %}
Explore this dataset with the [LeRobot Dataset Viewer](https://huggingface.co/spaces/lerobot/visualize_dataset?path={{ repo_id }}).
{% endif %}

## Dataset Description

{{ dataset_description | default("", true) }}

- **Homepage:** {{ url | default("[More Information Needed]", true)}}
- **Paper:** {{ paper | default("[More Information Needed]", true)}}
- **License:** {{ license | default("[More Information Needed]", true)}}

## Dataset Structure

{{ dataset_structure | default("[More Information Needed]", true)}}

## Sensor key map

<p align="center">
<img src="assets/sensor_key_map.png" width="760" alt="Sensor key map — each image stream keyed to its physical mount"/>
</p>

<p align="center"><em>Tactile keys map to fingertip sensors; wrist keys to gripper cameras;
head keys to the headset's stereo eyes.</em></p>

## Citation

**BibTeX:**

```bibtex
@misc{xense-taccap-lerobot,
    author = {XenseRobotics Team},
    title = {LeRobot-Xense: LeRobot with Xense Tactile Robotics Support},
    howpublished = {\url{https://github.com/XenseRobotics-AI/xense-taccap-lerobot}},
    year = {2026}
}{% if citation_bibtex is defined and citation_bibtex %}

{{ citation_bibtex }}{% endif %}
```
