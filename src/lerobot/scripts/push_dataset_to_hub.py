#!/usr/bin/env python
r"""
Push a local LeRobot dataset to the Hugging Face Hub.

This script is useful when:
1. The push_to_hub step failed during recording (e.g., network issues, SSL errors)
2. You want to push a previously recorded dataset to the Hub
3. You want to re-upload a dataset with different settings

Usage:
    # First login to Hugging Face with a token that can write datasets.
    huggingface-cli login

    # Basic usage with the installed console command.
    lerobot-push-dataset-to-hub \
        --repo-id Vertax/xense_flare_pick_and_place \
        --dataset-path ~/.cache/huggingface/lerobot/Vertax/xense_flare_pick_and_place

    # Equivalent module invocation, useful from a source checkout.
    python -m lerobot.scripts.push_dataset_to_hub \
        --repo-id Vertax/xense_flare_pick_and_place \
        --dataset-path ~/.cache/huggingface/lerobot/Vertax/xense_flare_pick_and_place

    # Push a dataset stored outside the default HF cache.
    lerobot-push-dataset-to-hub \
        --repo-id Xense/local_recording \
        --dataset-path /data/lerobot/local_recording

    # Limit the upload to selected files with glob patterns.
    lerobot-push-dataset-to-hub \
        --repo-id Xense/metadata_only_review \
        --dataset-path /data/lerobot/metadata_only_review \
        --allow-patterns "meta/**" "data/**"

    # Use the legacy upload_large_folder API when needed for an older hub setup.
    lerobot-push-dataset-to-hub \
        --repo-id Xense/forward-06_test \
        --dataset-path ~/.cache/huggingface/lerobot/Xense/forward-06_test \
        --upload-large-folder

    # Push as a private dataset.
    lerobot-push-dataset-to-hub \
        --repo-id Vertax/xense_flare_pick_and_place \
        --dataset-path ~/.cache/huggingface/lerobot/Vertax/xense_flare_pick_and_place \
        --private

    # Skip videos and upload only metadata, parquet files and the dataset card.
    lerobot-push-dataset-to-hub \
        --repo-id Vertax/xense_flare_pick_and_place \
        --dataset-path ~/.cache/huggingface/lerobot/Vertax/xense_flare_pick_and_place \
        --no-videos

    # Push to a branch, add card tags and override the dataset license.
    lerobot-push-dataset-to-hub \
        --repo-id Xense/experiment_20260902 \
        --dataset-path /data/lerobot/experiment_20260902 \
        --branch review \
        --tags taccap bimanual tactile \
        --license apache-2.0

    # Re-upload without updating the codebase-version tag.
    lerobot-push-dataset-to-hub \
        --repo-id Xense/experiment_20260902 \
        --dataset-path /data/lerobot/experiment_20260902 \
        --no-tag-version
"""

import argparse
import logging
import sys
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def setup_logging():
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(asctime)s %(filename)s:%(lineno)d %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def push_dataset_to_hub(
    dataset_path: Path,
    repo_id: str,
    branch: str | None = None,
    tags: list | None = None,
    license: str | None = "apache-2.0",
    tag_version: bool = True,
    push_videos: bool = True,
    private: bool = False,
    allow_patterns: list[str] | str | None = None,
    upload_large_folder: bool = False,
    **card_kwargs,
) -> None:
    """
    Push a local dataset to the Hugging Face Hub.

    Args:
        dataset_path: Path to the local dataset directory
        repo_id: Hub repository ID (e.g., "Vertax/xense_flare_pick_and_place")
        branch: Git branch to push to (default: main)
        tags: Tags to add to the dataset card
        license: License for the dataset
        tag_version: Whether to tag with codebase version
        push_videos: Whether to push video files
        private: Whether to make the repository private
        allow_patterns: Patterns of files to include
        upload_large_folder: Use upload_large_folder API for large datasets
        **card_kwargs: Additional arguments for the dataset card
    """
    dataset_path = Path(dataset_path).expanduser().resolve()

    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset path does not exist: {dataset_path}")

    logging.info(f"Pushing dataset to: {repo_id}")
    logging.info(f"Dataset path: {dataset_path}")
    logging.info(f"Private: {private}")
    logging.info(f"Push videos: {push_videos}")
    logging.info(f"Upload large folder: {upload_large_folder}")
    if not push_videos:
        logging.info("Skipping video files")

    logging.info("Loading local LeRobot dataset...")
    dataset = LeRobotDataset(repo_id=repo_id, root=dataset_path)

    logging.info("Starting upload...")
    try:
        dataset.push_to_hub(
            branch=branch,
            tags=tags,
            license=license,
            tag_version=tag_version,
            push_videos=push_videos,
            private=private,
            allow_patterns=allow_patterns,
            upload_large_folder=upload_large_folder,
            **card_kwargs,
        )
    except Exception as e:
        logging.error(f"Upload failed: {e}")
        logging.info("Tips:")
        logging.info("  - Try --upload-large-folder only if your installed huggingface_hub needs the legacy uploader")
        logging.info("  - Check your network connection")
        logging.info("  - Make sure you have write access to the repository")
        logging.info("  - Make sure you are logged in with: huggingface-cli login")
        raise

    logging.info(f"✅ Dataset successfully pushed to: https://huggingface.co/datasets/{repo_id}")


def main():
    parser = argparse.ArgumentParser(
        description="Push a local LeRobot dataset to the Hugging Face Hub",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "--dataset-path",
        type=str,
        required=True,
        help="Path to the local dataset directory (e.g., ~/.cache/huggingface/lerobot/username/dataset-name)",
    )
    parser.add_argument(
        "--repo-id",
        type=str,
        required=True,
        help="Hub repository ID (e.g., 'Vertax/xense_flare_pick_and_place_cubes_20260104')",
    )
    parser.add_argument(
        "--branch",
        type=str,
        default=None,
        help="Git branch to push to (default: main)",
    )
    parser.add_argument(
        "--tags",
        type=str,
        nargs="*",
        default=None,
        help="Tags to add to the dataset card",
    )
    parser.add_argument(
        "--license",
        type=str,
        default="apache-2.0",
        help="License for the dataset (default: apache-2.0)",
    )
    parser.add_argument(
        "--private",
        action="store_true",
        help="Make the repository private",
    )
    parser.add_argument(
        "--no-videos",
        action="store_true",
        help="Skip pushing video files",
    )
    parser.add_argument(
        "--upload-large-folder",
        action="store_true",
        help="Use the legacy upload_large_folder API instead of upload_folder",
    )
    parser.add_argument(
        "--allow-patterns",
        type=str,
        nargs="*",
        default=None,
        help="Only upload files matching these glob patterns (e.g., 'meta/**' 'data/**')",
    )
    parser.add_argument(
        "--no-tag-version",
        action="store_true",
        help="Do not tag with codebase version",
    )

    args = parser.parse_args()

    setup_logging()

    try:
        push_dataset_to_hub(
            dataset_path=args.dataset_path,
            repo_id=args.repo_id,
            branch=args.branch,
            tags=args.tags,
            license=args.license,
            tag_version=not args.no_tag_version,
            push_videos=not args.no_videos,
            private=args.private,
            allow_patterns=args.allow_patterns,
            upload_large_folder=args.upload_large_folder,
        )
    except KeyboardInterrupt:
        logging.info("\nUpload cancelled by user")
        sys.exit(1)
    except Exception as e:
        logging.error(f"Failed to push dataset: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
