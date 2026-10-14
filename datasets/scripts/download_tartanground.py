"""Download selected TartanGround (omni) trajectories: lidar + poses/metadata + imu + front RGB.

Usage: python download_tartanground.py ENV:TRAJ [ENV:TRAJ ...]
Data lands under datasets/data/ (git-ignored).
"""
import sys
from pathlib import Path

import tartanair as ta

ROOT = Path(__file__).resolve().parents[1] / "data"


def main(specs):
    ta.init(str(ROOT))
    for spec in specs:
        env, traj = spec.split(":")
        ta.download_ground(env=[env], version=["omni"], traj=[traj],
                           modality=["lidar", "meta", "imu", "image"],
                           camera_name=["lcam_front"],
                           unzip=True, delete_zip=True, num_workers=1,
                           data_source="huggingface")


if __name__ == "__main__":
    main(sys.argv[1:])
