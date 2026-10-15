#!/usr/bin/env python3
"""Download TartanGround (omni) trajectories from Hugging Face: lidar, poses/metadata, imu, front RGB.

    python3 datasets/scripts/download_tartanground.py                      # the experiment set (DEFAULT_TRAJECTORIES)
    python3 datasets/scripts/download_tartanground.py Hospital:P0005       # specific ENV:TRAJ
    python3 datasets/scripts/download_tartanground.py --no-images --dry-run
    python3 datasets/scripts/download_tartanground.py --modalities metadata imu   # small files only

Only needs `huggingface_hub` (pip install "huggingface_hub==1.33.0"); no torch/CUDA. Files come from the public
dataset theairlabcmu/TartanGround (CC-BY-4.0) at a pinned revision, so every download gets the same data.
Each <modality>.zip is unpacked into datasets/data/<ENV>/Data_omni/<TRAJ>/ (git-ignored) and then deleted.
Modalities already unpacked are skipped, so the script is safe to re-run; interrupted downloads resume.
Set HF_TOKEN for higher Hugging Face rate limits (optional; the dataset is not gated).
"""
import argparse
import sys
import zipfile
from pathlib import Path

from huggingface_hub import HfApi, hf_hub_download

REPO_ID = "theairlabcmu/TartanGround"
# Revision used for the published experiments (the `main` commit as of 2026-10-14)
REVISION = "388faf9c800568cfc6828fa47e063f8369397eb3"

DATASETS = Path(__file__).resolve().parents[1]

# The trajectories used for the Rolling Bonxai vs Bonxai experiments (omni version). fetch_datasets.sh reads this
# list via --print-default. Sizes of every omni trajectory are in datasets/tartanground_omni_sizes.csv.
DEFAULT_TRAJECTORIES = [
    "Hospital:P0005",             # indoor, changes floors (~11 m), revisits most of its path
    "Downtown:P0005",             # urban, flat
    "ForestEnv:P0010",            # forest, terrain varies by ~14 m
    "GreatMarsh:P0001",           # open marsh, flat
    "ModularNeighborhood:P0001",  # urban, longest route (1.8 km)
]
# modality -> what it unpacks to inside the trajectory directory (used to skip finished downloads)
MODALITIES = {
    "lidar": "lidar",
    "imu": "imu",
    "metadata": "pose_lcam_front.txt",
    "image_lcam_front": "image_lcam_front",
}


def unpacked(traj_dir, modality):
    target = traj_dir / MODALITIES[modality]
    return target.is_file() or (target.is_dir() and any(target.iterdir()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("specs", nargs="*", help="ENV:TRAJ, e.g. Hospital:P0005 (default: DEFAULT_TRAJECTORIES)")
    ap.add_argument("--print-default", action="store_true", help="print DEFAULT_TRAJECTORIES, one per line, and exit")
    ap.add_argument("--data-dir", type=Path, default=DATASETS / "data")
    ap.add_argument("--revision", default=REVISION, help="dataset commit to download (default: the pinned one)")
    ap.add_argument("--modalities", nargs="+", choices=list(MODALITIES), default=list(MODALITIES))
    ap.add_argument("--no-images", action="store_true", help="skip image_lcam_front (about half the download)")
    ap.add_argument("--dry-run", action="store_true", help="only show what would be downloaded")
    args = ap.parse_args()

    if args.print_default:
        print("\n".join(DEFAULT_TRAJECTORIES))
        return
    specs = args.specs or DEFAULT_TRAJECTORIES
    modalities = [m for m in args.modalities if not (args.no_images and m == "image_lcam_front")]

    # Plan: every <ENV>/Data_omni/<TRAJ>/<modality>.zip that isn't unpacked yet, with its size
    todo = []
    for spec in specs:
        env, traj = spec.split(":")
        traj_dir = args.data_dir / env / "Data_omni" / traj
        for m in modalities:
            if not unpacked(traj_dir, m):
                todo.append((f"{env}/Data_omni/{traj}/{m}.zip", traj_dir))
    if not todo:
        print("everything already downloaded")
        return
    sizes = {p.path: getattr(p, "size", 0) or 0
             for p in HfApi().get_paths_info(REPO_ID, [f for f, _ in todo], repo_type="dataset",
                                             revision=args.revision)}
    missing = [f for f, _ in todo if f not in sizes]
    if missing:
        sys.exit(f"not in {REPO_ID}@{args.revision[:8]}: {', '.join(missing)}")
    print(f"{len(todo)} files, {sum(sizes.values()) / 1e9:.1f} GB from {REPO_ID}@{args.revision[:8]}:")
    for f, _ in todo:
        print(f"  {sizes[f] / 1e9:6.2f} GB  {f}")
    if args.dry_run:
        return

    for i, (f, traj_dir) in enumerate(todo, 1):
        print(f"[{i}/{len(todo)}] {f}", flush=True)
        # local_dir mirrors the repo layout, so the zip lands inside its trajectory directory
        zip_path = Path(hf_hub_download(REPO_ID, f, repo_type="dataset", revision=args.revision,
                                        local_dir=args.data_dir))
        with zipfile.ZipFile(zip_path) as z:
            z.extractall(traj_dir)
        zip_path.unlink()
    print(f"done: {args.data_dir}")


if __name__ == "__main__":
    main()
