#!/usr/bin/env bash
# Get the experiment datasets end to end: download the TartanGround trajectories, convert each one to a rosbag2/MCAP
# bag, and check every bag has one scan per pose. Safe to re-run: finished downloads and existing bags are skipped.
#
#   src/Rolling-Bonxai/datasets/scripts/fetch_datasets.sh                    # the experiment set (DEFAULT_TRAJECTORIES)
#   src/Rolling-Bonxai/datasets/scripts/fetch_datasets.sh Hospital:P0005     # specific ENV:TRAJ
#   src/Rolling-Bonxai/datasets/scripts/fetch_datasets.sh --no-images        # skip camera images (and /camera in the bags)
#
# Needs ROS 2 (rosbag2_py + the MCAP storage plugin, both in the devcontainer) and huggingface_hub.
# Disk: ~27.5 GB downloaded (zips are deleted after unpacking), ~26 GB unpacked, ~26 GB of bags.
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATASETS="$(dirname "$SCRIPTS")"

no_images=""
specs=()
for arg in "$@"; do
    case "$arg" in
        --no-images) no_images="--no-images" ;;
        *) specs+=("$arg") ;;
    esac
done
if [[ ${#specs[@]} -eq 0 ]]; then
    mapfile -t specs < <(python3 -I "$SCRIPTS/download_tartanground.py" --print-default)
fi

# shellcheck disable=SC1090
source /opt/ros/"${ROS_DISTRO:-$(ls /opt/ros | head -1)}"/setup.bash

echo "=== download"
python3 -I "$SCRIPTS/download_tartanground.py" $no_images "${specs[@]}"

echo "=== convert"
for spec in "${specs[@]}"; do
    env="${spec%%:*}"; traj="${spec#*:}"
    if [[ -d "$DATASETS/bags/$env.bag" ]]; then
        echo "$env.bag exists; skipping (delete it to rebuild)"
    else
        python3 "$SCRIPTS/tartanground_to_mcap.py" "$DATASETS/data/$env/Data_omni/$traj" $no_images
    fi
done

echo "=== check"
python3 -I - "$DATASETS" "${specs[@]}" <<'EOF'
import json, sys
from pathlib import Path
import yaml
datasets, specs, bad = Path(sys.argv[1]), sys.argv[2:], 0
for spec in specs:
    env, traj = spec.split(":")
    meta = json.loads(next((datasets / "data" / env / "Data_omni" / traj).glob("*_metadata.json")).read_text())
    info = yaml.safe_load((datasets / "bags" / f"{env}.bag" / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    scans = next(t["message_count"] for t in info["topics_with_message_count"] if t["topic_metadata"]["name"] == "/lidar")
    ok = scans == meta["num_poses"]
    bad += not ok
    print(f"{'ok  ' if ok else 'BAD '} {env:20s} {scans:5d} scans / {meta['num_poses']:5d} poses, {meta['path_length']:7.1f} m")
sys.exit(1 if bad else 0)
EOF
echo "all bags ready in $DATASETS/bags"
