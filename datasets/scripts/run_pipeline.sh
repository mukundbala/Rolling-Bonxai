#!/usr/bin/env bash
# Full experiment pipeline: run the batch (run_experiments.py), extract the stats to CSVs, plot them, and check that
# every run processed all of its bag's scans.
#
#   nohup src/Rolling-Bonxai/datasets/scripts/run_pipeline.sh > ~/batch.log 2>&1 &
#   tail -f ~/batch.log
#
# Defaults: all 5 environments, both mappers, --rate 0.5 --perf. Extra arguments go to run_experiments.py and override
# these, e.g. `run_pipeline.sh --envs Hospital --mappers rolling_bonxai`.
# EXPERIMENTS_DIR and BAGS_DIR override where outputs go and where bags are read from.
# Exits non-zero if any run is incomplete or a step fails.
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$(dirname "$SCRIPTS")")"
EXPERIMENTS_DIR="${EXPERIMENTS_DIR:-$REPO/experiments}"
BAGS_DIR="${BAGS_DIR:-$REPO/datasets/bags}"

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

echo "=== [$(date +%T)] batch"
start_marker="$(mktemp)"
python3 "$SCRIPTS/run_experiments.py" --rate 0.5 --perf \
    --experiments-dir "$EXPERIMENTS_DIR" --bags-dir "$BAGS_DIR" "$@"

# The batch this run wrote: the newest run list created after we started
manifest="$(find "$EXPERIMENTS_DIR/batches" -name '*.csv' -newer "$start_marker" -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
rm -f "$start_marker"
if [[ -z "$manifest" ]]; then
    echo "no run list was written to $EXPERIMENTS_DIR/batches" >&2
    exit 1
fi
results="$EXPERIMENTS_DIR/results/$(basename "$manifest" .csv)"

echo "=== [$(date +%T)] extract: $manifest"
python3 "$SCRIPTS/extract_stats.py" "$manifest" --bags-dir "$BAGS_DIR"

echo "=== [$(date +%T)] plot: $results"
python3 "$SCRIPTS/plot_stats.py" "$results"

echo "=== [$(date +%T)] summary"
python3 -c 'import csv, sys; rows = list(csv.reader(open(sys.argv[1]))); w = [max(map(len, c)) for c in zip(*rows)]
[print("  ".join(v.ljust(n) for v, n in zip(r, w))) for r in rows]' "$results/summary.csv"

incomplete="$(awk -F, 'NR > 1 && $6 != "True" {print $2 " on " $1 ": " $4 "/" $5 " scans"}' "$results/summary.csv")"
if [[ -n "$incomplete" ]]; then
    echo "INCOMPLETE runs (don't use these):" >&2
    echo "$incomplete" >&2
    exit 2
fi
echo "all runs complete. results: $results"
