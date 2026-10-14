#!/usr/bin/env python3
"""Turn the stats bags of a run_experiments.py batch into CSVs.

Run with the workspace sourced (needs the rolling_bonxai_msgs / bonxai_msgs types):
    python3 datasets/scripts/extract_stats.py experiments/batches/<batch>.csv

Writes <experiments_dir>/results/<batch>/:
    <mapper>_<env>.csv  one row per stats message (1 Hz), keyed by point_clouds_processed. distance_m is the
                        ground-truth path length up to the last processed scan (from the dataset bag's /tf).
    summary.csv         one row per run: scans processed vs expected, final/peak memory, wall time.
"""
import argparse
import csv
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

REPO = Path(__file__).resolve().parents[2]

# Fields copied from each stats message, per mapper
FIELDS = {
    'rolling_bonxai': {
        '/rolling_bonxai/quick_occupancy_stats': [
            'point_clouds_processed', 'total_memory_mib', 'grid_memory_mib', 'buffer_memory_mib',
            'total_active_cells', 'total_insertions', 'hit_points_added', 'miss_points_added'],
        '/rolling_bonxai/chunk_stats': ['active_chunk_count', 'dirty_chunk_count', 'clean_chunk_count'],
    },
    'bonxai': {
        '/bonxai/occupancy_stats': [
            'point_clouds_processed', 'total_memory_mib', 'grid_memory_mib', 'buffer_memory_mib',
            'total_active_cells', 'total_insertions', 'hit_points_added', 'miss_points_added',
            'total_points_processed'],
    },
}


def read_bag(uri, topics):
    """Yield (topic, msg) for the given topics, in bag order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, types[topic])


def trajectory(dataset_bag):
    """(start stamp, path length in m after each scan); the converter writes one map -> base_link /tf per scan."""
    dist, prev, out, start = 0.0, None, [], None
    for _, msg in read_bag(dataset_bag, ['/tf']):
        for tf in msg.transforms:
            if tf.child_frame_id != 'base_link':
                continue
            start = stamp_s(tf) if start is None else start
            p = tf.transform.translation
            if prev is not None:
                dist += math.dist(prev, (p.x, p.y, p.z))
            prev = (p.x, p.y, p.z)
            out.append(dist)
    return start, out


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def extract_run(mapper, stats_bag, start, distance):
    """Rows keyed by stats header stamp; Rolling Bonxai publishes both its stats topics with the same stamp."""
    rows = {}
    for topic, msg in read_bag(stats_bag, FIELDS[mapper]):
        row = rows.setdefault(stamp_s(msg), {})
        row.update({f: getattr(msg, f) for f in FIELDS[mapper][topic]})
    out = []
    for stamp in sorted(rows):
        row = rows[stamp]
        if 'point_clouds_processed' not in row:
            continue
        n = row['point_clouds_processed']
        # Stats published before the bag's /clock arrives are stamped 0
        out.append({'sim_time_s': round(max(stamp - start, 0.0), 3),
                    'distance_m': round(distance[min(n, len(distance)) - 1], 3) if n else 0.0,
                    **row})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('manifest', type=Path)
    ap.add_argument('--bags-dir', type=Path, default=REPO / 'datasets' / 'bags')
    args = ap.parse_args()

    runs = list(csv.DictReader(args.manifest.open()))
    out_dir = args.manifest.parents[1] / 'results' / args.manifest.stem
    out_dir.mkdir(parents=True, exist_ok=True)

    trajectories, summary = {}, []
    for run in runs:
        env, mapper = run['env'], run['mapper']
        if env not in trajectories:
            trajectories[env] = trajectory(args.bags_dir / f'{env}.bag')
        rows = extract_run(mapper, run['stats_bag'], *trajectories[env])
        if not rows:
            print(f'{mapper} {env}: no stats in {run["stats_bag"]}')
            continue
        path = out_dir / f'{mapper}_{env}.csv'
        with path.open('w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[-1]))
            writer.writeheader()
            writer.writerows(rows)

        processed, expected = rows[-1]['point_clouds_processed'], int(run['expected_scans'])
        summary.append({
            'env': env, 'mapper': mapper, 'run_name': run['run_name'],
            'scans_processed': processed, 'scans_expected': expected,
            'complete': processed == expected,
            'distance_m': rows[-1]['distance_m'],
            'final_memory_mib': round(rows[-1]['total_memory_mib'], 2),
            'peak_memory_mib': round(max(r['total_memory_mib'] for r in rows), 2),
            'peak_active_cells': max(r['total_active_cells'] for r in rows),
            'wall_s': run['wall_s'], 'exit_code': run['exit_code'],
        })
        print(f'{mapper:15s} {env:20s} {processed}/{expected} scans -> {path}')

    with (out_dir / 'summary.csv').open('w', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    print(f'results: {out_dir}')


if __name__ == '__main__':
    main()
