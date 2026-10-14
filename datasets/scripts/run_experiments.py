#!/usr/bin/env python3
"""Run TartanGround bags through Rolling Bonxai and Bonxai, one after the other, recording stats.

Run with the workspace sourced:
    python3 datasets/scripts/run_experiments.py
    python3 datasets/scripts/run_experiments.py --envs Hospital --mappers rolling_bonxai
    python3 datasets/scripts/run_experiments.py --rate 0.5 --perf   # also profile every run

Each run is `<mapper>_bag_mapping.launch.py bag:=<env> record:=true exit_on_end:=true`, so the stats bag lands in
<experiments_dir>/<mapper>/<run_name> (Rolling Bonxai chunks in <experiments_dir>/rolling_bonxai/chunks/<run_name>).
Every run is appended to <experiments_dir>/batches/<batch>.csv, which extract_stats.py reads.

--perf attaches `perf record --call-graph fp` to the mapper's container for the whole run and writes
<experiments_dir>/profiles/<run_name>_<mapper>_<env>.data, plus a .perf text export (speedscope / Firefox Profiler).
Build with frame pointers first, or the stacks stop at the first function without one:
    colcon build --packages-select bonxai_map rolling_bonxai rolling_bonxai_ros bonxai_ros \
        --cmake-args -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer

Runs use their own ROS_DOMAIN_ID and Foxglove port, so they don't mix with anything else running on the machine.
"""
import argparse
import csv
import os
import re
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
PACKAGES = {'rolling_bonxai': 'rolling_bonxai_ros', 'bonxai': 'bonxai_ros'}
ALL_ENVS = ['Hospital', 'Downtown', 'ForestEnv', 'GreatMarsh', 'ModularNeighborhood']


def bag_info(bag_dir):
    """(lidar message count, duration in s) from a rosbag2 metadata.yaml."""
    info = yaml.safe_load((bag_dir / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    scans = next(t['message_count'] for t in info['topics_with_message_count']
                 if t['topic_metadata']['name'] == '/lidar')
    return scans, info['duration']['nanoseconds'] / 1e9


def container_pid(log, proc, timeout=60):
    """PID of the mapper's component_container, read from the launch log."""
    pattern = re.compile(r'component_container-1\]: process started with pid \[(\d+)\]')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and proc.poll() is None:
        match = pattern.search(log.read_text())
        if match:
            return int(match.group(1))
        time.sleep(0.2)
    return None


def start_perf(args, pid, out):
    """perf record exits by itself when the target process exits."""
    return subprocess.Popen(
        [args.perf_bin, 'record', '-F', str(args.perf_freq), '--call-graph', 'fp', '-p', str(pid), '-o', str(out)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def export_perf(args, data):
    with data.with_suffix('.perf').open('w') as fh:
        subprocess.run([args.perf_bin, 'script', '-i', str(data), '-F', 'comm,tid,time,event,ip,sym,dso'],
                       stdout=fh, stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--envs', nargs='+', default=ALL_ENVS)
    ap.add_argument('--mappers', nargs='+', default=['rolling_bonxai', 'bonxai'], choices=list(PACKAGES))
    ap.add_argument('--bags-dir', type=Path, default=REPO / 'datasets' / 'bags')
    ap.add_argument('--experiments-dir', type=Path, default=REPO / 'experiments')
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--domain-id', default='42', help='ROS_DOMAIN_ID for the runs')
    ap.add_argument('--foxglove-port', default='8766')
    ap.add_argument('--perf', action='store_true', help='profile each run with perf (see above)')
    ap.add_argument('--perf-freq', type=int, default=99, help='perf sampling rate (Hz)')
    ap.add_argument('--perf-bin', default='perf')
    args = ap.parse_args()

    batch = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    (args.experiments_dir / 'batches').mkdir(parents=True, exist_ok=True)
    (args.experiments_dir / 'logs').mkdir(parents=True, exist_ok=True)
    if args.perf:
        (args.experiments_dir / 'profiles').mkdir(parents=True, exist_ok=True)
    manifest = args.experiments_dir / 'batches' / f'{batch}.csv'
    env = dict(os.environ, ROS_DOMAIN_ID=args.domain_id)

    with manifest.open('w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(['run_name', 'mapper', 'env', 'stats_bag', 'expected_scans', 'exit_code', 'wall_s', 'profile'])
        for bag in args.envs:
            scans, duration = bag_info(args.bags_dir / f'{bag}.bag')
            for mapper in args.mappers:
                run_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                log = args.experiments_dir / 'logs' / f'{run_name}_{mapper}_{bag}.log'
                cmd = ['ros2', 'launch', PACKAGES[mapper], f'{mapper}_bag_mapping.launch.py',
                       f'bag:={bag}', f'bags_dir:={args.bags_dir}', f'rate:={args.rate}',
                       f'experiments_dir:={args.experiments_dir}', f'run_name:={run_name}',
                       f'foxglove_port:={args.foxglove_port}', 'record:=true', 'exit_on_end:=true']
                print(f'[{batch}] {mapper} on {bag}: {scans} scans, ~{duration / args.rate:.0f} s -> {log}', flush=True)

                start = time.monotonic()
                perf, profile = None, ''
                with log.open('w') as out:
                    proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, env=env)
                    if args.perf:
                        pid = container_pid(log, proc)
                        if pid is None:
                            print('  could not find the mapper pid; not profiling', flush=True)
                        else:
                            profile = args.experiments_dir / 'profiles' / f'{run_name}_{mapper}_{bag}.data'
                            perf = start_perf(args, pid, profile)
                    try:
                        code = proc.wait(timeout=duration / args.rate + 120)
                    except subprocess.TimeoutExpired:
                        print('  timed out; stopping', flush=True)
                        proc.send_signal(signal.SIGINT)
                        code = proc.wait()
                wall = time.monotonic() - start
                if perf is not None:
                    perf.wait(timeout=120)
                    export_perf(args, profile)
                writer.writerow([run_name, mapper, bag, args.experiments_dir / mapper / run_name, scans, code,
                                 f'{wall:.0f}', profile])
                fh.flush()
                print(f'  done in {wall:.0f} s (exit {code})', flush=True)
                time.sleep(3)  # let the previous run's processes and DDS participants go away

    print(f'manifest: {manifest}')


if __name__ == '__main__':
    main()
