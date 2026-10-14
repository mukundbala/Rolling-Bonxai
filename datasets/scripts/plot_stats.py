#!/usr/bin/env python3
"""Plot the CSVs from extract_stats.py against point_clouds_processed.

    python3 datasets/scripts/plot_stats.py experiments/results/<batch>

Writes one PNG per environment into the same folder: map memory (total solid, grid-only dashed), active cells and
memory per active cell for both mappers, plus Rolling Bonxai's chunk counts. The top axis shows the distance travelled.
hit/miss_points_added are not plotted: Rolling Bonxai counts every ray voxel as a miss, Bonxai only ray endpoints.
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COLORS = {'rolling_bonxai': '#1f77b4', 'bonxai': '#d62728'}
LABELS = {'rolling_bonxai': 'Rolling Bonxai', 'bonxai': 'Bonxai'}


def load(path):
    with path.open() as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def distance_axis(ax, data):
    """Secondary x axis: scans -> distance travelled (m), interpolated from the first run's samples."""
    scans, dist = data['point_clouds_processed'], data['distance_m']
    sec = ax.secondary_xaxis('top', functions=(lambda s: np.interp(s, scans, dist),
                                               lambda d: np.interp(d, dist, scans)))
    sec.set_xlabel('distance travelled (m)', fontsize=8)
    sec.tick_params(labelsize=7)


def plot_env(env, runs, out):
    panels = [('total_memory_mib', 'Map memory (MiB)'), ('total_active_cells', 'Active cells'),
              ('bytes_per_cell', 'Memory per active cell (bytes)'), ('chunks', 'Rolling Bonxai chunks')]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for ax, (key, title) in zip(axes.flat, panels):
        if key == 'chunks':
            d = runs.get('rolling_bonxai')
            if d is not None:
                for col, style in [('active_chunk_count', '-'), ('dirty_chunk_count', '--'), ('clean_chunk_count', ':')]:
                    ax.plot(d['point_clouds_processed'], d[col], style, color=COLORS['rolling_bonxai'],
                            label=col.replace('_chunk_count', ''))
        else:
            for mapper, d in runs.items():
                x = d['point_clouds_processed']
                if key == 'bytes_per_cell':
                    cells = d['total_active_cells']
                    y = np.divide(d['total_memory_mib'] * 2**20, cells, out=np.full_like(cells, np.nan), where=cells > 0)
                else:
                    y = d[key]
                ax.plot(x, y, color=COLORS[mapper], label=LABELS[mapper])
                if key == 'total_memory_mib':
                    ax.plot(x, d['grid_memory_mib'], '--', color=COLORS[mapper], label=f'{LABELS[mapper]} (grid only)')
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        distance_axis(ax, next(iter(runs.values())))
    for ax in axes[1]:
        ax.set_xlabel('point clouds processed')
    fig.suptitle(env)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('results_dir', type=Path)
    args = ap.parse_args()

    by_env = defaultdict(dict)
    for path in sorted(args.results_dir.glob('*.csv')):
        if path.name == 'summary.csv':
            continue
        for mapper in ('rolling_bonxai', 'bonxai'):  # longest prefix first
            if path.stem.startswith(mapper + '_'):
                by_env[path.stem[len(mapper) + 1:]][mapper] = load(path)
                break
    for env, runs in sorted(by_env.items()):
        out = args.results_dir / f'{env}.png'
        plot_env(env, runs, out)
        print(out)


if __name__ == '__main__':
    main()
