#!/usr/bin/env python3
"""Plot the CSVs from extract_stats.py.

    python3 datasets/scripts/plot_stats.py experiments/results/<batch>

Writes into the same folder:
    <env>.png           four panels against scans processed (distance travelled on the top axis):
                          1. map memory: the voxel grid only, both mappers
                          2. total memory: Rolling Bonxai's grid + ray-tracing buffers stacked, Bonxai's total as a line
                          3. voxels held in memory
                          4. Rolling Bonxai's loaded chunks, dirty vs clean
    summary_memory.png  peak total memory per run, Rolling Bonxai split into grid and buffers.

Memory terms: "grid" is the voxel map itself. "Buffers" are the hit/miss coordinate lists used while ray tracing a
scan; they are cleared after every scan but keep their capacity. Bonxai has one pair (<1 MiB); Rolling Bonxai has one
pair per loaded chunk, so its buffers dominate its total.
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

# Validated with the dataviz skill's validate_palette.py: blue/orange as categorical (CVD dE 24.7, normal 33.6),
# the two Rolling blues as an ordinal ramp (light end 2.06:1, so always direct-labelled or in the legend).
ROLLING, BONXAI = '#2a78d6', '#eb6834'
ROLLING_DARK, ROLLING_LIGHT = '#1c5cab', '#86b6ef'
SURFACE, INK, INK_2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
LABELS = {'rolling_bonxai': 'Rolling Bonxai', 'bonxai': 'Bonxai'}
COLORS = {'rolling_bonxai': ROLLING, 'bonxai': BONXAI}
LINE_W = 1.6

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK_2, 'axes.titlecolor': INK,
    'axes.titlesize': 11, 'axes.titleweight': 'semibold', 'axes.titlelocation': 'left', 'axes.titlepad': 10,
    'axes.labelsize': 9, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
    'xtick.color': INK_2, 'ytick.color': INK_2, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8, 'grid.linestyle': '-', 'axes.axisbelow': True,
    'legend.fontsize': 8, 'legend.frameon': False, 'legend.labelcolor': INK_2,
})


def load(path):
    with path.open() as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def distance_axis(ax, data):
    """Top axis: scans processed -> distance travelled (m)."""
    scans, dist = data['point_clouds_processed'], data['distance_m']
    sec = ax.secondary_xaxis('top', functions=(lambda s: np.interp(s, scans, dist),
                                               lambda d: np.interp(d, dist, scans)))
    sec.set_xlabel('distance travelled (m)', fontsize=8, color=INK_2)
    sec.tick_params(labelsize=7, colors=INK_2)
    sec.spines['top'].set_visible(False)


def end_label(ax, x, y, text, color):
    """Direct label at the end of a line: a coloured dot as the key, the text in ink."""
    ax.plot([x[-1]], [y[-1]], 'o', ms=4, color=color, clip_on=False, zorder=5)
    ax.annotate(text, (x[-1], y[-1]), xytext=(6, 0), textcoords='offset points', va='center',
                fontsize=8, color=INK, annotation_clip=False)


def mib(v):
    return f'{v:,.0f} MiB' if v >= 10 else f'{v:.1f} MiB'


def plot_env(env, runs, out):
    roll, bon = runs.get('rolling_bonxai'), runs.get('bonxai')
    fig, axes = plt.subplots(2, 2, figsize=(14, 8.5), sharex=True)
    (ax_grid, ax_total), (ax_cells, ax_chunks) = axes
    fig.subplots_adjust(left=0.07, right=0.88, top=0.80, bottom=0.08, hspace=0.45, wspace=0.32)

    # 1. Map memory: the voxel grid only
    for mapper, d in runs.items():
        x, y = d['point_clouds_processed'], d['grid_memory_mib']
        ax_grid.plot(x, y, color=COLORS[mapper], lw=LINE_W, label=LABELS[mapper])
        end_label(ax_grid, x, y, mib(y[-1]), COLORS[mapper])
    ax_grid.set_title('Map memory (voxel grid only)')
    ax_grid.set_ylabel('MiB')
    ax_grid.legend(loc='upper left')

    # 2. Total memory: Rolling stacked (grid + buffers), Bonxai total as a line
    if roll is not None:
        x = roll['point_clouds_processed']
        g, b = roll['grid_memory_mib'], roll['buffer_memory_mib']
        ax_total.fill_between(x, 0, g, color=ROLLING_DARK, lw=0, label='Rolling Bonxai: voxel grid')
        ax_total.fill_between(x, g, g + b, color=ROLLING_LIGHT, lw=0, label='Rolling Bonxai: ray-tracing buffers')
        ax_total.plot(x, g, color=SURFACE, lw=1.0)  # surface gap between the stacked fills
        end_label(ax_total, x, g + b, mib((g + b)[-1]), ROLLING)
    if bon is not None:
        x = bon['point_clouds_processed']
        ax_total.plot(x, bon['total_memory_mib'], color=BONXAI, lw=LINE_W, label='Bonxai: total (buffers < 1 MiB)')
        end_label(ax_total, x, bon['total_memory_mib'], mib(bon['total_memory_mib'][-1]), BONXAI)
    ax_total.set_title('Total memory, including ray-tracing buffers')
    ax_total.set_ylabel('MiB')
    ax_total.legend(loc='upper left')

    # 3. Voxels held in memory
    for mapper, d in runs.items():
        x, y = d['point_clouds_processed'], d['total_active_cells'] / 1e6
        ax_cells.plot(x, y, color=COLORS[mapper], lw=LINE_W, label=LABELS[mapper])
        end_label(ax_cells, x, y, f'{y[-1]:,.1f} M', COLORS[mapper])
    ax_cells.set_title('Voxels in memory (Rolling Bonxai: loaded chunks only)')
    ax_cells.set_ylabel('million voxels')
    ax_cells.legend(loc='upper left')

    # 4. Rolling chunks: dirty + clean stack to the active count (125 except mid-transition)
    if roll is not None:
        x, dirty, clean = roll['point_clouds_processed'], roll['dirty_chunk_count'], roll['clean_chunk_count']
        ax_chunks.fill_between(x, 0, dirty, color=ROLLING_DARK, lw=0, step='post',
                               label='dirty: changed since loaded, saved on eviction')
        ax_chunks.fill_between(x, dirty, dirty + clean, color=ROLLING_LIGHT, lw=0, step='post',
                               label='clean: unchanged, dropped on eviction')
        ax_chunks.set_ylim(0, max(130, (dirty + clean).max() * 1.05))
        ax_chunks.legend(loc='lower right', framealpha=0.9, frameon=True, edgecolor=SURFACE, facecolor=SURFACE)
    ax_chunks.set_title('Rolling Bonxai chunks in memory (top edge = loaded)')
    ax_chunks.set_ylabel('chunks')

    for ax in axes[1]:
        ax.set_xlabel('scans processed')
    ref = roll if roll is not None else bon
    for ax in axes[0]:
        distance_axis(ax, ref)
        ax.tick_params(labelbottom=True)
    for ax in axes.flat:
        ax.set_ylim(bottom=0)
        ax.set_xlim(0, ref['point_clouds_processed'][-1])

    fig.suptitle(f'{env}: {ref["distance_m"][-1]:,.0f} m, {int(ref["point_clouds_processed"][-1]):,} scans',
                 x=0.07, y=0.975, ha='left', fontsize=14, fontweight='semibold', color=INK)
    fig.text(0.07, 0.93, 'Rolling Bonxai keeps 125 chunks (5 m, radius 2) loaded; Bonxai keeps the whole map. '
             'Resolution 0.1 m, range 9 m.', fontsize=9, color=INK_2)
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_summary(summary_rows, by_env, out):
    """Peak total memory per run; Rolling's bar split into grid and buffers at the moment of its peak."""
    envs = sorted(by_env, key=lambda e: max(float(r['distance_m']) for r in summary_rows if r['env'] == e))
    fig, ax = plt.subplots(figsize=(11, 0.9 * len(envs) + 1.8))
    fig.subplots_adjust(left=0.2, right=0.9, top=0.76, bottom=0.12)
    h, gap = 0.36, 0.04
    xmax = 0
    for i, env in enumerate(envs):
        y = len(envs) - 1 - i
        runs = by_env[env]
        if 'bonxai' in runs:
            peak = runs['bonxai']['total_memory_mib'].max()
            ax.barh(y + (h + gap) / 2, peak, h, color=BONXAI, label='Bonxai' if i == 0 else None)
            ax.annotate(mib(peak), (peak, y + (h + gap) / 2), xytext=(4, 0), textcoords='offset points',
                        va='center', fontsize=8, color=INK)
            xmax = max(xmax, peak)
        if 'rolling_bonxai' in runs:
            d = runs['rolling_bonxai']
            k = int(np.argmax(d['total_memory_mib']))
            g, b = d['grid_memory_mib'][k], d['buffer_memory_mib'][k]
            yy = y - (h + gap) / 2
            ax.barh(yy, g, h, color=ROLLING_DARK, label='Rolling Bonxai: voxel grid' if i == 0 else None)
            ax.barh(yy, b, h, left=g, color=ROLLING_LIGHT, edgecolor=SURFACE, linewidth=1.5,
                    label='Rolling Bonxai: ray-tracing buffers' if i == 0 else None)
            ax.annotate(f'{mib(g + b)}  (grid {mib(g)})', (g + b, yy), xytext=(4, 0), textcoords='offset points',
                        va='center', fontsize=8, color=INK)
            xmax = max(xmax, g + b)
    dist = {}  # full route length: the longest distance any run on that env reached
    for r in summary_rows:
        dist[r['env']] = max(dist.get(r['env'], 0.0), float(r['distance_m']))
    ax.set_yticks(range(len(envs)))
    ax.set_yticklabels([f'{e}\n{dist[e]:,.0f} m' for e in reversed(envs)], fontsize=9, color=INK)
    ax.set_xlim(0, xmax * 1.25)
    ax.set_xlabel('MiB')
    ax.grid(axis='y', visible=False)
    ax.tick_params(axis='y', length=0)
    ax.legend(loc='lower left', bbox_to_anchor=(0, 1.01), ncols=3)
    fig.suptitle('Peak memory per run', x=0.2, y=0.97, ha='left', fontsize=14, fontweight='semibold', color=INK)
    fig.text(0.2, 0.89, 'Rolling Bonxai split into map and ray-tracing buffers at the moment of its peak; '
             'environments ordered by path length.', fontsize=9, color=INK_2)
    fig.savefig(out, dpi=130)
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

    summary = args.results_dir / 'summary.csv'
    if summary.exists():
        out = args.results_dir / 'summary_memory.png'
        plot_summary(list(csv.DictReader(summary.open())), by_env, out)
        print(out)


if __name__ == '__main__':
    main()
