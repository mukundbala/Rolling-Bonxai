"""Record the Bonxai stats topic to <experiments_dir>/bonxai/<YYYY-MM-DD_HH-MM-SS> (mcap).

Run alongside a mapping launch, e.g.:
    ros2 launch bonxai_ros bonxai_bag_mapping.launch.py bag:=Hospital
    ros2 launch bonxai_ros stats_recorder.launch.py
    ros2 launch bonxai_ros stats_recorder.launch.py record_voxels:=true   # also the occupied-voxel cloud

Quick vs. full stats are both on /bonxai/occupancy_stats, selected by the server's stats.quick_stats param
(expensive_stats_computed in the message says which). Ctrl-C to stop recording.
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

STATS_TOPICS = ['/bonxai/occupancy_stats']  # bonxai_msgs/OccupancyMapStats
VOXEL_TOPICS = ['/bonxai/occupied_voxels']  # sensor_msgs/PointCloud2, whole map each publish


def record(context):
    out_dir = os.path.join(LaunchConfiguration('experiments_dir').perform(context), 'bonxai')
    os.makedirs(out_dir, exist_ok=True)
    run_name = LaunchConfiguration('run_name').perform(context) or datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    bag = os.path.join(out_dir, run_name)

    topics = list(STATS_TOPICS)
    if LaunchConfiguration('record_voxels').perform(context).lower() in ('true', '1'):
        topics += VOXEL_TOPICS

    return [ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--storage', 'mcap', '-o', bag, '--topics', *topics],
        output='screen',
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('experiments_dir', default_value='/ros2_ws/src/Rolling-Bonxai/experiments',
                              description='Bags go to <experiments_dir>/bonxai/<datetime>'),
        DeclareLaunchArgument('record_voxels', default_value='false',
                              description='Also record /bonxai/occupied_voxels (large)'),
        DeclareLaunchArgument('run_name', default_value='',
                              description='Bag name; defaults to the current datetime'),
        OpaqueFunction(function=record),
    ])
