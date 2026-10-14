"""Record the Rolling Bonxai stats topics to <experiments_dir>/rolling_bonxai/<YYYY-MM-DD_HH-MM-SS> (mcap).

Run alongside a mapping launch, e.g.:
    ros2 launch rolling_bonxai_ros rolling_bonxai_bag_mapping.launch.py bag:=Hospital
    ros2 launch rolling_bonxai_ros stats_recorder.launch.py
    ros2 launch rolling_bonxai_ros stats_recorder.launch.py record_voxels:=true   # also the occupied-voxel cloud

Ctrl-C to stop recording.
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

STATS_TOPICS = [
    '/rolling_bonxai/quick_occupancy_stats',  # rolling_bonxai_msgs/QuickOccupancyStats
    '/rolling_bonxai/chunk_stats',            # rolling_bonxai_msgs/ChunkStats
    '/rolling_bonxai/transition_state',       # rolling_bonxai_msgs/RollingState
]
VOXEL_TOPICS = ['/rolling_bonxai/occupied_voxels']  # sensor_msgs/PointCloud2, whole map each publish


def record(context):
    out_dir = os.path.join(LaunchConfiguration('experiments_dir').perform(context), 'rolling_bonxai')
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
                              description='Bags go to <experiments_dir>/rolling_bonxai/<datetime>'),
        DeclareLaunchArgument('record_voxels', default_value='false',
                              description='Also record /rolling_bonxai/occupied_voxels (large)'),
        DeclareLaunchArgument('run_name', default_value='',
                              description='Bag name; defaults to the current datetime'),
        OpaqueFunction(function=record),
    ])
