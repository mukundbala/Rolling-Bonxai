"""Rolling Bonxai mapping fed by a TartanGround rosbag2 (datasets/bags/<bag>.bag).

Usage:
    ros2 launch rolling_bonxai_ros rolling_bonxai_bag_mapping.launch.py bag:=Hospital
    ros2 launch rolling_bonxai_ros rolling_bonxai_bag_mapping.launch.py bag:=ForestEnv rate:=2.0
    ros2 launch rolling_bonxai_ros rolling_bonxai_bag_mapping.launch.py bag:=Hospital record:=true
    ros2 launch rolling_bonxai_ros rolling_bonxai_bag_mapping.launch.py bag:=Hospital record:=true exit_on_end:=true  # scripted runs

Nodes run on sim time; the bag is played with --clock. record:=true also runs stats_recorder.launch.py
(stats bag in <experiments_dir>/rolling_bonxai/<datetime>), starting with playback. Every run saves its chunks to a
fresh <experiments_dir>/rolling_bonxai/chunks/<datetime>, the same timestamp as its stats bag.
"""
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bag', description='Bag name, e.g. Hospital -> <bags_dir>/Hospital.bag'),
        DeclareLaunchArgument('bags_dir', default_value='/ros2_ws/src/Rolling-Bonxai/datasets/bags',
                              description='Directory containing the <name>.bag directories'),
        DeclareLaunchArgument('rate', default_value='1.0', description='Bag playback rate'),
        DeclareLaunchArgument('play_delay', default_value='3.0',
                              description='Seconds to wait for the mapper before starting playback'),
        DeclareLaunchArgument('container_name', default_value='rolling_bonxai_container'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('record', default_value='false', description='Record the stats topics'),
        DeclareLaunchArgument('record_voxels', default_value='false',
                              description='With record:=true, also record the occupied-voxel cloud'),
        DeclareLaunchArgument('experiments_dir', default_value='/ros2_ws/src/Rolling-Bonxai/experiments'),
        DeclareLaunchArgument('run_name', default_value=datetime.now().strftime('%Y-%m-%d_%H-%M-%S'),
                              description='Name of the stats bag and chunk folder (default: launch datetime)'),
        DeclareLaunchArgument('publish_voxels', default_value='false',
                              description='Publish the occupied-voxel cloud (to see the map in Foxglove)'),
        DeclareLaunchArgument('exit_on_end', default_value='false',
                              description='Shut everything down a few seconds after the bag finishes'),
    ]

    params = PathJoinSubstitution([
        FindPackageShare('rolling_bonxai_ros'), 'params', 'rolling_bonxai_params.yaml'])

    mapper = ComposableNode(
        package='rolling_bonxai_ros',
        plugin='RollingBonxai::RollingBonxaiServer',
        name='rolling_bonxai_server_node',
        parameters=[params, {
            'use_sim_time': True,
            # TartanGround topics/frames (override the ZED defaults in the params file)
            'topic_in': '/lidar',
            'qos_reliable': True,  # best effort drops large clouds from the bag player
            'frame_id': 'map',
            'base_frame_id': 'base_link',
            'stats.publish_occupied_voxels': LaunchConfiguration('publish_voxels'),
            # Fresh chunk folder per run, so a run never loads a previous run's chunks
            'rolling.storage.full_save_folder_path': PathJoinSubstitution([
                LaunchConfiguration('experiments_dir'), 'rolling_bonxai', 'chunks', LaunchConfiguration('run_name')]),
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[mapper],
        output='screen',
        emulate_tty=True,
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'use_sim_time': True,
            'port': LaunchConfiguration('foxglove_port'),
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
        }],
        output='screen',
    )

    # Ground-truth trajectory (map -> base_link from the bag's /tf) as nav_msgs/Path on /trajectory
    trajectory = Node(
        package='rolling_bonxai_ros',
        executable='tf_path_publisher.py',
        name='tf_path_publisher',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'map',
            'child_frame_id': 'base_link',
            'topic': '/trajectory',
        }],
        output='screen',
    )

    bag_play = ExecuteProcess(
        # --delay: give the mapper's reliable subscription time to match, or the first scans are lost.
        # --wait-for-all-acked: keep the player alive until the last (large, reliable) scan is delivered.
        cmd=['ros2', 'bag', 'play', '--clock', '--delay', '1.0', '--wait-for-all-acked', '5000',
             '--rate', LaunchConfiguration('rate'),
             [LaunchConfiguration('bags_dir'), '/', LaunchConfiguration('bag'), '.bag']],
        output='screen',
        emulate_tty=True,
    )

    stats_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rolling_bonxai_ros'), 'launch', 'stats_recorder.launch.py'])),
        launch_arguments={
            'experiments_dir': LaunchConfiguration('experiments_dir'),
            'record_voxels': LaunchConfiguration('record_voxels'),
            'run_name': LaunchConfiguration('run_name'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('record')),
    )

    # Give the mapper time to drain its queue and publish final stats, then stop (closes the stats bag cleanly)
    exit_on_end = RegisterEventHandler(
        OnProcessExit(target_action=bag_play, on_exit=[
            TimerAction(period=5.0, actions=[EmitEvent(event=Shutdown(reason='bag finished'))])]),
        condition=IfCondition(LaunchConfiguration('exit_on_end')),
    )

    return LaunchDescription(args + [
        exit_on_end,
        container,
        foxglove,
        trajectory,
        TimerAction(period=LaunchConfiguration('play_delay'), actions=[bag_play, stats_recorder]),
    ])
