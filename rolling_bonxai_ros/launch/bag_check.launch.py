"""Play a TartanGround rosbag2 (datasets/bags/<bag>.bag) with the ground-truth trajectory, no mapper.

For eyeballing a dataset in Foxglove: /lidar, /camera/image_raw/compressed, /tf and the accumulated
map -> base_link path on /trajectory.

Usage:
    ros2 launch rolling_bonxai_ros bag_check.launch.py bag:=Hospital
    ros2 launch rolling_bonxai_ros bag_check.launch.py bag:=ForestEnv rate:=4.0 loop:=true

Nodes run on sim time; the bag is played with --clock.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bag', description='Bag name, e.g. Hospital -> <bags_dir>/Hospital.bag'),
        DeclareLaunchArgument('bags_dir', default_value='/ros2_ws/src/Rolling-Bonxai/datasets/bags',
                              description='Directory containing the <name>.bag directories'),
        DeclareLaunchArgument('rate', default_value='1.0', description='Bag playback rate'),
        DeclareLaunchArgument('loop', default_value='false', description='Loop playback'),
        DeclareLaunchArgument('play_delay', default_value='2.0',
                              description='Seconds to wait (e.g. to connect Foxglove) before starting playback'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
    ]

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

    # Ground-truth trajectory (map -> base_link from the bag's /tf) as nav_msgs/Path on /trajectory.
    # Clears itself when the clock jumps backwards, so loop:=true works.
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

    bag_path = [LaunchConfiguration('bags_dir'), '/', LaunchConfiguration('bag'), '.bag']
    play_cmd = ['ros2', 'bag', 'play', '--clock', '--rate', LaunchConfiguration('rate')]
    bag_play = ExecuteProcess(
        cmd=play_cmd + [bag_path],
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('loop')),
    )
    bag_play_loop = ExecuteProcess(
        cmd=play_cmd + ['--loop', bag_path],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('loop')),
    )

    return LaunchDescription(args + [
        foxglove,
        trajectory,
        TimerAction(period=LaunchConfiguration('play_delay'), actions=[bag_play, bag_play_loop]),
    ])
