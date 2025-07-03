from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def launch_setup(context, *args, **kwargs):
    param_file_input = LaunchConfiguration('param_file').perform(context)

    # Ensure .yaml extension (do not allow .yml)
    if param_file_input.endswith('.yml'):
        raise RuntimeError(f"'.yml' extension is not supported. Use '.yaml' only: {param_file_input}")
    if not param_file_input.endswith('.yaml'):
        param_file_input += '.yaml'

    package_name = 'rolling-map' 
    config_path = os.path.join(
        get_package_share_directory(package_name),
        'configs',
        param_file_input
    )

    if not os.path.exists(config_path):
        raise RuntimeError(f"Parameter file does not exist: {config_path}")

    # Load and validate chunk folder
    with open(config_path, 'r') as f:
        try:
            params = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise RuntimeError(f"Failed to parse parameter file: {e}")
    
    chunk_folder = params.get('c_chunk_folder_path', '')
    if not chunk_folder:
        raise RuntimeError("Missing 'c_chunk_folder' in parameter file.")
    if not os.path.exists(chunk_folder):
        raise RuntimeError(f"Chunk folder does not exist: {chunk_folder}")
    print(params)
    rolling_map_node = Node(
        package=package_name,
        executable='rolling_map_node',
        name='rolling_map_node',
        output='screen',
        parameters=[params],
        remappings=[
            ('/cloud_in', '/zed/zed_node/point_cloud/cloud_registered')
        ]
    )

    return [rolling_map_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value='rolling_map_params',
            description='YAML file name (with or without .yaml) in the config/ directory of the package'
        ),
        OpaqueFunction(function=launch_setup)
    ])
