#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import os
import yaml

def load_yaml_file(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def launch_setup(context, *args, **kwargs):
    # Retrieve the path to the YAML file from the config_file argument
    config_file_name = LaunchConfiguration('config_file').perform(context)

    # Prepend the path to the config directory
    config_directory = os.path.join(get_package_share_directory('gimbal_ros2'), 'config')
    config_file_path = os.path.join(config_directory, config_file_name)

    # Load parameters from the YAML file
    config_params = load_yaml_file(config_file_path)

    return [
        Node(
            package='gimbal_ros2',
            executable='gimbal_node.py',
            name='gimbal_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_params]
        )
    ]

def generate_launch_description():
    # Declare the YAML file path as a launch argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default_params.yaml',
        description='Path to the YAML config file'
    )

    # Create launch description using OpaqueFunction to ensure the config_file argument is processed at runtime
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()