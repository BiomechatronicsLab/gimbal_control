# test_example.py
import pytest
import launch_testing
import launch_testing.markers

from test_utilities.parameter_utility import ParameterGetter

import os
import launch_ros

from launch import LaunchDescription
from rclpy.node import Node
import launch_pytest
import time

# GLOBAL VARIABLES
node_name = "test_gimbal_node"

@launch_pytest.fixture
@launch_testing.markers.keep_alive
def launch_gimbal_ros2_node(config_params):

    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ['ROS_DOMAIN_ID'] = '42'

    return LaunchDescription([
        launch_ros.actions.Node(
            package='gimbal_ros2',  #
            executable='gimbal_node.py',  
            name=node_name,
            output='screen',
            parameters=[config_params]
        ),
        launch_testing.actions.ReadyToTest()
    ])

@pytest.fixture
def parameter_getter():
    parameter_getter = ParameterGetter(node_name)
    yield parameter_getter
    parameter_getter.destroy_node()

@pytest.mark.launch(fixture=launch_gimbal_ros2_node)
def test_params(config_params, parameter_getter):
    time.sleep(1)

    # Only checks configuration information set in the config file
    params_to_check = list(config_params.keys())
    parameter_getter.check_params(config_params, params_to_check)