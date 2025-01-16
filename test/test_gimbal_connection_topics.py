import pytest
import launch_pytest
import os
import launch_ros
import rclpy
from launch import LaunchDescription
from rclpy.node import Node
from test_utilities.parameter_utility import TopicChecker

node_name = "test_gimbal_node"

@launch_pytest.fixture
def launch_gimbal_ros2_node(config_params):

    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ['ROS_DOMAIN_ID'] = '42'

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gimbal_ros2",
                executable="gimbal_node.py",
                name=node_name,
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )

@pytest.fixture
def topic_checker():
    topic_checker = TopicChecker()
    yield topic_checker
    topic_checker.destroy_node()

@pytest.mark.launch(fixture=launch_gimbal_ros2_node)
@pytest.mark.parametrize(
    "topic_name_str", ["joint_command_topic", "joint_feedback_topic"]
)
def test_topic_exists(config_params, topic_checker, topic_name_str):
    """Checks if a topic is being published or subscribed to"""
    topic_to_check = config_params[topic_name_str]
    rclpy.spin_once(topic_checker, timeout_sec=3.0)
    topic_checker.topic_exists(topic_to_check)
