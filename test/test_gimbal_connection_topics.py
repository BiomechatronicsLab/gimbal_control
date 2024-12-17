import pytest
import launch_pytest
import os
import launch_ros
import rclpy
from launch import LaunchDescription
from rclpy.node import Node

node_name = "test_gimbal_node"

class TopicChecker(Node):
    def __init__(self):
        super().__init__("topic_checker")

    def topic_exists(self, topic_name):
        topic_list = self.get_topic_names_and_types()
        print(topic_list)
        for topic, _ in topic_list:
            if topic == topic_name:
                return True
        return False

@pytest.fixture(autouse=True, scope="session")
def initialize_rclpy():
    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ["ROS_DOMAIN_ID"] = "42"

    rclpy.init()
    yield
    rclpy.shutdown()

@launch_pytest.fixture
def launch_gimbal_ros2_node(config_params):
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
    topic_checker_ = TopicChecker()
    yield topic_checker_
    topic_checker_.destroy_node()

@pytest.mark.launch(fixture=launch_gimbal_ros2_node)
@pytest.mark.parametrize(
    "topic_name_str", ["joint_command_topic", "joint_feedback_topic"]
)
def test_topic_exists(config_params, topic_checker, topic_name_str):
    """Checks if a topic is being published or subscribed to"""
    topic_to_check = config_params[topic_name_str]
    rclpy.spin_once(topic_checker, timeout_sec=3.0)
    assert topic_checker.topic_exists(topic_to_check), (
        f'Topic "{topic_to_check}" not detected'
    )
