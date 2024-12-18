# test_example.py
import pytest
import launch_testing
import launch_testing.markers

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

import os
import launch_ros
import rclpy

from launch import LaunchDescription
from rclpy.node import Node
import launch_pytest
import time
from sensor_msgs.msg import JointState
import numpy as np

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
class GimbalPositionChecker(Node):
    def __init__(self, config_params):
        super().__init__('leap_position_checker')

        # Create publisher for joint states
        self.pub_joint_states = self.create_publisher(JointState, config_params["joint_command_topic"], 10)

        self.sub_joint_states = self.create_subscription(
            JointState, config_params["joint_feedback_topic"], self.command_callback, 10
        )
        self.feedback_position_deg = []

    def publish_position(self, positions_to_command):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["yaw", "pitch"]
        joint_state_msg.position = positions_to_command
        self.pub_joint_states.publish(joint_state_msg)

    def command_callback(self, msg):
        self.feedback_position_deg = msg.position 

    def spin_for_duration(self, duration_sec):
        """Spin the given node for the specified duration in seconds."""
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.1)  # Adjust timeout_sec for finer granularity if needed

@pytest.fixture(autouse=True, scope="session")
def initialize_rclpy():
    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ['ROS_DOMAIN_ID'] = '42'

    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def gimbal_position_checker(config_params):
    test_node = GimbalPositionChecker(config_params)
    yield test_node
    test_node.destroy_node()

@pytest.mark.launch(fixture=launch_gimbal_ros2_node)
def test_gimbal_end_to_end(gimbal_position_checker):
    tolerance_deg = 10.0

    rad_90deg = np.pi / 2
    # Define positions to command
    positions_to_command = [
        [0.0, 0.0],
        [rad_90deg, 0.0], # right
        [0.0, 0.0],
        [-rad_90deg, 0.0], # left
        [0.0, 0.0],
        [0.0, rad_90deg], # up
        [0.0, 0.0],
        [0.0, -rad_90deg], # down
        # [0.0, 0.0],
    ]

    for positions_rad in positions_to_command:
        # TODO: set the initial min/max position
        gimbal_position_checker.publish_position(positions_rad)
        gimbal_position_checker.spin_for_duration(duration_sec=1.5) # Time for motor to reach position, and to subscribe from leap_node publisher
        print(np.degrees(positions_rad))
        print(gimbal_position_checker.feedback_position_deg)
        position_comparison = [abs(a - b) for a, b in zip(np.degrees(positions_rad), gimbal_position_checker.feedback_position_deg)]
        print(position_comparison)
        comparison_result = [comparison < tolerance_deg for comparison in position_comparison]
        # comparison_result = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
        print(comparison_result)
        print("--------------------------")
        assert all(comparison_result)
