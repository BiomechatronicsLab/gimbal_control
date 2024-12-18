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

class ParameterChecker(Node):
    def __init__(self):
        super().__init__('parameter_checker')
        self.client = self.create_client(
            GetParameters,
            f'/{node_name}/get_parameters'
        )

        self.req = GetParameters.Request()

    def wait_for_service_ready(self):
        timeout = 5.0  # Total timeout in seconds
        start_time = time.time()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, retrying...')
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Service {self.client.srv_name} not available after {timeout} seconds.")
                raise TimeoutError("Service connection timed out.")

        self.get_logger().info('Service is available now.')

    # When using the GetParameters service, if you return an ENTIRE array 
    # of parameters to check, and one is missing it will return EMPTY 
    # but not tell you what parameter is not present in the parameter server
    # As a result, I choose to check each parameter individually to isolate which parameter is missing / incorrect

    def send_request(self, param_to_check):
        self.wait_for_service_ready()
        self.req.names = param_to_check
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=0.1)
        return self.future.result().values

@pytest.fixture(autouse=True, scope="session")
def initialize_rclpy():
    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ['ROS_DOMAIN_ID'] = '42'

    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def parameter_checker():
    parameter_checker = ParameterChecker()
    yield parameter_checker
    parameter_checker.destroy_node()

@pytest.mark.launch(fixture=launch_gimbal_ros2_node)
def test_params(config_params, parameter_checker):
    time.sleep(1)

    # Only checks configuration information set in the config file
    params_to_check = list(config_params.keys())
    
    for param in params_to_check:
        parameter_value = parameter_checker.send_request([param]) 

        # Determine the type of parameter and extract its value
        assert parameter_value, (
            f"Parameter '{param}': is not present in the nodes parameter server"
        )

         # Returns an array (with only a single element, because I check each parameter at at time)
        parameter_value = parameter_value[0]
        if parameter_value.type == ParameterType.PARAMETER_BOOL:
            actual_value = parameter_value.bool_value
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
            actual_value = parameter_value.integer_value
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
            actual_value = parameter_value.double_value
        elif parameter_value.type == ParameterType.PARAMETER_STRING:
            actual_value = parameter_value.string_value
        elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            actual_value = parameter_value.bool_array_value.tolist() # YAML reads as a LIST, ROS returns as a numpy array
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            actual_value = parameter_value.integer_array_value.tolist()
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            actual_value = parameter_value.double_array_value.tolist()
        elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            actual_value = parameter_value.string_array_value.tolist()
        
        print(f"Launch Config Value : {config_params[param]}, Actual Value in ROSNode : {actual_value}")
        # Assert that the actual value matches the expected value
        assert config_params[param] == actual_value, (
            f"Mismatch for parameter '{param}': Expected {config_params[param]}, Got {actual_value}"
        )