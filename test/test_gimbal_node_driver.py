#!/usr/bin/env python3
import pytest
import os
from sensor_msgs.msg import JointState
import rclpy
import time
from rclpy.node import Node
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, SetParametersResult
from gimbal_ros2.gimbal_node import GimbalNode
from rcl_interfaces.msg import ParameterType
from rclpy.executors import MultiThreadedExecutor
import threading
import numpy as np

stop_threads = False
pause_threads = False

node_name = "test_gimbal_node" 
class ParameterSetter(Node):
    def __init__(self):
        super().__init__('parameter_setter')
        
        self.client = self.create_client(
            SetParameters,
            f'/{node_name}/set_parameters'
        )

        self.req = SetParameters.Request()

    def wait_for_service_ready(self):
        timeout = 5.0  # Total timeout in seconds
        start_time = time.time()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, retrying...')
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Service {self.client.srv_name} not available after {timeout} seconds.")
                raise TimeoutError("Service connection timed out.")

        self.get_logger().info('Service is available now.')

    def send_request(self, req):
        self.wait_for_service_ready()
        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().results

@pytest.fixture(autouse=True, scope="session")
def initialize_rclpy():
    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ["ROS_DOMAIN_ID"] = "42"

    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def parameter_setter():
    parameter_setter = ParameterSetter()
    yield parameter_setter
    parameter_setter.destroy_node()

@pytest.fixture
def gimbal_ros2_node():
    gimbal_ros2_node = GimbalNode(test_flag=True, node_name="gimbal_node")
    yield gimbal_ros2_node
    gimbal_ros2_node.destroy_node()

def run_gimbal_node_thread(executor):
    """Spin the given node for the specified duration in seconds."""
    global stop_threads
    global pause_threads
    while True:
        if stop_threads:
            break
        elif pause_threads:
            pass
        else:             
            executor.spin_once(timeout_sec=0.1)

def spin_for_duration(executor, duration_sec):
    """Spin the given node for the specified duration in seconds."""
    start_time = time.time()
    while time.time() - start_time < duration_sec:
        executor.spin_once(timeout_sec=0.1)  # Adjust timeout_sec for finer granularity if needed

def set_new_params(config_params, param_names_to_check):
    params_to_set = [] # array of "sudo launch file params"
    for param_name in param_names_to_check:
        parameter_value = config_params[param_name]

        # creation of a new parameter (based off the launch file)
        new_param = Parameter()
        new_param.name = param_name

        if type(parameter_value) == bool:
            new_param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=parameter_value)
        elif type(parameter_value) == int:
            new_param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=parameter_value)
        elif type(parameter_value) == float:
            new_param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=parameter_value)
        elif type(parameter_value) == str:
            new_param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=parameter_value)
        elif type(parameter_value) == list and all(isinstance(x, bool) for x in parameter_value):
            new_param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY, bool_array_value=parameter_value)
        elif type(parameter_value) == list and all(isinstance(x, int) for x in parameter_value):
            new_param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY, integer_array_value=parameter_value)
        elif type(parameter_value) == list and all(isinstance(x, float) for x in parameter_value):
            new_param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=parameter_value)
        elif type(parameter_value) == list and all(isinstance(x, str) for x in parameter_value):
            new_param.value = ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=parameter_value)
        params_to_set.append(new_param)
    return params_to_set

def test_gimbal_node_gain_values(gimbal_ros2_node, parameter_setter, config_params):
    global stop_threads, pause_threads
    stop_threads = False
    pause_threads = False

    executor_gimbal_ros2_node = MultiThreadedExecutor()
    executor_gimbal_ros2_node.add_node(gimbal_ros2_node)
    thread1 = threading.Thread(target=run_gimbal_node_thread, args=(executor_gimbal_ros2_node,))
    thread1.start()

    executor_sub_and_client = MultiThreadedExecutor()
    executor_sub_and_client.add_node(parameter_setter)

    # set the params you want
    kP = 300
    kI = 10
    kD = 100 

    config_params["kP"] = kP
    config_params["kI"] = kI
    config_params["kD"] = kD

    param_names_to_check = list(config_params.keys())
    params_to_set = set_new_params(config_params, param_names_to_check)

    # Set the parameters using the server
    set_parameter_results = parameter_setter.send_request(SetParameters.Request(parameters=params_to_set))

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_sub_and_client, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    gimbal_ros2_node.setup()
    print("GIMBAL NODE SETUP FINISHED!")

    truth_kP = np.ones(len(gimbal_ros2_node.dynamixel_mgr.motor_ids)) * kP
    truth_kI = np.ones(len(gimbal_ros2_node.dynamixel_mgr.motor_ids)) * kI
    truth_kD = np.ones(len(gimbal_ros2_node.dynamixel_mgr.motor_ids)) * kD

    # Pause the leap_node from running so that I can check the actual driver information
    pause_threads = True
    time.sleep(1.0)

    # Check the driver
    test_kP = gimbal_ros2_node.dynamixel_mgr.get_kP(gimbal_ros2_node.dynamixel_mgr.motor_ids)
    test_kI = gimbal_ros2_node.dynamixel_mgr.get_kI(gimbal_ros2_node.dynamixel_mgr.motor_ids)
    test_kD = gimbal_ros2_node.dynamixel_mgr.get_kD(gimbal_ros2_node.dynamixel_mgr.motor_ids)

    # Stop the thread prior to assertion
    stop_threads = True
    time.sleep(1.0)

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert truth_kP.astype(int).tolist() == test_kP
    assert truth_kI.astype(int).tolist() == test_kI
    assert truth_kD.astype(int).tolist() == test_kD

def test_gimbal_node_start_position(gimbal_ros2_node, parameter_setter, config_params):
    global stop_threads, pause_threads
    stop_threads = False
    pause_threads = False

    executor_gimbal_ros2_node = MultiThreadedExecutor()
    executor_gimbal_ros2_node.add_node(gimbal_ros2_node)
    thread1 = threading.Thread(target=run_gimbal_node_thread, args=(executor_gimbal_ros2_node,))
    thread1.start()

    executor_sub_and_client = MultiThreadedExecutor()
    executor_sub_and_client.add_node(parameter_setter)

    # set the initial conditions that you want (parameters must be a DOUBLE array)
    truth_start_pos_deg = [45.0, 45.0] 
    config_params["start_pos_deg"] = truth_start_pos_deg

    # Technically relies params set by the min / max for the maximum and minimum position limits... 
    config_params["min_position_deg"] = [-90.0, -90.0]
    config_params["max_position_deg"] = [90.0, 90.0]

    param_names_to_check = list(config_params.keys())
    params_to_set = set_new_params(config_params, param_names_to_check)

    # Set the parameters using the server
    set_parameter_results = parameter_setter.send_request(SetParameters.Request(parameters=params_to_set))

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_sub_and_client, 1.0)
    print(set_parameter_results)

    # Setup the node so that it starts publishing with updated parameters
    gimbal_ros2_node.setup()
    print("GIMBAL NODE SETUP FINISHED!")
    
    # Pause the leap_node from running so that I can check the actual driver information
    pause_threads = True
    time.sleep(1.0)
    test_start_pos_deg = gimbal_ros2_node.dynamixel_mgr.get_position_deg(gimbal_ros2_node.dynamixel_mgr.motor_ids)
    gimbal_ros2_node.dynamixel_mgr.set_goal_position_deg(gimbal_ros2_node.dynamixel_mgr.motor_ids, np.zeros(len(gimbal_ros2_node.dynamixel_mgr.motor_ids))) # just reset it back to home, not 100% necessary...

    # Stop the thread prior to assertion
    stop_threads = True
    time.sleep(1.0)

    # Now actually check without the node
    position_comparison = [abs(a - b) for a, b in zip(truth_start_pos_deg, test_start_pos_deg)]
    print(position_comparison)

    # tolerance of 10 degrees, etc (could be adjusted)
    comparison_result = [comparison < 15.0 for comparison in position_comparison]

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert all(comparison_result)

def test_gimbal_node_min_position(gimbal_ros2_node, parameter_setter, config_params):
    global stop_threads, pause_threads
    stop_threads = False
    pause_threads = False

    executor_gimbal_ros2_node = MultiThreadedExecutor()
    executor_gimbal_ros2_node.add_node(gimbal_ros2_node)
    thread1 = threading.Thread(target=run_gimbal_node_thread, args=(executor_gimbal_ros2_node,))
    thread1.start()

    executor_sub_and_client = MultiThreadedExecutor()
    executor_sub_and_client.add_node(parameter_setter)

    # set the initial conditions that you want (parameters must be a DOUBLE array)
    truth_min_pos_deg = [-115.0, -105.0] 
    config_params["min_position_deg"] = truth_min_pos_deg

    param_names_to_check = list(config_params.keys())
    params_to_set = set_new_params(config_params, param_names_to_check)

    # Set the parameters using the server
    set_parameter_results = parameter_setter.send_request(SetParameters.Request(parameters=params_to_set))

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_sub_and_client, 1.0)
    print(set_parameter_results)

    # Setup the node so that it starts publishing with updated parameters
    gimbal_ros2_node.setup()
    
    # Pause the leap_node from running so that I can check the actual driver information
    pause_threads = True
    time.sleep(1.0)
    test_min_pos_deg = gimbal_ros2_node.dynamixel_mgr.get_min_position_deg(gimbal_ros2_node.dynamixel_mgr.motor_ids)

    # Stop the thread prior to assertion
    stop_threads = True
    time.sleep(1.0)

    # Now actually check without the node
    position_comparison = [abs(a - b) for a, b in zip(truth_min_pos_deg, test_min_pos_deg)]
    print(position_comparison)

    # should not really even have a tolerance, just floating point accuracy
    comparison_result = [comparison < 1.0 for comparison in position_comparison]

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert all(comparison_result)


def test_gimbal_node_max_position(gimbal_ros2_node, parameter_setter, config_params):
    global stop_threads, pause_threads
    stop_threads = False
    pause_threads = False

    executor_gimbal_ros2_node = MultiThreadedExecutor()
    executor_gimbal_ros2_node.add_node(gimbal_ros2_node)
    thread1 = threading.Thread(target=run_gimbal_node_thread, args=(executor_gimbal_ros2_node,))
    thread1.start()

    executor_sub_and_client = MultiThreadedExecutor()
    executor_sub_and_client.add_node(parameter_setter)

    # set the initial conditions that you want (parameters must be a DOUBLE array)
    truth_max_pos_deg = [115.0, 105.0] 
    config_params["max_position_deg"] = truth_max_pos_deg

    param_names_to_check = list(config_params.keys())
    params_to_set = set_new_params(config_params, param_names_to_check)

    # Set the parameters using the server
    set_parameter_results = parameter_setter.send_request(SetParameters.Request(parameters=params_to_set))

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_sub_and_client, 1.0)
    print(set_parameter_results)

    # Setup the node so that it starts publishing with updated parameters
    gimbal_ros2_node.setup()
    
    # Pause the leap_node from running so that I can check the actual driver information
    pause_threads = True
    time.sleep(1.0)
    test_max_pos_deg = gimbal_ros2_node.dynamixel_mgr.get_max_position_deg(gimbal_ros2_node.dynamixel_mgr.motor_ids)
    print(f'test_max_pos_deg: {test_max_pos_deg}')

    # Stop the thread prior to assertion
    stop_threads = True
    time.sleep(1.0)

    # Now actually check without the node
    position_comparison = [abs(a - b) for a, b in zip(truth_max_pos_deg, test_max_pos_deg)]
    print(position_comparison)

    # should not really even have a tolerance, just floating point accuracy
    comparison_result = [comparison < 1.0 for comparison in position_comparison]

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert all(comparison_result)

def test_gimbal_node_no_device_name(gimbal_ros2_node, parameter_setter, config_params):
    global stop_threads, pause_threads
    stop_threads = False
    pause_threads = False

    executor_gimbal_ros2_node = MultiThreadedExecutor()
    executor_gimbal_ros2_node.add_node(gimbal_ros2_node)
    thread1 = threading.Thread(target=run_gimbal_node_thread, args=(executor_gimbal_ros2_node,))
    thread1.start()

    executor_sub_and_client = MultiThreadedExecutor()
    executor_sub_and_client.add_node(parameter_setter)

    # remove the device_name parameter to cause the exception to be raised!
    del config_params["device_name"] 

    param_names_to_check = list(config_params.keys())
    params_to_set = set_new_params(config_params, param_names_to_check)

    # Set the parameters using the server
    set_parameter_results = parameter_setter.send_request(SetParameters.Request(parameters=params_to_set))

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_sub_and_client, 1.0)

    # Assert that a ValueError is raised
    with pytest.raises(ValueError, match="Please state the device_name in the configuration file."):
        # Setup the node so that it starts publishing with updated parameters
       gimbal_ros2_node.setup()

    # Pause the leap_node from running so that I can check the actual driver information
    pause_threads = True
    time.sleep(1.0)

    # Stop the thread prior to assertion
    stop_threads = True
    time.sleep(1.0)

    assert True

def test_gimbal_node_no_dynamixel_type(gimbal_ros2_node, parameter_setter, config_params):
    global stop_threads, pause_threads
    stop_threads = False
    pause_threads = False

    executor_gimbal_ros2_node = MultiThreadedExecutor()
    executor_gimbal_ros2_node.add_node(gimbal_ros2_node)
    thread1 = threading.Thread(target=run_gimbal_node_thread, args=(executor_gimbal_ros2_node,))
    thread1.start()

    executor_sub_and_client = MultiThreadedExecutor()
    executor_sub_and_client.add_node(parameter_setter)

    # remove the device_name parameter to cause the exception to be raised!
    del config_params["dynamixel_type"] 

    param_names_to_check = list(config_params.keys())
    params_to_set = set_new_params(config_params, param_names_to_check)

    # Set the parameters using the server
    set_parameter_results = parameter_setter.send_request(SetParameters.Request(parameters=params_to_set))

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_sub_and_client, 1.0)

    # Assert that a ValueError is raised
    with pytest.raises(ValueError, match="Please state the dynamixel_type in the configuration file."):
        # Setup the node so that it starts publishing with updated parameters
        gimbal_ros2_node.setup()

    # Pause the leap_node from running so that I can check the actual driver information
    pause_threads = True
    time.sleep(1.0)

    # Stop the thread prior to assertion
    stop_threads = True
    time.sleep(1.0)

    assert True
