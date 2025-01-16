#!/usr/bin/env python3
import pytest
from sensor_msgs.msg import JointState
import time
from rclpy.node import Node
import rclpy
from gimbal_ros2.gimbal_node import GimbalNode
from test_utilities.suspendable_thread import SuspendableThread 
from test_utilities.parameter_utility import ParameterSetter

from rcl_interfaces.msg import ParameterType
from rclpy.executors import SingleThreadedExecutor
import numpy as np

node_name = "test_gimbal_node" 

@pytest.fixture
def parameter_setter():
    parameter_setter = ParameterSetter(node_name)
    yield parameter_setter
    parameter_setter.destroy_node()

@pytest.fixture
def gimbal_ros2_node_and_thread():
    gimbal_ros2_node = GimbalNode(test_flag=True, node_name=node_name)
    executor = SingleThreadedExecutor()
    executor.add_node(gimbal_ros2_node)
    suspendable_thread = SuspendableThread(target=spin_node, args=(executor, ))
    suspendable_thread.start()

    yield gimbal_ros2_node, suspendable_thread

    executor.shutdown()
    gimbal_ros2_node.destroy_node()
    suspendable_thread.kill()

def spin_for_duration(executor, duration_sec):
    """Spin the given node for the specified duration in seconds."""
    start_time = time.time()
    while time.time() - start_time < duration_sec:
        spin_node(executor)  # Adjust timeout_sec for finer granularity if needed

def spin_node(executor):
    executor.spin_once(timeout_sec=0.01)

def test_gimbal_node_gain_values(gimbal_ros2_node_and_thread, parameter_setter, config_params):

    gimbal_ros2_node, suspendable_thread = gimbal_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # set the params you want
    kP = 300
    kI = 10
    kD = 100 

    config_params["kP"] = kP
    config_params["kI"] = kI
    config_params["kD"] = kD

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    gimbal_ros2_node.setup()
    print("GIMBAL NODE SETUP FINISHED!")

    truth_kP = np.ones(len(gimbal_ros2_node.dynamixel_mgr.motor_ids)) * kP
    truth_kI = np.ones(len(gimbal_ros2_node.dynamixel_mgr.motor_ids)) * kI
    truth_kD = np.ones(len(gimbal_ros2_node.dynamixel_mgr.motor_ids)) * kD

    # Pause the leap_node from running so that I can check the actual driver information
    suspendable_thread.suspend()
    time.sleep(1.0)

    # Check the driver
    test_kP = gimbal_ros2_node.dynamixel_mgr.get_kP(gimbal_ros2_node.dynamixel_mgr.motor_ids)
    test_kI = gimbal_ros2_node.dynamixel_mgr.get_kI(gimbal_ros2_node.dynamixel_mgr.motor_ids)
    test_kD = gimbal_ros2_node.dynamixel_mgr.get_kD(gimbal_ros2_node.dynamixel_mgr.motor_ids)

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert truth_kP.astype(int).tolist() == test_kP
    assert truth_kI.astype(int).tolist() == test_kI
    assert truth_kD.astype(int).tolist() == test_kD
    executor_test.shutdown()

def test_gimbal_node_start_position(gimbal_ros2_node_and_thread, parameter_setter, config_params):
    gimbal_ros2_node, suspendable_thread = gimbal_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # set the initial conditions that you want (parameters must be a DOUBLE array)
    truth_start_pos_deg = [45.0, 45.0] 
    config_params["start_pos_deg"] = truth_start_pos_deg

    # Technically relies params set by the min / max for the maximum and minimum position limits... 
    config_params["min_position_deg"] = [-90.0, -90.0]
    config_params["max_position_deg"] = [90.0, 90.0]

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    gimbal_ros2_node.setup()
    print("GIMBAL NODE SETUP FINISHED!")
    
    # Pause the leap_node from running so that I can check the actual driver information
    suspendable_thread.suspend()
    time.sleep(1.0)
    test_start_pos_deg = gimbal_ros2_node.dynamixel_mgr.get_position_deg(gimbal_ros2_node.dynamixel_mgr.motor_ids)
    gimbal_ros2_node.dynamixel_mgr.set_goal_position_deg(gimbal_ros2_node.dynamixel_mgr.motor_ids, np.zeros(len(gimbal_ros2_node.dynamixel_mgr.motor_ids))) # just reset it back to home, not 100% necessary...

    # Now actually check without the node
    position_comparison = [abs(a - b) for a, b in zip(truth_start_pos_deg, test_start_pos_deg)]
    print(position_comparison)

    # tolerance of 10 degrees, etc (could be adjusted)
    comparison_result = [comparison < 15.0 for comparison in position_comparison]

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert all(comparison_result)
    executor_test.shutdown()

def test_gimbal_node_min_position(gimbal_ros2_node_and_thread, parameter_setter, config_params):
    gimbal_ros2_node, suspendable_thread = gimbal_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # set the initial conditions that you want (parameters must be a DOUBLE array)
    truth_min_pos_deg = [-115.0, -105.0] 
    config_params["min_position_deg"] = truth_min_pos_deg

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    gimbal_ros2_node.setup()
    
    # Pause the leap_node from running so that I can check the actual driver information
    suspendable_thread.suspend()
    time.sleep(1.0)
    test_min_pos_deg = gimbal_ros2_node.dynamixel_mgr.get_min_position_deg(gimbal_ros2_node.dynamixel_mgr.motor_ids)

    # Now actually check without the node
    position_comparison = [abs(a - b) for a, b in zip(truth_min_pos_deg, test_min_pos_deg)]
    print(position_comparison)

    # should not really even have a tolerance, just floating point accuracy
    comparison_result = [comparison < 1.0 for comparison in position_comparison]

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert all(comparison_result)
    executor_test.shutdown()

def test_gimbal_node_max_position(gimbal_ros2_node_and_thread, parameter_setter, config_params):
    gimbal_ros2_node, suspendable_thread = gimbal_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # set the initial conditions that you want (parameters must be a DOUBLE array)
    truth_max_pos_deg = [115.0, 105.0] 
    config_params["max_position_deg"] = truth_max_pos_deg

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    gimbal_ros2_node.setup()
    
    # Pause the leap_node from running so that I can check the actual driver information
    suspendable_thread.suspend()
    time.sleep(1.0)
    test_max_pos_deg = gimbal_ros2_node.dynamixel_mgr.get_max_position_deg(gimbal_ros2_node.dynamixel_mgr.motor_ids)
    print(f'test_max_pos_deg: {test_max_pos_deg}')

    # Now actually check without the node
    position_comparison = [abs(a - b) for a, b in zip(truth_max_pos_deg, test_max_pos_deg)]
    print(position_comparison)

    # should not really even have a tolerance, just floating point accuracy
    comparison_result = [comparison < 1.0 for comparison in position_comparison]

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert all(comparison_result)
    executor_test.shutdown()

def test_gimbal_node_no_device_name(gimbal_ros2_node_and_thread, parameter_setter, config_params):
    gimbal_ros2_node, _ = gimbal_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # remove the device_name parameter to cause the exception to be raised!
    del config_params["device_name"] 

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Assert that a ValueError is raised
    with pytest.raises(ValueError, match="Please state the device_name in the configuration file."):
        # Setup the node so that it starts publishing with updated parameters
       gimbal_ros2_node.setup()

    assert True
    executor_test.shutdown()

def test_gimbal_node_no_dynamixel_type(gimbal_ros2_node_and_thread, parameter_setter, config_params):
    gimbal_ros2_node, _ = gimbal_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # remove the device_name parameter to cause the exception to be raised!
    del config_params["dynamixel_type"] 

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Assert that a ValueError is raised
    with pytest.raises(ValueError, match="Please state the dynamixel_type in the configuration file."):
        # Setup the node so that it starts publishing with updated parameters
        gimbal_ros2_node.setup()

    assert True
    executor_test.shutdown()
