#!/usr/bin/env python3
import numpy as np
import time
import pytest

# Test Parameters
position_mode_enum = 3
baud_rate_3M_enum = 5 # Equivalent to 3M [bps]
baud_rate_2M_enum = 4 # Equivalent to 2M [bps]
baud_rate_1M_enum = 3 # Equivalent to 1M [bps]

# Helper Function
def command_and_check_position(dynamixel_manager, truth_goal_position_deg, tolerance_deg):
    dynamixel_manager.set_goal_position_deg(dynamixel_manager.motor_ids, truth_goal_position_deg)
    time.sleep(0.5)
    test_position_deg = dynamixel_manager.get_position_deg(dynamixel_manager.motor_ids)
    print(truth_goal_position_deg)
    print(test_position_deg)
    position_comparison = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
    print(position_comparison)
    comparison_result = [comparison < tolerance_deg for comparison in position_comparison]
    # comparison_result = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
    print(comparison_result)
    print("--------------------------")
    return all(comparison_result)

def test_kP(dynamixel_manager):
    kP = 600
    truth_kP = np.ones(len(dynamixel_manager.motor_ids)) * kP
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_kP(dynamixel_manager.motor_ids, truth_kP)
    test_kP = dynamixel_manager.get_kP(dynamixel_manager.motor_ids)
    assert truth_kP.tolist() == test_kP, "kP values not set correctly"
    time.sleep(0.5)

def test_kI(dynamixel_manager):
    kI = 0
    truth_kI = np.ones(len(dynamixel_manager.motor_ids)) * kI
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_kI(dynamixel_manager.motor_ids, truth_kI)
    test_kI = dynamixel_manager.get_kI(dynamixel_manager.motor_ids)
    assert truth_kI.tolist() == test_kI, "kI values not set correctly"
    time.sleep(0.5)

def test_kD(dynamixel_manager):
    kD = 200
    truth_kD = np.ones(len(dynamixel_manager.motor_ids)) * kD
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_kD(dynamixel_manager.motor_ids, truth_kD)
    test_kD = dynamixel_manager.get_kD(dynamixel_manager.motor_ids)
    assert truth_kD.tolist() == test_kD, "kD values not set correctly"
    time.sleep(0.5)

# TODO: this test is a little weird because if i change the baudrate (which is taken from the config right now, it will no longer connect)
def test_baud_rate(dynamixel_manager):
    truth_baud_rate = np.ones(len(dynamixel_manager.motor_ids)) * baud_rate_3M_enum
    dynamixel_manager.set_baud_rate(dynamixel_manager.motor_ids, truth_baud_rate)
    test_baud_rate = dynamixel_manager.get_baud_rate(dynamixel_manager.motor_ids)
    assert truth_baud_rate.tolist() == test_baud_rate, "baud_rate not set correctly"
    time.sleep(0.5)

def test_position_mode(dynamixel_manager):
    truth_operating_mode = np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum # Position Mode
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, truth_operating_mode)
    test_operating_mode = dynamixel_manager.get_operating_mode(dynamixel_manager.motor_ids)
    assert truth_operating_mode.tolist() == test_operating_mode
    time.sleep(0.5)

def test_velocity(dynamixel_manager):
    test_velocity = dynamixel_manager.get_velocity(dynamixel_manager.motor_ids)
    print(test_velocity)
    assert all([abs(val) <= 2.0 for val in test_velocity]), "velocity is practically 0"

    # TODO: could do more work to see if values are changing when moving around...

def test_torque_enable(dynamixel_manager):
    truth_torque_enable = np.zeros(len(dynamixel_manager.motor_ids))
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, truth_torque_enable)
    test_torque_enable = dynamixel_manager.get_torque_enable(dynamixel_manager.motor_ids)
    assert truth_torque_enable.tolist() == test_torque_enable # TORQUE OFF

    truth_torque_enable = np.ones(len(dynamixel_manager.motor_ids))
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, truth_torque_enable)
    test_torque_enable = dynamixel_manager.get_torque_enable(dynamixel_manager.motor_ids)
    assert truth_torque_enable.tolist() == test_torque_enable # TORQUE ON
    time.sleep(0.5)

def test_max_position(dynamixel_manager):
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))
    truth_max_position = np.ones(len(dynamixel_manager.motor_ids)) * 100.0
    dynamixel_manager.set_max_position_deg(dynamixel_manager.motor_ids, truth_max_position)
    test_max_position = dynamixel_manager.get_max_position_deg(dynamixel_manager.motor_ids)
    assert truth_max_position.tolist() == test_max_position 

def test_min_position(dynamixel_manager):
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))
    truth_min_position = np.ones(len(dynamixel_manager.motor_ids)) * -100.0
    dynamixel_manager.set_min_position_deg(dynamixel_manager.motor_ids, truth_min_position)
    test_min_position = dynamixel_manager.get_min_position_deg(dynamixel_manager.motor_ids)
    assert truth_min_position.tolist() == test_min_position 

@pytest.mark.parametrize(
    "truth_goal_position_deg", 
    [
        [0.0, 0.0],
        [90.0, 0.0], # right
        [0.0, 0.0],
        [-90.0, 0.0], # left
        [0.0, 0.0],
        [0.0, 90.0], # up
        [0.0, 0.0],
        [0.0, -90.0], # down
        [0.0, 0.0],
    ]
)

def test_position(dynamixel_manager, truth_goal_position_deg):
    # Test Parameters
    kP_gains = np.ones(len(dynamixel_manager.motor_ids)) * 600
    kI_gains = np.ones(len(dynamixel_manager.motor_ids)) * 0
    kD_gains = np.ones(len(dynamixel_manager.motor_ids)) * 200
    tolerance_deg = 10.0 # how much allowable error there can be (was never given any requirements...)

    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))
    dynamixel_manager.initialize_gains(dynamixel_manager.motor_ids, kP_gains, kI_gains, kD_gains)

    # Command Position!
    assert command_and_check_position(dynamixel_manager, truth_goal_position_deg, tolerance_deg)
    time.sleep(0.5)

