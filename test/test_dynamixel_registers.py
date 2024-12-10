#!/usr/bin/env python3
import unittest
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from dynamixel_driver.XL430_W250_manager import XL430W250Manager

import numpy as np
import time
import pytest

# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory('gimbal_ros2'), 'config')
config_file_path = os.path.join(config_directory, "test_params.yaml")

def load_yaml_file(file_path):
    # Print the path to the YAML file
    print(f"Loading configuration from: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

class TestDynamixelConnection(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        # Load configuration parameters
        self.config_params = load_yaml_file(config_file_path)
        self.motor_ids = list(range(2))

        if self.config_params["dynamixel_type"] == "XL430-W250":
            self.dynamixel_manager = XL430W250Manager(list(range(2)), self.config_params["baud_rate"],
                                                self.config_params["device_name"],
                                                self.config_params["kP"],
                                                self.config_params["kI"],
                                                self.config_params["kD"],
                                                )
            
    @classmethod
    def tearDownClass(self):
        # Close the port if it's open
        if self.dynamixel_manager.port_handler:
            self.dynamixel_manager.port_handler.closePort()

    
    @pytest.mark.run(order=1)
    def test_kP(self):
        test_kP = self.dynamixel_manager.get_kP(self.motor_ids)
        truth_kP = np.ones(len(self.motor_ids)) * self.config_params["kP"]
        # truth_kP[[0, 4, 8]] = np.ones(3) * (self.config_params["kP"] * 0.75) 
        self.assertListEqual(truth_kP.tolist(), test_kP, "kP values not set correctly")

    @pytest.mark.run(order=2)
    def test_kI(self):
        test_kI = self.dynamixel_manager.get_kI(self.motor_ids)
        truth_kI = np.ones(len(self.motor_ids)) * self.config_params["kI"]
        # truth_kI[[0, 4, 8]] = np.ones(3) * (self.config_params["kI"] * 0.75) 
        self.assertListEqual(truth_kI.tolist(), test_kI, "kI values not set correctly")

    @pytest.mark.run(order=3)
    def test_kD(self):
        test_kD = self.dynamixel_manager.get_kD(self.motor_ids)
        truth_kD = np.ones(len(self.motor_ids)) * self.config_params["kD"]
        # truth_kD[[0, 4, 8]] = np.ones(3) * (self.config_params["kD"] * 0.75) 
        self.assertListEqual(truth_kD.tolist(), test_kD, "kD values not set correctly")

    @pytest.mark.run(order=4)
    def test_baud_rate(self):
        # truth_baud_rate = np.ones(len(self.motor_ids)) * self.config_params["baud_rate"]
        # TODO: make this parameterized to the ENUM (as presented in the SDK)
        truth_baud_rate = np.ones(len(self.motor_ids)) * 5
        test_baud_rate = self.dynamixel_manager.get_baud_rate(self.motor_ids)
        self.assertEqual(truth_baud_rate.tolist(), test_baud_rate)

    @pytest.mark.run(order=6)
    def test_position_mode(self):
        truth_operating_mode = np.ones(len(self.motor_ids)) * 3 # Position Mode
        test_operating_mode = self.dynamixel_manager.get_operating_mode(self.motor_ids)
        self.assertEqual(truth_operating_mode.tolist(), test_operating_mode)

    @pytest.mark.run(order=7)
    def test_torque_enable(self):
        truth_torque_enable = np.ones(len(self.motor_ids))
        test_torque_enable = self.dynamixel_manager.get_torque_enable(self.motor_ids)
        self.assertEqual(truth_torque_enable.tolist(), test_torque_enable)

    @pytest.mark.run(order=9)
    def test_multiple_positions(self):
                # Define positions to command


        # Define positions to command
        positions_to_command = [
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

        for truth_goal_position_deg in positions_to_command:
            truth_goal_position_ticks = self.dynamixel_manager.degrees_to_ticks_list(truth_goal_position_deg)
            self.dynamixel_manager.set_goal_position(self.motor_ids, truth_goal_position_ticks)
            time.sleep(1.0)
            test_position_ticks = self.dynamixel_manager.get_position(self.motor_ids)
            test_position_deg = self.dynamixel_manager.ticks_to_degrees_list(test_position_ticks)

            # Tolerance value
            tolerance = 10  
            # Compare element-wise and create boolean list based on the tolerance
            position_comparison = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
            comparison_result = [comparison < tolerance for comparison in position_comparison]
           
            # hardware_status = self.dynamixel_manager.get_hardware_error_status(self.motor_ids)

            print(truth_goal_position_deg)
            print(test_position_deg)
            print(position_comparison)
            print(comparison_result)
            # print(hardware_status)
            print("------------------------------------")
            # Assert that all elements in comparison_result are True
            self.assertTrue(all(comparison_result))  # This will pass only if all values are True



if __name__ == '__main__':
    unittest.main()
