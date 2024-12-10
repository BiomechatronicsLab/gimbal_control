#!/usr/bin/env python3
import unittest
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from dynamixel_driver.XL430_W250_manager import XL430W250Manager

# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory('gimbal_ros2'), 'config')
config_file_path = os.path.join(config_directory, "test_params.yaml")

def load_yaml_file(file_path):
    # Print the path to the YAML file
    print(f"Loading configuration from: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

class TestDynamixelConnection(unittest.TestCase):
    
    def setUp(self):
        # Load configuration parameters
        self.config_params = load_yaml_file(config_file_path)

        if self.config_params["dynamixel_type"] == "XL430-W250":
            self.dynamixel_manager = XL430W250Manager(list(range(2)), self.config_params["baud_rate"],
                                                self.config_params["device_name"],
                                                self.config_params["kP"],
                                                self.config_params["kI"],
                                                self.config_params["kD"],
                                                )
    def tearDown(self):
        # Close the port if it's open
        if self.dynamixel_manager.port_handler:
            self.dynamixel_manager.port_handler.closePort()

    def test_motor_connection(self):
        packet_handler = self.dynamixel_manager.packet_handler
        port_handler = self.dynamixel_manager.port_handler

        for motor_id in self.dynamixel_manager.motor_ids:
            print(motor_id) # Print motor_ids
            model_number, comm_result, error = packet_handler.ping(port_handler, motor_id)
            self.assertEqual(model_number, self.dynamixel_manager.model_number, "Ping failed: Invalid model number") # Specific ID for the dynamixel
            self.assertEqual(comm_result, 0, "Ping failed: Communication error")
            self.assertEqual(error, 0, f"Ping failed: Motor error {error}")

if __name__ == '__main__':
    unittest.main()