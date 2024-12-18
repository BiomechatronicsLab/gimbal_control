import pytest
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from dynamixel_driver.XL430_W250_manager import XL430W250Manager

config_directory = os.path.join(get_package_share_directory("gimbal_ros2"), "config")
config_file_path = os.path.join(config_directory, "test_params.yaml")

@pytest.fixture(scope="function")
def config_params():
    print(f"Loading configuration from: {config_file_path}")

    with open(config_file_path, "r") as file:
        return yaml.safe_load(file)
    
@pytest.fixture(scope="function")
def dynamixel_manager(config_params):
    baud_rate = config_params["baud_rate"]
    device_name = config_params['device_name']

    motors_ids = list(range(2)) # Just two motors for the gimbal
    try:
        if config_params["dynamixel_type"] == "XL430-W250":
            dynamixel_mgr = XL430W250Manager(motors_ids, baud_rate, device_name)

        yield dynamixel_mgr

    except:
        raise ValueError(f"Double check config for dynamixel setting")

