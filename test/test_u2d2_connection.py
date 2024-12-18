#!/usr/bin/env python3

import pytest
import dynamixel_sdk as dxl

def test_port_connection(config_params):
    """Test to ensure successful connection to Dynamixel device."""
    try:
        # Initialize PortHandler and PacketHandler instances
        device_name = config_params['device_name']
        print(device_name)
        port_handler = dxl.PortHandler(device_name)

        # Attempt to open the port
        port_open = port_handler.openPort()

        # Verify that the port is open
        assert port_open, f"Failed to open port on device {device_name}"
                
        if port_handler:
            port_handler.closePort()

    except Exception as e:
        # Fail the test with the exception message
        pytest.fail(f"An exception occurred while trying to connect to the device: {e}", pytrace=True)

