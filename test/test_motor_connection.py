#!/usr/bin/env python3
def test_motor_connection(dynamixel_manager):
    packet_handler = dynamixel_manager.packet_handler
    port_handler = dynamixel_manager.port_handler

    for motor_id in dynamixel_manager.motor_ids:
        print(motor_id) # Print motor_ids
        model_number, comm_result, error = packet_handler.ping(port_handler, motor_id)
        assert model_number == dynamixel_manager.model_number, "Ping failed: Invalid model number" # Specific ID for the dynamixel
        assert comm_result == 0, "Ping failed: Communication error"
        assert error == 0, f"Ping failed: Motor error {error}"
