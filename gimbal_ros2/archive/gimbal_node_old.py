#!/usr/bin/env python3

"""
Developed By: Lionel

Updated to ROS2 by: S. M. Asjad

Update by S.Mock 10/16/24
"""

import dynamixel_sdk as dxl
import sys

from dynamixel_sdk import COMM_SUCCESS
import rclpy
from rclpy.node import Node
import rclpy.logging

from sensor_msgs.msg import JointState
import numpy as np
import time

#####

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_VELOCITY = 104
ADDR_VELOCITY_LIMIT = 44
ADDR_VELOCITY_PROFILE = 112
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_MINIMUM_POSITION = 52
ADDR_MAXIMUM_POSITION = 48
ADDR_PRESENT_POSITION = 132
ADDR_CONTROL_MODE = 11
POSITION_MODE = 3
ADDR_P_GAIN = 84
ADDR_I_GAIN = 82
ADDR_D_GAIN = 80

PROTOCOL_VERSION = 2.0
TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

DXL_ID_LINK_1 = 0 # PAN
DXL_ID_LINK_2 = 1 # TILT


# Set maximum Velocity
HIGH_VEL = 1023
LOW_VEL = int(1023/1)

class GimbalDriver(Node):
    def __init__(self):
        super().__init__("gimbal_driver")

        # Declare parameters with default values
        self.declare_parameter('baudrate', 3000000)
        self.declare_parameter('device_name', '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5UZP-if00-port0')
        self.declare_parameter('start_pos', [0.0, 0.0])
        self.declare_parameter('min_position_deg', [-100.0, -100.0])
        self.declare_parameter('max_position_deg', [100.0, 100.0])
        self.declare_parameter('kP', 500)
        self.declare_parameter('kI', 0)
        self.declare_parameter('kD', 200)

        # Get parameters from the parameter sever
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.device_name = self.get_parameter('device_name').get_parameter_value().string_value
        self.start_pos = self.get_parameter('start_pos').get_parameter_value().double_array_value
        self.min_position_deg = self.get_parameter('min_position_deg').get_parameter_value().double_array_value
        self.max_position_deg = self.get_parameter('max_position_deg').get_parameter_value().double_array_value
        self.kP = self.get_parameter('kP').get_parameter_value().integer_value
        self.kI = self.get_parameter('kI').get_parameter_value().integer_value
        self.kD = self.get_parameter('kD').get_parameter_value().integer_value

        # Intialize PortHandler and PackerHandler instances
        self.portHandler = dxl.PortHandler(self.device_name)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

         # Open port
        if self.portHandler.openPort():
            self.get_logger().info("Succeeded to open port for Gimbal Driver")
        else:
            self.get_logger().error("Failed to open the port for Gimbal Driver")
            sys.exit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            self.get_logger().info("Succeeded to change the baudrate for Gimbal Driver")
        else:
            self.get_logger().error("Failed to change the baudrate for Gimbal Driver")
            sys.exit()

        # Disable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID_LINK_1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID_LINK_2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )

        # Place Dynamixel in position control mode
        self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID_LINK_1, ADDR_CONTROL_MODE, POSITION_MODE
        )
        self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID_LINK_2, ADDR_CONTROL_MODE, POSITION_MODE
        )

        # Set Safety Position Limits for Min / Max, PID Gains
        for i in range(2):
            self.packetHandler.write4ByteTxRx(self.portHandler, i, ADDR_MINIMUM_POSITION, self.degrees_to_ticks(self.min_position_deg[i]))
            self.packetHandler.write4ByteTxRx(self.portHandler, i, ADDR_MAXIMUM_POSITION, self.degrees_to_ticks(self.max_position_deg[i]))
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_P_GAIN, self.kP)
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_I_GAIN, self.kI)
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_D_GAIN, self.kD)

        # Register 44
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_LINK_1, ADDR_VELOCITY_LIMIT, HIGH_VEL)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_LINK_2, ADDR_VELOCITY_LIMIT, HIGH_VEL)

        # # Register 112
        # self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_LINK_1, ADDR_VELOCITY_PROFILE, LOW_VEL)
        # self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_LINK_2, ADDR_VELOCITY_PROFILE, LOW_VEL)
        
        # Enable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID_LINK_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID_LINK_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )

        self.position = [0, 0]

        # Get the current position of the gimbal (prior to starting)
        for i in range(2):
            self.position[i] = self.ticks_to_degrees(self.packetHandler.read4ByteTxRx(self.portHandler, i, ADDR_PRESENT_POSITION)[0])

        # Set Start Position 
        self.update_servo(DXL_ID_LINK_1, self.start_pos[0])
        self.update_servo(DXL_ID_LINK_2, self.start_pos[1])

        self.sub_gimbal_joint = self.create_subscription(
            JointState, "gimbal", self.listener, 10
        )
        self.packet_num = 0


    def listener(self, msg):
        self.packet_num += 1
        self.time = time.time()
        self.yaw = msg.position[msg.name.index("yaw")]
        self.pitch = msg.position[msg.name.index("pitch")]

        # Recieve Messages in form of Radians, perform math to have them between -180 to 180 deg
        yaw = self.yaw % (2 * np.pi)
        pitch = self.pitch % (2 * np.pi)
        if yaw > np.pi:
            yaw -= 2 * np.pi
        if pitch > np.pi:
            pitch -= 2 * np.pi
        self.update_servo(DXL_ID_LINK_1, yaw)
        self.update_servo(DXL_ID_LINK_2, pitch)
        self.get_logger().info("Packet Number: %d, Time: %f, Yaw: %f, Pitch: %f \n" % (self.packet_num, self.time, yaw, pitch))


    def ticks_to_degrees(self, ticks):
        return (ticks - 2048) * 0.088

    def degrees_to_ticks(self, degrees):
        return int(degrees / 0.088) + 2048

        
    def update_servo(self, motorID, angle_in_radian):

        # Have to multiple by * -1 because reverse of gimbal movement to head tracking
        angle_in_deg = -1 * angle_in_radian * 180 / np.pi
        servo_pos_tick = self.degrees_to_ticks(angle_in_deg)
        # print("Requested Angle: " + str(angle_in_deg))
        # print("Current Position: " + str(self.position[motorID]))
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motorID, ADDR_GOAL_POSITION, servo_pos_tick
        )

        self.position[motorID] = angle_in_deg
        self.get_logger().info("Motor ID: %d, Position (DEG): %d" % (motorID, angle_in_deg))
    
    def __del__(self):

        # Disable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID_LINK_1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID_LINK_2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )

        # Close port
        self.portHandler.closePort()
        print("Gimbal Driver Port Closed")


def main():
    rclpy.init()
    gimbal_driver = GimbalDriver()
    rclpy.spin(gimbal_driver)
    rclpy.shutdown()
    gimbal_driver.portHandler.closePort()
    print("Gimbal Driver Port Closed")

if __name__ == "__main__":
    main()


