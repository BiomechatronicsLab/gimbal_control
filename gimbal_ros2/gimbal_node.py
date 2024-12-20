#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.logging
from dynamixel_driver.XL430_W250_manager import XL430W250Manager
from sensor_msgs.msg import JointState
import numpy as np
import time

# TODO: I don't love this because you have to set your position enums in the leaphand node, when thats a driver specific implementation. TBD what to do.
POSITION_MODE_ENUM = 3

class GimbalNode(Node):
    def __init__(self, test_flag=False, node_name="gimbal_node"):
        super().__init__(node_name)
        # Declare parameters with default values
        self.declare_parameter("joint_command_topic", "/gimbal/peripheral/feedback_joint_states" )
        self.declare_parameter("joint_feedback_topic", "/gimbal/peripheral/dynamixel_joint_states")
        self.declare_parameter('baud_rate', 3000000)
        self.declare_parameter('device_name', "")
        self.declare_parameter("dynamixel_type", "")
        self.declare_parameter('min_position_deg', [-100.0, -100.0])
        self.declare_parameter('max_position_deg', [100.0, 100.0])
        self.declare_parameter('kP', 500)
        self.declare_parameter('kI', 0)
        self.declare_parameter('kD', 200)
        self.declare_parameter('start_pos_deg', [0.0, 0.0])
        self.dynamixel_mgr = None 
        
        if not test_flag:
            self.setup()

    def setup(self):
        # Obtain parameter values from the parameter server
        self.joint_command_topic = self.get_parameter("joint_command_topic").get_parameter_value().string_value
        self.joint_feedback_topic = self.get_parameter("joint_feedback_topic").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.device_name = self.get_parameter("device_name").get_parameter_value().string_value
        if self.device_name == "":
            raise ValueError(
                "Please state the device_name in the configuration file."
            )

        self.dynamixel_type = self.get_parameter("dynamixel_type").get_parameter_value().string_value
        if self.dynamixel_type == "":
            raise ValueError(
                "Please state the dynamixel_type in the configuration file."
            )

        self.kP = self.get_parameter("kP").get_parameter_value().integer_value
        self.kI = self.get_parameter("kI").get_parameter_value().integer_value
        self.kD = self.get_parameter("kD").get_parameter_value().integer_value
        self.start_pos_deg = self.get_parameter("start_pos_deg").get_parameter_value().double_array_value
        self.min_position_deg = self.get_parameter('min_position_deg').get_parameter_value().double_array_value
        self.max_position_deg = self.get_parameter('max_position_deg').get_parameter_value().double_array_value

        motor_ids = list(range(2))
        
        try:
            if self.dynamixel_type == "XL430-W250":
                self.dynamixel_mgr = XL430W250Manager(motor_ids, self.baud_rate, self.device_name)
        except:
            raise ValueError(f"Double check config for dynamixel setting")

        # Create publisher
        self.publisher = self.create_publisher(
            JointState, self.joint_feedback_topic, 10
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            JointState, self.joint_command_topic, self.command_callback, 10
        )

        # Initialize gains and operating mode
        # Currently operating mode is only set to position and cannot be changed (TBD!)
        self.dynamixel_mgr.set_torque_enable(self.dynamixel_mgr.motor_ids, np.zeros(len(self.dynamixel_mgr.motor_ids))) # Disable torques just in case
        self.dynamixel_mgr.set_operating_mode(self.dynamixel_mgr.motor_ids, np.ones(len(self.dynamixel_mgr.motor_ids)) * POSITION_MODE_ENUM)
        self.dynamixel_mgr.set_min_position_deg(self.dynamixel_mgr.motor_ids, self.min_position_deg)
        self.dynamixel_mgr.set_max_position_deg(self.dynamixel_mgr.motor_ids, self.max_position_deg)
        self.dynamixel_mgr.set_torque_enable(self.dynamixel_mgr.motor_ids, np.ones(len(self.dynamixel_mgr.motor_ids))) # Enable torques
        self.initialize_gains()

        # Set initial position
        self.dynamixel_mgr.set_goal_position_deg(self.dynamixel_mgr.motor_ids, self.start_pos_deg)
        
        # Create timer to publish data
        timer_period = 1.0 / 60.0
        self.timer = self.create_timer(timer_period, self.read_and_publish_data)

    def __del__(self):
        del self.dynamixel_mgr # Kill dynamixel manager object        

    def initialize_gains(self):
            try:
                kP = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kP                
                kI = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kI
                kD = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kD
                self.dynamixel_mgr.initialize_gains(self.dynamixel_mgr.motor_ids, kP, kI, kD)

            except Exception as e:
                self.get_logger().error(f"Error initializing gains: {str(e)}")

    def command_callback(self, msg):
        self.time = time.time()
        self.yaw = msg.position[msg.name.index("yaw")]
        self.pitch = msg.position[msg.name.index("pitch")]

        # Recieve Messages in form of Radians, perform math to convert them between -180 to 180 deg
        yaw = self.yaw % (2 * np.pi)
        pitch = self.pitch % (2 * np.pi)
        if yaw > np.pi:
            yaw -= 2 * np.pi
        if pitch > np.pi:
            pitch -= 2 * np.pi

        yaw_deg = np.degrees(yaw)
        pitch_deg = np.degrees(pitch)

        #TODO: TBD if you need to multiple by * -1 because reverse of gimbal movement to head tracking
        try:
            self.dynamixel_mgr.set_goal_position_deg(self.dynamixel_mgr.motor_ids, [yaw_deg, pitch_deg])
        except Exception as e:
            self.get_logger().error(f"Error in command callback: {str(e)}")

        self.get_logger().info("Time: %f, Yaw: %f, Pitch: %f \n" % (self.time, yaw, pitch))

    def read_and_publish_data(self):
        try:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            joint_state_msg.position = self.dynamixel_mgr.get_position_deg(self.dynamixel_mgr.motor_ids)
            velocity_readings = self.dynamixel_mgr.get_velocity(self.dynamixel_mgr.motor_ids)
            # because of the two's compliments, have to convert from a list of ints to a list of floats!
            joint_state_msg.velocity = list(map(float, velocity_readings))
            self.publisher.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading and publishing data: {str(e)}")

def main(args=None): 
    rclpy.init(args=args)
    gimbal_driver = GimbalNode()
    rclpy.spin(gimbal_driver)
    gimbal_driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


