#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import numpy as np

class DemoGimbalMotion(Node):
    def __init__(self):
        super().__init__('demo_gimbal_node')


        rad_90deg = np.pi / 2
        # Define positions to command
        self.positions_to_command = [
            [0.0, 0.0],
            [rad_90deg, 0.0], # right
            [0.0, 0.0],
            [-rad_90deg, 0.0], # left
            [0.0, 0.0],
            [0.0, rad_90deg], # up
            [0.0, 0.0],
            [0.0, -rad_90deg], # down
        ]
        
        # Initialize index for current position
        self.current_pose_index = 0

        # Create publisher for joint states
        self.pub_joint_states = self.create_publisher(JointState, 'gimbal', 10)

        # Create timer to publish joint states every 5 seconds
        self.timer = self.create_timer(0.5, self.publish_joint_states)

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["yaw", "pitch"]
        joint_state_msg.position = self.positions_to_command[self.current_pose_index]
        self.pub_joint_states.publish(joint_state_msg)
        self.current_pose_index = (self.current_pose_index + 1) % len(self.positions_to_command)

def main(args=None):
    rclpy.init(args=args)
    node = DemoGimbalMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()