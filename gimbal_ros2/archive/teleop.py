#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
import threading
import rclpy.logging
import time
import pygame

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(JointState, 'gimbal', 10)
        self.pitch = 0
        self.yaw = 0    
        self.logger = self.get_logger()
        self.angle_increment = 2.5 # Adjust this value as needed

    def publish_yaw_pitch(self):
        msg = JointState()
        msg.name = ["yaw", "pitch"]
        msg.position = [float(self.yaw), float(self.pitch)]
        self.publisher_.publish(msg)
        # Log a message
        self.logger.info("Publishing New Yaw and Pitch Values: %f, %f" % (self.yaw, self.pitch))

    def run(self):
        # Initialize pygame
        pygame.init()
        screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Teleop Node")

        # Key state tracking
        keys = {
            pygame.K_w: False,
            pygame.K_x: False,
            pygame.K_a: False,
            pygame.K_d: False,
            pygame.K_e: False,
            pygame.K_z: False,
            pygame.K_c: False,
            pygame.K_q: False,
            pygame.K_s: False
        }

        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rclpy.shutdown()
                    break
                
                # Key down events
                if event.type == pygame.KEYDOWN:
                    if event.key in keys:
                        keys[event.key] = True
                    if event.key == pygame.K_s:
                        return  # Exit on 's' key press

                # Key up events
                if event.type == pygame.KEYUP:
                    if event.key in keys:
                        keys[event.key] = False

            # Update pitch and yaw based on the keys pressed
            if keys[pygame.K_w]:
                self.pitch += self.angle_increment * np.pi / 180
            if keys[pygame.K_x]:
                self.pitch -= self.angle_increment * np.pi / 180
            if keys[pygame.K_a]:
                self.yaw -= self.angle_increment * np.pi / 180
            if keys[pygame.K_d]:
                self.yaw += self.angle_increment * np.pi / 180
            if keys[pygame.K_e]:
                self.pitch += self.angle_increment * np.pi / 180
                self.yaw += self.angle_increment * np.pi / 180
            if keys[pygame.K_z]:
                self.pitch -= self.angle_increment * np.pi / 180
                self.yaw -= self.angle_increment * np.pi / 180
            if keys[pygame.K_c]:
                self.pitch -= self.angle_increment * np.pi / 180
                self.yaw += self.angle_increment * np.pi / 180
            if keys[pygame.K_q]:
                self.pitch += self.angle_increment * np.pi / 180
                self.yaw -= self.angle_increment * np.pi / 180
            
            self.publish_yaw_pitch()
            time.sleep(0.1)  # Adjust the loop rate as needed

        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    # Create a separate thread for running the teleop_node
    thread = threading.Thread(target=teleop_node.run)
    thread.start()
    rclpy.spin(teleop_node)

if __name__ == '__main__':
    main()
