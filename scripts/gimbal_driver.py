#!/usr/bin/env python3

import rclpy
from gimbal_ros2.gimbal_node import GimbalNode

def main(args=None):
    rclpy.init(args=args)

    gimbal = GimbalNode()

    try:
        rclpy.spin(gimbal)
    except Exception as e:
        print(e)
    finally:
        gimbal.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()