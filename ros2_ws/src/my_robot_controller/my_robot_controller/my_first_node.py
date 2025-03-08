#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("hello ros2")
        self.counter_=0
        self.create_timer(1.0, self.timer_callback)
    def timer_callback(self):
        self.counter_+=1
        self.get_logger().info(f"Hello from timer, this is "+str(self.counter_)+"th message")
def main(args=None):
    rclpy.init()
    node=MyNode()
    rclpy.spin(node)

    rclpy.shutdown()
if __name__=='__main__':
    main()