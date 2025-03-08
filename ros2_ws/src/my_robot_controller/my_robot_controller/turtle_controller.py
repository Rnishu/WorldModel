#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist
from functools import partial
import sys

class TurtleControllerNode(Node):
    def __init__(self,name:str="turtle1"):
        super().__init__(f"{name}_controller")
        self.previous_x_=0
        self.pose_subscriber_=self.create_subscription(Pose,"/turtle1/pose",self.bound_check_and_move,10)
        self.cmd_vel_pub_=self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info(f"{name} controller has started")
    def bound_check_and_move(self, pose:Pose):
        msg=Twist()
        if pose.x>9.0 or pose.x<2.0 or pose.y>9 or pose.y<2:
            msg.linear.x=1.0
            msg.angular.z=0.9
        else:
            msg.linear.x=20.0
            msg.angular.z=0.0

        if pose.x>5.5 and self.previous_x_<=5.5:
            self.get_logger().info("Set color to red")
            self.call_set_pen_service(255,0,0,3,0)
        elif pose.x<5.5 and self.previous_x_>=5.5:
            self.get_logger().info("Set color to green")

            self.call_set_pen_service(0,255,0,3,0)
        self.cmd_vel_pub_.publish(msg)
        self.previous_x_=pose.x
    def call_set_pen_service(self, r, g, b, width, off):
        client=self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service...")
        request=SetPen.Request()
        request.r=r
        request.g=g
        request.b=b
        request.width=width
        request.off=off
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    def callback_set_pen(self,future):
        try:
            response=future.result()
        except Exception as e:
            self.get_logger().error("service call fail: %r" % (e,))


def main(args=None):
    cli_args=sys.argv[1:]
    rclpy.init(args=args)
    node1=TurtleControllerNode("turtle1")
    node2=TurtleControllerNode("turtle2")

    rclpy.spin(node1)
    rclpy.spin(node2)

    rclpy.shutdown()