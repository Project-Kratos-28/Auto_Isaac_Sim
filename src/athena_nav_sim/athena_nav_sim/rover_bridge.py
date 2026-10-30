#!/usr/bin/env python3
"""Sim stand-in for the rover's rover_bridge: gates Nav2's velocity onto /cmd_vel.

The Kratos GUI's AUTO/MANUAL button calls /rover_bridge/set_auto. In AUTO, Nav2's
output (cmd_vel_auto) is forwarded to /cmd_vel; in MANUAL it is dropped, so the
joystick owns /cmd_vel. Moving a joystick stick drops back to MANUAL, like on the rover.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class RoverBridge(Node):
    def __init__(self):
        super().__init__('rover_bridge')

        self.declare_parameter('nav_topic', 'cmd_vel_auto')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('joy_deadband', 0.2)

        self.auto = False
        self.joy_deadband = self.get_parameter('joy_deadband').value

        self.pub_cmd_vel = self.create_publisher(
            Twist, self.get_parameter('out_topic').value, 10)
        self.sub_nav = self.create_subscription(
            Twist, self.get_parameter('nav_topic').value, self.nav_callback, 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.srv = self.create_service(SetBool, '~/set_auto', self.set_auto_callback)

        self.get_logger().info("rover_bridge (sim) ready in MANUAL")

    def set_mode(self, auto, reason):
        if auto == self.auto:
            return
        self.auto = auto
        if not auto:
            self.pub_cmd_vel.publish(Twist())   # stop whatever Nav2 last commanded
        self.get_logger().info(f"{'AUTO' if auto else 'MANUAL'} ({reason})")

    def set_auto_callback(self, request, response):
        self.set_mode(request.data, 'GUI')
        response.success = True
        response.message = 'AUTO' if self.auto else 'MANUAL'
        return response

    def nav_callback(self, msg: Twist):
        if self.auto:
            self.pub_cmd_vel.publish(msg)

    def joy_callback(self, msg: Joy):
        if self.auto and any(abs(a) > self.joy_deadband for a in msg.axes):
            self.set_mode(False, 'joystick moved')


def main(args=None):
    rclpy.init(args=args)
    node = RoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
