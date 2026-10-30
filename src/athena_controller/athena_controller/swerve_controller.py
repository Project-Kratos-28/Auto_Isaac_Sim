#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')

        # ---- ROVER GEOMETRY — FILL IN ----
        self.wheels = {
            'front_left':  ( -0.393,  0.392),
            'front_right': ( -0.393, -0.392),
            'rear_left':   (0.345,  0.379),
            'rear_right':  (0.345, -0.379),
        }
        self.wheel_radius = 0.10
        self.steer_limit  = math.radians(90)

        self.steer_joints = {
            'front_left':  'steer_front_left',
            'front_right': 'steer_front_right',
            'rear_left':   'steer_rear_left',
            'rear_right':  'steer_rear_right',
        }
        self.drive_joints = {
            'front_left':  'wheel_front_left_spin',
            'front_right': 'wheel_front_right_spin',
            'rear_left':   'wheel_rear_left_spin',
            'rear_right':  'wheel_rear_right_spin',
        }

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.get_logger().info('Swerve controller started. Listening on /cmd_vel')

    def cmd_cb(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        names, positions, velocities = [], [], []

        for corner, (x, y) in self.wheels.items():
            wheel_vx = vx - wz * y
            wheel_vy = vy + wz * x
            steer_angle = math.atan2(wheel_vy, wheel_vx)
            speed       = math.hypot(wheel_vx, wheel_vy)

            # Keep steering inside its physical +/-90 degree range without
            # losing the requested travel direction.  In particular, a pure
            # reverse command produces atan2(0, negative) == pi: point the
            # wheel straight and rotate it backwards instead of clamping the
            # steering to +90 degrees and always using a positive speed.
            if steer_angle > self.steer_limit:
                steer_angle -= math.pi
                speed = -speed
            elif steer_angle < -self.steer_limit:
                steer_angle += math.pi
                speed = -speed

            wheel_spin  = speed / self.wheel_radius
            steer_angle = max(-self.steer_limit, min(self.steer_limit, steer_angle))

            names.append(self.steer_joints[corner]); positions.append(steer_angle); velocities.append(0.0)
            names.append(self.drive_joints[corner]); positions.append(0.0);         velocities.append(wheel_spin)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name, js.position, js.velocity = names, positions, velocities
        self.pub.publish(js)


def main():
    rclpy.init()
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
