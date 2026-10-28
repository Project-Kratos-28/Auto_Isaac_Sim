#!/usr/bin/env python3
"""sim_adapter: bridge the electronics drive stack to Isaac Sim.

Replaces command_splitter.py (which targets ros2_control forward controllers
on the real rover) for the SIMULATION.

It converts the single 8-element command from drive_controller.py:
    /drive_controller/commands  Float64MultiArray
        data[0:4] = steering angles  (rad from center)  wheel order FL, FR, RL, RR
        data[4:8] = wheel velocities (rad/s)             wheel order FL, FR, RL, RR

into a sensor_msgs/JointState on /joint_command that the Isaac "rover_drive"
Action Graph (ROS2 Subscribe Joint State -> Articulation Controller) consumes:
    steer joints -> position (rad), velocity 0.0
    spin  joints -> position 0.0,   velocity (rad/s)

The Articulation Controller matches by joint NAME, so message order is not
important as long as each name carries its own value.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class SimAdapter(Node):
    def __init__(self):
        super().__init__('sim_adapter')

        # Wheel order of /drive_controller/commands is FL, FR, RL, RR.
        # These names MUST match the joints in your Isaac articulation / URDF.
        self.steer_joints = [
            'steer_front_left',
            'steer_front_right',
            'steer_rear_left',
            'steer_rear_right',
        ]
        self.spin_joints = [
            'wheel_front_left_spin',
            'wheel_front_right_spin',
            'wheel_rear_left_spin',
            'wheel_rear_right_spin',
        ]

        self.sub = self.create_subscription(
            Float64MultiArray, '/drive_controller/commands', self.cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.get_logger().info(
            'sim_adapter active: /drive_controller/commands (8) -> /joint_command (JointState)')

    def cb(self, msg: Float64MultiArray):
        if len(msg.data) != 8:
            self.get_logger().warn(
                f'Expected 8 values, got {len(msg.data)}; ignoring.',
                throttle_duration_sec=2.0)
            return

        steer = [float(v) for v in msg.data[0:4]]   # rad from center, FL FR RL RR
        vel   = [float(v) for v in msg.data[4:8]]   # rad/s,           FL FR RL RR

        names, positions, velocities = [], [], []
        for name, ang in zip(self.steer_joints, steer):
            names.append(name)
            positions.append(ang)
            velocities.append(0.0)
        for name, w in zip(self.spin_joints, vel):
            names.append(name)
            positions.append(0.0)
            velocities.append(w)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = names
        js.position = positions
        js.velocity = velocities
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
