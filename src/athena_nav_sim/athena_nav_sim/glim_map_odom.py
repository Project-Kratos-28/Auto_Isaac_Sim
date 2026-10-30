#!/usr/bin/env python3
"""Publishes map -> odom from GLIM while Isaac keeps publishing odom -> base_link.

Isaac's Odom graph already owns odom -> base_link (ground truth), so GLIM runs with its own
frame names (map -> glim_odom -> glim_imu) and this node closes the tree, like AMCL does:

    T_map_odom = T_map_glim_imu * inverse(T_odom_imu_link)

glim_imu is where GLIM thinks the IMU is, imu_link is where Isaac's odometry puts it
(odom -> base_link -> livox_frame -> imu_link).
"""
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def to_matrix(t):
    m = np.eye(4)
    q = t.transform.rotation
    m[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    v = t.transform.translation
    m[:3, 3] = [v.x, v.y, v.z]
    return m


class GlimMapOdom(Node):
    def __init__(self):
        super().__init__('glim_map_odom')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('glim_imu_frame', 'glim_imu')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('rate', 20.0)
        # Future-date map -> odom like AMCL, so lookups at "now" don't extrapolate.
        self.declare_parameter('transform_tolerance', 0.1)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.glim_imu_frame = self.get_parameter('glim_imu_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.tolerance = Duration(seconds=self.get_parameter('transform_tolerance').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_stamp = None
        self.create_timer(1.0 / self.get_parameter('rate').value, self.update)

        self.get_logger().info(
            f"{self.map_frame} -> {self.odom_frame} from GLIM's {self.glim_imu_frame}, "
            f"waiting for GLIM...")

    def update(self):
        try:
            glim = self.tf_buffer.lookup_transform(self.map_frame, self.glim_imu_frame, Time())
            stamp = Time.from_msg(glim.header.stamp)
            # Isaac's odometry at the same instant as GLIM's pose.
            odom = self.tf_buffer.lookup_transform(self.odom_frame, self.imu_frame, stamp)
        except TransformException:
            return
        if self.last_stamp is None:
            self.get_logger().info('GLIM pose received, publishing map -> odom')
        self.last_stamp = stamp

        m = to_matrix(glim) @ np.linalg.inv(to_matrix(odom))
        q = Rotation.from_matrix(m[:3, :3]).as_quat()
        out = TransformStamped()
        out.header.stamp = (self.get_clock().now() + self.tolerance).to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = m[:3, 3]
        (out.transform.rotation.x, out.transform.rotation.y,
         out.transform.rotation.z, out.transform.rotation.w) = q
        self.tf_broadcaster.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    node = GlimMapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
