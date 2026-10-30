#!/usr/bin/env python3
"""Blank /map until pcd2pgm has one, so Nav2 can plan before GLIM closes its first submap.

Nav2's global static layer blocks planning until a map arrives, and GLIM only publishes
/glim_ros/map (-> pcd2pgm) once a submap closes. This node publishes an all-unknown grid
with pcd2pgm's fixed geometry (so the costmap never resizes), then relays pcd2pgm's map
(remapped to /map_glim) onto /map. One publisher on /map, latched like pcd2pgm's.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapSeed(Node):
    def __init__(self):
        super().__init__('map_seed')

        # Must match pcd2pgm's fixed grid (pcd2pgm_sim.yaml).
        self.declare_parameter('origin_x', -25.0)
        self.declare_parameter('origin_y', -25.0)
        self.declare_parameter('width_m', 50.0)
        self.declare_parameter('height_m', 50.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('frame_id', 'map')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.sub = self.create_subscription(OccupancyGrid, '/map_glim', self.relay, qos)
        self.got_real_map = False

        self.pub.publish(self.blank_map())
        self.get_logger().info("Published blank /map, waiting for pcd2pgm's map on /map_glim")

    def blank_map(self):
        res = self.get_parameter('resolution').value
        msg = OccupancyGrid()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = res
        msg.info.width = round(self.get_parameter('width_m').value / res)
        msg.info.height = round(self.get_parameter('height_m').value / res)
        msg.info.origin.position.x = self.get_parameter('origin_x').value
        msg.info.origin.position.y = self.get_parameter('origin_y').value
        msg.info.origin.orientation.w = 1.0
        msg.data = [-1] * (msg.info.width * msg.info.height)
        return msg

    def relay(self, msg: OccupancyGrid):
        if not self.got_real_map:
            self.got_real_map = True
            self.get_logger().info(f'First map from pcd2pgm ({msg.info.width}x{msg.info.height}), relaying')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapSeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
