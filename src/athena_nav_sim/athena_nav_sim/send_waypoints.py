#!/usr/bin/env python3
"""Drive through x,y waypoints (map frame) with Nav2, and report how close the rover got.

    ros2 run athena_nav_sim send_waypoints 2,0 2,2 0,2
    ros2 run athena_nav_sim send_waypoints --relative 3,0      # 3 m ahead of the rover

--relative: waypoints are in the rover's frame at start (x forward, y left).
--frame F: waypoints are in frame F (e.g. odom, Isaac's ground truth), converted to map once at start.
Each waypoint's heading points from the previous one. Switches rover_bridge to AUTO for the
run (if it is running) and back to MANUAL at the end, also on Ctrl+C.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


class SendWaypoints(Node):
    def __init__(self):
        super().__init__('send_waypoints', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.set_auto = self.create_client(SetBool, '/rover_bridge/set_auto')
        self.nav = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.goal_handle = None

    def rover_pose(self, timeout=10.0, frame='base_link'):
        """Pose of `frame` in map: (x, y, yaw)."""
        # Wall-clock deadline: sim time jumps from 0 when the first /clock arrives.
        end = time.monotonic() + timeout
        while rclpy.ok():
            try:
                t = self.tf_buffer.lookup_transform('map', frame, Time())
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return t.transform.translation.x, t.transform.translation.y, yaw
            except TransformException:
                if time.monotonic() > end:
                    raise
                rclpy.spin_once(self, timeout_sec=0.1)

    def bridge(self, auto):
        if not self.set_auto.wait_for_service(timeout_sec=1.0):
            return
        future = self.set_auto.call_async(SetBool.Request(data=auto))
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info(f"rover_bridge -> {'AUTO' if auto else 'MANUAL'}")

    def run(self, points, relative, frame):
        x0, y0, yaw0 = self.rover_pose()
        if relative:
            frame_x, frame_y, frame_yaw = x0, y0, yaw0
        elif frame != 'map':
            frame_x, frame_y, frame_yaw = self.rover_pose(frame=frame)
        else:
            frame_x, frame_y, frame_yaw = 0.0, 0.0, 0.0
        c, s = math.cos(frame_yaw), math.sin(frame_yaw)
        points = [(frame_x + c * px - s * py, frame_y + s * px + c * py) for px, py in points]

        goal = NavigateThroughPoses.Goal()
        prev = (x0, y0)
        for x, y in points:
            yaw = math.atan2(y - prev[1], x - prev[0])
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            goal.poses.append(pose)
            prev = (x, y)
        route = ' -> '.join(f'({x:.2f}, {y:.2f})' for x, y in points)
        self.get_logger().info(f'Start ({x0:.2f}, {y0:.2f}) -> {route}')

        if not self.nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_through_poses not available (is Nav2 up?)')
            return 1
        self.bridge(True)
        send = self.nav.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        self.goal_handle = send.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.bridge(False)
            return 1
        result = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        self.bridge(False)

        status = result.result().status
        x, y, _ = self.rover_pose()
        gx, gy = points[-1]
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f"{'SUCCEEDED' if ok else f'FAILED (status {status}, error {result.result().result.error_code})'}: "
            f'rover at ({x:.2f}, {y:.2f}), {math.hypot(x - gx, y - gy):.2f} m from the last waypoint')
        return 0 if ok else 1

    def on_feedback(self, msg):
        f = msg.feedback
        self.get_logger().info(
            f'{f.number_of_poses_remaining} waypoint(s) left, {f.distance_remaining:.2f} m to go',
            throttle_duration_sec=3.0)

    def cancel(self):
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            self.get_logger().info('Goal cancelled')
        self.bridge(False)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('waypoints', nargs='+', help='x,y pairs, e.g. 2,0 2,2')
    parser.add_argument('--relative', action='store_true', help="x,y in the rover's start frame")
    parser.add_argument('--frame', default='map', help='frame the x,y are in (default map)')
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    # '--' first, so negative waypoints like -1.6,3.6 aren't read as options.
    flags = []
    for i, a in enumerate(argv):
        if a in ('-h', '--help', '--relative'):
            flags.append(a)
        elif a == '--frame' and i + 1 < len(argv):
            flags += [a, argv[i + 1]]
    rest = [a for i, a in enumerate(argv) if a not in flags and not (i > 0 and argv[i - 1] == '--frame')]
    args = parser.parse_args(flags + ['--'] + rest)
    points = [tuple(float(v) for v in p.split(',')) for p in args.waypoints]

    # Python's own SIGINT handler (KeyboardInterrupt), so Ctrl+C can still cancel the goal.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = SendWaypoints()
    code = 1
    try:
        code = node.run(points, args.relative, args.frame)
    except KeyboardInterrupt:
        node.cancel()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
