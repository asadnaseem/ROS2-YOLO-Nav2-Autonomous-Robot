#!/usr/bin/env python3
"""Continuous, fault-tolerant Nav2 waypoint patrol manager."""

import math
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')
        self.declare_parameter('pause_at_waypoint', 2.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('goal_timeout', 90.0)
        self.waypoints = [
            (-4.0, -3.0, 0.0), (-4.8, 1.0, 1.57), (-3.0, 4.0, 0.0),
            (0.0, 4.0, 0.0), (4.8, 4.0, -1.57), (5.0, 0.0, -1.57),
            (4.8, -2.5, 3.14), (2.0, -4.1, 3.14), (-2.0, -4.1, 3.14),
        ]
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.index, self.retry_count = 1, 0
        self.goal_handle = self.goal_started = self.next_goal_time = None
        self.cancel_requested = False
        self.wait_logged = False
        self.timer = self.create_timer(0.5, self.tick)

    def status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def tick(self):
        now = self.get_clock().now()
        if self.goal_handle is not None:
            timeout = float(self.get_parameter('goal_timeout').value)
            if ((now - self.goal_started).nanoseconds / 1e9 > timeout
                    and not self.cancel_requested):
                self.status('TIMEOUT: cancelling goal and continuing patrol')
                self.goal_handle.cancel_goal_async()
                self.cancel_requested = True
            return
        if self.next_goal_time is not None and now < self.next_goal_time:
            return
        if not self.client.wait_for_server(timeout_sec=0.05):
            if not self.wait_logged:
                self.get_logger().info('Waiting for Nav2...')
                self.wait_logged = True
            return
        self.wait_logged = False
        self.send_goal()

    def send_goal(self):
        x, y, yaw = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.status(f'PATROL: waypoint {self.index + 1}/{len(self.waypoints)} '
                    f'({x:.1f}, {y:.1f}), attempt {self.retry_count + 1}')
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)
        self.next_goal_time = None

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.status('REJECTED: retrying or skipping waypoint')
            self.retry_or_advance()
            return
        self.goal_handle = handle
        self.goal_started = self.get_clock().now()
        self.cancel_requested = False
        result = handle.get_result_async()
        result.add_done_callback(self.result_cb)

    def feedback_cb(self, message):
        self.status_pub.publish(String(
            data=f'MOVING: {message.feedback.distance_remaining:.2f} m remaining'))

    def result_cb(self, future):
        wrapped = future.result()
        self.goal_handle = self.goal_started = None
        self.cancel_requested = False
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.status(f'REACHED: waypoint {self.index + 1}')
            self.retry_count = 0
            self.index = (self.index + 1) % len(self.waypoints)
            delay = float(self.get_parameter('pause_at_waypoint').value)
            self.next_goal_time = self.get_clock().now() + Duration(seconds=delay)
        else:
            self.status(f'FAILED: Nav2 status {wrapped.status}')
            self.retry_or_advance()

    def retry_or_advance(self):
        self.retry_count += 1
        if self.retry_count > int(self.get_parameter('max_retries').value):
            self.status('SKIPPING unreachable waypoint; patrol continues')
            self.retry_count = 0
            self.index = (self.index + 1) % len(self.waypoints)
        self.next_goal_time = self.get_clock().now() + Duration(seconds=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
