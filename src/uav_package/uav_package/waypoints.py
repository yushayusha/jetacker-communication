#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import csv
from pathlib import Path

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped


class CSVStepPublisher(Node):

    def __init__(self):
        super().__init__('csv_step_publisher')

        # ── Parameters ─────────────────────────────
        self.declare_parameter('csv_file', 'waypoints.csv')
        csv_path = Path(self.get_parameter('csv_file').value)

        # ── Load CSV ──────────────────────────────
        if not csv_path.exists():
            self.get_logger().error(f'CSV file not found: {csv_path}')
            raise FileNotFoundError(csv_path)

        self.points = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                try:
                    x = float(row[1])
                    y = float(row[2])
                    self.points.append((x, y))
                except:
                    continue

        if not self.points:
            raise RuntimeError('No valid coordinates in CSV')

        self.get_logger().info(f'Loaded {len(self.points)} waypoints')

        # ── State ────────────────────────────────
        self.index = 0

        # ── ROS interfaces ──────────────────────
        self.sub = self.create_subscription(
            Bool,
            '/next_waypoint',
            self.trigger_callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/waypoints',
            10
        )

    # ── Trigger callback ─────────────────────────
    def trigger_callback(self, msg: Bool):
        # Only act on TRUE signals
        if not msg.data:
            return

        if self.index >= len(self.points):
            self.get_logger().warn('End of CSV reached')
            return

        x, y = self.points[self.index]

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        out.point.x = x
        out.point.y = y
        out.point.z = 0.0

        self.pub.publish(out)

        self.get_logger().info(f'Published waypoint {self.index}: ({x}, {y})')

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = CSVStepPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()