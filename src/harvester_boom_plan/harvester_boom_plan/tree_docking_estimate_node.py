"""Publish the tree docking estimate (trunk-top world point) for the boom planner.

This is a thin bridge that turns the offline tree-height analysis result into a
live ``/harvester/tree/docking_estimate`` (``geometry_msgs/PointStamped``) that
the boom-plan node consumes.

In the reference simulation the trunk top is at world z = 12.0 m and the trunk
axis is at world (8.5, 0).  In a live deployment the value would come from the
LiDAR sweep estimator (``analyze_tree_scan_v2.py``) instead of a static value.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Header


class TreeDockingEstimateNode(Node):
    def __init__(self):
        super().__init__("tree_docking_estimate_node")

        self.declare_parameter("trunk_top_height_m", 12.0)
        self.declare_parameter("trunk_x_m", 8.5)
        self.declare_parameter("trunk_y_m", 0.0)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate_hz", 5.0)

        self.height = float(self.get_parameter("trunk_top_height_m").value)
        self.x = float(self.get_parameter("trunk_x_m").value)
        self.y = float(self.get_parameter("trunk_y_m").value)
        self.frame = str(self.get_parameter("frame_id").value)
        rate = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(PointStamped, "/harvester/tree/docking_estimate", 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"Publishing trunk-top docking estimate at world "
            f"({self.x}, {self.y}, {self.height}) on "
            f"/harvester/tree/docking_estimate")

    def _publish(self):
        msg = PointStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.height
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TreeDockingEstimateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
