#!/usr/bin/env python3
"""Republish a Gazebo PointCloud2 with a TF-compatible latest timestamp.

Gazebo Classic emits sensor messages in simulation time.  This project uses
joint_state_publisher_gui and robot_state_publisher for the interactive model,
which use system time.  Foxy RViz requires the cloud and its dynamic TF chain
to share one time domain.  A zero ROS timestamp asks TF2 for the latest
available transform, avoiding an unreliable exact-time lookup through the
moving arm.  The relay preserves every point and field; it only replaces the
outgoing header timestamp.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class LidarTimestampBridge(Node):
    def __init__(self, source_topic: str, output_topic: str):
        super().__init__('lidar_timestamp_bridge')
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(
            PointCloud2, source_topic, self.on_cloud, sensor_qos)
        self.received_cloud_count = 0
        self.get_logger().info(
            f'Republishing {source_topic} to {output_topic} with latest-TF timestamps')

    def on_cloud(self, cloud: PointCloud2):
        # ``Time(0)`` is the TF2 convention for "latest available transform".
        # The raw Gazebo topic retains its original simulation timestamp for
        # algorithms that need sensor acquisition time.
        cloud.header.stamp.sec = 0
        cloud.header.stamp.nanosec = 0
        self.publisher.publish(cloud)
        self.received_cloud_count += 1
        if self.received_cloud_count == 1:
            self.get_logger().info(
                'Forwarding LiDAR clouds with latest-TF timestamps for RViz')


def main():
    source_topic = sys.argv[1] if len(sys.argv) > 1 else '/harvester/lidar/raw_points'
    output_topic = sys.argv[2] if len(sys.argv) > 2 else '/harvester/lidar/points'
    rclpy.init()
    node = LidarTimestampBridge(source_topic, output_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
