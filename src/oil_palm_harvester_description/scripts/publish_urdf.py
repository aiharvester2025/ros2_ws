#!/usr/bin/env python3
"""Publish the package URDF on the /robot_description topic (transient local QoS).

Usage: publish_urdf.py /path/to/robot.urdf
"""
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


class URDFPublisher(Node):
    def __init__(self, urdf_path: str):
        super().__init__('publish_urdf')
        content = Path(urdf_path).read_text()
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(String, 'robot_description', qos)
        self.msg = String()
        self.msg.data = content
        # publish a few times and exit so tools get the URDF without repeatedly
        # re-publishing which can cause other nodes (e.g. joint_state_publisher)
        # to reinitialize their state.
        for _ in range(5):
            self.pub.publish(self.msg)
            self.get_logger().info('Published /robot_description')
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)


def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print('Usage: publish_urdf.py /path/to/robot.urdf')
        return
    node = URDFPublisher(sys.argv[1])
    try:
        # allow a short window for late subscribers to connect
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
