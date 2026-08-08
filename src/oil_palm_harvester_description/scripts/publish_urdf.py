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
    def __init__(self, urdf_path: str, topic: str = 'robot_description', node_name: str = 'publish_urdf'):
        super().__init__(node_name)
        content = Path(urdf_path).read_text()
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(String, topic, qos)
        self.msg = String()
        self.msg.data = content
        self._publish()

    def _publish(self):
        self.pub.publish(self.msg)
        self.get_logger().info(f'Published {self.pub.topic_name}')

    def start(self):
        self._timer = self.create_timer(1.0, self._publish)


def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print('Usage: publish_urdf.py /path/to/robot.urdf [topic] [node_name]')
        return
    urdf_path = sys.argv[1]
    robot_description_topic = sys.argv[2] if len(sys.argv) > 2 else 'robot_description'
    if len(sys.argv) > 3:
        node_name = sys.argv[3]
    else:
        sanitized_topic = robot_description_topic.strip('/').replace('/', '_')
        node_name = f'publish_urdf_{sanitized_topic}'
    node = URDFPublisher(urdf_path, robot_description_topic, node_name)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
