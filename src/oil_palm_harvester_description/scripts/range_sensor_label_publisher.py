#!/usr/bin/env python3
"""Publish live five-range-sensor values as RViz text markers.

Markers are attached to the sensor frames, so their labels remain next to the
corresponding physical sensor while the platform and boom move.  A zero stamp
asks TF2/RViz to use the latest transform, avoiding simulation-time versus GUI
time mismatches in this kinematic scene.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray


SENSORS = (
    ('center', 'Centre', '/harvester/center_range', 'center_range_sensor_link'),
    ('left_45', 'Left 45 deg', '/harvester/left_45_range', 'left_45_range_sensor_link'),
    ('right_45', 'Right 45 deg', '/harvester/right_45_range', 'right_45_range_sensor_link'),
    ('left_side', 'Left side', '/harvester/left_side_range', 'left_side_range_sensor_link'),
    ('right_side', 'Right side', '/harvester/right_side_range', 'right_side_range_sensor_link'),
)


class RangeSensorLabelPublisher(Node):
    def __init__(self):
        super().__init__('range_sensor_label_publisher')
        self.values = {}
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/harvester/range_sensor_labels', 10)
        # ``Node.subscriptions`` is a read-only rclpy property in Foxy; retain
        # these handles under our own name so the subscriptions stay alive.
        self.range_subscriptions = [
            self.create_subscription(
                Range, topic,
                lambda message, key=key: self.on_range(key, message),
                qos_profile_sensor_data)
            for key, _label, topic, _frame in SENSORS
        ]
        self.timer = self.create_timer(0.1, self.publish_labels)
        self.get_logger().info(
            'Publishing RViz range labels on /harvester/range_sensor_labels')

    def on_range(self, key, message):
        self.values[key] = (message.range, message.max_range)

    def format_value(self, key):
        value = self.values.get(key)
        if value is None:
            return 'waiting'
        measured_range, maximum = value
        if not math.isfinite(measured_range) or measured_range >= maximum - 0.002:
            return 'out of range'
        return f'{measured_range:.2f} m'

    def publish_labels(self):
        markers = MarkerArray()
        for marker_id, (key, label, _topic, frame) in enumerate(SENSORS):
            marker = Marker()
            marker.header.frame_id = frame
            # Leave stamp at Time(0): use the newest transform in RViz.
            marker.ns = 'range_sensor_values'
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            # Keep labels clear of the physical sensor meshes.  Frame locking
            # makes RViz redraw them at the current sensor pose while the
            # platform/boom moves, without requiring matching wall and
            # simulation timestamps.
            marker.frame_locked = True
            marker.pose.position.x = 0.18
            marker.pose.position.z = 0.18
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.16
            marker.color.r = 1.0 if key == 'center' else 0.2
            marker.color.g = 0.75
            marker.color.b = 0.15 if key == 'center' else 1.0
            marker.color.a = 1.0
            marker.text = f'{label}: {self.format_value(key)}'
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)


def main():
    rclpy.init()
    node = RangeSensorLabelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
