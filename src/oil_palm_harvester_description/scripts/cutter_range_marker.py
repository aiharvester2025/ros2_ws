#!/usr/bin/env python3
"""Publish an RViz ray for the cutter-attached left range sensor.

This stays separate from ``range_sensor_calibration.py``: that calibration
uses five sensors that are rigid relative to ``c_channel_reference``, whereas
this sensor moves with the cutter rail, lift and extension.  The marker is
frame-locked in the sensor frame, so RViz uses the latest cutter TF and no
simulation-time/wall-time TF lookup is required.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray


class CutterRangeMarker(Node):
    """Render only a fresh, valid cutter-side measurement as a green ray."""

    def __init__(self):
        super().__init__('cutter_range_marker')
        self.frame_id = 'cutting_tool_left_range_sensor_link'
        self.last_reading = None
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/harvester/cutter/range_markers', 10)
        self.subscription = self.create_subscription(
            Range, '/harvester/cutting_tool_left_range', self.on_range,
            qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.publish_marker)
        self.get_logger().info(
            'Publishing cutter range ray on /harvester/cutter/range_markers')

    def on_range(self, message):
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            self.get_logger().warning(
                'Ignoring cutter range with unexpected frame %r (expected %r)' %
                (message.header.frame_id, self.frame_id))
            return
        self.last_reading = (
            float(message.range), float(message.min_range),
            float(message.max_range), time.monotonic())

    def valid_reading(self):
        if self.last_reading is None:
            return None
        measured_range, minimum, maximum, receipt_time = self.last_reading
        if time.monotonic() - receipt_time > 0.30:
            return None
        if not all(math.isfinite(value) for value in (measured_range, minimum, maximum)):
            return None
        if measured_range < minimum or measured_range >= maximum - 0.002:
            return None
        return measured_range

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        # Leave the stamp at Time(0): RViz resolves the moving cutter frame at
        # its latest transform, which is reliable in the mixed Foxy clock path.
        marker.frame_locked = True
        marker.ns = 'cutter_range_ray'
        marker.id = 0

        measured_range = self.valid_reading()
        if measured_range is None:
            marker.action = Marker.DELETE
        else:
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.018
            marker.color.r = 1.00
            marker.color.g = 0.85
            marker.color.b = 0.05
            marker.color.a = 0.95
            marker.points = [Point(x=0.0, y=0.0, z=0.0),
                             Point(x=measured_range, y=0.0, z=0.0)]

        markers = MarkerArray()
        markers.markers.append(marker)
        self.marker_publisher.publish(markers)


def main():
    rclpy.init()
    node = CutterRangeMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
