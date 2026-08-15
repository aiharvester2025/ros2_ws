#!/usr/bin/env python3
"""Select one of the two Gazebo RGB camera streams for the single RViz view.

The relay is deliberately passive: it preserves Image/CameraInfo headers and
payloads, publishes no TF, and never changes Gazebo, the robot state, or the
raw camera topics.  The cutter camera is the default so existing perception
and camera--LiDAR projection paths continue to use their established streams.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class CameraViewSelector(Node):
    """Header-preserving two-camera selector for one RViz Image display."""

    STREAMS = {
        'cutter': {
            'image': '/harvester/platform_camera/depth/image_raw',
            'camera_info': '/harvester/platform_camera/depth/camera_info',
        },
        'docking': {
            'image': '/harvester/docking_camera/depth/image_raw',
            'camera_info': '/harvester/docking_camera/depth/camera_info',
        },
    }
    SELECTION_TOPIC = '/harvester/camera_view/select'
    IMAGE_OUTPUT_TOPIC = '/harvester/camera_view/image_raw'
    CAMERA_INFO_OUTPUT_TOPIC = '/harvester/camera_view/camera_info'
    STATUS_TOPIC = '/harvester/camera_view/status'

    def __init__(self):
        super().__init__('camera_view_selector')
        self.selected = 'cutter'
        self.latest_images = {name: None for name in self.STREAMS}
        self.latest_infos = {name: None for name in self.STREAMS}
        self.ready_streams = set()

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        selection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.image_publisher = self.create_publisher(
            Image, self.IMAGE_OUTPUT_TOPIC, output_qos)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, self.CAMERA_INFO_OUTPUT_TOPIC, output_qos)
        self.status_publisher = self.create_publisher(String, self.STATUS_TOPIC, selection_qos)
        self.selection_subscription = self.create_subscription(
            String, self.SELECTION_TOPIC, self.on_selection, selection_qos)

        self.image_subscriptions = []
        self.info_subscriptions = []
        for name, topics in self.STREAMS.items():
            self.image_subscriptions.append(self.create_subscription(
                Image, topics['image'],
                lambda message, source=name: self.on_image(source, message),
                qos_profile_sensor_data))
            self.info_subscriptions.append(self.create_subscription(
                CameraInfo, topics['camera_info'],
                lambda message, source=name: self.on_camera_info(source, message),
                qos_profile_sensor_data))

        self.publish_status('waiting for cutter camera data')
        self.get_logger().info(
            'Camera selector ready: default=cutter; publish cutter or docking on '
            f'{self.SELECTION_TOPIC}')

    def on_selection(self, message: String):
        requested = message.data.strip().lower()
        if requested not in self.STREAMS:
            self.get_logger().warning(
                f'Ignoring unsupported camera selection {message.data!r}; '
                'expected cutter or docking')
            self.publish_status(f'invalid selection: {message.data!r}')
            return

        changed = requested != self.selected
        self.selected = requested
        self.forward_latest_selected()
        ready = requested in self.ready_streams
        self.publish_status(
            f'selected={requested}; ' +
            ('forwarding available stream' if ready else 'waiting for camera data'))
        if changed:
            self.get_logger().info(f'Active RViz camera changed to {requested}')

    def on_image(self, source: str, message: Image):
        self.latest_images[source] = message
        if source not in self.ready_streams:
            self.ready_streams.add(source)
            self.get_logger().info(f'{source} camera image stream is available')
        if source == self.selected:
            self.image_publisher.publish(message)

    def on_camera_info(self, source: str, message: CameraInfo):
        self.latest_infos[source] = message
        if source == self.selected:
            self.camera_info_publisher.publish(message)

    def forward_latest_selected(self):
        """Switch promptly, but never send an image/info from the old camera."""
        latest_image = self.latest_images[self.selected]
        if latest_image is not None:
            self.image_publisher.publish(latest_image)
        latest_info = self.latest_infos[self.selected]
        if latest_info is not None:
            self.camera_info_publisher.publish(latest_info)

    def publish_status(self, detail: str):
        status = String()
        status.data = detail
        self.status_publisher.publish(status)


def main():
    rclpy.init(args=sys.argv)
    node = CameraViewSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
