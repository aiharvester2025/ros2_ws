#!/usr/bin/env python3
"""Read existing Gazebo ROS 2 telemetry and publish canonical ZeroMQ v1.

This node is deliberately read-only: it creates no ROS publishers, TF
broadcasters, command subscribers, or Gazebo services.  It can therefore run
beside the known-good combined Gazebo/RViz launch without changing its control
or sensor behaviour.
"""

import json
import time
from collections import defaultdict, deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, Range
from std_msgs.msg import String
import zmq

from harvester_telemetry_contract import ProtocolError, pack_message

from .encoders import (
    camera_info_json,
    depth_to_uint16_mm,
    header_frame_id,
    image_to_jpeg,
    pointcloud_to_xyz_f32,
    stamp_to_ns,
)
from .recording import PacketRecorder


DOCKING_RANGES = (
    ('center_line', '/harvester/center_range'),
    ('left_45_deg', '/harvester/left_45_range'),
    ('right_45_deg', '/harvester/right_45_range'),
    ('left_side', '/harvester/left_side_range'),
    ('right_side', '/harvester/right_side_range'),
)


class TelemetryGateway(Node):
    def __init__(self):
        super().__init__('harvester_telemetry_gateway')
        self.declare_parameter('pub_endpoint', 'tcp://*:5590')
        self.declare_parameter('status_endpoint', 'tcp://*:5600')
        self.declare_parameter('source_id', 'xavier')
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('queue_depth', 2)
        self.declare_parameter('socket_hwm', 8)
        self.declare_parameter('lidar_stride', 2)
        self.declare_parameter('lidar_roi.min_x', float('nan'))
        self.declare_parameter('lidar_roi.max_x', float('nan'))
        self.declare_parameter('lidar_roi.min_y', float('nan'))
        self.declare_parameter('lidar_roi.max_y', float('nan'))
        self.declare_parameter('lidar_roi.min_z', float('nan'))
        self.declare_parameter('lidar_roi.max_z', float('nan'))
        self.declare_parameter('cutter_calibration_id', 'gazebo_nominal_camera_lidar_v1')
        self.declare_parameter('docking_calibration_id', 'gazebo_nominal_docking_camera_v1')
        self.declare_parameter('range_calibration_id', 'gazebo_nominal_c_channel_v1')
        # Empty disables disk writes.  Recording is opt-in because a full
        # camera/LiDAR audit can consume storage rapidly.
        self.declare_parameter('record_dir', '')

        self.source_id = self.get_parameter('source_id').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.queue_depth = max(1, int(self.get_parameter('queue_depth').value))
        self.lidar_stride = max(1, int(self.get_parameter('lidar_stride').value))
        self.lidar_roi = self._roi_from_parameters()
        self.sequence = defaultdict(int)
        self.queues = defaultdict(lambda: deque(maxlen=self.queue_depth))
        self.last_stream_status = {}
        self.drop_counts = defaultdict(int)
        self.docking_ranges = {}
        self.started_monotonic_ns = time.monotonic_ns()
        self.recorder = PacketRecorder(str(self.get_parameter('record_dir').value))
        # These are source capabilities, not live-stream health.  The latter
        # remains in v1/system/status so a missing camera is never masked.
        self.capabilities = {
            'camera.cutter.rgb': True,
            'camera.cutter.depth': True,
            'camera.cutter.camera_info': True,
            'camera.docking.rgb': True,
            'camera.docking.depth': True,
            'camera.docking.camera_info': True,
            'lidar.raw_xyz': True,
            'lidar.intensity': False,
            'lidar.point_time': False,
            'range.docking': True,
            'range.cutter': True,
            'docking.trunk_estimate': True,
            'calibration.status': True,
            'packet.recording': True,
            'packet.replay': True,
            'target.world_fixed': False,
        }

        context = zmq.Context.instance()
        self.pub_socket = context.socket(zmq.PUB)
        self.pub_socket.setsockopt(zmq.LINGER, 0)
        self.pub_socket.setsockopt(zmq.SNDHWM, int(self.get_parameter('socket_hwm').value))
        self.pub_socket.bind(self.get_parameter('pub_endpoint').value)
        self.status_socket = context.socket(zmq.REP)
        self.status_socket.setsockopt(zmq.LINGER, 0)
        self.status_socket.bind(self.get_parameter('status_endpoint').value)

        self.create_subscription(
            Image, '/harvester/platform_camera/depth/image_raw', self.on_cutter_rgb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/harvester/platform_camera/depth/depth/image_raw', self.on_cutter_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/harvester/platform_camera/depth/camera_info', self.on_cutter_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/harvester/docking_camera/depth/image_raw', self.on_docking_rgb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/harvester/docking_camera/depth/depth/image_raw', self.on_docking_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/harvester/docking_camera/depth/camera_info', self.on_docking_info,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/harvester/lidar/raw_points', self.on_lidar,
            qos_profile_sensor_data)
        for key, topic in DOCKING_RANGES:
            self.create_subscription(
                Range, topic, lambda message, range_key=key: self.on_docking_range(range_key, message),
                qos_profile_sensor_data)
        self.create_subscription(
            Range, '/harvester/cutting_tool_left_range', self.on_cutter_range,
            qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/harvester/docking/trunk_center', self.on_trunk_estimate,
            QoSProfile(depth=10))
        self.create_subscription(
            String, '/harvester/docking/calibration_status', self.on_calibration_status,
            QoSProfile(depth=10))

        self.create_timer(0.002, self.flush_one_packet)
        self.create_timer(0.05, self.handle_status_request)
        self.create_timer(1.0, self.publish_system_status)
        self.get_logger().info(
            'Read-only canonical ZeroMQ gateway ready: PUB {} ; status REP {}'.format(
                self.get_parameter('pub_endpoint').value,
                self.get_parameter('status_endpoint').value))
        if self.recorder.enabled:
            self.get_logger().info('Canonical packet recording enabled: {}'.format(
                self.recorder.status()['directory']))

    def _roi_from_parameters(self):
        roi = {}
        for axis in ('x', 'y', 'z'):
            for bound in ('min', 'max'):
                value = float(self.get_parameter('lidar_roi.{}_{}'.format(bound, axis)).value)
                if value == value:  # NaN means disabled.
                    roi['{}_{}'.format(bound, axis)] = value
        return roi

    def _header(self, message_header, calibration_id, codec):
        return {
            'schema_version': 1,
            'source_mode': 'simulation',
            'source_id': self.source_id,
            'sequence': 0,
            'frame_id': header_frame_id(message_header),
            'acquisition_timestamp_ns': stamp_to_ns(message_header.stamp),
            'clock_domain': 'ros_sim_time',
            'gateway_monotonic_ns': time.monotonic_ns(),
            'calibration_id': calibration_id,
            'codec': codec,
            'capabilities': dict(self.capabilities),
        }

    def _enqueue(self, channel, header, payload):
        self.sequence[channel] += 1
        header['sequence'] = self.sequence[channel]
        header['source_id'] = self.source_id
        header['gateway_monotonic_ns'] = time.monotonic_ns()
        try:
            frames = pack_message(channel, header, payload)
        except ProtocolError as error:
            self.get_logger().error('Rejected {} packet: {}'.format(channel, error))
            self.last_stream_status[channel] = {'enabled': False, 'error': str(error)}
            return
        if self.recorder.enabled:
            try:
                self.recorder.write(frames)
            except OSError as error:
                # Audit storage errors must never stop the live telemetry path.
                self.get_logger().error('Recording {} failed: {}'.format(channel, error))
        queue = self.queues[channel]
        if len(queue) == queue.maxlen:
            self.drop_counts[channel] += 1
        queue.append(frames)
        self.last_stream_status[channel] = {
            'enabled': True,
            'last_sequence': header['sequence'],
            'last_acquisition_timestamp_ns': header['acquisition_timestamp_ns'],
            'frame_id': header['frame_id'],
        }

    def _image_header(self, message, calibration_id, codec, pixel_encoding=None):
        header = self._header(message.header, calibration_id, codec)
        header.update({'width': int(message.width), 'height': int(message.height)})
        if pixel_encoding is not None:
            header['pixel_encoding'] = pixel_encoding
        return header

    def on_cutter_rgb(self, message):
        try:
            self._enqueue(
                'v1/camera/cutter/rgb',
                self._image_header(message, self.get_parameter('cutter_calibration_id').value, 'jpeg', 'RGB8'),
                image_to_jpeg(message, self.jpeg_quality))
        except ValueError as error:
            self.get_logger().warning('Cutter RGB skipped: {}'.format(error))

    def on_docking_rgb(self, message):
        try:
            self._enqueue(
                'v1/camera/docking/rgb',
                self._image_header(message, self.get_parameter('docking_calibration_id').value, 'jpeg', 'RGB8'),
                image_to_jpeg(message, self.jpeg_quality))
        except ValueError as error:
            self.get_logger().warning('Docking RGB skipped: {}'.format(error))

    def _on_depth(self, channel, calibration_id, message):
        try:
            self._enqueue(
                channel,
                self._image_header(message, calibration_id, 'depth_uint16_le'),
                depth_to_uint16_mm(message))
        except ValueError as error:
            self.get_logger().warning('{} skipped: {}'.format(channel, error))

    def on_cutter_depth(self, message):
        self._on_depth('v1/camera/cutter/depth', self.get_parameter('cutter_calibration_id').value, message)

    def on_docking_depth(self, message):
        self._on_depth('v1/camera/docking/depth', self.get_parameter('docking_calibration_id').value, message)

    def _on_info(self, channel, calibration_id, message):
        header = self._header(message.header, calibration_id, 'json')
        header.update({'width': int(message.width), 'height': int(message.height)})
        self._enqueue(channel, header, camera_info_json(message))

    def on_cutter_info(self, message):
        self._on_info('v1/camera/cutter/camera_info', self.get_parameter('cutter_calibration_id').value, message)

    def on_docking_info(self, message):
        self._on_info('v1/camera/docking/camera_info', self.get_parameter('docking_calibration_id').value, message)

    def on_lidar(self, message):
        try:
            payload, point_count = pointcloud_to_xyz_f32(
                message, stride=self.lidar_stride, roi=self.lidar_roi)
            header = self._header(
                message.header, self.get_parameter('cutter_calibration_id').value, 'lidar_xyz_f32')
            header.update({
                'point_count': point_count,
                'point_stride_bytes': 12,
                'point_fields': [
                    {'name': 'x', 'type': 'float32'},
                    {'name': 'y', 'type': 'float32'},
                    {'name': 'z', 'type': 'float32'},
                ],
            })
            self._enqueue('v1/lidar/raw', header, payload)
        except ValueError as error:
            self.get_logger().warning('LiDAR skipped: {}'.format(error))

    def _range_record(self, key, message):
        maximum = float(message.max_range)
        valid = (
            message.min_range <= message.range <= maximum and
            message.range == message.range and
            message.range != float('inf'))
        return {
            'telemetry_key': key,
            'distance_m': float(message.range) if valid else None,
            'valid': bool(valid),
            'frame_id': header_frame_id(message.header),
            'acquisition_timestamp_ns': stamp_to_ns(message.header.stamp),
            'calibration_id': self.get_parameter('range_calibration_id').value,
            'min_range_m': float(message.min_range),
            'max_range_m': maximum,
        }

    def on_docking_range(self, key, message):
        self.docking_ranges[key] = self._range_record(key, message)
        records = [self.docking_ranges[range_key] for range_key, _ in DOCKING_RANGES if range_key in self.docking_ranges]
        header = self._header(message.header, self.get_parameter('range_calibration_id').value, 'json')
        self._enqueue('v1/range/docking', header, json.dumps(records, separators=(',', ':')).encode('utf-8'))

    def on_cutter_range(self, message):
        record = self._range_record('cutter_forward', message)
        header = self._header(message.header, self.get_parameter('range_calibration_id').value, 'json')
        self._enqueue('v1/range/cutter', header, json.dumps(record, separators=(',', ':')).encode('utf-8'))

    def on_trunk_estimate(self, message):
        header = self._header(message.header, self.get_parameter('range_calibration_id').value, 'json')
        payload = {
            'pose': {
                'position': {
                    'x': message.pose.pose.position.x,
                    'y': message.pose.pose.position.y,
                    'z': message.pose.pose.position.z,
                },
                'orientation': {
                    'x': message.pose.pose.orientation.x,
                    'y': message.pose.pose.orientation.y,
                    'z': message.pose.pose.orientation.z,
                    'w': message.pose.pose.orientation.w,
                },
            },
            'covariance': list(message.pose.covariance),
        }
        self._enqueue('v1/docking/trunk_estimate', header, json.dumps(payload, separators=(',', ':')).encode('utf-8'))

    def on_calibration_status(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {'status': 'UNPARSEABLE_SOURCE_STATUS', 'raw': message.data}
        header = {
            'schema_version': 1,
            'source_mode': 'simulation',
            'source_id': self.source_id,
            'sequence': 0,
            'frame_id': 'c_channel_reference',
            # The source status has no ROS header.  Do not label gateway wall
            # time as Gazebo simulation time; consumers use it only for the
            # freshness of this status message.
            'acquisition_timestamp_ns': time.time_ns(),
            'clock_domain': 'utc_host',
            'gateway_monotonic_ns': time.monotonic_ns(),
            'calibration_id': str(payload.get('calibration_id', self.get_parameter('range_calibration_id').value)),
            'codec': 'json',
            'capabilities': dict(self.capabilities),
            'transform_valid': payload.get('status') in (None, 'VALID', 'READY'),
            'transform_freshness_s': None,
        }
        self._enqueue('v1/calibration/status', header, json.dumps(payload, separators=(',', ':')).encode('utf-8'))

    def flush_one_packet(self):
        for channel in sorted(self.queues):
            queue = self.queues[channel]
            if not queue:
                continue
            frames = queue.pop()  # newest wins; discard stale complete packets.
            dropped = len(queue)
            if dropped:
                self.drop_counts[channel] += dropped
                queue.clear()
            try:
                self.pub_socket.send_multipart(frames, flags=zmq.NOBLOCK)
            except zmq.Again:
                self.drop_counts[channel] += 1
            return

    def publish_system_status(self):
        header = {
            'schema_version': 1,
            'source_mode': 'simulation',
            'source_id': self.source_id,
            'sequence': 0,
            'frame_id': '',
            'acquisition_timestamp_ns': time.time_ns(),
            'clock_domain': 'utc_host',
            'gateway_monotonic_ns': time.monotonic_ns(),
            'calibration_id': 'none',
            'codec': 'json',
            'capabilities': dict(self.capabilities),
        }
        payload = {
            'source_id': self.source_id,
            'source_mode': 'simulation',
            'uptime_s': (time.monotonic_ns() - self.started_monotonic_ns) / 1e9,
            'streams': self.last_stream_status,
            'dropped_packets': dict(self.drop_counts),
            'errors': [],
            'recording': self.recorder.status(),
            'capabilities': self.capabilities,
        }
        self._enqueue('v1/system/status', header, json.dumps(payload, separators=(',', ':')).encode('utf-8'))

    def handle_status_request(self):
        try:
            # Drain every request frame before replying.  A REQ client may
            # include a JSON request body even though this endpoint is
            # deliberately read-only.
            self.status_socket.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        response = {
            'schema_version': 1,
            'active_profile': 'simulation',
            'calibration_revision': {
                'cutter': self.get_parameter('cutter_calibration_id').value,
                'docking': self.get_parameter('docking_calibration_id').value,
                'ranges': self.get_parameter('range_calibration_id').value,
            },
            'streams': self.last_stream_status,
            'dropped_packets': dict(self.drop_counts),
            'recording': self.recorder.status(),
            'capabilities': self.capabilities,
            'latest_status': 'OK',
        }
        self.status_socket.send(json.dumps(response, separators=(',', ':')).encode('utf-8'))

    def destroy_node(self):
        self.pub_socket.close(0)
        self.status_socket.close(0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
