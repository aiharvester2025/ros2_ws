#!/usr/bin/env python3
"""Project the raw arm-mounted LiDAR cloud onto the RGB camera image.

This optional, simulation-only node is deliberately additive.  It reads the
fixed camera-to-LiDAR relationship from the active URDF and uses only the raw
Gazebo acquisition-time topics.  It does not publish TF, modify the URDF,
republish joint states, or consume the zero-stamped RViz-only LiDAR stream.
"""

import json
import math
import struct
import sys
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String

from camera_lidar_calibration_common import (
    derive_camera_optical_T_calibrated_lidar,
    matrix_to_quaternion,
    matrix_translation,
    require_simulation_camera_lidar_config,
)
from range_sensor_calibration_common import load_json, parse_fixed_joint_graph


POINT_FIELD_FORMATS = {
    PointField.INT8: 'b',
    PointField.UINT8: 'B',
    PointField.INT16: 'h',
    PointField.UINT16: 'H',
    PointField.INT32: 'i',
    PointField.UINT32: 'I',
    PointField.FLOAT32: 'f',
    PointField.FLOAT64: 'd',
}

IMAGE_ENCODINGS = {
    'rgb8': (3, (0, 1, 2)),
    'bgr8': (3, (2, 1, 0)),
    'rgba8': (4, (0, 1, 2)),
    'bgra8': (4, (2, 1, 0)),
}


def default_share_directory():
    return Path(__file__).resolve().parent.parent


def normalized_frame(frame):
    return (frame or '').lstrip('/')


def stamp_seconds(stamp):
    if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def copy_header(destination, source, frame_id=None, zero_stamp=False):
    destination.frame_id = frame_id if frame_id is not None else source.frame_id
    if zero_stamp:
        destination.stamp.sec = 0
        destination.stamp.nanosec = 0
    else:
        destination.stamp.sec = source.stamp.sec
        destination.stamp.nanosec = source.stamp.nanosec


def transform_xyz(matrix, x, y, z):
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
    )


class CameraLidarProjection(Node):
    """Timestamp-gated camera/LiDAR overlay without dynamic TF lookup."""

    def __init__(self, config_path, urdf_path):
        super().__init__('camera_lidar_projection')
        self.config_path = Path(config_path)
        self.urdf_path = Path(urdf_path)
        self.config = require_simulation_camera_lidar_config(load_json(self.config_path))
        self.camera = self.config['camera']
        self.lidar = self.config['lidar']
        self.time_policy = self.config['time_policy']
        self.runtime = self.config['runtime']
        self.outputs = self.config['outputs']

        _, graph = parse_fixed_joint_graph(self.urdf_path)
        self.camera_T_lidar = derive_camera_optical_T_calibrated_lidar(self.config, graph)
        self.camera_optical_frame = self.camera['optical_frame']
        self.lidar_frame = self.lidar['frame']

        self.camera_info = None
        self.images = deque(maxlen=6)
        self.clouds = deque(maxlen=6)
        self.last_processed_pair = None
        self.last_projection_monotonic_s = 0.0
        self.last_status_signature = None
        self.last_status_publish_monotonic_s = 0.0
        self.last_projection_summary = None

        self.overlay_publisher = self.create_publisher(
            Image, self.outputs['overlay_image_topic'], 5)
        self.raw_points_publisher = self.create_publisher(
            PointCloud2, self.outputs['visible_points_raw_topic'], 5)
        self.rviz_points_publisher = self.create_publisher(
            PointCloud2, self.outputs['visible_points_rviz_topic'], 5)
        self.status_publisher = self.create_publisher(
            String, self.outputs['status_topic'], 5)

        # Request sensor-data QoS: it connects to both Gazebo Best Effort and
        # Reliable sensor publishers, while avoiding a dependency on cv_bridge,
        # PCL, OpenCV, or a second RViz process.
        self.image_subscription = self.create_subscription(
            Image, self.camera['color_image_topic'], self.on_image,
            qos_profile_sensor_data)
        self.info_subscription = self.create_subscription(
            CameraInfo, self.camera['color_camera_info_topic'], self.on_camera_info,
            qos_profile_sensor_data)
        self.cloud_subscription = self.create_subscription(
            PointCloud2, self.lidar['raw_points_topic'], self.on_cloud,
            qos_profile_sensor_data)

        translation = matrix_translation(self.camera_T_lidar)
        quaternion = matrix_to_quaternion(self.camera_T_lidar)
        self.get_logger().info(
            f"Loaded '{self.config['calibration_id']}'. Derived "
            f"T_{self.camera_optical_frame}_{self.lidar_frame}: "
            f"xyz=({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) m, "
            f"quaternion=({quaternion[0]:.3f}, {quaternion[1]:.3f}, "
            f"{quaternion[2]:.3f}, {quaternion[3]:.3f}).")
        self.get_logger().info(
            'Using raw camera and raw LiDAR acquisition timestamps only; the '
            'zero-stamped /harvester/lidar/points RViz stream is never used for fusion.')
        self.publish_status('WAITING_FOR_CAMERA_INFO')

    def on_camera_info(self, message):
        self.camera_info = message
        self.try_project()

    def on_image(self, message):
        self.images.append(message)
        self.try_project()

    def on_cloud(self, message):
        self.clouds.append(message)
        self.try_project()

    def select_pair(self):
        """Return the closest non-zero-stamped image/cloud pair, if any."""
        if not self.images or not self.clouds:
            return None, 'WAITING_FOR_RAW_MESSAGES', None
        candidate = None
        for image in self.images:
            image_stamp = stamp_seconds(image.header.stamp)
            if image_stamp is None:
                continue
            for cloud in self.clouds:
                cloud_stamp = stamp_seconds(cloud.header.stamp)
                if cloud_stamp is None:
                    continue
                skew = abs(image_stamp - cloud_stamp)
                if candidate is None or skew < candidate[0]:
                    candidate = (skew, image, cloud)
        if candidate is None:
            return None, 'REJECTED_ZERO_SOURCE_TIMESTAMP', None
        maximum_skew = float(self.time_policy['maximum_image_lidar_skew_s'])
        if candidate[0] > maximum_skew:
            return None, 'REJECTED_TIMESTAMP_SKEW', candidate[0]
        return (candidate[1], candidate[2]), 'READY', candidate[0]

    def try_project(self):
        pair, state, skew = self.select_pair()
        if pair is None:
            self.publish_status(state, pair_skew_s=skew)
            return
        if self.camera_info is None:
            self.publish_status('WAITING_FOR_CAMERA_INFO', pair_skew_s=skew)
            return
        image, cloud = pair
        pair_key = (
            image.header.stamp.sec, image.header.stamp.nanosec,
            cloud.header.stamp.sec, cloud.header.stamp.nanosec,
        )
        if pair_key == self.last_processed_pair:
            return
        now = time.monotonic()
        minimum_period = 1.0 / float(self.runtime['projection_rate_hz'])
        if now - self.last_projection_monotonic_s < minimum_period:
            return

        try:
            self.validate_runtime_frames(image, cloud, self.camera_info)
            overlay, visible_points, total_points = self.project(image, cloud, self.camera_info)
        except ValueError as error:
            self.publish_status('REJECTED_' + str(error), pair_skew_s=skew)
            return

        self.overlay_publisher.publish(overlay)
        raw_points = self.camera_frame_cloud(visible_points, image.header, zero_stamp=False)
        rviz_points = self.camera_frame_cloud(visible_points, image.header, zero_stamp=True)
        self.raw_points_publisher.publish(raw_points)
        self.rviz_points_publisher.publish(rviz_points)
        self.last_processed_pair = pair_key
        self.last_projection_monotonic_s = now
        self.last_projection_summary = {
            'total_points_in_cloud': total_points,
            'visible_projected_points': len(visible_points),
            'pair_skew_s': skew,
            'camera_stamp_s': stamp_seconds(image.header.stamp),
            'lidar_stamp_s': stamp_seconds(cloud.header.stamp),
        }
        self.publish_status('VALID', force=True)

    def validate_runtime_frames(self, image, cloud, camera_info):
        if normalized_frame(image.header.frame_id) != self.camera_optical_frame:
            raise ValueError('CAMERA_IMAGE_FRAME_MISMATCH')
        if normalized_frame(camera_info.header.frame_id) != self.camera_optical_frame:
            raise ValueError('CAMERA_INFO_FRAME_MISMATCH')
        if normalized_frame(cloud.header.frame_id) != self.lidar_frame:
            raise ValueError('LIDAR_FRAME_MISMATCH')
        if int(image.width) <= 0 or int(image.height) <= 0:
            raise ValueError('EMPTY_CAMERA_IMAGE')
        if int(camera_info.width) != int(image.width) or int(camera_info.height) != int(image.height):
            raise ValueError('CAMERA_INFO_DIMENSIONS_MISMATCH')

    @staticmethod
    def camera_intrinsics(camera_info):
        # P is the rectified projection matrix and is preferred where present.
        projection = list(camera_info.p)
        if len(projection) >= 7 and projection[0] > 0.0 and projection[5] > 0.0:
            return projection[0], projection[5], projection[2], projection[6]
        intrinsic = list(camera_info.k)
        if len(intrinsic) >= 6 and intrinsic[0] > 0.0 and intrinsic[4] > 0.0:
            return intrinsic[0], intrinsic[4], intrinsic[2], intrinsic[5]
        raise ValueError('INVALID_CAMERA_INTRINSICS')

    @staticmethod
    def xyz_readers(cloud):
        fields = {field.name: field for field in cloud.fields}
        readers = []
        prefix = '>' if cloud.is_bigendian else '<'
        for name in ('x', 'y', 'z'):
            field = fields.get(name)
            if field is None or field.datatype not in POINT_FIELD_FORMATS or field.count < 1:
                raise ValueError('POINTCLOUD_MISSING_XYZ')
            readers.append((field.offset, prefix + POINT_FIELD_FORMATS[field.datatype]))
        return readers

    def project(self, image, cloud, camera_info):
        encoding = image.encoding.lower()
        if encoding not in IMAGE_ENCODINGS:
            raise ValueError('UNSUPPORTED_IMAGE_ENCODING_' + image.encoding)
        channels, color_indices = IMAGE_ENCODINGS[encoding]
        if int(image.step) < int(image.width) * channels:
            raise ValueError('INVALID_IMAGE_STEP')
        data = bytearray(image.data)
        if len(data) < int(image.step) * int(image.height):
            raise ValueError('TRUNCATED_IMAGE_DATA')
        fx, fy, cx, cy = self.camera_intrinsics(camera_info)
        readers = self.xyz_readers(cloud)
        if int(cloud.point_step) <= 0 or int(cloud.row_step) < int(cloud.point_step) * int(cloud.width):
            raise ValueError('INVALID_POINTCLOUD_LAYOUT')
        if len(cloud.data) < int(cloud.row_step) * int(cloud.height):
            raise ValueError('TRUNCATED_POINTCLOUD_DATA')

        total = int(cloud.width) * int(cloud.height)
        stride = max(1, int(math.ceil(total / float(self.runtime['maximum_points_per_cloud']))))
        minimum_depth, maximum_depth = [float(value) for value in (
            self.camera['nominal_image_geometry']['clip_range_m'])]
        nearest_by_pixel = {}
        counter = 0
        data_view = cloud.data
        for row in range(int(cloud.height)):
            row_offset = row * int(cloud.row_step)
            for column in range(int(cloud.width)):
                if counter % stride:
                    counter += 1
                    continue
                counter += 1
                point_offset = row_offset + column * int(cloud.point_step)
                try:
                    x = struct.unpack_from(readers[0][1], data_view, point_offset + readers[0][0])[0]
                    y = struct.unpack_from(readers[1][1], data_view, point_offset + readers[1][0])[0]
                    z = struct.unpack_from(readers[2][1], data_view, point_offset + readers[2][0])[0]
                except struct.error:
                    raise ValueError('TRUNCATED_POINTCLOUD_DATA')
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                camera_x, camera_y, camera_z = transform_xyz(self.camera_T_lidar, x, y, z)
                if not (minimum_depth <= camera_z <= maximum_depth):
                    continue
                pixel_x = int(round(fx * camera_x / camera_z + cx))
                pixel_y = int(round(fy * camera_y / camera_z + cy))
                if not (0 <= pixel_x < int(image.width) and 0 <= pixel_y < int(image.height)):
                    continue
                pixel_key = pixel_y * int(image.width) + pixel_x
                previous = nearest_by_pixel.get(pixel_key)
                if previous is None or camera_z < previous[0]:
                    nearest_by_pixel[pixel_key] = (camera_z, pixel_x, pixel_y,
                                                   camera_x, camera_y)

        visible_points = []
        for camera_z, pixel_x, pixel_y, camera_x, camera_y in nearest_by_pixel.values():
            self.draw_point(
                data, image, pixel_x, pixel_y,
                self.depth_color(camera_z, minimum_depth, maximum_depth),
                channels, color_indices)
            visible_points.append((camera_x, camera_y, camera_z))
        overlay = Image()
        copy_header(overlay.header, image.header, frame_id=self.camera_optical_frame)
        overlay.height = image.height
        overlay.width = image.width
        overlay.encoding = image.encoding
        overlay.is_bigendian = image.is_bigendian
        overlay.step = image.step
        overlay.data = bytes(data)
        return overlay, visible_points, total

    def draw_point(self, data, image, pixel_x, pixel_y, color, channels, color_indices):
        radius = int(self.runtime['point_radius_px'])
        red_index, green_index, blue_index = color_indices
        for y in range(max(0, pixel_y - radius), min(int(image.height), pixel_y + radius + 1)):
            for x in range(max(0, pixel_x - radius), min(int(image.width), pixel_x + radius + 1)):
                if (x - pixel_x) ** 2 + (y - pixel_y) ** 2 > radius ** 2:
                    continue
                offset = y * int(image.step) + x * channels
                data[offset + red_index] = color[0]
                data[offset + green_index] = color[1]
                data[offset + blue_index] = color[2]
                if channels == 4:
                    data[offset + 3] = 255

    @staticmethod
    def depth_color(depth, minimum, maximum):
        ratio = min(1.0, max(0.0, (depth - minimum) / (maximum - minimum)))
        # Near points are red/yellow; far points trend blue for quick depth reading.
        return (
            int(255.0 * (1.0 - ratio)),
            int(190.0 * (1.0 - abs(2.0 * ratio - 1.0))),
            int(255.0 * ratio),
        )

    def camera_frame_cloud(self, points, source_header, zero_stamp):
        cloud = PointCloud2()
        copy_header(cloud.header, source_header, self.camera_optical_frame, zero_stamp)
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        payload = bytearray(cloud.row_step)
        for index, point in enumerate(points):
            struct.pack_into('<fff', payload, index * cloud.point_step, *point)
        cloud.data = bytes(payload)
        return cloud

    def publish_status(self, state, pair_skew_s=None, force=False):
        document = {
            'calibration_id': self.config['calibration_id'],
            'mode': self.config['mode'],
            'state': state,
            'camera_optical_frame': self.camera_optical_frame,
            'lidar_frame': self.lidar_frame,
            'rviz_lidar_stream_used_for_fusion': False,
            'time_policy': self.time_policy['fusion_input_time_domain'],
            'pair_skew_s': pair_skew_s,
            'last_projection': self.last_projection_summary,
        }
        translation = matrix_translation(self.camera_T_lidar)
        document['camera_optical_T_calibrated_lidar'] = {
            'translation_m': translation,
            'quaternion_xyzw': matrix_to_quaternion(self.camera_T_lidar),
        }
        serialized = json.dumps(document, allow_nan=False, separators=(',', ':'), sort_keys=True)
        now = time.monotonic()
        if (not force and serialized == self.last_status_signature and
                now - self.last_status_publish_monotonic_s < 1.0):
            return
        message = String()
        message.data = serialized
        self.status_publisher.publish(message)
        self.last_status_signature = serialized
        self.last_status_publish_monotonic_s = now


def main():
    share = default_share_directory()
    config_path = sys.argv[1] if len(sys.argv) > 1 else str(
        share / 'config' / 'camera_lidar_calibration.nominal.json')
    urdf_path = sys.argv[2] if len(sys.argv) > 2 else str(
        share / 'urdf' / 'oil_palm_harvester_kinematic.urdf')
    rclpy.init()
    node = None
    try:
        node = CameraLidarProjection(config_path, urdf_path)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
