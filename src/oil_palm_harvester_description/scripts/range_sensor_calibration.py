#!/usr/bin/env python3
"""Project the five Gazebo range readings into a calibrated C-channel frame.

Raw Gazebo ``sensor_msgs/Range`` topics remain unchanged.  This node adds a
calibrated, body-relative interpretation for docking: endpoint topics,
uncluttered RViz ray markers, and a side-pair trunk centre estimate.  All
five source frames are rigid children of the C-channel assembly, so their
relative transforms can be read once from TF using ``Time(0)``.  That avoids
the Gazebo-simulation-time versus GUI-wall-time mismatch in this Foxy scene.
"""

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from range_sensor_calibration_common import (
    fixed_transform_between,
    load_json,
    matrix_from_pose,
    matrix_from_rotation_translation,
    matrix_multiply,
    parse_fixed_joint_graph,
    quaternion_to_rotation,
    require_simulation_config,
    transform_point,
)


def default_share_directory():
    return Path(__file__).resolve().parent.parent


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def copy_stamp(destination, source):
    destination.sec = source.sec
    destination.nanosec = source.nanosec


def point_message(values):
    point = Point()
    point.x, point.y, point.z = values
    return point


def json_safe(value):
    """Convert telemetry to strict JSON without allowing NaN/Infinity tokens."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    return value


class RangeSensorCalibration(Node):
    """Calibration projector for the five rigid C-channel range sensors."""

    def __init__(self, config_path, urdf_path):
        super().__init__('range_sensor_calibration')
        self.config_path = Path(config_path)
        self.urdf_path = Path(urdf_path)
        self.config = require_simulation_config(load_json(self.config_path))
        self.reference_frame = self.config['reference_frame']
        self.runtime = self.config['runtime']
        self.sensors = self.config['sensors']
        self.validate_urdf_contract()
        self.transforms = {}
        self.states = {}
        self.frame_mismatch_warned = set()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/harvester/docking/range_markers', 10)
        self.status_publisher = self.create_publisher(
            String, '/harvester/docking/calibration_status', 10)
        self.hit_publishers = {
            name: self.create_publisher(
                PointStamped, f'/harvester/docking/range_hits/{name}', 10)
            for name in self.sensors
        }
        self.trunk_center_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/harvester/docking/trunk_center', 10)
        # ``Node.subscriptions`` is a read-only rclpy property in Foxy.
        # Retain our handles under a project-owned name so they stay alive.
        self.range_subscriptions = [
            self.create_subscription(
                Range, sensor['topic'],
                lambda message, key=name: self.on_range(key, message),
                qos_profile_sensor_data)
            for name, sensor in self.sensors.items()
        ]
        self.transform_timer = self.create_timer(0.2, self.cache_static_transforms)
        update_period = 1.0 / float(self.runtime['visualization_rate_hz'])
        self.visualization_timer = self.create_timer(update_period, self.publish_visualization)
        self.get_logger().info(
            f"Loaded simulation-only calibration '{self.config['calibration_id']}' from "
            f"{self.config_path}; reference frame is '{self.reference_frame}'.")
        self.get_logger().info(
            f"Physical sensor mounts remain defined by {self.urdf_path}; raw Range topics "
            'are preserved unchanged.')

    def validate_urdf_contract(self):
        """Reject a configuration whose frames are not fixed URDF descendants.

        The configuration purposefully does not duplicate mount poses.  This
        validation makes the active URDF the sole source for the physical
        Gazebo ray geometry and ensures each calibration transform is local to
        the moving C-channel assembly.
        """
        links, graph = parse_fixed_joint_graph(self.urdf_path)
        if self.reference_frame not in links:
            raise ValueError(
                f"URDF '{self.urdf_path}' does not define '{self.reference_frame}'")
        for name, sensor in self.sensors.items():
            source = sensor['expected_frame_id']
            if source not in links:
                raise ValueError(f"URDF does not define sensor frame '{source}' for '{name}'")
            fixed_transform_between(graph, self.reference_frame, source)

    def cache_static_transforms(self):
        for name, sensor in self.sensors.items():
            if name in self.transforms:
                continue
            source_frame = sensor['expected_frame_id']
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame, source_frame, Time(),
                    timeout=Duration(seconds=0.05))
            except Exception:
                continue
            rotation = quaternion_to_rotation(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w)
            sensor_to_reference = matrix_from_rotation_translation(
                rotation,
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ])
            correction = sensor['sensor_T_calibrated_beam']
            sensor_to_beam = matrix_from_pose(
                correction['translation_m'], correction['rpy_rad'])
            self.transforms[name] = matrix_multiply(sensor_to_reference, sensor_to_beam)
            translation = [self.transforms[name][row][3] for row in range(3)]
            self.get_logger().info(
                f"Cached T_{self.reference_frame}_{source_frame}: "
                f"({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) m")

        if len(self.transforms) == len(self.sensors):
            self.transform_timer.cancel()
            self.get_logger().info(
                'All fixed range-sensor transforms are available; publishing calibrated endpoints.')

    def on_range(self, name, message):
        sensor = self.sensors[name]
        expected_frame = sensor['expected_frame_id']
        message_frame = message.header.frame_id
        if message_frame != expected_frame:
            if name not in self.frame_mismatch_warned:
                self.get_logger().warning(
                    f"{name} expected frame '{expected_frame}' but received '{message_frame}'; "
                    'the reading is rejected until its publisher is corrected.')
                self.frame_mismatch_warned.add(name)
            self.states[name] = self.invalid_state('FRAME_MISMATCH', message)
            return

        if name not in self.transforms:
            self.states[name] = self.invalid_state('WAITING_FOR_STATIC_TF', message)
            return

        raw_range = float(message.range)
        minimum, maximum = [float(value) for value in sensor['valid_range_m']]
        message_maximum = float(message.max_range) if message.max_range > 0.0 else maximum
        valid_raw = (
            math.isfinite(raw_range) and raw_range >= minimum and
            raw_range < min(maximum, message_maximum) - 0.0005)
        if not valid_raw:
            self.states[name] = self.invalid_state('NO_RETURN', message, raw_range)
            return

        corrected_range = float(sensor['range_scale']) * raw_range + float(sensor['range_bias_m'])
        if not (math.isfinite(corrected_range) and minimum <= corrected_range <= maximum):
            self.states[name] = self.invalid_state('OUTSIDE_CALIBRATED_LIMITS', message, raw_range)
            return

        endpoint = transform_point(self.transforms[name], [corrected_range, 0.0, 0.0])
        origin = transform_point(self.transforms[name], [0.0, 0.0, 0.0])
        self.states[name] = {
            'status': 'VALID',
            'raw_range_m': raw_range,
            'corrected_range_m': corrected_range,
            'endpoint_m': endpoint,
            'origin_m': origin,
            'receipt_monotonic_s': time.monotonic(),
            'source_stamp': message.header.stamp,
            'source_stamp_s': stamp_seconds(message.header.stamp),
            'source_frame_id': message.header.frame_id,
        }
        hit = PointStamped()
        hit.header.frame_id = self.reference_frame
        copy_stamp(hit.header.stamp, message.header.stamp)
        hit.point = point_message(endpoint)
        self.hit_publishers[name].publish(hit)

    @staticmethod
    def invalid_state(status, message, raw_range=None):
        return {
            'status': status,
            'raw_range_m': raw_range,
            'corrected_range_m': None,
            'endpoint_m': None,
            'origin_m': None,
            'receipt_monotonic_s': time.monotonic(),
            'source_stamp': message.header.stamp,
            'source_stamp_s': stamp_seconds(message.header.stamp),
            'source_frame_id': message.header.frame_id,
        }

    def fresh_state(self, name):
        state = self.states.get(name)
        if state is None:
            return None
        if time.monotonic() - state['receipt_monotonic_s'] > float(
                self.runtime['stale_receipt_timeout_s']):
            return None
        if state['status'] != 'VALID':
            return None
        return state

    def side_pair_estimate(self):
        settings = self.config['side_pair_estimator']
        left = self.fresh_state(settings['left_sensor'])
        right = self.fresh_state(settings['right_sensor'])
        if left is None or right is None:
            return None

        source_skew = abs(left['source_stamp_s'] - right['source_stamp_s'])
        if left['source_stamp_s'] > 0.0 and right['source_stamp_s'] > 0.0:
            if source_skew > float(self.runtime['maximum_pair_skew_s']):
                return None
        elif abs(left['receipt_monotonic_s'] - right['receipt_monotonic_s']) > float(
                self.runtime['maximum_pair_skew_s']):
            return None

        left_point = left['endpoint_m']
        right_point = right['endpoint_m']
        longitudinal_difference = abs(left_point[0] - right_point[0])
        if longitudinal_difference > float(settings['maximum_longitudinal_difference_m']):
            return None
        diameter = math.dist(left_point, right_point)
        if not (float(settings['minimum_diameter_m']) <= diameter <=
                float(settings['maximum_diameter_m'])):
            return None
        return {
            'center_m': [(left_point[index] + right_point[index]) * 0.5 for index in range(3)],
            'diameter_m': diameter,
            'source_stamp': left['source_stamp'] if left['source_stamp_s'] >= right['source_stamp_s']
            else right['source_stamp'],
            'source_stamp_s': max(left['source_stamp_s'], right['source_stamp_s']),
        }

    @staticmethod
    def marker_header(marker, frame):
        marker.header.frame_id = frame
        # Zero time is intentional for RViz only: use its latest C-channel TF.
        # Algorithm outputs retain original Range acquisition timestamps.
        marker.frame_locked = True

    def marker_for_ray(self, marker_id, state, valid):
        marker = Marker()
        self.marker_header(marker, self.reference_frame)
        marker.ns = 'calibrated_docking_ranges'
        marker.id = marker_id
        if not valid:
            marker.action = Marker.DELETE
            return marker
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.018
        marker.color.r = 0.1
        marker.color.g = 0.95
        marker.color.b = 0.65
        marker.color.a = 0.95
        marker.points = [point_message(state['origin_m']), point_message(state['endpoint_m'])]
        return marker

    def marker_for_hit(self, marker_id, state, valid):
        marker = Marker()
        self.marker_header(marker, self.reference_frame)
        marker.ns = 'calibrated_docking_hits'
        marker.id = marker_id
        if not valid:
            marker.action = Marker.DELETE
            return marker
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point_message(state['endpoint_m'])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.075
        marker.scale.y = 0.075
        marker.scale.z = 0.075
        marker.color.r = 1.0
        marker.color.g = 0.56
        marker.color.b = 0.08
        marker.color.a = 0.95
        return marker

    def marker_for_trunk(self, estimate):
        marker = Marker()
        self.marker_header(marker, self.reference_frame)
        marker.ns = 'calibrated_trunk_estimate'
        marker.id = 100
        if estimate is None:
            marker.action = Marker.DELETE
            return marker
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position = point_message([
            estimate['center_m'][0], estimate['center_m'][1], 0.03])
        marker.pose.orientation.w = 1.0
        marker.scale.x = estimate['diameter_m']
        marker.scale.y = estimate['diameter_m']
        marker.scale.z = 0.06
        marker.color.r = 0.15
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = 0.55
        return marker

    def publish_trunk_center(self, estimate):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.reference_frame
        copy_stamp(message.header.stamp, estimate['source_stamp'])
        message.pose.pose.position = point_message(estimate['center_m'])
        message.pose.pose.orientation.w = 1.0
        variance = float(self.config['side_pair_estimator']['position_stddev_m']) ** 2
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[14] = variance
        message.pose.covariance[21] = 1.0
        message.pose.covariance[28] = 1.0
        message.pose.covariance[35] = 1.0
        self.trunk_center_publisher.publish(message)

    def status_document(self, estimate):
        now = time.monotonic()
        sensor_status = {}
        for name in self.sensors:
            state = self.states.get(name)
            if state is None:
                sensor_status[name] = {'status': 'WAITING_FOR_MESSAGE'}
                continue
            age = now - state['receipt_monotonic_s']
            status = state['status'] if age <= float(self.runtime['stale_receipt_timeout_s']) else 'STALE'
            sensor_status[name] = {
                'status': status,
                'raw_range_m': state['raw_range_m'],
                'corrected_range_m': state['corrected_range_m'],
                'hit_point_m': state['endpoint_m'],
                'source_frame_id': state['source_frame_id'],
                'source_stamp_s': state['source_stamp_s'],
                'receipt_age_s': round(age, 4),
            }
        serialized_estimate = None
        if estimate is not None:
            serialized_estimate = {
                'center_m': estimate['center_m'],
                'diameter_m': estimate['diameter_m'],
                'source_stamp_s': estimate['source_stamp_s'],
            }
        return {
            'calibration_id': self.config['calibration_id'],
            'mode': self.config['mode'],
            'reference_frame': self.reference_frame,
            'sensors': sensor_status,
            'trunk_estimate': serialized_estimate,
        }

    def publish_visualization(self):
        markers = MarkerArray()
        for index, name in enumerate(self.sensors):
            state = self.fresh_state(name)
            markers.markers.append(self.marker_for_ray(index * 2, state, state is not None))
            markers.markers.append(self.marker_for_hit(index * 2 + 1, state, state is not None))
        estimate = self.side_pair_estimate()
        markers.markers.append(self.marker_for_trunk(estimate))
        self.marker_publisher.publish(markers)
        if estimate is not None:
            self.publish_trunk_center(estimate)
        status = String()
        status.data = json.dumps(
            json_safe(self.status_document(estimate)),
            allow_nan=False,
            separators=(',', ':'),
            sort_keys=True)
        self.status_publisher.publish(status)


def main():
    share = default_share_directory()
    config_path = sys.argv[1] if len(sys.argv) > 1 else str(
        share / 'config' / 'range_sensor_calibration.nominal.json')
    urdf_path = sys.argv[2] if len(sys.argv) > 2 else str(
        share / 'urdf' / 'oil_palm_harvester_kinematic.urdf')
    rclpy.init()
    node = None
    try:
        node = RangeSensorCalibration(config_path, urdf_path)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        logger = rclpy.logging.get_logger('range_sensor_calibration')
        logger.fatal(f'Calibration projector stopped: {error}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
