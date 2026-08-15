#!/usr/bin/env python3
"""Dependency-free helpers for the camera--LiDAR calibration contract.

The active URDF remains the single source of physical mount geometry.  This
module validates a data-only calibration profile and derives the fixed camera
optical-frame to LiDAR transform from that URDF.  A calibration correction is
applied only inside perception; it never publishes or overrides TF.
"""

import math

from range_sensor_calibration_common import (
    fixed_transform_between,
    inverse_rigid,
    matrix_from_rotation_translation,
    matrix_multiply,
    quaternion_to_rotation,
)


def _is_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _require_string(container, field, context):
    value = container.get(field)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{context}.{field} must be a non-empty string")


def _require_vector(container, field, length, context):
    values = container.get(field)
    if (not isinstance(values, list) or len(values) != length or
            not all(_is_number(value) for value in values)):
        raise ValueError(f"{context}.{field} must contain {length} finite numbers")
    return [float(value) for value in values]


def _require_positive_number(container, field, context):
    value = container.get(field)
    if not _is_number(value) or float(value) <= 0.0:
        raise ValueError(f"{context}.{field} must be a positive finite number")
    return float(value)


def _require_positive_integer(container, field, context):
    value = container.get(field)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{context}.{field} must be a positive integer")
    return value


def _validate_transform(transform, context):
    if not isinstance(transform, dict):
        raise ValueError(f"{context} must be an object")
    translation = _require_vector(transform, 'translation_m', 3, context)
    quaternion = _require_vector(transform, 'quaternion_xyzw', 4, context)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 1e-12:
        raise ValueError(f"{context}.quaternion_xyzw must not be zero")
    return translation, [component / norm for component in quaternion]


def require_simulation_camera_lidar_config(config):
    """Validate and return a simulation-only camera--LiDAR profile.

    Deployment templates are deliberately rejected by the runtime projector.
    They document the fields that must be measured on a physical machine but
    must not become an accidental source of simulated or hardware geometry.
    """
    if config.get('schema_version') != 1:
        raise ValueError('schema_version must be 1')
    if config.get('mode') != 'simulation_only' or config.get('deployment_allowed') is not False:
        raise ValueError(
            'The projector accepts only a simulation_only profile with '
            'deployment_allowed set to false.')
    _require_string(config, 'calibration_id', 'config')
    _require_string(config, 'reference_frame', 'config')

    camera = config.get('camera')
    if not isinstance(camera, dict):
        raise ValueError('camera must be an object')
    for field in (
            'body_frame', 'optical_frame', 'color_image_topic',
            'color_camera_info_topic', 'depth_image_topic',
            'depth_camera_info_topic', 'depth_points_topic',
            'intrinsics_source'):
        _require_string(camera, field, 'camera')
    if camera['intrinsics_source'] != 'camera_info_topic':
        raise ValueError('camera.intrinsics_source must be camera_info_topic')
    geometry = camera.get('nominal_image_geometry')
    if not isinstance(geometry, dict):
        raise ValueError('camera.nominal_image_geometry must be an object')
    size = geometry.get('size_px')
    if (not isinstance(size, list) or len(size) != 2 or
            any(not isinstance(value, int) or isinstance(value, bool) or value <= 0
                for value in size)):
        raise ValueError('camera.nominal_image_geometry.size_px must be [width, height]')
    _require_positive_number(geometry, 'horizontal_fov_rad', 'camera.nominal_image_geometry')
    clip = geometry.get('clip_range_m')
    if (not isinstance(clip, list) or len(clip) != 2 or
            not all(_is_number(value) for value in clip) or float(clip[0]) <= 0.0 or
            float(clip[0]) >= float(clip[1])):
        raise ValueError('camera.nominal_image_geometry.clip_range_m is invalid')
    _require_positive_number(geometry, 'update_rate_hz', 'camera.nominal_image_geometry')

    lidar = config.get('lidar')
    if not isinstance(lidar, dict):
        raise ValueError('lidar must be an object')
    for field in ('frame', 'raw_points_topic', 'rviz_points_topic'):
        _require_string(lidar, field, 'lidar')
    lidar_geometry = lidar.get('nominal_scan_geometry')
    if not isinstance(lidar_geometry, dict):
        raise ValueError('lidar.nominal_scan_geometry must be an object')
    for field in ('horizontal_samples', 'vertical_samples'):
        _require_positive_integer(lidar_geometry, field, 'lidar.nominal_scan_geometry')
    for field in ('horizontal_fov_rad', 'vertical_fov_rad', 'range_m'):
        values = lidar_geometry.get(field)
        if (not isinstance(values, list) or len(values) != 2 or
                not all(_is_number(value) for value in values) or
                float(values[0]) >= float(values[1])):
            raise ValueError(f'lidar.nominal_scan_geometry.{field} is invalid')
    _require_positive_number(lidar_geometry, 'update_rate_hz', 'lidar.nominal_scan_geometry')

    extrinsics = config.get('relative_extrinsics')
    if not isinstance(extrinsics, dict):
        raise ValueError('relative_extrinsics must be an object')
    expected = extrinsics.get('urdf_expected_camera_optical_T_lidar')
    _validate_transform(expected, 'relative_extrinsics.urdf_expected_camera_optical_T_lidar')
    correction = extrinsics.get('nominal_lidar_T_calibrated_lidar')
    _validate_transform(correction, 'relative_extrinsics.nominal_lidar_T_calibrated_lidar')
    for field in ('translation_stddev_m', 'rotation_stddev_rad'):
        values = _require_vector(extrinsics, field, 3, 'relative_extrinsics')
        if any(value < 0.0 for value in values):
            raise ValueError(f'relative_extrinsics.{field} must be non-negative')

    time_policy = config.get('time_policy')
    if not isinstance(time_policy, dict):
        raise ValueError('time_policy must be an object')
    for field in ('fusion_input_time_domain', 'rviz_lidar_topic_policy'):
        _require_string(time_policy, field, 'time_policy')
    if time_policy['rviz_lidar_topic_policy'] != 'visualization_only_latest_tf':
        raise ValueError(
            'time_policy.rviz_lidar_topic_policy must explicitly mark the '
            'RViz LiDAR copy as visualization_only_latest_tf')
    _require_positive_number(time_policy, 'maximum_image_lidar_skew_s', 'time_policy')
    if time_policy.get('reject_zero_source_timestamps') is not True:
        raise ValueError('time_policy.reject_zero_source_timestamps must be true')

    runtime = config.get('runtime')
    if not isinstance(runtime, dict):
        raise ValueError('runtime must be an object')
    _require_positive_number(runtime, 'projection_rate_hz', 'runtime')
    _require_positive_integer(runtime, 'maximum_points_per_cloud', 'runtime')
    point_radius = _require_positive_integer(runtime, 'point_radius_px', 'runtime')
    if point_radius > 8:
        raise ValueError('runtime.point_radius_px must not exceed 8')

    outputs = config.get('outputs')
    if not isinstance(outputs, dict):
        raise ValueError('outputs must be an object')
    for field in ('overlay_image_topic', 'visible_points_raw_topic',
                  'visible_points_rviz_topic', 'status_topic'):
        _require_string(outputs, field, 'outputs')
    return config


def transform_from_config(transform):
    """Return the matrix described by a calibration transform object."""
    translation, quaternion = _validate_transform(transform, 'transform')
    return matrix_from_rotation_translation(
        quaternion_to_rotation(*quaternion), translation)


def derive_camera_optical_T_lidar(config, graph):
    """Derive ``T_camera_optical_lidar`` from the active fixed URDF tree."""
    reference = config['reference_frame']
    # Explicitly retain the common arm-base contract: a future sensor mounted
    # across a movable joint must not silently reuse this rigid-pair projector.
    for frame in (config['camera']['body_frame'], config['camera']['optical_frame'],
                  config['lidar']['frame']):
        fixed_transform_between(graph, reference, frame)
    return fixed_transform_between(
        graph, config['camera']['optical_frame'], config['lidar']['frame'])


def derive_camera_optical_T_calibrated_lidar(config, graph):
    """Return the perception-only camera to calibrated-LiDAR transform.

    ``nominal_lidar_T_calibrated_lidar`` maps coordinates expressed in the
    calibrated LiDAR frame into the nominal URDF LiDAR frame.  It is identity
    in this Gazebo profile.  This is intentionally a matrix used by the
    projector only; no corrective TF is ever broadcast.
    """
    nominal = derive_camera_optical_T_lidar(config, graph)
    correction = transform_from_config(
        config['relative_extrinsics']['nominal_lidar_T_calibrated_lidar'])
    return matrix_multiply(nominal, correction)


def matrix_translation(matrix):
    return [float(matrix[row][3]) for row in range(3)]


def matrix_rotation_angle(left, right):
    """Return the angle between two rigid-transform rotations in radians."""
    relative = matrix_multiply(left, inverse_rigid(right))
    trace = relative[0][0] + relative[1][1] + relative[2][2]
    return math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5)))


def matrix_to_quaternion(matrix):
    """Return a normalized ``[x, y, z, w]`` quaternion for a rotation matrix."""
    r00, r01, r02 = matrix[0][0], matrix[0][1], matrix[0][2]
    r10, r11, r12 = matrix[1][0], matrix[1][1], matrix[1][2]
    r20, r21, r22 = matrix[2][0], matrix[2][1], matrix[2][2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = [(r21 - r12) / scale, (r02 - r20) / scale,
                      (r10 - r01) / scale, 0.25 * scale]
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        quaternion = [0.25 * scale, (r01 + r10) / scale,
                      (r02 + r20) / scale, (r21 - r12) / scale]
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        quaternion = [(r01 + r10) / scale, 0.25 * scale,
                      (r12 + r21) / scale, (r02 - r20) / scale]
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        quaternion = [(r02 + r20) / scale, (r12 + r21) / scale,
                      0.25 * scale, (r10 - r01) / scale]
    norm = math.sqrt(sum(value * value for value in quaternion))
    return [value / norm for value in quaternion]
