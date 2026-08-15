#!/usr/bin/env python3
"""Validate the camera--LiDAR calibration contract against the active URDF.

This is intentionally an offline check.  It verifies nominal fixed geometry,
Gazebo sensor settings, and the data-only calibration profile without starting
Gazebo, publishing TF, or changing any runtime sensor/control topic.
"""

import argparse
import math
from pathlib import Path
from xml.etree import ElementTree as ET

from camera_lidar_calibration_common import (
    derive_camera_optical_T_lidar,
    matrix_rotation_angle,
    matrix_to_quaternion,
    matrix_translation,
    require_simulation_camera_lidar_config,
    transform_from_config,
)
from range_sensor_calibration_common import load_json, parse_fixed_joint_graph


def default_share_directory():
    return Path(__file__).resolve().parent.parent


def require_text(element, path, context):
    child = element.find(path)
    if child is None or child.text is None:
        raise ValueError(f"{context} is missing '{path}'")
    return child.text.strip()


def approximately_equal(actual, expected, tolerance=1e-9):
    return abs(float(actual) - float(expected)) <= tolerance


def gazebo_reference(root, reference):
    for gazebo in root.findall('gazebo'):
        if gazebo.get('reference') == reference:
            return gazebo
    raise ValueError(f"URDF has no <gazebo reference='{reference}'> block")


def validate_gazebo_camera(root, config):
    camera = config['camera']
    geometry = camera['nominal_image_geometry']
    gazebo = gazebo_reference(root, camera['body_frame'])
    sensor = gazebo.find('sensor')
    if sensor is None or sensor.get('type') != 'depth':
        raise ValueError('camera Gazebo reference must contain a depth sensor')
    width = int(require_text(sensor, 'camera/image/width', 'camera sensor'))
    height = int(require_text(sensor, 'camera/image/height', 'camera sensor'))
    expected_width, expected_height = geometry['size_px']
    if [width, height] != [expected_width, expected_height]:
        raise ValueError(
            f'camera image size is {width}x{height}, expected '
            f'{expected_width}x{expected_height} from calibration config')
    hfov = float(require_text(sensor, 'camera/horizontal_fov', 'camera sensor'))
    if not approximately_equal(hfov, geometry['horizontal_fov_rad'], 1e-9):
        raise ValueError(f'camera horizontal FOV is {hfov}, expected {geometry["horizontal_fov_rad"]}')
    near = float(require_text(sensor, 'camera/clip/near', 'camera sensor'))
    far = float(require_text(sensor, 'camera/clip/far', 'camera sensor'))
    if [near, far] != [float(value) for value in geometry['clip_range_m']]:
        raise ValueError(f'camera clip range is [{near}, {far}], expected {geometry["clip_range_m"]}')
    update_rate = float(require_text(sensor, 'update_rate', 'camera sensor'))
    if not approximately_equal(update_rate, geometry['update_rate_hz']):
        raise ValueError(
            f'camera update rate is {update_rate}, expected {geometry["update_rate_hz"]}')
    plugin = sensor.find('plugin')
    if plugin is None:
        raise ValueError('camera sensor has no Gazebo ROS plugin')
    frame_name = require_text(plugin, 'frame_name', 'camera plugin')
    if frame_name != camera['optical_frame']:
        raise ValueError(
            f"camera plugin frame is '{frame_name}', expected '{camera['optical_frame']}'")


def validate_gazebo_lidar(root, config):
    lidar = config['lidar']
    geometry = lidar['nominal_scan_geometry']
    gazebo = gazebo_reference(root, lidar['frame'])
    sensor = gazebo.find('sensor')
    if sensor is None or sensor.get('type') != 'gpu_ray':
        raise ValueError('LiDAR Gazebo reference must contain a gpu_ray sensor')
    horizontal_samples = int(require_text(sensor, 'ray/scan/horizontal/samples', 'LiDAR sensor'))
    vertical_samples = int(require_text(sensor, 'ray/scan/vertical/samples', 'LiDAR sensor'))
    if horizontal_samples != geometry['horizontal_samples']:
        raise ValueError('LiDAR horizontal sample count differs from calibration config')
    if vertical_samples != geometry['vertical_samples']:
        raise ValueError('LiDAR vertical sample count differs from calibration config')
    for path, expected in (
            ('ray/scan/horizontal/min_angle', geometry['horizontal_fov_rad'][0]),
            ('ray/scan/horizontal/max_angle', geometry['horizontal_fov_rad'][1]),
            ('ray/scan/vertical/min_angle', geometry['vertical_fov_rad'][0]),
            ('ray/scan/vertical/max_angle', geometry['vertical_fov_rad'][1]),
            ('ray/range/min', geometry['range_m'][0]),
            ('ray/range/max', geometry['range_m'][1]),
            ('update_rate', geometry['update_rate_hz'])):
        actual = float(require_text(sensor, path, 'LiDAR sensor'))
        if not approximately_equal(actual, expected, 1e-9):
            raise ValueError(f"LiDAR '{path}' is {actual}, expected {expected}")
    plugin = sensor.find('plugin')
    if plugin is None:
        raise ValueError('LiDAR sensor has no Gazebo ROS plugin')
    if require_text(plugin, 'frame_name', 'LiDAR plugin') != lidar['frame']:
        raise ValueError('LiDAR plugin frame does not match calibration config')
    if require_text(plugin, 'output_type', 'LiDAR plugin') != 'sensor_msgs/PointCloud2':
        raise ValueError('LiDAR plugin must publish sensor_msgs/PointCloud2')
    remapping = require_text(plugin, 'ros/remapping', 'LiDAR plugin')
    raw_remap_target = lidar['raw_points_topic']
    if raw_remap_target.startswith('/harvester/'):
        raw_remap_target = raw_remap_target[len('/harvester/'):]
    else:
        raw_remap_target = raw_remap_target.lstrip('/')
    if not remapping.endswith(f':={raw_remap_target}'):
        raise ValueError(
            f"LiDAR raw remapping '{remapping}' does not end with ':={raw_remap_target}'")


def validate_deployment_template(config):
    if config.get('mode') != 'deployment_template':
        raise ValueError('--mode planning accepts only a deployment_template profile')
    if config.get('deployment_allowed') is not False:
        raise ValueError('deployment template must remain deployment_allowed=false')
    commissioning = config.get('commissioning')
    if not isinstance(commissioning, dict) or commissioning.get('verified') is not False:
        raise ValueError('deployment template must explicitly remain unverified')


def main():
    share = default_share_directory()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', default=str(share / 'config' / 'camera_lidar_calibration.nominal.json'))
    parser.add_argument(
        '--urdf', default=str(share / 'urdf' / 'oil_palm_harvester_kinematic.urdf'))
    parser.add_argument('--mode', choices=('simulation', 'planning'), default='simulation')
    arguments = parser.parse_args()

    config = load_json(arguments.config)
    if arguments.mode == 'planning':
        validate_deployment_template(config)
        print('PASS: deployment template is intentionally unverified and cannot be used at runtime.')
        return

    require_simulation_camera_lidar_config(config)
    urdf_path = Path(arguments.urdf)
    root = ET.parse(str(urdf_path)).getroot()
    links, graph = parse_fixed_joint_graph(urdf_path)
    for frame in (
            config['reference_frame'], config['camera']['body_frame'],
            config['camera']['optical_frame'], config['lidar']['frame']):
        if frame not in links:
            raise ValueError(f"URDF does not define required frame '{frame}'")

    actual = derive_camera_optical_T_lidar(config, graph)
    expected = transform_from_config(
        config['relative_extrinsics']['urdf_expected_camera_optical_T_lidar'])
    translation_error = math.sqrt(sum(
        (matrix_translation(actual)[index] - matrix_translation(expected)[index]) ** 2
        for index in range(3)))
    rotation_error = matrix_rotation_angle(actual, expected)
    if translation_error > 1e-6 or rotation_error > 1e-6:
        raise ValueError(
            'URDF camera--LiDAR transform differs from the nominal contract: '
            f'{translation_error:.9f} m, {rotation_error:.9f} rad')

    validate_gazebo_camera(root, config)
    validate_gazebo_lidar(root, config)
    translation = matrix_translation(actual)
    quaternion = matrix_to_quaternion(actual)
    print(f"Calibration ID: {config['calibration_id']}")
    print(
        f"T_{config['camera']['optical_frame']}_{config['lidar']['frame']}: "
        f"xyz=({translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}) m, "
        f"quaternion_xyzw=({quaternion[0]:.6f}, {quaternion[1]:.6f}, "
        f"{quaternion[2]:.6f}, {quaternion[3]:.6f})")
    print('PASS: nominal camera/LiDAR geometry and Gazebo sensor settings match the contract.')


if __name__ == '__main__':
    main()
