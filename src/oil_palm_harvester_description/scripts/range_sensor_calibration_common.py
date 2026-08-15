#!/usr/bin/env python3
"""Small dependency-free helpers for docking-range calibration files.

The active URDF owns nominal physical sensor mount geometry.  The simulation
configuration owns only measurement corrections, validity limits, and the
common docking frame.  These helpers intentionally have no ROS dependency so
the configuration can be validated before a ROS graph is running.
"""

import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET


def load_json(path):
    with Path(path).open('r', encoding='utf-8') as stream:
        return json.load(stream)


def _is_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def require_simulation_config(config):
    if config.get('mode') != 'simulation_only' or config.get('deployment_allowed') is not False:
        raise ValueError(
            'The calibration projector accepts only a simulation_only configuration. '
            'A deployment template must be surveyed and verified before use.')
    if not isinstance(config.get('reference_frame'), str) or not config['reference_frame']:
        raise ValueError('reference_frame must be a non-empty string')
    sensors = config.get('sensors')
    if not isinstance(sensors, dict) or not sensors:
        raise ValueError('sensors must be a non-empty object')
    for name, sensor in sensors.items():
        if not isinstance(sensor.get('topic'), str) or not sensor['topic']:
            raise ValueError(f"sensor '{name}' has no topic")
        if not isinstance(sensor.get('expected_frame_id'), str) or not sensor['expected_frame_id']:
            raise ValueError(f"sensor '{name}' has no expected_frame_id")
        correction = sensor.get('sensor_T_calibrated_beam')
        if not isinstance(correction, dict):
            raise ValueError(f"sensor '{name}' has no sensor_T_calibrated_beam")
        for field in ('translation_m', 'rpy_rad'):
            values = correction.get(field)
            if not isinstance(values, list) or len(values) != 3 or not all(_is_number(v) for v in values):
                raise ValueError(f"sensor '{name}' has invalid {field}")
        for field in ('range_scale', 'range_bias_m', 'standard_deviation_m'):
            if not _is_number(sensor.get(field)):
                raise ValueError(f"sensor '{name}' has invalid {field}")
        valid_range = sensor.get('valid_range_m')
        if (not isinstance(valid_range, list) or len(valid_range) != 2 or
                not all(_is_number(v) for v in valid_range) or
                float(valid_range[0]) >= float(valid_range[1])):
            raise ValueError(f"sensor '{name}' has invalid valid_range_m")
    return config


def identity_matrix():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matrix_multiply(left, right):
    return [
        [sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def rpy_to_rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF/SDF RPY: Rz(yaw) * Ry(pitch) * Rx(roll).
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def quaternion_to_rotation(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError('quaternion cannot be zero')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def matrix_from_rotation_translation(rotation, translation):
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], float(translation[0])],
        [rotation[1][0], rotation[1][1], rotation[1][2], float(translation[1])],
        [rotation[2][0], rotation[2][1], rotation[2][2], float(translation[2])],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matrix_from_pose(translation, rpy):
    return matrix_from_rotation_translation(
        rpy_to_rotation(float(rpy[0]), float(rpy[1]), float(rpy[2])), translation)


def inverse_rigid(matrix):
    rotation = [[matrix[row][column] for column in range(3)] for row in range(3)]
    translation = [matrix[row][3] for row in range(3)]
    transposed = [[rotation[column][row] for column in range(3)] for row in range(3)]
    inverted_translation = [
        -sum(transposed[row][column] * translation[column] for column in range(3))
        for row in range(3)
    ]
    return matrix_from_rotation_translation(transposed, inverted_translation)


def transform_point(matrix, point):
    return [
        sum(matrix[row][column] * float(point[column]) for column in range(3)) + matrix[row][3]
        for row in range(3)
    ]


def parse_fixed_joint_graph(urdf_path):
    root = ET.parse(str(urdf_path)).getroot()
    graph = {}
    links = {link.get('name') for link in root.findall('link') if link.get('name')}
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed':
            continue
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is None or child is None:
            continue
        parent_name = parent.get('link')
        child_name = child.get('link')
        if not parent_name or not child_name:
            continue
        origin = joint.find('origin')
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get('xyz'):
                xyz = [float(value) for value in origin.get('xyz').split()]
            if origin.get('rpy'):
                rpy = [float(value) for value in origin.get('rpy').split()]
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError(f"joint '{joint.get('name')}' has invalid origin")
        # Entry is T_parent_child: a child-frame point expressed in parent.
        graph[child_name] = (parent_name, matrix_from_pose(xyz, rpy))
    return links, graph


def fixed_transform_between(graph, reference_frame, source_frame):
    """Return T_reference_source when both frames share a fixed-joint tree."""
    def to_root(frame):
        transform = identity_matrix()
        current = frame
        visited = set()
        while current in graph:
            if current in visited:
                raise ValueError(f'fixed-joint cycle at {current}')
            visited.add(current)
            parent, parent_child = graph[current]
            transform = matrix_multiply(parent_child, transform)
            current = parent
        return current, transform

    reference_root, root_reference = to_root(reference_frame)
    source_root, root_source = to_root(source_frame)
    if reference_root != source_root:
        raise ValueError(
            f"'{reference_frame}' and '{source_frame}' do not share a fixed-joint root "
            f"({reference_root} versus {source_root})")
    return matrix_multiply(inverse_rigid(root_reference), root_source)


def matrix_to_rpy(matrix):
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(abs(pitch) - math.pi / 2.0) < 1e-9:
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    else:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    return [roll, pitch, yaw]
