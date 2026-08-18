"""ROS-message to canonical-payload encoders with no ROS node side effects."""

import io
import json
import math
import struct

import numpy as np
from PIL import Image as PilImage
from sensor_msgs.msg import PointField


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


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def header_frame_id(header):
    return (header.frame_id or '').lstrip('/')


def image_to_jpeg(message, quality):
    """Convert common ROS color encodings to an RGB JPEG payload."""
    encoding = (message.encoding or '').lower()
    channels = {
        'rgb8': 3,
        'bgr8': 3,
        'rgba8': 4,
        'bgra8': 4,
        'mono8': 1,
    }.get(encoding)
    if channels is None:
        raise ValueError('unsupported color image encoding {!r}'.format(message.encoding))
    row_bytes = int(message.width) * channels
    if int(message.step) < row_bytes:
        raise ValueError('image step is smaller than one image row')
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    expected = int(message.height) * int(message.step)
    if raw.size < expected:
        raise ValueError('image payload is shorter than height * step')
    rows = raw[:expected].reshape((int(message.height), int(message.step)))
    image = rows[:, :row_bytes].reshape((int(message.height), int(message.width), channels))
    if encoding == 'bgr8':
        image = image[:, :, ::-1]
    elif encoding == 'bgra8':
        image = image[:, :, [2, 1, 0, 3]]
    if encoding in ('rgba8', 'bgra8'):
        image = image[:, :, :3]
    if encoding == 'mono8':
        image = np.repeat(image, 3, axis=2)
    output = io.BytesIO()
    PilImage.fromarray(np.ascontiguousarray(image), mode='RGB').save(
        output, format='JPEG', quality=int(quality), optimize=False)
    return output.getvalue()


def depth_to_uint16_mm(message):
    """Normalize ROS 16UC1/32FC1 depth into canonical uint16 millimetres."""
    encoding = (message.encoding or '').lower()
    type_map = {
        '16uc1': ('u2', 1.0),
        '32fc1': ('f4', 1000.0),
        '64fc1': ('f8', 1000.0),
    }
    if encoding not in type_map:
        raise ValueError('unsupported depth image encoding {!r}'.format(message.encoding))
    dtype_code, scale = type_map[encoding]
    dtype = np.dtype(('>' if message.is_bigendian else '<') + dtype_code)
    pixel_bytes = dtype.itemsize
    row_bytes = int(message.width) * pixel_bytes
    if int(message.step) < row_bytes:
        raise ValueError('depth step is smaller than one depth row')
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    expected = int(message.height) * int(message.step)
    if raw.size < expected:
        raise ValueError('depth payload is shorter than height * step')
    rows = raw[:expected].reshape((int(message.height), int(message.step)))
    values = np.ascontiguousarray(rows[:, :row_bytes]).view(dtype).reshape(
        (int(message.height), int(message.width)))
    if encoding == '16uc1':
        millimetres = values.astype(np.uint16, copy=False)
    else:
        millimetres_float = values.astype(np.float64, copy=False) * scale
        valid = np.isfinite(millimetres_float) & (millimetres_float > 0.0)
        millimetres = np.zeros(values.shape, dtype=np.uint16)
        clipped = np.clip(np.rint(millimetres_float[valid]), 1, 65535)
        millimetres[valid] = clipped.astype(np.uint16)
    return np.ascontiguousarray(millimetres.astype('<u2', copy=False)).tobytes()


def camera_info_json(message):
    return json.dumps({
        'width': int(message.width),
        'height': int(message.height),
        'distortion_model': message.distortion_model,
        'd': list(message.d),
        'k': list(message.k),
        'r': list(message.r),
        'p': list(message.p),
        'binning_x': int(message.binning_x),
        'binning_y': int(message.binning_y),
        'roi': {
            'x_offset': int(message.roi.x_offset),
            'y_offset': int(message.roi.y_offset),
            'height': int(message.roi.height),
            'width': int(message.roi.width),
            'do_rectify': bool(message.roi.do_rectify),
        },
    }, separators=(',', ':'), allow_nan=False).encode('utf-8')


def _field_reader(message, required_name):
    for field in message.fields:
        if field.name == required_name:
            fmt = POINT_FIELD_FORMATS.get(field.datatype)
            if fmt is None or int(field.count) != 1:
                raise ValueError('unsupported PointCloud2 field {!r}'.format(required_name))
            prefix = '>' if message.is_bigendian else '<'
            return int(field.offset), struct.Struct(prefix + fmt)
    raise ValueError('PointCloud2 is missing {!r}'.format(required_name))


def pointcloud_to_xyz_f32(message, stride=1, roi=None):
    """Extract a configurable XYZ-only canonical cloud from PointCloud2."""
    stride = max(1, int(stride))
    x_offset, x_reader = _field_reader(message, 'x')
    y_offset, y_reader = _field_reader(message, 'y')
    z_offset, z_reader = _field_reader(message, 'z')
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if point_step <= 0 or row_step < int(message.width) * point_step:
        raise ValueError('invalid PointCloud2 point_step/row_step')
    payload = bytes(message.data)
    if len(payload) < int(message.height) * row_step:
        raise ValueError('PointCloud2 payload is shorter than height * row_step')
    roi = roi or {}
    minimum = {axis: roi.get('min_' + axis) for axis in ('x', 'y', 'z')}
    maximum = {axis: roi.get('max_' + axis) for axis in ('x', 'y', 'z')}
    output = bytearray()
    count = 0
    for row in range(0, int(message.height), stride):
        row_base = row * row_step
        for column in range(0, int(message.width), stride):
            point_base = row_base + column * point_step
            x = x_reader.unpack_from(payload, point_base + x_offset)[0]
            y = y_reader.unpack_from(payload, point_base + y_offset)[0]
            z = z_reader.unpack_from(payload, point_base + z_offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            values = {'x': x, 'y': y, 'z': z}
            if any(minimum[axis] is not None and values[axis] < minimum[axis] for axis in values):
                continue
            if any(maximum[axis] is not None and values[axis] > maximum[axis] for axis in values):
                continue
            output.extend(struct.pack('<fff', x, y, z))
            count += 1
    return bytes(output), count
