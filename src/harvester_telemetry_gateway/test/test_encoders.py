import json
import struct
import unittest

import numpy as np
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField

from harvester_telemetry_gateway.encoders import (
    depth_to_uint16_mm,
    image_to_jpeg,
    imu_json,
    pointcloud_to_xyz_f32,
)


class EncoderTest(unittest.TestCase):
    def test_rgb8_encodes_to_jpeg(self):
        message = Image()
        message.width = 2
        message.height = 1
        message.encoding = 'rgb8'
        message.step = 6
        message.data = bytes([255, 0, 0, 0, 255, 0])
        self.assertTrue(image_to_jpeg(message, 85).startswith(b'\xff\xd8'))

    def test_float_depth_is_normalized_to_millimetres(self):
        message = Image()
        message.width = 2
        message.height = 1
        message.encoding = '32FC1'
        message.step = 8
        message.data = struct.pack('<ff', 1.25, float('nan'))
        self.assertEqual(depth_to_uint16_mm(message), struct.pack('<HH', 1250, 0))

    def test_cloud_extracts_xyz(self):
        message = PointCloud2()
        message.width = 1
        message.height = 1
        message.point_step = 12
        message.row_step = 12
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.data = struct.pack('<fff', 1.0, 2.0, 3.0)
        payload, count = pointcloud_to_xyz_f32(message)
        self.assertEqual(count, 1)
        self.assertEqual(payload, struct.pack('<fff', 1.0, 2.0, 3.0))

    def test_imu_encodes_to_json_with_covariance(self):
        message = Imu()
        message.orientation.x = 0.0
        message.orientation.y = 0.0
        message.orientation.z = 0.0
        message.orientation.w = 1.0
        message.angular_velocity.x = 0.1
        message.angular_velocity.y = -0.2
        message.angular_velocity.z = 0.3
        message.linear_acceleration.x = 0.0
        message.linear_acceleration.y = 0.0
        message.linear_acceleration.z = 9.81
        payload = json.loads(imu_json(message).decode('utf-8'))
        self.assertEqual(payload['orientation']['w'], 1.0)
        self.assertEqual(payload['angular_velocity']['x'], 0.1)
        self.assertEqual(payload['linear_acceleration']['z'], 9.81)
        self.assertEqual(len(payload['orientation_covariance']), 9)
        self.assertEqual(len(payload['angular_velocity_covariance']), 9)
        self.assertEqual(len(payload['linear_acceleration_covariance']), 9)


if __name__ == '__main__':
    unittest.main()
