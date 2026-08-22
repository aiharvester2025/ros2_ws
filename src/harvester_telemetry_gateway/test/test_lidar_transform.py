import math
import unittest

from harvester_telemetry_gateway.encoders import (
    quaternion_to_rotation_matrix,
    rotate_point,
)


def pitch_quaternion(theta):
    """Rotation about the +y axis (sensor pitch) by theta radians."""
    half = theta / 2.0
    return (0.0, math.sin(half), 0.0, math.cos(half))  # x, y, z, w


class LidarTransformTest(unittest.TestCase):
    def test_identity_rotation_leaves_point_unchanged(self):
        r = quaternion_to_rotation_matrix(0.0, 0.0, 0.0, 1.0)
        self.assertEqual(rotate_point(r, 1.0, 2.0, 3.0), (1.0, 2.0, 3.0))

    def test_pitch_up_keeps_world_vertical_point_when_undone(self):
        # A tree point in the world is straight up: (0, 0, 5) in world frame.
        # Expressed in the sensor frame after the sensor pitches up by theta,
        # it is rotated by -theta. Applying the forward sensor->world rotation
        # must recover (0, 0, 5) up to floating error.
        theta = 0.5  # radians of "look up"
        r = quaternion_to_rotation_matrix(*pitch_quaternion(theta))
        # Sensor-frame coordinates of a world point at (0,0,5):
        # world = R_sensor_to_world * sensor  =>  sensor = R^T * world.
        world = (0.0, 0.0, 5.0)
        sensor = rotate_point(r, world[0], world[1], world[2],
                              # inverse of R is its transpose (pure rotation)
                              )
        # rotate_point applies R + translation; to invert use R^T.
        rt = tuple(tuple(r[j][i] for j in range(3)) for i in range(3))
        sensor = rotate_point(rt, world[0], world[1], world[2])
        recovered = rotate_point(r, *sensor)
        for a, b in zip(recovered, world):
            self.assertAlmostEqual(a, b, places=6)

    def test_translation_is_applied(self):
        r = quaternion_to_rotation_matrix(0.0, 0.0, 0.0, 1.0)
        self.assertEqual(
            rotate_point(r, 1.0, 2.0, 3.0, tx=10.0, ty=-5.0, tz=1.0),
            (11.0, -3.0, 4.0))


if __name__ == '__main__':
    unittest.main()
