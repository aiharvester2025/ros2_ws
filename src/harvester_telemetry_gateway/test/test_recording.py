import tempfile
import unittest

from harvester_telemetry_contract import pack_message
from harvester_telemetry_gateway.recording import PacketRecorder, iter_recordings


def image_header(sequence):
    return {
        'schema_version': 1,
        'source_mode': 'simulation',
        'source_id': 'test',
        'sequence': sequence,
        'frame_id': 'camera_frame',
        'acquisition_timestamp_ns': 1,
        'clock_domain': 'ros_sim_time',
        'gateway_monotonic_ns': sequence,
        'calibration_id': 'test_calibration',
        'capabilities': {'packet.recording': True},
        'codec': 'jpeg',
        'pixel_encoding': 'RGB8',
        'width': 1,
        'height': 1,
    }


class RecordingTest(unittest.TestCase):
    def test_recording_preserves_exact_three_frames(self):
        frames = pack_message('v1/camera/cutter/rgb', image_header(1), b'jpeg-data')
        with tempfile.TemporaryDirectory() as directory:
            recorder = PacketRecorder(directory)
            recorder.write(frames)
            records = list(iter_recordings(directory))
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0][1], frames)


if __name__ == '__main__':
    unittest.main()
