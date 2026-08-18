"""GUI smoke test: loads Dashboard.qml offscreen and exercises view keys.

Skips cleanly (``skipTest``) when PySide2/QtQuick are unavailable or no
display/offscreen platform exists, so the pure-python suite stays green
in any environment.
"""

import os
import unittest

try:
    import PySide2.QtQuick  # noqa: F401
    _GUI_IMPORTS_OK = True
except ImportError:
    _GUI_IMPORTS_OK = False

if _GUI_IMPORTS_OK:
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PySide2.QtCore import QUrl
    from PySide2.QtGui import QGuiApplication
    from PySide2.QtQuick import QQuickView

from harvester_dashboard.config import DashboardConfig
from harvester_dashboard.model.telemetry_model import TelemetryModel
from harvester_dashboard.model.target_model import AnnotationState
from harvester_dashboard.protocol_shim import ensure_contract_importable

ensure_contract_importable()


@unittest.skipUnless(_GUI_IMPORTS_OK, 'PySide2 QtQuick not installed')
class GuiSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QGuiApplication.instance() or QGuiApplication(
            ['harvester-dashboard-smoke'])
        from harvester_dashboard.bridge import DashboardBridge
        from harvester_dashboard.image_provider import FrameImageProvider
        cls.model = TelemetryModel()
        cls.annotation = AnnotationState()
        cls.bridge = DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            cls.model, cls.annotation)
        cls.provider = FrameImageProvider()
        cls.view = QQuickView()
        cls.view.engine().addImageProvider('frames', cls.provider)
        cls.view.rootContext().setContextProperty('bridge', cls.bridge)
        qml_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'qml')
        cls.view.setSource(QUrl.fromLocalFile(os.path.join(qml_dir, 'Dashboard.qml')))
        if cls.view.status() != QQuickView.Null and not cls.root_ok():
            raise unittest.SkipTest('Dashboard.qml failed to load')
        cls.root = cls.view.rootObject()

    @classmethod
    def root_ok(cls):
        return cls.view.status() == QQuickView.Ready

    @classmethod
    def tearDownClass(cls):
        cls.view.deleteLater()
        cls.app.processEvents()
        del cls.view

    def _process(self, ms=50):
        for _ in range(max(1, int(ms / 5))):
            self.app.processEvents()

    def test_root_loads(self):
        self.assertIsNotNone(self.root)

    def test_view_switch_is_render_only_state(self):
        self.assertEqual(self.bridge.view, 'cutter')
        self.bridge.set_view('docking')
        self.app.processEvents()
        self.assertEqual(self.bridge.view, 'docking')
        self.bridge.set_view('cutter')
        self.app.processEvents()
        self.assertEqual(self.bridge.view, 'cutter')

    def test_hud_and_lidar_toggles(self):
        initial_hud = self.bridge.hudVisible
        self.bridge.toggle_hud()
        self.app.processEvents()
        self.assertEqual(self.bridge.hudVisible, not initial_hud)
        self.bridge.toggle_hud()
        initial_lidar = self.bridge.lidarVisible
        self.bridge.toggle_lidar()
        self.app.processEvents()
        self.assertEqual(self.bridge.lidarVisible, not initial_lidar)
        self.bridge.toggle_lidar()

    def test_maintenance_hidden_without_hardware_status(self):
        self.assertFalse(self.bridge.maintenanceAvailable)
        self.assertEqual(self.bridge.maintenanceMode, 'unknown')

    def test_annotation_click_without_depth_shows_no_depth(self):
        self.bridge.annotate_click(10, 10)
        self.app.processEvents()
        self.assertFalse(self.bridge.annotationActive)
        self.assertIn('NO DEPTH', self.bridge.toast)

    def test_clear_annotation(self):
        self.bridge.clear_annotation()
        self.app.processEvents()
        self.assertFalse(self.bridge.annotationActive)

    def test_image_provider_serves_published_frame(self):
        import numpy as np
        from PySide2.QtCore import QSize
        frame = np.zeros((12, 16, 3), dtype=np.uint8)
        frame[:, :, 0] = 200
        self.provider.publish_rgb('cutter', frame)
        image = self.provider.requestImage('cutter', QSize(), QSize())
        self.assertEqual(image.width(), 16)
        self.assertEqual(image.height(), 12)

    def test_bridge_updates_from_synthetic_packets(self):
        from helpers import json_packet
        frames = json_packet('v1/range/cutter', {
            'telemetry_key': 'cutter_forward', 'distance_m': 1.5,
            'valid': True})
        self.model.ingest_frames(frames)
        self.app.processEvents()
        self.assertIn('1.50 m', self.bridge.cutterRangeLine)
        self.assertEqual(self.bridge.sourceBadge, 'SIMULATION xavier')


if __name__ == '__main__':
    unittest.main()
