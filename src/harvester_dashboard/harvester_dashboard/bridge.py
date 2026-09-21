"""QObject bridge exposing dashboard state and controls to QML.

Everything QML touches lives here as Qt properties/slots so the views
stay declarative.  The bridge deliberately provides **no** slot that
writes to any telemetry socket: view switching is render-only, and
maintenance controls exist but are hidden unless the status source
reports hardware mode (and even then remain inert until a control
endpoint is defined).
"""

from __future__ import annotations

import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

try:
    from PySide2.QtCore import Property, QObject, QTimer, Signal, Slot
    _QT_AVAILABLE = True
except ImportError:  # pragma: no cover - headless pure-python tests
    _QT_AVAILABLE = False
    QObject = object

from .config import DashboardConfig
from .model.telemetry_model import TelemetryModel
from .model.target_model import AnnotationState
from .safety_guidance import (
    DANGER, NO_DATA, Guidance, SafetyConfig, default_config_path, evaluate,
    state_message)
from .status_client import StatusClient


if _QT_AVAILABLE:

    class DashboardBridge(QObject):
        """Single context property ``bridge`` for Dashboard.qml."""

        # --- QML-bound notifications ---------------------------------------
        view_changed = Signal()
        hud_visible_changed = Signal()
        lidar_visible_changed = Signal()
        source_badge_changed = Signal()
        ranges_changed = Signal()
        trunk_changed = Signal()
        calibration_changed = Signal()
        stream_rows_changed = Signal()
        annotation_changed = Signal()
        toast_changed = Signal()
        status_summary_changed = Signal()
        maintenance_changed = Signal()
        lidar_points_changed = Signal()
        lidar_view_changed = Signal()
        frame_tick = Signal()
        dock_state_changed = Signal()
        dock_plan_changed = Signal()
        dock_speed_changed = Signal()
        dock_safety_changed = Signal()

        STATUS_TIMEOUT_MS = 600

        def __init__(self, config: DashboardConfig, model: TelemetryModel,
                     annotation: AnnotationState, annotation_publisher=None,
                     dock_publisher=None, parent=None):
            super().__init__(parent)
            self.config = config
            self.model = model
            self.annotation = annotation
            self.annotation_publisher = annotation_publisher
            self.dock_publisher = dock_publisher
            self.status_client = (
                StatusClient(config.status_endpoint,
                             timeout_ms=self.STATUS_TIMEOUT_MS)
                if config.status_enabled else None)
            self._view = 'cutter'
            self._hud_visible = True
            self._lidar_visible = True
            self._lidar_view_index = 0
            self._frame_counters = {'cutter': 0, 'docking': 0}
            self._depth_counters = {'cutter': 0, 'docking': 0}
            self._latest_frames: Dict[str, Any] = {}
            self._toast = ''
            self._toast_until = 0.0
            self._last_status_response: Optional[Dict[str, Any]] = None
            self._maintenance_mode = 'unknown'
            self._lidar_points: List[List[float]] = []
            self._dock_state = 'SWEEP'
            self._dock_plan: Dict[str, Any] = {}
            # Rolling (timestamp_ns, distance_m) samples of the center range
            # sensor, used to estimate approach speed toward the trunk.
            self._center_range_samples: Deque[Tuple[int, float]] = deque()
            self._dock_speed_cm_s: Optional[float] = None
            self._dock_center_distance_m: Optional[float] = None
            # Monotonic receipt time of the latest valid center-range record,
            # used to flag the range stream as stale (NO_DATA) in the HUD.
            self._center_range_recv_monotonic: Optional[float] = None
            # Smoothed closing speed (EMA) + safety-guidance state.
            self._dock_speed_smoothed: Optional[float] = None
            self._safety_config = SafetyConfig.load(default_config_path())
            self._guidance: Guidance = Guidance(
                'no_data', None, 0.0, None, None, None, None,
                'approach: awaiting center range')
            # Hysteresis debounce: track the last guidance state and the
            # monotonic time it was first entered so a boundary crossing must
            # persist for ``debounce_s`` before the HUD changes colour.
            self._guidance_state = 'no_data'
            self._guidance_state_since = 0.0
            self._refresh = QTimer(self)
            self._refresh.timeout.connect(self.refresh)
            self._refresh.start(200)
            if self.status_client is not None:
                self._status_timer = QTimer(self)
                self._status_timer.timeout.connect(self.poll_status)
                self._status_timer.start(int(config.status_interval_s * 1000))
                QTimer.singleShot(0, self.poll_status)

        # =================================================================
        # View switching — render-only by construction: no socket write.
        # =================================================================
        @Slot(str)
        def set_view(self, view: str) -> None:
            if view in ('cutter', 'docking') and view != self._view:
                self._view = view
                self.view_changed.emit()

        def _get_view(self) -> str:
            return self._view

        @Slot()
        def toggle_hud(self) -> None:
            self._hud_visible = not self._hud_visible
            self.hud_visible_changed.emit()

        def _get_hud_visible(self) -> bool:
            return self._hud_visible

        @Slot()
        def toggle_lidar(self) -> None:
            self._lidar_visible = not self._lidar_visible
            self.lidar_visible_changed.emit()

        def _get_lidar_visible(self) -> bool:
            return self._lidar_visible

        # =================================================================
        # LiDAR view cycling — render-only, cycles on key 5.
        # =================================================================
        # Ordered projection modes: top-down, front, left, right, isometric.
        _LIDAR_VIEWS = ('top', 'front', 'left', 'right', 'iso')

        @Slot()
        def cycle_lidar_view(self) -> None:
            self._lidar_view_index = (
                self._lidar_view_index + 1) % len(self._LIDAR_VIEWS)
            self.lidar_view_changed.emit()

        def _get_lidar_view(self) -> str:
            return self._LIDAR_VIEWS[self._lidar_view_index]

        def _get_lidar_view_label(self) -> str:
            return {
                'top': 'top-down (x-y)',
                'front': 'front (x-z)',
                'left': 'left (y-z)',
                'right': 'right (y-z)',
                'iso': 'isometric',
            }.get(self._LIDAR_VIEWS[self._lidar_view_index], '')

        # =================================================================
        # Frame ingestion hook (UI thread; called from TelemetrySource)
        # =================================================================
        def on_frame_decoded(self, channel: str, decoded) -> None:
            self._latest_frames[channel] = decoded
            if channel.endswith('/rgb'):
                camera = 'cutter' if '/cutter/' in channel else 'docking'
                self._frame_counters[camera] += 1
            elif channel.endswith('/depth'):
                camera = 'cutter' if '/cutter/' in channel else 'docking'
                self._depth_counters[camera] += 1
            elif channel == 'v1/lidar/raw':
                self._set_lidar_points(decoded)
            elif channel == 'v1/docking/plan':
                self._on_dock_plan(decoded)
            self.frame_tick.emit()

        _LIDAR_HUD_SIZE_M = 8.0  # mirror of LidarInset.qml range_limit_m

        def _set_lidar_points(self, points) -> None:
            try:
                from .decoders.lidar_decoder import LidarDecoder
                limited = LidarDecoder().limit(
                    points, self.config.lidar_max_points)
            except Exception:
                limited = points
            self._lidar_points = (
                limited.tolist() if limited is not None else [])
            self.lidar_points_changed.emit()

        @Slot(float, float, float, str, result='QVariantList')
        def project_lidar_point(self, x: float, y: float, z: float,
                                view: str) -> list:
            """Project one LiDAR point to screen coords for the given view.

            Returns ``[screen_x, screen_y]`` with the HUD origin (0, 0)
            in the top-left corner, matching the Canvas draw coordinate
            system used by ``LidarInset.qml``.  The caller supplies ``cx``,
            ``cy``, and ``scale`` so the projection can be reused at any
            HUD size.
            """
            from .projection import project_points
            return [project_points([[x, y, z]], view, 0, 0, 1)[0][:2]]

        @Slot(float, float, float, result='QVariantList')
        def project_lidar_point_current(self, x: float, y: float, z: float) -> list:
            """Project one point using the current LiDAR view."""
            return self.project_lidar_point(x, y, z, self._get_lidar_view())

        def latest_rgb(self, camera: str):
            return self._latest_frames.get(
                'v1/camera/{}/rgb'.format(camera))

        def latest_depth(self, camera: str):
            return self._latest_frames.get(
                'v1/camera/{}/depth'.format(camera))

        # =================================================================
        # Annotation
        # =================================================================
        @Slot(int, int)
        def annotate_click(self, u: int, v: int) -> None:
            camera = self._view
            depth = self.latest_depth(camera)
            camera_info = self.model.snapshot_camera_info(camera)
            header = self.model.state(
                'v1/camera/{}/rgb'.format(camera)).last_header
            frame_id = (header or {}).get('frame_id', '')
            if depth is None:
                depth_m = None
            else:
                from .decoders.depth_decoder import DepthDecoder
                depth_m = DepthDecoder().depth_at(
                    depth, u, v, window=self.config.annotation_depth_window_px)
            _accepted, message = self.annotation.build(
                camera, u, v, depth_m, camera_info, frame_id)
            self._forward_annotation('created')
            self._toast_message(message)
            self.annotation_changed.emit()

        @Slot()
        def clear_annotation(self) -> None:
            had = self.annotation.active
            self.annotation.clear(reason='operator')
            if had:
                self._forward_annotation('cleared')
            self.annotation_changed.emit()

        # =================================================================
        # DOCK button — out-of-contract command forwarder
        # =================================================================
        @Slot()
        def dock_pressed(self) -> None:
            """Forward one DOCK press on the out-of-contract command PUB.

            The orchestrator owns the FSM and advances one step per press; this
            client sends only the one-bit ``{"action": "dock"}`` request.  When
            the publisher is disabled (default), it only toasts a notice.
            """
            if self.dock_publisher is not None and self.dock_publisher.enabled:
                ok = self.dock_publisher.publish_dock()
                self._toast_message(
                    'DOCK press sent' if ok else 'DOCK publish failed')
            else:
                self._toast_message(
                    'docking command endpoint not configured (--dock-pub)')

        def _on_dock_plan(self, payload: Dict[str, Any]) -> None:
            self._dock_plan = payload if isinstance(payload, dict) else {}
            state = self._dock_plan.get('state', 'SWEEP')
            if state != self._dock_state:
                self._dock_state = state
                self.dock_state_changed.emit()
            self.dock_plan_changed.emit()

        def _get_dock_state(self) -> str:
            return self._dock_state

        def _get_dock_plan_line(self) -> str:
            if not self._dock_plan:
                return 'dock: idle'
            p = self._dock_plan
            state = p.get('state', '?')
            if p.get('reachable') is False:
                return ('dock: {} — MOVE PRIME MOVER{}'.format(
                    state,
                    (' +{:.2f} m'.format(p['needed_advance_m'])
                     if p.get('needed_advance_m') else '')))
            parts = ['dock: {}'.format(state)]
            if p.get('tree_height_m') is not None:
                parts.append('H={:.2f}'.format(p['tree_height_m']))
            if p.get('distance_m') is not None:
                parts.append('d={:.2f}'.format(p['distance_m']))
            if p.get('boom_angle_deg') is not None:
                parts.append('theta={:.1f}'.format(p['boom_angle_deg']))
            if p.get('boom_extension_m') is not None:
                parts.append('e={:.2f}'.format(p['boom_extension_m']))
            return '  '.join(parts)

        def _forward_annotation(self, action: str) -> None:
            if self.annotation_publisher is not None:
                self.annotation_publisher.publish_annotation(
                    self.annotation, action=action)

        def _get_annotation_active(self) -> bool:
            return self.annotation.active

        def _get_annotation_label(self) -> str:
            return self.annotation.label()

        def _get_annotation_camera(self) -> str:
            return self.annotation.camera

        def _get_annotation_u(self) -> int:
            return self.annotation.pixel[0]

        def _get_annotation_v(self) -> int:
            return self.annotation.pixel[1]

        # =================================================================
        # Docking approach speed (center range derivative)
        # =================================================================
        # Max sample age (seconds): samples older than this are dropped from the
        # slope estimate, so a stale speed reading never lingers on screen.
        _SPEED_MAX_SAMPLE_AGE_S = 1.5

        def _update_dock_speed(self) -> None:
            """Estimate approach speed (cm/s) from the center range sensor.

            Speed = -d(distance)/dt: a *decreasing* center range means the
            platform is closing on the trunk, reported as a positive value.
            Uses a least-squares slope over the last ~1 s of samples so a
            single noisy reading doesn't thrash the readout.  Returns ``None``
            (no measurement) when there aren't enough fresh valid samples.
            """
            records, _cutter = self.model.snapshot_ranges()
            center = None
            for record in (records or []):
                if isinstance(record, dict) and \
                        record.get('telemetry_key') == 'center_line':
                    center = record
                    break
            if center is None or not center.get('valid'):
                # No (or invalid) center range: keep the last value briefly,
                # but age it out so stale speed doesn't linger.
                self._prune_center_samples()
                if not self._center_range_samples:
                    self._set_dock_speed(None)
                    self._set_dock_center_distance(None)
                return

            ts_ns = center.get('acquisition_timestamp_ns')
            distance_m = float(center['distance_m'])
            self._set_dock_center_distance(distance_m)
            self._center_range_recv_monotonic = time.monotonic()
            if ts_ns is None:
                self._set_dock_speed(None)
                return

            # Drop samples with timestamps that move backwards or repeat.
            if not self._center_range_samples or \
                    ts_ns > self._center_range_samples[-1][0]:
                self._center_range_samples.append((int(ts_ns), distance_m))
            self._prune_center_samples()

            if len(self._center_range_samples) < 2:
                self._set_dock_speed(None)
                return

            # Least-squares slope of distance (m) vs time (s) over the window.
            first_ts = self._center_range_samples[0][0]
            n = len(self._center_range_samples)
            sum_t = 0.0
            sum_d = 0.0
            sum_tt = 0.0
            sum_td = 0.0
            for ts, d in self._center_range_samples:
                t = (ts - first_ts) * 1e-9
                sum_t += t
                sum_d += d
                sum_tt += t * t
                sum_td += t * d
            denom = (n * sum_tt) - (sum_t * sum_t)
            if denom <= 0.0:
                self._set_dock_speed(None)
                return
            slope_m_per_s = ((n * sum_td) - (sum_t * sum_d)) / denom

            # Closing on trunk (distance shrinking) => positive speed.
            self._set_dock_speed(-slope_m_per_s * 100.0)

        def _prune_center_samples(self) -> None:
            """Drop center-range samples older than the tracking window."""
            if not self._center_range_samples:
                return
            newest_ns = self._center_range_samples[-1][0]
            cutoff_ns = newest_ns - int(self._SPEED_MAX_SAMPLE_AGE_S * 1e9)
            while self._center_range_samples and \
                    self._center_range_samples[0][0] < cutoff_ns:
                self._center_range_samples.popleft()

        def _set_dock_speed(self, value: Optional[float]) -> None:
            changed = value != self._dock_speed_cm_s
            self._dock_speed_cm_s = value
            if changed:
                self.dock_speed_changed.emit()

        def _get_dock_speed(self) -> float:
            # QML needs a number; return NaN when there is no measurement so
            # the view can render an explicit "—".
            if self._dock_speed_cm_s is None:
                return float('nan')
            return self._dock_speed_cm_s

        def _set_dock_center_distance(self, value: Optional[float]) -> None:
            changed = value != self._dock_center_distance_m
            self._dock_center_distance_m = value
            if changed:
                self.dock_speed_changed.emit()

        def _get_dock_center_distance(self) -> float:
            if self._dock_center_distance_m is None:
                return float('nan')
            return self._dock_center_distance_m

        # =================================================================
        # Safety guidance (stopping-distance + TTC — see safety_guidance.py)
        # =================================================================
        def _recompute_safety_guidance(self) -> None:
            """EMA-smooth the raw speed, then map (speed, distance) -> state.

            Staleness (range older than ``stale_s``) and hysteresis debounce are
            applied here: the raw evaluation is stateless, but the *displayed*
            state only changes after a candidate state has persisted for
            ``debounce_s`` (anti-flicker), and falls back to NO_DATA when the
            range stream is absent or stale.
            """
            raw = self._dock_speed_cm_s
            if raw is None:
                self._dock_speed_smoothed = None
            elif self._dock_speed_smoothed is None:
                self._dock_speed_smoothed = raw
            else:
                alpha = self._safety_config.speed_ema_alpha
                self._dock_speed_smoothed = (
                    alpha * raw + (1.0 - alpha) * self._dock_speed_smoothed)

            stale = False
            if self._center_range_recv_monotonic is not None:
                stale = (time.monotonic() - self._center_range_recv_monotonic
                         ) > self._safety_config.stale_s
            elif self._dock_center_distance_m is not None:
                stale = True

            candidate = evaluate(
                self._dock_speed_smoothed,
                self._dock_center_distance_m,
                self._safety_config,
                stale=stale)

            # Hysteresis: only adopt a *new* state after it has persisted for
            # ``debounce_s``; NO_DATA / DANGER are adopted immediately (a loss
            # of telemetry or a collision warning must never be delayed).
            now = time.monotonic()
            if candidate.state != self._guidance_state:
                if (candidate.state in (NO_DATA, DANGER)
                        or now - self._guidance_state_since
                        >= self._safety_config.debounce_s):
                    self._guidance_state = candidate.state
                    self._guidance_state_since = now
            # Recompute the displayed guidance from the (possibly held) state
            # and the candidate's measurements.  When the state is held (still
            # debouncing toward a new state), the message must match the held
            # state, not the candidate's, so the HUD text and colour never
            # disagree.
            message = candidate.message
            if candidate.state != self._guidance_state:
                message = state_message(self._guidance_state, candidate)
            guidance = Guidance(
                self._guidance_state,
                candidate.ttc_s,
                candidate.speed_cm_s,
                candidate.distance_m,
                candidate.stop_distance_m,
                candidate.max_speed_cm_s,
                candidate.recommended_speed_cm_s,
                message)
            changed = guidance != self._guidance
            self._guidance = guidance
            if changed:
                self.dock_safety_changed.emit()

        def _get_dock_safety_state(self) -> str:
            return self._guidance.state

        def _get_dock_guidance_text(self) -> str:
            return self._guidance.message

        def _get_dock_recommended_speed(self) -> float:
            if self._guidance.recommended_speed_cm_s is None:
                return float('nan')
            return self._guidance.recommended_speed_cm_s

        def _get_dock_max_speed(self) -> float:
            if self._guidance.max_speed_cm_s is None:
                return float('nan')
            return self._guidance.max_speed_cm_s

        def _get_dock_stop_distance(self) -> float:
            if self._guidance.stop_distance_m is None:
                return float('nan')
            return self._guidance.stop_distance_m

        def _get_dock_ttc(self) -> float:
            if self._guidance.ttc_s is None:
                return float('nan')
            return self._guidance.ttc_s

        def _get_dock_speed_smoothed(self) -> float:
            if self._dock_speed_smoothed is None:
                return float('nan')
            return self._dock_speed_smoothed

        # =================================================================
        # Periodic refresh — recompute derived display state
        # =================================================================
        @Slot()
        def refresh(self) -> None:
            self.source_badge_changed.emit()
            self.ranges_changed.emit()
            self.trunk_changed.emit()
            self.calibration_changed.emit()
            self.stream_rows_changed.emit()
            self.status_summary_changed.emit()
            self.frame_tick.emit()
            self._update_dock_speed()
            self._recompute_safety_guidance()
            if self._toast and time.monotonic() > self._toast_until:
                self._toast = ''
                self.toast_changed.emit()

        # -- source badge ---------------------------------------------------
        def _get_source_badge(self) -> str:
            mode = self.model.source_mode()
            ids = self.model.source_ids()
            return '{} {}'.format(mode, ids).strip()

        def _get_source_mixed(self) -> bool:
            return self.model.is_mixed()

        def _get_capabilities_line(self) -> str:
            caps = self.model.latest_capabilities()
            notable = [name for name in sorted(caps) if not caps[name]]
            if 'target.world_fixed' in caps:
                world = 'target.world_fixed={} '.format(
                    caps['target.world_fixed'])
            else:
                world = ''
            return world + 'off-capabilities: ' + (
                ', '.join(notable) if notable else 'none')

        # -- freshness of the active camera ---------------------------------
        def _active_rgb_channel(self) -> str:
            return 'v1/camera/{}/rgb'.format(self._view)

        def _get_active_camera_stale(self) -> bool:
            return self.model.state(self._active_rgb_channel()).is_stale(
                time.monotonic(), self.config.stale_after_s)

        def _get_active_timestamp_line(self) -> str:
            state = self.model.state(self._active_rgb_channel())
            header = state.last_header
            if header is None:
                return 'camera {}: no packet yet'.format(self._view)
            return '{}: {} ({}) seq {}'.format(
                self._view,
                header.get('acquisition_timestamp_ns', '?'),
                header.get('clock_domain', '?'),
                header.get('sequence', '?'))

        @Slot(str, result=bool)
        def stream_stale(self, channel: str) -> bool:
            return self.model.state(channel).is_stale(
                time.monotonic(), self.config.stale_after_s)

        @Slot(str, result=str)
        def stream_age_line(self, channel: str) -> str:
            age = self.model.state(channel).age_s(time.monotonic())
            return '—' if age is None else '{:.1f}s'.format(age)

        # -- ranges -----------------------------------------------------------
        def _get_docking_range_rows(self):
            records, _cutter = self.model.snapshot_ranges()
            rows = []
            for record in (records or []):
                if not isinstance(record, dict):
                    continue
                rows.append({
                    'key': str(record.get('telemetry_key', '?')),
                    'distance': (None if record.get('distance_m') is None
                                 else float(record['distance_m'])),
                    'valid': bool(record.get('valid', False)),
                })
            return rows

        def _get_cutter_range_line(self) -> str:
            _records, cutter = self.model.snapshot_ranges()
            if not cutter:
                return 'cutter: —'
            distance = cutter.get('distance_m')
            if distance is None:
                return 'cutter: INVALID'
            return 'cutter: {:.2f} m'.format(float(distance))

        # -- trunk / calibration ---------------------------------------------
        def _get_trunk_line(self) -> str:
            trunk = self.model.snapshot_trunk()
            if not isinstance(trunk, dict):
                return 'trunk: —'
            position = ((trunk.get('pose') or {}).get('position') or {})
            return 'trunk: ({:+.2f}, {:+.2f}, {:+.2f}) m'.format(
                float(position.get('x', 0.0)),
                float(position.get('y', 0.0)),
                float(position.get('z', 0.0)))

        def _get_calibration_line(self) -> str:
            payload, valid = self.model.snapshot_calibration()
            if payload is None:
                return 'calibration: —'
            status = payload.get('status', '?') if isinstance(payload, dict) else '?'
            calibration_id = (
                payload.get('calibration_id', '')
                if isinstance(payload, dict) else '')
            flag = 'VALID' if valid else 'UNCONFIRMED'
            return 'calibration: {} [{}] {}'.format(
                status, flag, calibration_id)

        # -- stream rows / errors panel ----------------------------------------
        def _get_stream_rows(self):
            rows = self.model.summary_rows(
                stale_after_s=self.config.stale_after_s)
            rendered = []
            for row in rows:
                age = row['age_s']
                rendered.append({
                    'channel': row['channel'],
                    'age': '—' if age is None else '{:.1f}s'.format(age),
                    'stale': row['stale'],
                    'ever_seen': row['ever_seen'],
                    'gaps': row['sequence_gaps'],
                    'drops': row['drops'],
                    'decode_errors': row['decode_errors'],
                    'error': row['last_error'],
                })
            return rendered

        def _get_received_packets(self) -> int:
            return int(getattr(self.model, 'source_received', 0))

        def _get_dropped_packets(self) -> int:
            local = sum(state.drops for state in self.model.states())
            reported = int(getattr(self.model, 'source_dropped', 0))
            return max(local, reported)

        # -- image counters -----------------------------------------------------
        def _get_frame_counter(self) -> int:
            return self._frame_counters.get(self._view, 0)

        def _get_depth_counter(self) -> int:
            return self._depth_counters.get(self._view, 0)

        # -- LiDAR points ---------------------------------------------------------
        def _get_lidar_points(self):
            return self._lidar_points

        # -- status REP ------------------------------------------------------------
        def poll_status(self) -> None:
            if self.status_client is None:
                return
            response = self.status_client.query()
            self._last_status_response = response
            mode = StatusClient.source_mode_of(response)
            if mode != self._maintenance_mode:
                self._maintenance_mode = mode
                self.maintenance_changed.emit()
            self.status_summary_changed.emit()

        def _get_status_line(self) -> str:
            if self.status_client is None:
                return 'status: disabled (replay)'
            response = self._last_status_response
            if response is None:
                return 'status: unreachable'
            drops = response.get('dropped_packets') or {}
            total_drops = sum(
                value if isinstance(value, int) else 0
                for value in drops.values())
            recording = (response.get('recording') or {}).get('enabled', False)
            return 'status: {} | drops {} | rec {}'.format(
                response.get('active_profile', '?'), total_drops,
                'on' if recording else 'off')

        def _get_maintenance_available(self) -> bool:
            return self._maintenance_mode == 'hardware'

        def _get_maintenance_mode(self) -> str:
            return self._maintenance_mode

        # Maintenance actions are intentionally inert: Phase 1 defines no
        # control endpoint client.  They never touch a socket.
        @Slot(str)
        def request_stream_toggle(self, channel: str) -> None:
            self._toast_message(
                'maintenance control endpoint not configured '
                '(source mode: {})'.format(self._maintenance_mode))

        # -- toast -------------------------------------------------------------------
        def _toast_message(self, message: str) -> None:
            self._toast = str(message)
            self._toast_until = time.monotonic() + 4.0
            self.toast_changed.emit()

        def _get_toast(self) -> str:
            return self._toast

        # =====================================================================
        # QML property plumbing
        # =====================================================================
        view = Property(str, _get_view, set_view, notify=view_changed)
        hudVisible = Property(
            bool, _get_hud_visible, notify=hud_visible_changed)
        lidarVisible = Property(
            bool, _get_lidar_visible, notify=lidar_visible_changed)
        sourceBadge = Property(
            str, _get_source_badge, notify=source_badge_changed)
        sourceMixed = Property(
            bool, _get_source_mixed, notify=source_badge_changed)
        capabilitiesLine = Property(
            str, _get_capabilities_line, notify=source_badge_changed)
        activeCameraStale = Property(
            bool, _get_active_camera_stale, notify=frame_tick)
        activeTimestampLine = Property(
            str, _get_active_timestamp_line, notify=frame_tick)
        dockingRangeRows = Property(
            'QVariantList', _get_docking_range_rows, notify=ranges_changed)
        cutterRangeLine = Property(
            str, _get_cutter_range_line, notify=ranges_changed)
        trunkLine = Property(str, _get_trunk_line, notify=trunk_changed)
        calibrationLine = Property(
            str, _get_calibration_line, notify=calibration_changed)
        streamRows = Property(
            'QVariantList', _get_stream_rows, notify=stream_rows_changed)
        receivedPackets = Property(
            int, _get_received_packets, notify=stream_rows_changed)
        droppedPackets = Property(
            int, _get_dropped_packets, notify=stream_rows_changed)
        frameCounter = Property(int, _get_frame_counter, notify=frame_tick)
        depthCounter = Property(int, _get_depth_counter, notify=frame_tick)
        lidarPoints = Property(
            'QVariantList', _get_lidar_points, notify=lidar_points_changed)
        lidarView = Property(
            str, _get_lidar_view, notify=lidar_view_changed)
        lidarViewLabel = Property(
            str, _get_lidar_view_label, notify=lidar_view_changed)
        statusLine = Property(
            str, _get_status_line, notify=status_summary_changed)
        maintenanceAvailable = Property(
            bool, _get_maintenance_available, notify=maintenance_changed)
        maintenanceMode = Property(
            str, _get_maintenance_mode, notify=maintenance_changed)
        annotationActive = Property(
            bool, _get_annotation_active, notify=annotation_changed)
        annotationLabel = Property(
            str, _get_annotation_label, notify=annotation_changed)
        annotationCamera = Property(
            str, _get_annotation_camera, notify=annotation_changed)
        annotationU = Property(
            int, _get_annotation_u, notify=annotation_changed)
        annotationV = Property(
            int, _get_annotation_v, notify=annotation_changed)
        toast = Property(str, _get_toast, notify=toast_changed)
        dockState = Property(
            str, _get_dock_state, notify=dock_state_changed)
        dockPlanLine = Property(
            str, _get_dock_plan_line, notify=dock_plan_changed)
        dockSpeedCmS = Property(
            float, _get_dock_speed, notify=dock_speed_changed)
        dockCenterDistanceM = Property(
            float, _get_dock_center_distance, notify=dock_speed_changed)
        dockSafetyState = Property(
            str, _get_dock_safety_state, notify=dock_safety_changed)
        dockGuidanceText = Property(
            str, _get_dock_guidance_text, notify=dock_safety_changed)
        dockRecommendedSpeedCmS = Property(
            float, _get_dock_recommended_speed, notify=dock_safety_changed)
        dockMaxSpeedCmS = Property(
            float, _get_dock_max_speed, notify=dock_safety_changed)
        dockStopDistanceM = Property(
            float, _get_dock_stop_distance, notify=dock_safety_changed)
        dockTtcS = Property(
            float, _get_dock_ttc, notify=dock_safety_changed)
        dockSpeedSmoothedCmS = Property(
            float, _get_dock_speed_smoothed, notify=dock_safety_changed)


__all__ = ['DashboardBridge', '_QT_AVAILABLE']
