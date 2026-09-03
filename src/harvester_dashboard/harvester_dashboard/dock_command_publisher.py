"""Optional out-of-contract docking command forwarder.

Disabled by default.  When enabled via ``--dock-pub`` it binds a **separate**
PUB endpoint (suggested ``tcp://127.0.0.1:5593``) and publishes a single
``v1/operator/dock_request`` packet (``{"action": "dock"}``) each time the DOCK
button is pressed.  It never binds or writes the canonical 5590/5600 endpoints
and never carries a joint/velocity value — it is a one-bit "advance the docking
state machine" request, not a motion command.
"""

from __future__ import annotations

import json
import time

from .protocol_shim import ProtocolError, pack_message


class DockCommandPublisher:
    """Publishes DOCK button presses on a dedicated, non-canonical PUB."""

    def __init__(self, endpoint: str = '', source_id: str = 'dashboard',
                 context=None):
        self.endpoint = endpoint
        self.source_id = source_id
        self.sequence = 0
        self.socket = None
        self.sent = 0
        self.errors = 0
        self.last_error = ''
        self._owns_context = False
        if endpoint:
            import zmq
            if context is None:
                self._context = zmq.Context.instance()
                self._owns_context = True
            else:
                self._context = context
            self.socket = self._context.socket(zmq.PUB)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.bind(endpoint)

    @property
    def enabled(self) -> bool:
        return self.socket is not None

    def publish_dock(self) -> bool:
        """Forward one DOCK press; returns True when sent."""
        if not self.enabled:
            return False
        self.sequence += 1
        payload = {'action': 'dock'}
        header = {
            'schema_version': 1,
            'source_mode': 'simulation',
            'source_id': self.source_id,
            'sequence': self.sequence,
            'frame_id': 'world',
            'acquisition_timestamp_ns': time.time_ns(),
            'clock_domain': 'utc_host',
            'gateway_monotonic_ns': time.monotonic_ns(),
            'calibration_id': 'none',
            'codec': 'json',
            'capabilities': {'dock.autonomous': True},
        }
        try:
            frames = pack_message(
                'v1/operator/dock_request', header,
                json.dumps(payload, separators=(',', ':')).encode('utf-8'))
        except ProtocolError as error:
            self.errors += 1
            self.last_error = str(error)
            return False
        try:
            self.socket.send_multipart(frames, flags=1)  # zmq.NOBLOCK == 1
            self.sent += 1
            return True
        except Exception as error:
            self.errors += 1
            self.last_error = str(error)
            return False

    def close(self) -> None:
        if self.socket is not None:
            try:
                self.socket.close(0)
            except Exception:
                pass
            self.socket = None


__all__ = ['DockCommandPublisher']
