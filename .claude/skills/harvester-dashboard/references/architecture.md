# Dashboard architecture + data model

Source of truth: `src/harvester_dashboard/**`, `docs/canonical_zmq_v1.md`.

## Principle: source-agnostic, ROS-free

The dashboard subscribes **only** to canonical ZeroMQ. It contains no ROS 2 assumptions, no TF, no
rclpy. On Xavier the source is the gateway (replay or live); on Orin the source is the hardware
adapter publishing the same wire format. This is what makes the dashboard portable — the plan
deliberately skips the Orin adapter and builds the dashboard on Xavier first, because the data source
is agnostic.

## Components

- `config.py` — endpoint/port/display config.
- `protocol_shim.py` — imports `harvester_telemetry_contract` (a colcon package) via `sys.path`
  insertion to `src/harvester_telemetry_contract`, or vendors the pure `protocol.py` (zero deps
  beyond msgpack).
- `decoders/` — mirror-image of the gateway's encoders: `jpeg_decoder.py` (PIL),
  `h264_decoder.py` / `h265_decoder.py` (stubs raising on `h264|h265`), `lidar_decoder.py`.
- `bridge.py` / `projection.py` — ZMQ → UI data model and LiDAR point projection.
- `qml/` — views: main viewport (cutter/docking), `SensorPanel.qml`, `LidarInset.qml`, touch buttons.

## Controls

- `1`: cutter camera view.
- `2`: docking camera view.
- `3`: toggle sensor HUD.
- `4`: toggle LiDAR inset/overlay.
- `5`: cycle LiDAR projection view (top-down → front → left → right → isometric).
- `0` / Esc: clear the selected target.
- click on camera: annotate (depth-validated; "NO DEPTH" toast if depth invalid).

Camera switching changes **only the rendered source** — never emits a stream enable/disable. Keys
`1`/`2` are render-only and enforced by a wire-level test (`test_no_emit_proof.py`).

## HUD contents

Source badge (SIMULATION / HARDWARE), camera + LiDAR freshness, five docking ranges, cutter range,
trunk estimate, calibration ID/validity, selected target state + out-of-view/stale warnings.

## Target selection (non-actuating annotation)

On a valid click: back-project the depth pixel, transform to `tree_base`/map via the timestamp-correct
pose export (simulation only), reproject into subsequent frames. Published only as a non-actuating
operator annotation event (`v1/operator/target_selection`). No control command to the robot.

In **Phase 1 (current)** annotations are **camera-relative only**: `tree_base_xyz` stays `null` and
the UI never claims a world-fixed target. A hardware annotation also leaves `tree_base_xyz` null
(no validated localization source). Depth is validated — invalid depth shows a "NO DEPTH" toast and
no annotation is placed.

## Data model behavior

- `v1/range/docking` republishes all five records on every single-sensor update → the model should
  **overwrite** state per packet, not accumulate.
- Render whatever `telemetry_key` strings arrive (the gateway and Pi use different key names — see the
  telemetry-gateway skill's wire-format reference). Don't assume one convention.
- Per-channel `deque(maxlen=4)`; count drops for stale indicators.

## Testing convention

Pure-python tests use `unittest` + synthetic contract-built packets (no live ZMQ/recording fixture
needed). GUI tests guard with import checks so they skip headless or pre-apt-install.
