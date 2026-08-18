# Canonical ZeroMQ Telemetry v1

This contract is shared by the Xavier simulation gateway, the Orin hardware
adapter, and the operator dashboard.  It is transport and ROS independent.
It carries observations and non-actuating operator annotations only; it never
carries a joint, hydraulic, PLC, solenoid, or motion command.

## Endpoints and queue policy

Each source exposes one configurable PUB endpoint.  The default is
`tcp://*:5590`; a dashboard selects the Xavier or the Orin endpoint in its
profile.  A source may expose a separate configurable REQ/REP status endpoint
(default `tcp://*:5600`).

PUB/SUB consumers subscribe by channel prefix.  Senders and consumers use
bounded application queues and discard complete old packets when a newer
packet is available.  `ZMQ_CONFLATE` is prohibited because it is unsafe with
multipart messages.

## Wire format

Every PUB/SUB observation is exactly three frames:

```text
[ UTF-8 channel bytes, MessagePack header bytes, binary payload bytes ]
```

The first frame is the ZeroMQ subscription topic, for example
`v1/camera/cutter/rgb`.  The header is a MessagePack map.  The payload is
codec-specific binary data; JSON payloads are UTF-8 encoded JSON bytes.

All timestamp fields are integer nanoseconds.  A timestamp is meaningful only
within its declared `clock_domain`; it must not be compared directly with a
timestamp from another domain.  `gateway_monotonic_ns` is for local stream
freshness and latency measurements, not cross-host clock synchronization.

## Header

Every message has these fields:

| Field | Type | Meaning |
|---|---|---|
| `schema_version` | integer | Must be `1`. |
| `source_mode` | string | `simulation` or `hardware`. |
| `source_id` | string | Publisher identity, for example `xavier` or `orin`. |
| `sequence` | integer | Non-negative, monotonically increasing per channel/source. |
| `frame_id` | string | Measurement frame; may be empty only for source-global status. |
| `acquisition_timestamp_ns` | integer | Source-clock acquisition timestamp. |
| `clock_domain` | string | `ros_sim_time`, `utc_host`, or `plc_rtc_utc`. |
| `gateway_monotonic_ns` | integer | Gateway monotonic time at publication. |
| `calibration_id` | string | Calibration/configuration revision, or `none`. |
| `capabilities` | map of string to boolean | Source feature flags; present in every packet. |

Channel-specific fields are required only when their channel requires them:

| Field | Type | Used by |
|---|---|---|
| `codec` | string | Image, depth, LiDAR, and JSON payloads. |
| `pixel_encoding` | string | RGB image payloads. |
| `width`, `height` | integer | Image/depth payloads. |
| `transform_valid` | boolean | Data that references a transform. |
| `transform_freshness_s` | number or `null` | Age of that transform in its source clock. |
| `point_count`, `point_stride_bytes`, `point_fields` | integer/integer/list | LiDAR payloads. |
| `keyframe` | boolean | H.264/H.265 payloads when known. |

The implementation validates global fields and validates required
channel-specific fields before publishing or consuming a packet.

`capabilities` describes installed/source-supported features, for example
`camera.cutter.depth`, `lidar.intensity`, `range.cutter`, and
`target.world_fixed`. It is distinct from live-stream health: the current
availability, drops, and errors belong in `v1/system/status`. This lets the
same dashboard adapt to Xavier simulation and Orin hardware without guessing
which sensors exist.

## Canonical channels and payloads

| Channel | Codec | Payload |
|---|---|---|
| `v1/camera/cutter/rgb` | `jpeg`, `h264`, or `h265` | JPEG bytes or Annex-B encoded video access unit. |
| `v1/camera/cutter/depth` | `depth_uint16_le` | Tight row-major little-endian `uint16` depth in millimetres. |
| `v1/camera/cutter/camera_info` | `json` | CameraInfo-compatible JSON: K, D, R, P, dimensions, and distortion model. |
| `v1/camera/docking/rgb` | `jpeg`, `h264`, or `h265` | Same representation as cutter RGB. |
| `v1/camera/docking/depth` | `depth_uint16_le` | Same representation as cutter depth. |
| `v1/camera/docking/camera_info` | `json` | Same representation as cutter CameraInfo. |
| `v1/lidar/raw` | `lidar_xyz_f32` | Little-endian point records.  Simulation starts with XYZ float32; hardware may declare optional intensity, tag, line, and point-time fields. |
| `v1/range/docking` | `json` | Array of the five named docking-sensor readings. |
| `v1/range/cutter` | `json` | One cutter-range reading. |
| `v1/docking/trunk_estimate` | `json` | Existing calibrated side-pair trunk estimate. |
| `v1/calibration/status` | `json` | Calibration revision, validity, and nominal/physical status. |
| `v1/system/status` | `json` | Source heartbeat, stream state, drops, and errors. |
| `v1/operator/target_selection` | `json` | Non-actuating operator annotation only. |

### Payload conventions

- RGB: `pixel_encoding` is `RGB8`, `BGR8`, `H264`, or `H265` as appropriate.
  A decoder follows `codec`, not an assumed camera type.
- Depth: `width * height * 2` bytes exactly.  `0` means invalid/no return.
  Gateways convert source `32FC1` depth to millimetres before publication.
- LiDAR: `point_fields` declares ordered records, for example
  `[{"name":"x","type":"float32"}, {"name":"y","type":"float32"},
  {"name":"z","type":"float32"}]`.  Consumers must not assume intensity
  exists.
- JSON: payload is UTF-8 JSON and the header `codec` is `json`.
- Range readings contain `telemetry_key`, `distance_m`, `valid`, `frame_id`,
  `acquisition_timestamp_ns`, and `calibration_id`.
- A simulation target annotation may include `tree_base_xyz`; a hardware
  annotation leaves it `null` unless a separately validated localization
  source exists.

## Status request

A REQ/REP status response is UTF-8 JSON containing `schema_version`,
`active_profile`, `calibration_revision`, and the latest stream state.  It is
read-only.  It cannot enable cameras, change robot state, or command hardware.

## Audit recording and replay

Recording is optional but uses the same v1 wire contract on Xavier and Orin.
When enabled, a producer records **each complete three-frame packet** before
its bounded live-output queue can discard older packets. Each `.msgpack`
record contains `record_format_version`, `recorded_monotonic_ns`, and the
three binary frames unchanged. It never stores image or LiDAR data as JSON or
hexadecimal text.

Replayers validate every stored packet with this contract and publish its
original frames on a separate configurable PUB endpoint. Recordings are thus
suitable for offline dashboard development, protocol regression tests, tuning,
and audits from simulation or real hardware. Capture directories must be
managed by the operator because image/depth/LiDAR audits can consume storage
quickly.

## Compatibility rules

- A v1 consumer rejects a different `schema_version`.
- Unknown v1 header fields and unknown channels under `v1/` are ignored only
  when the consumer does not need them.
- Producers must retain the three-frame layout throughout v1.
- Simulation and hardware use the same channels and header names; source
  adapters, not the dashboard, perform source-specific translation.
