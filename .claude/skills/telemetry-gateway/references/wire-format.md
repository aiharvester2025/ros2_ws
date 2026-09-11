# Canonical ZeroMQ v1 wire format

Source of truth: `docs/canonical_zmq_v1.md` and
`src/harvester_telemetry_contract/harvester_telemetry_contract/protocol.py`.

## Packet structure

Every telemetry packet is exactly 3 frames (PUB/SUB multipart):

```
[channel bytes, MessagePack header bytes, binary payload bytes]
```

The first frame is the channel (UTF-8 string) so ZeroMQ topic filtering can route. Header is a
MessagePack dict; payload is raw bytes whose meaning depends on the channel's codec.

## Header fields

Required: `schema_version` (must be `1`), `source_mode` (`simulation` | `hardware`), `source_id`
(`xavier` | `orin`), `sequence` (monotonic per channel/source), `frame_id`,
`acquisition_timestamp_ns` (integer nanoseconds), `clock_domain` (`ros_sim_time` | `utc_host` |
`plc_rtc_utc`), `gateway_monotonic_ns` (local freshness only), `calibration_id`, and a required
`capabilities` map (string→boolean) in **every** header.

Channel-specific fields (required only when the channel needs them): `codec`, `pixel_encoding`
(RGB: `RGB8`/`BGR8`/`H264`/`H265`), `width`/`height`, `transform_valid`, `transform_freshness_s`,
`point_count`/`point_stride_bytes`/`point_fields` (LiDAR), `keyframe` (H.264/H.265).

Key deltas from the original plan (now frozen):
- Integer-nanosecond timestamps `acquisition_timestamp_ns` and `gateway_monotonic_ns` (not
  `timestamp_us` / `gateway_monotonic_timestamp`).
- Required `capabilities` map in every header.
- Transform fields made optional.

## Canonical channels

```
v1/camera/cutter/rgb, depth, camera_info
v1/camera/docking/rgb, depth, camera_info
v1/lidar/raw
v1/range/docking, v1/range/cutter
v1/docking/trunk_estimate
v1/docking/plan                (docking FSM state + measured height/distance/boom plan; observation)
v1/calibration/status, v1/system/status
v1/operator/target_selection   (annotation only, non-actuating)
v1/imu/lidar, v1/imu/camera    (added for the IMU-assisted height strategy)
```

`v1/operator/dock_request` (`{"action":"dock"}`) is an **out-of-contract** request on the dedicated
endpoint `tcp://127.0.0.1:5593`, consumed only by the docking orchestrator (which owns the FSM). It
is NOT a canonical `5590`/`5600` channel — it carries no joint/velocity value.

`CANONICAL_CHANNELS` is a frozenset in `protocol.py`. Add new channels there (and route header
validation) only when needed.

## Payload conventions

- **Depth:** normalized to row-major uint16 millimetres regardless of 16UC1/32FC1 source (NaN→0,
  clip 1–65535). See `encoders.depth_to_uint16_mm`.
- **RGB:** JPEG on simulation; hardware may use H.264/H.265 (Annex-B) with decoder/key-frame metadata
  in the header `codec`.
- **LiDAR:** header declares `point_count`, `point_stride_bytes`, `point_fields`. Simulation sends
  XYZ-only (`lidar_xyz_f32`, `point_count * 12` bytes); hardware may additionally retain intensity,
  tag, line, and point-time fields.
- **Ranges:** `v1/range/docking` carries the full 5-record array in one payload (republished on every
  single-sensor update).

## Telemetry key mismatch (Orin work)

The gateway's `telemetry_key` set is `center_line`, `left_45_deg`, `right_45_deg`, `left_side`,
`right_side`. The Raspberry Pi / hardware uses `diagonal_left_45deg`, `diagonal_right_45deg`,
`c_channel_left`, `c_channel_right` — 4 of 5 disagree. The Orin ingest must normalize keys, or the
dashboard needs two rendering paths. The dashboard should render whatever `telemetry_key` strings
arrive rather than assume one convention.

## Endpoints

- PUB `tcp://*:5590` (one configurable endpoint per source; Xavier and Orin both default 5590).
- Status REQ/REP `tcp://*:5600` (schema version, active profile, calibration revision, stream
  availability, latest-status snapshot; read-only, one request per 50 ms tick).
- Replay PUB `tcp://*:5591`.
- Dashboard annotation PUB `tcp://127.0.0.1:5592` (`v1/operator/target_selection`, non-actuating,
  default disabled).
- Dock-request PUB `tcp://127.0.0.1:5593` (`v1/operator/dock_request`) — out-of-contract, consumed
  by the docking orchestrator only.
- Camera-control PULL `5566`/`5567` and Pi telemetry `5555` are **legacy harvester_vision ports**,
  not part of the canonical v1 map.

## Anti-patterns

- No `ZMQ_CONFLATE` with multipart — use bounded newest-wins queues and drop complete old packets.
- Never compare `acquisition_timestamp` across clock domains/hosts; gateway monotonic time is for
  network freshness only.
- Do not compare Gazebo sim time directly to Orin UTC.
