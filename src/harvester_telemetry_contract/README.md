# Canonical telemetry contract

`harvester_telemetry_contract` is the pure-Python validation and packing layer
shared by the Xavier simulation gateway, the future Orin adapter, and the
operator dashboard. It contains no ROS or ZeroMQ socket ownership.

The protocol authority is
[`../../docs/canonical_zmq_v1.md`](../../docs/canonical_zmq_v1.md). Operational
context and the current implementation boundary are in
[`../../docs/TELEMETRY_HANDOFF.md`](../../docs/TELEMETRY_HANDOFF.md).

Use `pack_message()` to create exactly three frames and `unpack_message()` to
validate received frames. Any v1 producer or consumer must preserve that
layout, include every mandatory header field (including `capabilities`), and
avoid `ZMQ_CONFLATE` for multipart packets.
