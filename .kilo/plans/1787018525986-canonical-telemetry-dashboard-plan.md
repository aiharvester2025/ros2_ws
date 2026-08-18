# Canonical Xavier–Orin Telemetry and Operator Dashboard — Implementation Plan (Phase 1, re-baselined)

**Phase:** 1 of 2 — Non-actuating operator annotation and guidance only.
**Scope boundary:** Telemetry gateway (done), **Qt Quick dashboard (next — build now on Xavier, source-agnostic, portable to Orin)**, pose exporter (simulation), Orin hardware adapter. No hydraulic actuation, solenoid/PLC control, joint commands, or Eye-in-Hand closed-loop guidance — those are Phase 2.
**Baseline constraint:** Do not modify any file in `src/oil_palm_harvester_description/` or `src/oil_palm_tree_description/`. All new work is additive, including launch files (run new nodes in separate terminals/launches).
**Hardware constraint:** The real Orin robot is human-operated: no joint encoders, no joint command interface, no software path to the hydraulics. The dashboard is a passive guidance tool only.

## Implemented baseline — 2026-08-18 (verified, do not redo)

Verified against source and tests (8 passing: 4 protocol, 3 encoder, 1 recording; `git status` clean and additive):

- `docs/canonical_zmq_v1.md` — protocol authority (frozen).
- `docs/TELEMETRY_HANDOFF.md` — operational handoff.
- `src/harvester_telemetry_contract/` — pure-Python (no ROS/ZMQ deps) pack/validate for the exact three-frame `[channel bytes, MessagePack header, payload bytes]` wire format; `CANONICAL_CHANNELS` frozenset; strict header validation.
- `src/harvester_telemetry_gateway/` — read-only rclpy gateway subscribing to existing Gazebo topics only (cutter/docking RGB+depth+CameraInfo, `/harvester/lidar/raw_points`, five docking ranges, cutter range, trunk center, calibration status), publishing canonical v1; opt-in exact-packet recording + replay.

Frozen contract facts the remaining work must follow:

| Fact | Value |
|---|---|
| PUB endpoint | **One per source**, default `tcp://*:5590`; all `v1/*` channels multiplexed (first frame = subscription prefix). Xavier gateway owns 5590 on Xavier; the future Orin **aggregator** owns 5590 on Orin. Replay uses `tcp://*:5591`. |
| Status endpoint | Read-only REQ/REP, default `tcp://*:5600`; same response shape on both hosts. |
| Queue policy | Bounded newest-wins queues; drop complete old packets; `ZMQ_CONFLATE` prohibited. |
| Header | `schema_version`, `source_mode` (`simulation`/`hardware`), `source_id`, `sequence`, `frame_id`, `acquisition_timestamp_ns`, `clock_domain` (`ros_sim_time`/`utc_host`/`plc_rtc_utc`), `gateway_monotonic_ns`, `calibration_id`, `capabilities` (required, string→bool, per packet) + channel-specific `codec`, `pixel_encoding`, `width`/`height`, `point_count`/`point_stride_bytes`/`point_fields`, optional `transform_valid`/`transform_freshness_s`, `keyframe`. |
| Depth | `depth_uint16_le`, row-major LE uint16 **millimetres**, exactly `width*height*2` bytes, `0` = invalid. Gateway normalizes 16UC1/32FC1. |
| LiDAR | `lidar_xyz_f32`, LE float32 XYZ; layout declared in `point_fields`; consumers must not assume extra fields. |
| Timestamps | Integer ns; meaningful only within `clock_domain`; `gateway_monotonic_ns` is local freshness only. |
| Capability flags | e.g. `target.world_fixed: false` today — flips true only when the pose exporter is validated. |

---

## NEXT TASK — Dashboard v1 (Xavier, source-agnostic, portable)

**Rationale:** the canonical bus is source-agnostic, so a dashboard validated against Xavier simulation/replay works unchanged on Orin against the real aggregator. Build it here first; skip Orin adapter work.

### D0. Environment facts (measured on this Xavier, drive the design)

- Active `python3` = anaconda 3.8.8 (has `zmq` 20.0.0, `msgpack`, `numpy`, `PIL`; **no PySide2**). System `/usr/bin/python3` = 3.8.10 (has PySide2 QtCore/QtGui; **missing** QtQuick bindings, `zmq`, `msgpack`). **No `cv2` anywhere.** `DISPLAY=:1` is live.
- **Dashboard runtime = `/usr/bin/python3`** with apt-provided `python3-pyside2.qtquick`, `python3-zmq`, `python3-msgpack` (+ QML modules). All six candidates verified present in apt (pyside2.qtquick 5.14.0, zmq 18.1.1, msgpack 0.6.2, qtquick2/window2/layouts 5.12.8). Sudo is available (user password provided in conversation — do **not** write it into any file or script; prompt/run interactively). First implementation step:
  `sudo apt install python3-pyside2.qtquick python3-zmq python3-msgpack qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtquick-layouts`
- PySide2 5.14 has **no QtQuickControls2 bindings** → QML uses **QtQuick 2 primitives only** (Item, Rectangle, Text, Image, Canvas, Keys, ColumnLayout/RowLayout from qtquick-layouts). No Controls2, no cv2, no ROS imports anywhere in `dashboard/`.
- JPEG decode via **PIL**; H.264/H.265 decode is a stub raising a clear "hardware decode unavailable on this host" error, selected by header `codec` (uniform decoder interface; real Jetson path drops in later on Orin).
- The gateway launch runs under anaconda python (unchanged); the dashboard is an independent process under system python. Zero changes to either interpreter's existing roles.

### D1. Package layout — `src/harvester_dashboard/` (ament_python-style, zero ROS imports)

Portable to Orin by copying the directory (or `pip install .`); colcon-visible for future wiring but fully runnable from checkout.

```
src/harvester_dashboard/
  harvester_dashboard/
    __init__.py
    main.py              # /usr/bin/python3 entry: QApplication, engine, wiring, CLI args
    config.py            # endpoint/timeout/limit args -> dataclass
    protocol_shim.py     # sys.path bootstrap to harvester_telemetry_contract (or vendored copy)
    zmq_source.py        # one SUB socket; prefix subs; RCVHWM; drain loop; per-channel
                         # deque(maxlen<=4); drops counted; frames -> parsed dicts (worker thread)
    status_client.py     # REQ to tcp://<host>:5600 (read-only): profile, calibration_revision,
                         # streams, dropped_packets, recording, capabilities
    decoders/
      jpeg_decoder.py    # PIL -> numpy RGB
      h264_decoder.py    # stub (codec-aware interface)
      h265_decoder.py    # stub
      depth_decoder.py   # uint16 LE mm -> float32 m (numpy)
      lidar_decoder.py   # point_fields-driven struct unpack -> Nx3 float32
    model/
      telemetry_model.py # per-stream last header/payload, recv monotonic time, stale flag,
                         # sequence-gap + drop counters, source badge aggregation
      target_model.py    # camera-relative annotation state (see D4)
    image_provider.py    # QQuickImageProvider bridging latest decoded frames to QML Image
    bridge.py            # QObject exposing models/controls to QML (setContextProperty)
  qml/
    Dashboard.qml        # root: layout, Keys handler, view/HUD/LiDAR visibility state
    CameraView.qml       # Image from provider + click MouseArea + annotation overlay
    HudOverlay.qml       # source badge, freshness, ranges, trunk, calibration, stream errors
    SensorPanel.qml      # five docking ranges + cutter range rows (renders whatever
                         # telemetry_key strings arrive; Orin ingest normalizes Pi keys)
    LidarInset.qml       # Canvas top-down (x-y) scatter, range-coloured, ≤2000 pts
    Annotation.qml       # crosshair + camera-relative point label
  test/
    test_model.py        # pure-python: packet->model updates, stale logic, gaps
    test_decoders.py     # depth/lidar/jpeg decode from synthetic packets (contract-built)
    test_zmq_source.py   # inproc PUB -> subscriber drain/drop semantics
    test_smoke_gui.py    # guarded: skipped unless PySide2+QtQuick importable
```

### D2. Control semantics (user-specified; supersedes earlier plan text)

- `1` → render cutter view. `2` → render docking view. **Render-only**: switching changes nothing upstream; both subscriptions stay live; no enable/disable is ever sent on `1`/`2`.
- `3` → toggle sensor HUD. `4` → toggle LiDAR inset/overlay. `0` / `Esc` → clear current annotation.
- **Hardware stream enable/disable is a separate, explicit maintenance action** in a maintenance area of the UI, **unavailable in simulation** (hidden/disabled unless the status REP reports `source_mode: hardware`). It is never triggered by view switching. Simulation source has no control endpoint at all — the dashboard must not attempt one.
- Touch equivalents of 1/2/3/4/0 as on-screen buttons (QtQuick primitives).
- All dashboard events, including target clicks, are **non-actuating annotations only**.

### D3. Data model & freshness

- One subscriber, prefix subscriptions for all canonical channels; worker `QThread` drains with `recv_multipart(NOBLOCK)` until `Again`; decode in worker; post results to UI via queued signals.
- Freshness/staleness from **local receipt monotonic time** (works for live and replay); red > 2.0 s (configurable). Header `acquisition_timestamp_ns` displayed with its `clock_domain`; never compared across domains or hosts.
- Source badge from `source_mode` of active streams; if active streams disagree → `MIXED` + warning row. `capabilities` shown in an info line (e.g. `target.world_fixed: false`).
- Stream errors panel: local drop/sequence-gap counts per channel + `v1/system/status` payload (`streams`, `dropped_packets`, `errors`, `recording`).
- Non-image payloads: docking ranges (array of `{telemetry_key, distance_m|null, valid, ...}`), cutter range (single record), trunk estimate (pose+covariance, render position + status), calibration status (`calibration_id`, validity).

### D4. Annotation (Phase 1 scope: camera-relative)

- Click `(u,v)` on the active camera view → look up depth via the depth decoder using latest `camera_info` K (nearest valid pixel in a small window). Depth `0`/missing → "NO DEPTH" toast, nothing published.
- Back-project `P = ((u-cx)/fx·z, (v-cy)/fy·z, z)` in the packet's `frame_id`; store camera-relative 3-D point + pixel; overlay crosshair + distance label on every frame until cleared (`0`/`Esc`).
- World-fixed anchoring is **deferred** until `v1/pose/*` exists and `target.world_fixed` is true; the UI must not claim a world-fixed target.
- Annotation events are logged in-app and optionally forwarded to a **separate configurable annotation PUB** (default **disabled**, e.g. `tcp://127.0.0.1:5592`); the dashboard never binds the canonical 5590/5600 endpoints.

### D5. Run & validation procedure (all without breaking status quo)

1. First implementation step: run the D0 apt install (sudo; prompt for password interactively — never embed it in scripts/docs).
2. Fixture: during a normal sim session, run the gateway once with `record_dir` set (30–60 s), or reuse an existing audit. No sim/Gazebo needed afterwards. If no recording exists yet, the pure-python tests build synthetic packets via the contract, so decoder/model work can start before any fixture exists.
3. `python3 -m harvester_telemetry_gateway.replay <audit_dir> --endpoint tcp://*:5591` (anaconda python, existing tool).
4. `/usr/bin/python3 -m harvester_dashboard.main --pub tcp://127.0.0.1:5591 --status <status-host-or-disable>` on `DISPLAY=:1` → verify D2/D3/D4 acceptance below.
5. Live: point `--pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600` while sim+gateway run; confirm identical rendering and that sliders/Gazebo/RViz behave exactly as before (gateway untouched).
6. Tests: `PYTHONPATH=src/harvester_telemetry_contract:src/harvester_dashboard /usr/bin/python3 -m unittest discover -s src/harvester_dashboard/test -v` (pure tests must pass even before apt install; GUI smoke test skips cleanly).

**Acceptance (dashboard v1):**
- [ ] Visualizes all channels from replay with no Gazebo/ROS running (source-agnostic proof).
- [ ] `1`/`2` switch rendered view only; wire-level confirmation that no control traffic is emitted (no socket other than SUB + optional annotation PUB/REQ).
- [ ] Maintenance enable/disable absent/disabled in simulation.
- [ ] Freshness red after 2 s of silence; badge correct; range/trunk/calibration panels populate; stream errors visible.
- [ ] Click → crosshair + camera-relative annotation; `0`/`Esc` clears; no-depth rejected.
- [ ] H.264/H.265 stub returns a clear error (no crash) when fed synthetic `codec: h264` packet.
- [ ] Bounded memory: slow drain (artificial sleep) does not grow RSS; drops counted.
- [ ] Regression: gateway/sim untouched (git diff clean for `oil_palm_*` and telemetry packages except additive dashboard dir).

---

## 1. Additive schema extension for pose transport (after dashboard v1)

- New canonical channels: `v1/pose/cutter`, `v1/pose/docking` — `json`: `T_tree_base_camera` sampled at the paired image acquisition time; payload `{child_frame_id, translation_m, quaternion_xyzw, transform_status, transform_age_s, joint_state_age_s, image_sequence}`; header `clock_domain: ros_sim_time`, `frame_id: tree_base`, `transform_valid`, `transform_freshness_s`, `capabilities` incl. `target.world_fixed: true` **only after** validation.
- Add to `harvester_telemetry_contract.CANONICAL_CHANNELS` + validation + `docs/canonical_zmq_v1.md`.

## 2. Xavier: `harvester_pose_exporter` (new ROS 2 package) + gateway wiring

**Package:** `src/harvester_pose_exporter/` (ament_python, rclpy). **Simulation only.**

- Subscribes `/harvester/joint_states`, both camera image topics; publishes `/harvester/pose_exporter/{cutter,docking}_to_tree_base` (JSON String, header stamp = paired image stamp).
- **FK not TF2** (sim contracts prohibit exact-stamp dynamic TF): URDF forward kinematics (`urdf_parser_py` + `kdl_parser`/PyKDL) from measured joint state at-or-before image stamp; cache static sub-chains from `oil_palm_harvester_kinematic.urdf`; reject joint-state age > 0.1 s. `T_world_base_link` via TF2 at image sim stamp (single plugin-owned dynamic link; latest-TF fallback for this one link only, age-gated). `T_tree_base_world` static `(8.5, 0, 0)`. Compose `T_tree_base_camera = T_tree_base_world · T_world_base_link · T_base_link_camera_optical`.
- Gateway (our package) gains an additive subscription to both exporter topics → `v1/pose/*`, config `pose_channels_enabled`, no crash when exporter absent.
- **Validation gate** (`scripts/validate_pose_exporter.py`): finite, age < 0.1 s, anchor = static `tree_base`, world-fixed point doesn't drift while sliders move the arm. Until it passes, dashboard must not claim world-fixed targets and `target.world_fixed` stays false.

## 3. Orin hardware adapter (harvester_vision repo, new `adapter/`) — deferred until dashboard v1 validated

- **One aggregator owns the canonical bus**: `adapter/aggregator.py` binds `tcp://*:5590` + REP `5600` (same response shape as Xavier gateway), owns per-channel sequences, bounded queues, and the same exact-packet recorder.
- Ingest modules **connect to** the aggregator (PUSH/PULL, e.g. 5570–5579), never bind canonical PUB:
  - `oak_capture.py` per camera role: DepthAI v3 as today; **H.264/H.265 primary** (`H264_HP`/`H265_MAIN`, Annex-B, header `codec`, `pixel_encoding`, `keyframe`), MJPEG fallback; headers `source_mode: hardware`, `clock_domain: plc_rtc_utc`, `acquisition_timestamp_ns` from `time_sync.capture_timestamp_us()*1000`; existing PULL 5566/5567 control for enable/disable (pipeline fully stopped when disabled).
  - `range_ingest.py`: Pi `harvester.sensor-telemetry.v2` (tcp://192.168.50.40:5555) → **normalize Pi keys** (`diagonal_left_45deg`→`left_45_deg` etc.) to the canonical `telemetry_key` set → `v1/range/docking`.
  - `cutter_range_ingest.py` → `v1/range/cutter`; `lidar_ingest.py` (lightweight MID-360 UDP parser, 10 Hz, XYZ; deskew = Phase 2) → `v1/lidar/raw`; camera_info from OAK EEPROM calibration handler; depth channels capability-flagged off until OAK depth enabled.
- Hardware profiles `adapter/hardware_profiles/*.yaml` per stream; systemd units for aggregator + ingests (`Restart=always`, after network/chrony); dashboard unit user-session only.
- Legacy `oak_rgb_publisher.py` ports 5556/5557 keep running until the dashboard is validated on Orin, then retire; `oak_rgb_viewer.py` replaced by the dashboard (canonical multipart ≠ legacy single-part; never both on one port).

## 4. Dashboard v2 on Orin (post-adapter)

Same `src/harvester_dashboard` codebase: swap `--pub tcp://127.0.0.1:5590`, point status at the Orin aggregator, install the Jetson H.264/H.265 decoder module (PyNvCodec preferred), enable the maintenance stream controls (hardware mode now permits them). H.264/H.265 becomes primary there; JPEG remains the fallback.

## 5. Validation summary (Phase 1 complete when all pass)

**Xavier:** contract+gateway tests stay green (8 + new pose tests); pose-exporter gate passes; `v1/pose/*` paired by image stamp; regression: `gz model -l` both models, single `/robot_description` publisher, sliders move sim+RViz, no static `world→base_link`, description packages unmodified.
**Dashboard:** D5 acceptance list.
**Orin (later):** aggregator publishes every configured channel; packets pass contract validation; REP shape matches; Jetson hw decode works with JPEG fallback verified; camera switch render-only + maintenance toggles act on exactly one pipeline; Pi range keys normalized; bounded memory under RCVHWM hit; hardware annotation camera-relative (`tree_base_xyz` null); simulation-linked world-fixed target holds while arm moves; **no-actuation audit** (tcpdump shows only canonical PUB/REP/control + optional annotation port).

## 6. Delivery order (re-baselined)

1. ~~Freeze schema; contract + gateway + recording/replay~~ — **done, verified.**
2. **Dashboard v1 on Xavier (NEXT):** D0 apt install → package skeleton → zmq_source/status_client + decoders + models → QML views → replay-fixture validation → D5 acceptance.
3. Freeze additive `v1/pose/*` channels; build `harvester_pose_exporter`; wire into gateway; pass validation gate; dashboard gains world-fixed annotation behind `target.world_fixed`.
4. Orin `adapter/` (aggregator + ingest + profiles + systemd); `tests/validate_adapter.py`.
5. Dashboard v2 on Orin (decoder swap + maintenance controls); end-to-end acceptance incl. no-actuation audit.

---

## Phase 2 (out of scope) — Eye-in-Hand Human-in-the-Loop Guidance

Strictly non-actuating: tool-mounted sensing, 3-stage tracking, richer guidance overlays; operator keeps full manual hydraulic control; reuses v1 additively.

- **2.1 Sensors:** tool depth camera + tool LiDAR on `cutting_tool_link` (second Gazebo sensors in sim; second OAK + dedicated LiDAR on hardware); dual IMUs (OAK + LiDAR) 200–500 Hz. Eye-in-Hand reduces guidance to `e(t) = P_target^C(t) − P_crosshair^C` (no global FK). Sim drives existing `/harvester/joint_commands` joints; no Gazebo controller changes.
- **2.2 Stages:** Far >1.0 m (tool LiDAR+camera coarse lock); Mid 0.3–1.0 m (IMU-stabilized vision, crosshair precision); Terminal <0.3 m (cutter micro-range sensors, contact/tilt, `CONTACT READY`). `v1/operator/target_lock`: stage, distance, primary sensor, quality, lock state.
- **2.3 Tracking:** click → depth back-projection + nearest-LiDAR refinement (~5 cm); crosshair = fixed calibrated tool-camera offset; `calcOpticalFlowPyrLK` tracking; deskewed-LiDAR depth updates; LiDAR re-init on flow loss; IMU dead-reckoning ≤0.5 s through occlusions else `TRACK LOST`. HUD: crosshair, tracked dot, error vector, ΔX/ΔY/ΔZ, stage, lock quality, distance.
- **2.4 IMU compensation:** per-point LiDAR deskew via IMU orientation interpolation; camera rolling-shutter/blur mitigation (Laplacian trigger, `MOTION` warning on jerk); high-pass (>20 Hz) whip filtering for dead-reckoning.
- **2.5 Terminal:** auto-handoff ≤0.30 m; multi-sensor distance+tilt; <20 cm freeze last camera pixel, range-only depth; range failure → `SENSOR BLIND — MANUAL ENGAGE`; regression >0.30 m → `TARGET LOST — RETREAT`.
- **2.6 Software:** Xavier `harvester_tool_tracking`, `harvester_imu_deskew`, `harvester_range_fusion`; Orin tool ingest into aggregator; dashboard `CrosshairOverlay/TargetTrackOverlay/StageIndicator` + `tracking_engine`/`handoff_controller`. Additive channels: `v1/camera/tool/{rgb,depth,camera_info}`, `v1/lidar/tool_raw`, `v1/imu/tool_camera`, `v1/imu/tool_lidar`, `v1/operator/target_lock`.
- **2.7 Safety/validation:** unchanged boundaries (no valve/PLC/joint output; physical E-stop external). Handled failure modes: occlusion, blind zone, foliage saturation (radius filter), IMU dropout. Validation: sensor URDF+topics; deskew vs static truth; tracking vs known motion; terminal handoff into trunk contact; scripted 3-stage approach; zero-actuation packet audit.
- **2.8 Order:** freeze additive channels → URDF/launch sensors → `harvester_imu_deskew` → `harvester_tool_tracking` → `harvester_range_fusion` → Orin tool ingest → dashboard overlays → end-to-end + no-actuation audit.

---

## Open questions (implementer-level, non-blocking)

1. **Qt binding on Orin:** reuse PySide2 5.14 stack (matching Xavier) vs PySide6 — keep PySide2 for portability unless the Orin image already ships PySide6.
2. **MID-360 ingestion:** lightweight UDP parser vs Livox SDK — parser first for CPU; revisit only if lossy.
3. **Annotation sink:** default-disabled local PUB (5592). If a ROS 2 logger is needed later, bridge subscribes to that port; ROS never enters the dashboard.
