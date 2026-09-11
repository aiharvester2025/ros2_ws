# Simulation vs hardware behavior

The dashboard must behave differently on Xavier (simulation) vs Orin (hardware), especially around
the pose exporter and target selection. This split is a deliberate safety boundary.

## Pose exporter (timestamp-correct, simulation-only)

The pose exporter publishes, for every cutter/docking image, the camera-to-`tree_base` transform
sampled in the **same Gazebo time domain** as that image, plus validity and age.

- Uses `/harvester/joint_states` (Gazebo measured feedback) + URDF forward kinematics — **not** TF2
  lookup at exact sim timestamps (the contracts prohibit dynamic TF at exact Gazebo sim timestamps).
- Never uses "latest TF" as a substitute for the measured joint state.
- In simulation, targets anchor in `tree_base`. Hardware later uses a configured stable map or
  tree-local frame.
- Until the exporter passes validation, the dashboard may show imagery/ranges/LiDAR but **must not
  claim a world-fixed clicked target**.

**Hardware (no encoders):** there is no pose exporter. The robot is human-operated, hydraulics via
solenoid/PLC/valve, no joint cmd. So on hardware the pose exporter cannot run.

## Target selection

- **Simulation:** back-project depth pixel → transform to `tree_base` via pose exporter → reproject.
- **Hardware (no encoders):** back-project → store as **camera-relative annotation only** (crosshair
  on-image). Do NOT transform to `tree_base`/world. HUD shows `ANNOTATION (camera-relative)`.

Both are non-actuating operator annotations.

## Codecs

- Simulation: JPEG (PIL decode).
- Hardware: H.264/H.265 primary via DepthAI `VideoEncoder` (Profiles `H264_HP`/`H265_MAIN`), Annex-B
  byte-stream payloads, config profiles with `bitrate_kbps`, `profile`, `keyframe_interval_s`. MJPEG
  fallback for debug only.
- Dashboard H.264/H.265 decoders are **stubs** (raise on `codec: h264|h265`) until Jetson hardware
  decode (PyNvCodec or Gst-nvdec) is wired on Orin. JPEG via OpenCV/PIL is the Xavier-only fallback.

## Out-of-scope future phase: Eye-in-Hand guidance

The operator asked about "mark tool crosshair → click target → guide the cutter crosshair to the
target" (Eye-on-Tool). This is a **separate, later phase** because it adds closed-loop actuation and
safety-critical control — and on the real machine there is no actuation path anyway (human-operated,
guide-only). The frozen ZeroMQ schema already supports it; keep it out of the current non-actuating
dashboard. The 3-stage handoff (LiDAR+camera >1 m → IMU-stabilized vision 0.3–1 m → tool range
sensors <0.3 m) is documented but not implemented.

## Cutter range sensor

Confirmed: Orin hardware **does** have the cutter range sensor, so `v1/range/cutter` is not
simulation-only — the Orin adapter will publish it.
