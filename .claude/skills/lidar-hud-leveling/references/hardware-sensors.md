# Real-hardware sensor inventory + leveling source

The "no encoders" statement in the handoff docs is **too broad**. The settled truth, assembled across
the session:

## What the real machine has / lacks

| Joint / DOF | Sensor |
|---|---|
| Boom pivot (elevation) angle | ✅ angle sensor |
| Boom extension (4 prismatic stages) | ❌ no encoder |
| Platform tilt (2-axis, pitch/roll) | ✅ tilt sensor (no yaw) |
| Boom turret yaw | ❌ no encoder |
| Cutting arm (lift + extension) | ❌ no encoder |
| Cutter-forward range (0.05–3.0 m) | ✅ |
| 5 docking range sensors | ✅ |
| OAK camera IMU | ✅ |
| Mid-360 LiDAR IMU | ✅ |
| GPS / absolute pose | ❌ none |

Transport: boom angle + platform tilt + range sensors come from **Modbus → PLC → ZMQ publisher**;
LiDAR via Livox-SDK2 → ZMQ; OAK via DepthAI → ZMQ.

## Key implication for leveling

Leveling (the "tree stands straight" fix) needs **only orientation**, not position. Orientation is
available from three independent sources:

1. Boom angle + platform tilt (via PLC/Modbus) — pitch/roll, **no yaw**.
2. Mid-360 built-in IMU — full orientation (with yaw drift).
3. OAK camera IMUs — orientation.

"**No yaw is fine for leveling**" — height/leveling depend on pitch/roll, not yaw.

The missing encoders (boom extension, turret, cutter arm) are the **translation** terms that the
rotation-only leveling fix **already drops**. So the leveled-sensor-frame model sidesteps them
entirely.

## URDF role (the user asked "do I need ROS 2 for a URDF?")

- **No — a URDF is just XML; you don't need ROS to read it or compute FK** (pure trig/numpy).
- A URDF is inert without joint states; the missing extension/turret/cutter encoders mean the dynamic
  chain is incomplete (orientation-only, no position).
- **Useful as a static calibration registry** (sensor mounts, camera↔LiDAR extrinsic, boom pivot
  geometry), NOT for full world registration on this machine.
- Keep the URDF as **data**, do the math in plain Python/numpy (the harvester_vision repo's
  `geometry/transforms.py` is already designed for this).

## Recommended leveling source

Read the **Mid-360 built-in IMU** (self-contained, co-located with the LiDAR) via Livox-SDK2, but
structure it as a **pluggable "leveling source"** so boom/tilt sensors can be swapped in. The core
leveling math (quaternion → rotation-only transform) is identical regardless of source.

## IMU caveats

- IMU gives roll/pitch (gravity-referenced), **not yaw** (unobservable without magnetometer/GNSS).
- IMU-vs-gravity reference is not "level" on a slope; platform + sensor IMUs must share a calibrated
  gravity reference.
- For a single sweep (~2–3 min) IMU drift is negligible; factory bias offsets still required.
- Time-sync: IMU streams must be time-synchronized with LiDAR/range reads or the `d_short` "same step"
  assumption breaks (see tree-height-estimation skill).
