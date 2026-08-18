# Xavier ROS 2 to ZeroMQ gateway

This package is additive and read-only.  It subscribes to the existing
simulation camera, depth, LiDAR, range, trunk-estimate, and calibration topics
and publishes canonical three-frame ZeroMQ v1 packets.  It does not publish a
ROS topic, TF transform, joint command, velocity command, or Gazebo service.

## Run

Start the normal simulation in one terminal:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py
```

In a second terminal, start the gateway:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch harvester_telemetry_gateway gateway.launch.py
```

The default PUB endpoint is `tcp://*:5590`; all `v1/*` channels share it.  The
read-only status endpoint is `tcp://*:5600`.  Set a different configuration
file using a real absolute path with:

```bash
ros2 launch harvester_telemetry_gateway gateway.launch.py \
  config:=/home/ubuntu/harvester_gateway_audit.yaml
```

The supplied configuration starts with JPEG quality 85 and LiDAR stride two
to limit Xavier load.  It transports raw simulation LiDAR from
`/harvester/lidar/raw_points`, not the RViz-only zero-stamped cloud.

## Audit recording and offline replay

Recording is disabled by default. Copy `config/gateway.yaml`, set `record_dir`
to a new directory with sufficient free space, and start the gateway using
that copy. Every complete canonical packet is written as an exact three-frame
MessagePack record. This includes RGB, depth, CameraInfo, raw LiDAR, all range
channels, trunk estimate, and status packets; it does not affect Gazebo, RViz,
or robot controls.

Replay a completed recording on a different endpoint from the live gateway:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 -m harvester_telemetry_gateway.replay \
  /home/ubuntu/harvester_audits/run01 --endpoint tcp://*:5591 --speed 1.0
```

Replay preserves packet order and bounded timing, but caps long gaps at 0.25
seconds by default so offline UI tests do not appear stuck.

`docs/TELEMETRY_HANDOFF.md` is the operational handoff for this gateway,
record/replay workflow, time policy, and the future Orin adapter boundary.

## Tests

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway:${PYTHONPATH} \
  python3 -m unittest discover -s src/harvester_telemetry_gateway/test -v
```
