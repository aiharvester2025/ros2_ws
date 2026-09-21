#!/usr/bin/env bash
# run_simulation.sh — single-command launcher for the harvester simulation.
#
# Starts the processes that would otherwise each need their own terminal.
# Two control modes are supported (see MODE below):
#
#   auto    (default) — autonomous docking: slider GUI off, dock orchestrator on.
#   manual  — manual joint control: slider GUI ON (joint_state_publisher_gui),
#             dock orchestrator OFF. Move the joints yourself with the sliders.
#
# Processes:
#   T1 — Gazebo (headless) + RViz
#   T2 — canonical telemetry gateway
#   T3 — dock orchestrator   (auto mode only)
#   T4 — telemetry dashboard (system python, on the xrdp display :10)
#
# Usage:
#   cd ~/ros2_ws
#   ./run_simulation.sh            # auto-docking
#   ./run_simulation.sh manual     # manual slider control
#   MODE=manual ./run_simulation.sh
#
# Ctrl-C stops all processes.  The dashboard + RViz windows open on DISPLAY=:10
# (the xrdp session) — ensure you are connected to that session to see them.

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
DISPLAY_NUM="${DISPLAY_NUM:-:10}"          # xrdp session display
LOG_DIR="${LOG_DIR:-$HOME/ros2_ws/.sim_logs}"

# Control mode: "auto" (autonomous dock) or "manual" (slider GUI).
# Accepts the first positional argument or the MODE env var.
MODE="${1:-${MODE:-auto}}"
case "$MODE" in
    auto|manual) ;;
    *)
        echo "ERROR: unknown MODE '$MODE'. Use 'auto' or 'manual'." >&2
        exit 2
        ;;
esac

# ---------------------------------------------------------------------------
# Paths / interpreters (do NOT mix these — see the dashboard README)
# ---------------------------------------------------------------------------
ROS_SOURCE="/opt/ros/foxy/setup.bash"
WS_SOURCE="$ROS2_WS/install/setup.bash"
SYSTEM_PYTHON="/usr/bin/python3"

# ---------------------------------------------------------------------------
# Helper: run a command in its own process group and remember the group ID.
# `setsid` gives each launch its own session/process group so that killing the
# group tears down every child too (ros2 launch spawns gzserver, gateway_node,
# robot_state_publisher, etc. — killing only the parent leaves them orphaned
# and still holding ports like 5590).
# ---------------------------------------------------------------------------
PGIDS=()
launch() {
    local name="$1"; shift
    local log="$LOG_DIR/$name.log"
    mkdir -p "$LOG_DIR"
    echo "[launch] $name -> $log"
    setsid "$@" >"$log" 2>&1 &
    PGIDS+=($!)
}

cleanup() {
    echo ""
    echo "[stop] terminating all simulation processes..."
    for pgid in "${PGIDS[@]}"; do
        # Negative PID = signal the whole process group.
        kill -- -"$pgid" 2>/dev/null || true
    done
    sleep 2
    for pgid in "${PGIDS[@]}"; do
        kill -9 -- -"$pgid" 2>/dev/null || true
    done
    # Belt-and-suspenders: sweep any straggler ROS/gazebo/gateway processes
    # that escaped their process group.
    pkill -9 -f 'gzserver' 2>/dev/null || true
    pkill -9 -f 'harvester_telemetry_gateway.gateway_node' 2>/dev/null || true
    pkill -9 -f 'harvester_dock.dock_orchestrator' 2>/dev/null || true
    pkill -9 -f 'harvester_dashboard.main' 2>/dev/null || true
    echo "[stop] done."
}
trap cleanup EXIT INT TERM

cd "$ROS2_WS"

# ---------------------------------------------------------------------------
# Decide joint_gui based on the mode.
#   auto   -> joint_gui:=false  (slider off; orchestrator owns the joints)
#   manual -> joint_gui:=true   (slider on; orchestrator must NOT run)
# boom_plan stays false in BOTH modes: the old boom_plan stack also publishes
# to /harvester/joint_commands and would fight both the orchestrator and the
# slider GUI.
# ---------------------------------------------------------------------------
if [ "$MODE" = "manual" ]; then
    JOINT_GUI="true"
else
    JOINT_GUI="false"
fi

echo "=============================================="
echo " Harvester simulation launcher"
echo " MODE   : $MODE"
echo " joint_gui : $JOINT_GUI  (slider control)"
echo " orchestrator: $([ "$MODE" = "auto" ] && echo ON || echo OFF)"
echo " DISPLAY: $DISPLAY_NUM"
echo "=============================================="

# ---------------------------------------------------------------------------
# T1 — Gazebo headless + RViz
#      boom_plan:=false in BOTH modes. joint_gui toggles the slider GUI.
# ---------------------------------------------------------------------------
echo "[start] T1 — Gazebo + RViz (joint_gui=$JOINT_GUI)"
bash -c "
  source '$ROS_SOURCE' && source '$WS_SOURCE' && \
  ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
    gui:=false rviz:=true harvester_collision_mode:=off \
    articulation_control_mode:=kinematic docking_camera:=true \
    joint_gui:=$JOINT_GUI boom_plan:=false
" &
T1_PID=$!
PGIDS+=($T1_PID)

# ---------------------------------------------------------------------------
# T2 — canonical telemetry gateway (ROS-sourced terminal, anaconda python)
# ---------------------------------------------------------------------------
echo "[start] T2 — telemetry gateway"
bash -c "
  source '$ROS_SOURCE' && source '$WS_SOURCE' && \
  ros2 launch harvester_telemetry_gateway gateway.launch.py
" &
T2_PID=$!
PGIDS+=($T2_PID)

# Give Gazebo a head start so the gateway/orchestrator topics exist.
echo "[wait] giving Gazebo a few seconds to come up..."
sleep 5

# ---------------------------------------------------------------------------
# T3 — dock orchestrator (auto mode only).
#      In manual mode this MUST NOT run: it publishes to /harvester/joint_commands,
#      the same topic the slider GUI writes to — they would fight each other.
# ---------------------------------------------------------------------------
if [ "$MODE" = "auto" ]; then
    echo "[start] T3 — dock orchestrator (auto-docking)"
    bash -c "
      source '$ROS_SOURCE' && source '$WS_SOURCE' && \
      python3 -m harvester_dock.dock_orchestrator
    " &
    T3_PID=$!
    PGIDS+=($T3_PID)
else
    T3_PID="(disabled)"
    echo "[skip]  T3 — dock orchestrator (manual mode: use the sliders instead)"
fi

# ---------------------------------------------------------------------------
# T4 — dashboard (SYSTEM python, no ROS source, on the xrdp display)
#      The --dock-pub button only matters in auto mode (the orchestrator listens
#      on it); in manual mode it is harmless, so we keep it for consistency.
# ---------------------------------------------------------------------------
echo "[start] T4 — dashboard (DISPLAY=$DISPLAY_NUM)"
bash -c "
  export DISPLAY='$DISPLAY_NUM' && \
  export PYTHONPATH='$ROS2_WS/src/harvester_dashboard' && \
  '$SYSTEM_PYTHON' -m harvester_dashboard.main \
    --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600 \
    --dock-pub tcp://127.0.0.1:5593
" &
T4_PID=$!
PGIDS+=($T4_PID)

echo ""
echo "All processes launched:"
echo "  T1 Gazebo+RViz  (PID $T1_PID)  log: $LOG_DIR/t1.log"
echo "  T2 gateway      (PID $T2_PID)  log: $LOG_DIR/t2.log"
echo "  T3 orchestrator (PID $T3_PID)  log: $LOG_DIR/t3.log"
echo "  T4 dashboard    (PID $T4_PID)  log: $LOG_DIR/t4.log"
echo ""

if [ "$MODE" = "auto" ]; then
    echo "AUTO-DOCKING: press DOCK three times in the dashboard (sweep → dock → undock)."
else
    echo "MANUAL CONTROL: use the joint_state_publisher_gui sliders to move the joints."
    echo "  (The slider window opens beside RViz on DISPLAY=$DISPLAY_NUM.)"
fi

echo "Ctrl-C to stop everything."
echo ""

# Wait on the first process (Gazebo) to keep this launcher alive.
wait $T1_PID
