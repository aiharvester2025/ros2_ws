// Sensor HUD: source badge, ranges, trunk, calibration, stream errors.
import QtQuick 2.12

Item {
    id: hud
    property real panel_opacity: 0.82

    // MIXED source warning row (top center).
    Rectangle {
        visible: bridge.sourceMixed
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 8
        width: mixed_text.width + 24
        height: 30
        radius: 5
        color: "#b26a00"
        opacity: panel_opacity
        Text {
            id: mixed_text
            anchors.centerIn: parent
            text: "MIXED SOURCES — timestamps not comparable"
            color: "white"
            font.pixelSize: 13
        }
    }

    // Left panel: docking ranges + cutter range.
    SensorPanel {
        id: sensor_panel
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 8
    }

    // Right panel: trunk + calibration + capabilities.
    Column {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 8
        spacing: 4

        Rectangle {
            width: trunk_cal_text.width + 16
            height: trunk_cal_text.height + 12
            radius: 5
            color: "#000000"
            opacity: panel_opacity
            Text {
                id: trunk_cal_text
                anchors.centerIn: parent
                text: bridge.trunkLine + "\n" + bridge.calibrationLine + "\n"
                      + bridge.capabilitiesLine
                color: "#cfe3f5"
                font.pixelSize: 12
                horizontalAlignment: Text.AlignRight
            }
        }

        // Docking plan readout (visible whenever the dock orchestrator runs).
        Rectangle {
            visible: bridge.dockPlanLine !== "dock: idle"
            width: dock_plan_text.width + 16
            height: dock_plan_text.height + 12
            radius: 5
            color: "#102a18"
            opacity: panel_opacity
            Text {
                id: dock_plan_text
                anchors.centerIn: parent
                text: bridge.dockPlanLine
                color: "#a8e0a8"
                font.pixelSize: 12
                horizontalAlignment: Text.AlignRight
            }
        }
    }

    // Center-bottom: docking safety-guidance HUD (docking camera view only).
    // Physics-based operator alert: green = safe, orange = warn (slow down),
    // red = danger (stop), grey = no data.  Shows closing speed, gap, TTC,
    // the required stopping distance, and a concrete "slow to X cm/s"
    // recommendation.  A stop-bar compares the current gap against the
    // stopping distance so the operator can read remaining margin at a glance.
    // Pulses in DANGER.
    Rectangle {
        id: dock_guidance_hud
        visible: bridge.view === "docking"
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: errors_panel.top
        anchors.bottomMargin: 10
        width: Math.max(stopbar.width, Math.max(guidance_text.width, metrics_text.width)) + 32
        height: guidance_text.height + stopbar.height + metrics_text.height + 40
        radius: 8

        // State -> palette (grey = no data).
        property string stateColor: bridge.dockSafetyState === "danger" ? "#e23c3c"
                                  : bridge.dockSafetyState === "warn"   ? "#f0a030"
                                  : bridge.dockSafetyState === "no_data" ? "#9fb4c7"
                                  :                                        "#40c040"
        property color borderCol: stateColor
        property color bgCol: bridge.dockSafetyState === "danger" ? "#3a1414"
                            : bridge.dockSafetyState === "warn"   ? "#3a2a10"
                            : bridge.dockSafetyState === "no_data" ? "#1a1f24"
                            :                                        "#102a18"

        color: bgCol
        border.color: borderCol
        border.width: bridge.dockSafetyState === "danger" ? 3 : 2
        opacity: panel_opacity

        // Pulse the border/background in DANGER by animating opacity; the
        // NumberAnimation takes over opacity only while running.
        NumberAnimation on opacity {
            id: pulse_anim
            running: bridge.dockSafetyState === "danger"
            loops: Animation.Infinite
            from: 1.0
            to: 0.55
            duration: 500
            easing.type: Easing.InOutSine
        }

        Column {
            anchors.centerIn: parent
            spacing: 5

            // Guidance / notification line (the actionable message).
            Text {
                id: guidance_text
                anchors.horizontalCenter: parent.horizontalCenter
                text: bridge.dockGuidanceText
                color: dock_guidance_hud.stateColor
                font.pixelSize: 15
                font.bold: true
            }

            // Stop-bar: current gap (fill) vs required stopping distance
            // (marker).  The bar is scaled to a fixed reference range so the
            // marker approaches the fill as the platform gets too close/fast.
            // When the gap (fill) is shorter than the stopping distance the
            // bar is red; otherwise it tracks the state colour.
            Rectangle {
                id: stopbar
                anchors.horizontalCenter: parent.horizontalCenter
                width: 220
                height: 14
                radius: 4
                color: "#2a3a4a"

                // Fill = current gap, scaled to stopbar_range_m.
                // Full-scale is the near field (0..1.5 m) where the gap
                // actually competes with the stopping distance; at a_max=0.10,
                // latency=0.30 the stop distance at 60 cm/s is ~1.98 m, so a
                // 1.5 m scale keeps the marker on-bar through the whole
                // approach while reserving the right side for the danger zone.
                property real stopbar_range_m: 1.5
                property real gap_m: isFinite(bridge.dockCenterDistanceM)
                    ? Math.max(0.0, Math.min(1.0, bridge.dockCenterDistanceM / stopbar_range_m))
                    : 0.0
                property real stop_m: isFinite(bridge.dockStopDistanceM)
                    ? Math.max(0.0, Math.min(1.0, bridge.dockStopDistanceM / stopbar_range_m))
                    : 0.0
                property bool stopping: isFinite(bridge.dockStopDistanceM)
                    && isFinite(bridge.dockCenterDistanceM)
                    && bridge.dockStopDistanceM >= bridge.dockCenterDistanceM

                Rectangle {
                    id: gap_fill
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    width: parent.width * stopbar.gap_m
                    radius: 4
                    color: stopbar.stopping ? "#e23c3c" : dock_guidance_hud.stateColor
                }

                // STOP marker at the stopping-distance position.
                Rectangle {
                    id: stop_marker
                    visible: isFinite(bridge.dockStopDistanceM)
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    x: parent.width * stopbar.stop_m - 2
                    width: 4
                    color: "#ffffff"
                }
            }

            // Metrics row: speed · gap · TTC · max safe speed.
            Text {
                id: metrics_text
                anchors.horizontalCenter: parent.horizontalCenter
                text: (isFinite(bridge.dockSpeedSmoothedCmS)
                          ? bridge.dockSpeedSmoothedCmS.toFixed(1) + " cm/s" : "—")
                      + "  ·  "
                      + (isFinite(bridge.dockCenterDistanceM)
                          ? bridge.dockCenterDistanceM.toFixed(2) + " m" : "—")
                      + "  ·  "
                      + (isFinite(bridge.dockTtcS)
                          ? "TTC " + bridge.dockTtcS.toFixed(1) + " s" : "TTC —")
                      + "  ·  max "
                      + (isFinite(bridge.dockMaxSpeedCmS)
                          ? bridge.dockMaxSpeedCmS.toFixed(0) + " cm/s" : "—")
                color: "#cfe3f5"
                font.pixelSize: 12
            }
        }
    }

    // Center-bottom: cutter safety-guidance HUD (cutter camera view only).
    // Mirrors the docking guidance panel but for the cutting arm: a colour-coded
    // clearance alert plus the cut-sequence prompt (approach -> stop/ready ->
    // open scissors -> advance -> cut).  Advisory/operator-facing only.
    Rectangle {
        id: cutter_guidance_hud
        visible: bridge.view === "cutter"
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: errors_panel.top
        anchors.bottomMargin: 10
        width: Math.max(cutter_bar.width,
                        Math.max(cutter_phase_text.width,
                                 cutter_metrics_text.width)) + 32
        height: cutter_phase_text.height + cutter_guidance_text.height
                + cutter_bar.height + cutter_metrics_text.height + 44
        radius: 8

        property string stateColor: bridge.cutterSafetyState === "danger" ? "#e23c3c"
                                  : bridge.cutterSafetyState === "warn"   ? "#f0a030"
                                  : bridge.cutterSafetyState === "no_data" ? "#9fb4c7"
                                  :                                          "#40c040"
        property color bgCol: bridge.cutterSafetyState === "danger" ? "#3a1414"
                            : bridge.cutterSafetyState === "warn"   ? "#3a2a10"
                            : bridge.cutterSafetyState === "no_data" ? "#1a1f24"
                            :                                          "#102a18"

        color: bgCol
        border.color: cutter_guidance_hud.stateColor
        border.width: bridge.cutterSafetyState === "danger" ? 3 : 2
        opacity: panel_opacity

        NumberAnimation on opacity {
            id: cutter_pulse
            running: bridge.cutterSafetyState === "danger"
            loops: Animation.Infinite
            from: 1.0
            to: 0.55
            duration: 500
            easing.type: Easing.InOutSine
        }

        Column {
            anchors.centerIn: parent
            spacing: 5

            // Cut-sequence phase banner (the operator prompt).
            Text {
                id: cutter_phase_text
                anchors.horizontalCenter: parent.horizontalCenter
                text: bridge.cutterPhaseText
                color: "#ffffff"
                font.pixelSize: 15
                font.bold: true
            }

            // Clearance alert line (safe/warn/danger).
            Text {
                id: cutter_guidance_text
                anchors.horizontalCenter: parent.horizontalCenter
                text: bridge.cutterGuidanceText
                color: cutter_guidance_hud.stateColor
                font.pixelSize: 13
            }

            // Clearance stop-bar: tip clearance (fill) vs required stopping
            // distance (white marker).  Red when the marker meets the fill.
            Rectangle {
                id: cutter_bar
                anchors.horizontalCenter: parent.horizontalCenter
                width: 220
                height: 14
                radius: 4
                color: "#2a3a4a"

                property real bar_range_m: 1.0
                property real clear_m: isFinite(bridge.cutterClearanceM)
                    ? Math.max(0.0, Math.min(1.0, bridge.cutterClearanceM / bar_range_m))
                    : 0.0
                property real stop_m: isFinite(bridge.cutterStopDistanceM)
                    ? Math.max(0.0, Math.min(1.0, bridge.cutterStopDistanceM / bar_range_m))
                    : 0.0
                property bool stopping: isFinite(bridge.cutterStopDistanceM)
                    && isFinite(bridge.cutterClearanceM)
                    && bridge.cutterStopDistanceM >= bridge.cutterClearanceM

                Rectangle {
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    width: parent.width * cutter_bar.clear_m
                    radius: 4
                    color: cutter_bar.stopping ? "#e23c3c"
                                               : cutter_guidance_hud.stateColor
                }

                Rectangle {
                    visible: isFinite(bridge.cutterStopDistanceM)
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    x: parent.width * cutter_bar.stop_m - 2
                    width: 4
                    color: "#ffffff"
                }
            }

            // Metrics: tip clearance · closing speed · max safe speed.
            Text {
                id: cutter_metrics_text
                anchors.horizontalCenter: parent.horizontalCenter
                text: "tip "
                      + (isFinite(bridge.cutterClearanceM)
                          ? bridge.cutterClearanceM.toFixed(2) + " m" : "—")
                      + "  ·  "
                      + (isFinite(bridge.cutterSpeedSmoothedCmS)
                          ? bridge.cutterSpeedSmoothedCmS.toFixed(1) + " cm/s" : "—")
                      + "  ·  max "
                      + (isFinite(bridge.cutterMaxSpeedCmS)
                          ? bridge.cutterMaxSpeedCmS.toFixed(0) + " cm/s" : "—")
                color: "#cfe3f5"
                font.pixelSize: 12
            }

            // Operator confirmation: advance the cut sequence one step
            // (open / advance / cut have no sensors, so they are confirmed here).
            // Disabled while DANGER/NO_DATA — the cut sequence must not advance
            // with the tip too close or with no range.
            Rectangle {
                id: confirm_button
                property bool blocked: bridge.cutterSafetyState === "danger"
                                       || bridge.cutterSafetyState === "no_data"
                visible: bridge.cutterPhase !== "idle"
                         && bridge.cutterPhase !== "approach"
                anchors.horizontalCenter: parent.horizontalCenter
                width: confirm_text.width + 24
                height: 24
                radius: 5
                color: !enabled ? "#1a2028"
                               : (confirm_touch.pressed ? "#3a4a5a" : "#22303f")
                border.color: enabled ? "#4fc3f7" : "#5a6470"
                border.width: 1
                enabled: !blocked
                Text {
                    id: confirm_text
                    anchors.centerIn: parent
                    text: confirm_button.blocked ? "WAIT — tip too close"
                                                 : "CONFIRM STEP"
                    color: confirm_button.blocked ? "#8a94a0" : "#e8eef4"
                    font.pixelSize: 12
                }
                MouseArea {
                    id: confirm_touch
                    anchors.fill: parent
                    enabled: confirm_button.enabled
                    onClicked: bridge.cutter_confirm_phase()
                }
            }
        }
    }

    // Bottom: stream errors panel (collapsible rows per channel).
    Rectangle {
        id: errors_panel
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: 8
        height: Math.min(210, 24 + bridge.streamRows.length * 22)
        radius: 5
        color: "#000000"
        opacity: 0.78
        clip: true

        Text {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.margins: 6
            text: "streams  recv " + bridge.receivedPackets
                  + "  drops " + bridge.droppedPackets
            color: "#9fb4c7"
            font.pixelSize: 12
        }

        // Maintenance placeholder: only meaningful in hardware mode.
        Rectangle {
            visible: bridge.maintenanceAvailable
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.margins: 6
            width: maintenance_text.width + 16
            height: 22
            radius: 4
            color: "#23415e"
            border.color: "#4fc3f7"
            Text {
                id: maintenance_text
                anchors.centerIn: parent
                text: "maintenance: hardware controls pending"
                color: "#bfe3ff"
                font.pixelSize: 11
            }
        }

        Column {
            anchors.top: parent.top
            anchors.topMargin: 24
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 0

            Repeater {
                model: bridge.streamRows
                delegate: Text {
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.leftMargin: 8
                    text: {
                        var base = modelData.channel + "  " + modelData.age;
                        if (modelData.gaps > 0) base += "  gaps " + modelData.gaps;
                        if (modelData.drops > 0) base += "  drops " + modelData.drops;
                        if (modelData.decode_errors > 0)
                            base += "  dec-err " + modelData.decode_errors;
                        if (modelData.error.length > 0) base += "  [" + modelData.error + "]";
                        return base;
                    }
                    color: !modelData.ever_seen ? "#7a8a99"
                         : modelData.stale ? "#e2a63c"
                         : modelData.decode_errors > 0 ? "#e25c5c"
                         : "#a8d08d"
                    font.pixelSize: 11
                    elide: Text.ElideMiddle
                }
            }
        }
    }
}
