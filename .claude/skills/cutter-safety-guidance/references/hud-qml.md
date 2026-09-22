# Cutter Qt Quick HUD

The cutter guidance panel lives in `src/harvester_dashboard/qml/HudOverlay.qml`, shown only when
`bridge.view === "cutter"`.  Recreate it on Orin with **QtQuick 2 primitives only** (no Controls2 on
PySide2 5.14).

## Panel structure (one `Rectangle`)

1. **Phase banner** — bold `Text` = `bridge.cutterPhaseText` (white), the operator prompt
   (e.g. "STOP — ready to cut (open scissors wide next)", "OPEN the cutting scissors WIDE",
   "ADVANCE cutter forward 5 cm", "CUT now (gripper holds the FFB)").
2. **Clearance alert line** — `Text` = `bridge.cutterGuidanceText`, coloured by state.
3. **Clearance stop-bar** — tip clearance (fill) vs required stopping distance (white marker).
4. **Metrics row** — `tip <clearance> m · <speed> cm/s · max <v_max> cm/s`.
5. **CONFIRM STEP button** — a `Rectangle` + `MouseArea` calling `bridge.cutter_confirm_phase()`;
   visible only when `cutterPhase` is not `idle`/`approach`.  It is **disabled and relabelled
   "WAIT — tip too close"** while the state is `danger` or `no_data`.

## State → colour map (same as docking)

| State | border/banner colour | background |
|---|---|---|
| `danger` | `#e23c3c` | `#3a1414` |
| `warn` | `#f0a030` | `#3a2a10` |
| `no_data` | `#9fb4c7` | `#1a1f24` |
| `safe` (default) | `#40c040` | `#102a18` |

`danger` pulses (opacity 1.0→0.55, `InOutSine`, `Infinite`); `border.width` 3 in danger else 2.

## Stop-bar semantics

- Track `Rectangle` (220×14, `#2a3a4a`).
- Fill = `cutterClearanceM / bar_range_m` (clamped 0..1); `bar_range_m = 1.0` m full-scale (cutter
  clearances are small) — red when `cutterStopDistanceM >= cutterClearanceM`, else the state colour.
- Marker = thin white `Rectangle` at `cutterStopDistanceM / bar_range_m`.
- Guard every value with `isFinite(...)`.

## Invariants

- Visible only on the cutter view (hide on docking).
- `no_data` grey and distinct from `safe` green — never a false green.
- The CONFIRM STEP button is the only interactive element and only advances the *prompt*; it never
  commands the arm.  It must be disabled (and show "WAIT — tip too close") while DANGER/NO_DATA.
