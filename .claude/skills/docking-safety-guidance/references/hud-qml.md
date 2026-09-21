# Qt Quick HUD

The docking guidance panel lives in `src/harvester_dashboard/qml/HudOverlay.qml`, shown only when
`bridge.view === "docking"`.  Recreate it on Orin using **QtQuick 2 primitives only** — PySide2 5.14
(Ubuntu 20.04) has **no QtQuickControls2**, so no `Button`/`Slider`/Controls2; build everything from
`Rectangle` + `Text` + `MouseArea`.

## Panel structure (one `Rectangle`)

1. **State banner** — a bold `Text` showing `bridge.dockGuidanceText`, coloured by state.
2. **Stop-bar** — the primary glanceable element (see below).
3. **Metrics row** — `speed · gap · TTC · max safe speed` (one `Text`).

## State → colour map (must match exactly)

| State | border/banner colour | background |
|---|---|---|
| `danger` | `#e23c3c` (red) | `#3a1414` |
| `warn` | `#f0a030` (orange) | `#3a2a10` |
| `no_data` | `#9fb4c7` (grey) | `#1a1f24` |
| `safe` (default) | `#40c040` (green) | `#102a18` |

`danger` also pulses (a `NumberAnimation` on `opacity` from 1.0→0.55, `InOutSine`, `Infinite`).
`border.width` is 3 in `danger`, 2 otherwise.

## Stop-bar semantics

- A track `Rectangle` (e.g. 220×14, `#2a3a4a`).
- **Fill** = current gap, width = `gap / stopbar_range_m` (clamped 0..1), coloured red when
  `d_stop(v) >= gap` else the state colour.
- **Marker** = a thin white `Rectangle` (4 px wide) at `x = width * (d_stop(v) / stopbar_range_m)`.
- `stopbar_range_m = 1.5` — the full-scale near field (0..1.5 m) where the gap actually competes
  with the stopping distance.
- Guard every numeric property with `isFinite(...)` and render `—` (or hide the marker) when the
  value is NaN.

## Metrics row text

```
(speed.toFixed(1) + " cm/s" | "—") · (gap.toFixed(2) + " m" | "—")
   · ("TTC " + ttc.toFixed(1) + " s" | "TTC —") · ("max " + maxSpeed.toFixed(0) + " cm/s" | "—")
```

## Invariants

- The panel must be hidden on the cutter view (`visible: bridge.view === "docking"`).
- `no_data` is grey and distinct from `safe` green — never render a false green.
- `stateColor` should ideally be declared `property color` (QML auto-coerces a string, but `color`
  is more idiomatic).
- Keep the existing `errors_panel` anchoring: the guidance panel sits above it
  (`anchors.bottom: errors_panel.top`).
