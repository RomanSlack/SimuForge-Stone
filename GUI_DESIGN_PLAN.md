# SimuForge-Stone GUI Redesign Plan

## Vision

**Palantir** data density meets **Apple** restraint meets **Blender** viewport-centric workflow.

A dark, professional simulation cockpit where the 3D viewport dominates and
information-rich panels frame it without competing for attention. Every pixel
earns its place. The interface should feel like piloting an industrial system,
not using a toy.

---

## 1. Layout Architecture

```
+--[ Header Bar (32px) ]------------------------------------------+
| [>] SimuForge-Stone   |  Mode: CARVING  |  1000x  |  FPS: 60   |
+--------+-------------------------------------------------+------+
|        |                                                 |      |
| Left   |                                                 | Right|
| Panel  |              3D Viewport                        | Panel|
| 280px  |           (PBR Scene)                           | 260px|
|        |                                                 |      |
|  Arm   |                                                 | Props|
|  State |                                                 | Mat  |
|  Joints|                                                 | Tool |
|  Torque|                                                 | G-cd |
|        |                                                 |      |
+--------+-----------+-----------------------------------+-+------+
|  Timeline / Progress Bar (48px)                                  |
|  [|<] [<] [>||] [>] [>|]   =====[===|=========]======  0:42/2:15|
+------------------------------------------------------------------+
```

### Panel Behavior

| Panel | Content | Width | Behavior |
|-------|---------|-------|----------|
| **Header** | App name, mode indicator, speed, FPS | Full width, 32px | Always visible |
| **Left** | Arm state: joints, torques, IK status | 280px | Collapsible sections |
| **Right** | Properties: material, tool, workpiece, G-code | 260px | Collapsible sections, scrollable |
| **Bottom** | Timeline/progress for carving, log output | Full width, 48px | Expandable to ~200px |
| **Viewport** | 3D scene (PBR + sky + grid) | Remaining space | Receives mouse input when not over panels |

### Input Routing

egui reports `response.consumed` per event. When the cursor is over a panel,
egui eats the input. When over the viewport, mouse events route to the orbit
camera. Keyboard shortcuts (Space, G, R, +/-) are global and bypass egui.

---

## 2. Color System

### Foundation

Derived from Blender 4.x dark theme, refined with Palantir's precision and
Apple's restraint. All colors in sRGB.

```
Background Scale (darkest → lightest):
  bg-0   #0D0E12   (viewport clear / deepest)
  bg-1   #181A20   (panel fill)
  bg-2   #1E2028   (panel header / section bg)
  bg-3   #282A32   (input field bg)
  bg-4   #353840   (hover state)
  bg-5   #454850   (active/pressed state)

Border Scale:
  border-0  #2A2D35  (subtle separator)
  border-1  #3A3D45  (panel edge)
  border-2  #505560  (focused input)

Text Scale:
  text-0  #F0F1F4  (primary — headings, values)
  text-1  #B0B4BC  (secondary — labels)
  text-2  #6E7380  (tertiary — hints, disabled)
  text-3  #454850  (placeholder)
```

### Accent Colors

```
Primary (Orange — like Blender selection):
  orange-400  #FF8C38  (hover)
  orange-500  #F57C20  (default — buttons, active indicators)
  orange-600  #D46A15  (pressed)

Secondary (Blue — Apple-influenced):
  blue-400   #5B9CF5  (hover)
  blue-500   #4A8AE5  (default — links, selection highlight)
  blue-600   #3A72C0  (pressed)

Status:
  green-500  #3DCC60  (running, OK, inside block)
  red-500    #E5453A  (error, IK fail, outside block)
  yellow-500 #F5C842  (warning, paused, speed != 1x)
  cyan-500   #42BFD9  (info, manual mode)
```

### Panel Transparency

Panels use semi-transparent fills to let the 3D scene subtly bleed through:

```
Panel fill:        #181A20  alpha 230/255 (~90%)
Panel header fill: #1E2028  alpha 240/255 (~94%)
Tooltip fill:      #1E2028  alpha 245/255 (~96%)
```

This creates depth without needing a blur pass. The dark viewport behind
ensures readability while the slight transparency creates a floating,
glass-like feel.

---

## 3. Typography

### Font Stack

```
Primary:    Inter (proportional) — clean, Apple-influenced sans-serif
Monospace:  JetBrains Mono — for values, joint angles, coordinates
Fallback:   egui default (Hack/NotoSans built-in)
```

Both fonts are open source (OFL). Embed as static bytes via `include_bytes!`.

### Scale

```
Title:      Inter 18px  semi-bold  (app name, major headings)
Heading:    Inter 14px  medium     (section headers)
Body:       Inter 12px  regular    (labels, descriptions)
Caption:    Inter 10px  regular    (hints, timestamps)
Value:      JetBrains Mono 13px    (numbers, angles, coordinates)
Value-sm:   JetBrains Mono 11px    (bar labels, secondary values)
```

### Usage Rules

- Section headers: `Heading` in `text-1`, ALL CAPS with 1px letter-spacing
- Numeric values: Always `Value` monospace for alignment
- Units: Appended in `text-2` (e.g., `  45.2` `deg` where deg is dimmer)
- Labels: `Body` in `text-1`, sentence case

---

## 4. Panel Designs

### 4.1 Header Bar

```
+------------------------------------------------------------------+
| [S] SimuForge  |  CARVING [===|====]  78%  |  1000x  |  62 FPS  |
+------------------------------------------------------------------+
```

- Height: 32px, bg: `bg-2`
- Left: App icon (orange S glyph) + "SimuForge" in `Title`
- Center: Mode badge + progress mini-bar (only in carving mode)
- Right: Speed selector (dropdown or +/- buttons) + FPS counter
- Bottom border: 1px `border-0`

### 4.2 Left Panel — Arm State

```
+-- ARM STATE (280px) ----------------+
|                                      |
|  v JOINTS                            |
|  ┌──────────────────────────────┐    |
|  │ J1   -12.4°   [========|--] │    |
|  │ J2    45.2°   [------|====] │    |
|  │ J3   -28.7°   [====|------] │    |
|  │ J4     3.1°   [-----|=====] │    |
|  │ J5    89.0°   [---------|=] │    |
|  │ J6   -15.3°   [=====|----] │    |
|  └──────────────────────────────┘    |
|                                      |
|  v TORQUES                           |
|  ┌──────────────────────────────┐    |
|  │ J1   -278.4 Nm  [====|    ] │    |
|  │ J2    156.2 Nm  [    |===  ] │    |
|  │ J3     42.8 Nm  [   |=    ] │    |
|  │ J4     -8.3 Nm  [  =|     ] │    |
|  │ J5     12.1 Nm  [   |=    ] │    |
|  │ J6     -2.4 Nm  [  =|     ] │    |
|  └──────────────────────────────┘    |
|                                      |
|  v POSITION                          |
|  ┌──────────────────────────────┐    |
|  │ IK: OK (converged)    [grn] │    |
|  │                              │    |
|  │ Target                       │    |
|  │   X  450.0  Y    0.0  Z  95│    |
|  │ Actual                       │    |
|  │   X  449.8  Y    0.2  Z  95│    |
|  │ Error: 0.28 mm               │    |
|  │                              │    |
|  │ Status: IN BLOCK      [grn] │    |
|  └──────────────────────────────┘    |
|                                      |
|  v CUTTING                           |
|  ┌──────────────────────────────┐    |
|  │ Spindle: ON            [grn] │    |
|  │ Force:  12.4 N               │    |
|  │ RPM:    24000                 │    |
|  └──────────────────────────────┘    |
+--------------------------------------+
```

- Each section is a `CollapsingHeader` with orange triangle indicator
- Joint bars: thin (4px), orange for J1-J3, cyan for J4-J6
- Torque bars: centered at zero, green=positive, red=negative
- Position values: monospace, right-aligned
- IK status: colored dot indicator (green/red)

### 4.3 Right Panel — Properties

```
+-- PROPERTIES (260px) ---------------+
|                                      |
|  v SIMULATION                        |
|  ┌──────────────────────────────┐    |
|  │ Time     2.847 s             │    |
|  │ Physics  1000 Hz             │    |
|  │ Mesh     24 chunks  186K tri │    |
|  └──────────────────────────────┘    |
|                                      |
|  v WORKPIECE                         |
|  ┌──────────────────────────────┐    |
|  │ Material   Marble            │    |
|  │ Size       100 x 100 x 100  │    |
|  │ Center     450, 0, -50  mm  │    |
|  │ Resolution 0.5 mm           │    |
|  └──────────────────────────────┘    |
|                                      |
|  v TOOL                              |
|  ┌──────────────────────────────┐    |
|  │ Type       Ball Nose         │    |
|  │ Radius     2.0 mm            │    |
|  │ Length     20.0 mm            │    |
|  │ Offset    170.0 mm            │    |
|  └──────────────────────────────┘    |
|                                      |
|  v G-CODE                            |
|  ┌──────────────────────────────┐    |
|  │ File: test_carve.gcode       │    |
|  │ Waypoints: 1,247             │    |
|  │ Est. time: 142s              │    |
|  │                              │    |
|  │ [  Load G-code...  ]         │    |
|  └──────────────────────────────┘    |
+--------------------------------------+
```

- Read-only property display (not editable yet — phase 2)
- Values right-aligned in monospace
- Labels left-aligned in proportional
- "Load G-code" button with orange accent

### 4.4 Bottom Panel — Timeline

```
+------------------------------------------------------------------+
| [|<] [<] [ > ] [>] [>|]   [====|====================]  0:42/2:15|
| RUNNING   Speed: [  1000x  v]                   ETA: 98s  78.2% |
+------------------------------------------------------------------+
```

- Transport controls: standard media player icons (skip, play/pause, stop)
- Scrubber bar: orange fill on `bg-3` track, white playhead
- Speed: dropdown selector with presets
- Right side: ETA, percentage, elapsed/total time
- Collapsed height: 48px. Can expand to show log/console output (~200px)

---

## 5. Interactive Elements

### Buttons

```
Default:    bg-3 fill, border-1 stroke, text-1 text, 4px radius
Hover:      bg-4 fill, border-2 stroke, text-0 text
Pressed:    bg-5 fill, border-2 stroke, text-0 text
Primary:    orange-500 fill, no stroke, #000 text (for key actions)
Danger:     red-500 fill, no stroke, white text (for destructive actions)
```

### Sliders (future — phase 2)

- Rail: 4px tall, `bg-3`, 2px corner radius
- Fill: orange-500 (or section accent color)
- Handle: 12px circle, `bg-5` with `border-2` stroke
- Hover: handle grows to 14px

### Collapsible Sections

```
v JOINTS               (expanded — orange triangle, heading text)
> JOINTS               (collapsed — gray triangle, dimmer text)
```

- Triangle rotates 90deg on collapse (egui CollapsingHeader default)
- Override triangle color: orange-500 when expanded, text-2 when collapsed
- Section content indented 8px from section header

### Status Indicators

Small colored dots (6px diameter) next to status text:

```
[*] Running     green-500
[*] Paused      yellow-500
[*] IK OK       green-500
[*] IK FAIL     red-500
[*] IN BLOCK    green-500
[*] OUTSIDE     red-500
```

### Progress Bars

```
Track:    bg-3, 6px tall, 3px radius
Fill:     green-500 (or orange-500 for speed-modified)
Text:     percentage overlaid in Value-sm, centered
```

---

## 6. Viewport Integration

### Clear Color

Sky gradient rendered as a full-screen quad before PBR:

```
Zenith (top):    #0A0D17   deep navy
Horizon (mid):   #1E2438   dark blue-gray
Nadir (bottom):  #0A0D17   (below ground, not visible)
```

### Ground Plane

- Infinite grid with distance fade (already in pbr.wgsl)
- Grid color: `border-1` (#3A3D45)
- Major grid lines every 100mm, minor every 10mm

### Viewport Overlay (future — phase 2)

- Axis gizmo in bottom-left corner (RGB = XYZ)
- Tool path preview lines (existing, keep as-is)
- Selection highlight glow on workpiece

---

## 7. Implementation Plan

### Phase 1: Core egui Integration (this sprint)

**Goal:** Replace custom overlay with egui. All existing information displayed
in proper panels. No new interactive features yet — just display.

#### Step 1: Dependencies

```toml
# workspace Cargo.toml [workspace.dependencies]
egui = "0.31"
egui-wgpu = "0.31"
egui-winit = "0.31"

# crates/sim/Cargo.toml
egui = { workspace = true }
egui-wgpu = { workspace = true }
egui-winit = { workspace = true }

# crates/render/Cargo.toml
egui = { workspace = true }
```

Version 0.31 matches our existing wgpu 24 + winit 0.30. No wgpu upgrade needed.

#### Step 2: egui State in App

```rust
struct App {
    // ... existing fields ...
    egui_ctx: egui::Context,
    egui_winit_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
}
```

Initialize in `App::resumed()` after creating RenderContext.

#### Step 3: Event Routing

In `window_event()`:
1. Pass event to `egui_winit_state.on_window_event()`
2. If `response.consumed` — skip camera/keyboard handling
3. Else — proceed with existing input handling

Exception: global shortcuts (Space, Escape, G, R, +/-) always pass through.

#### Step 4: Theme Setup

Apply the color system from Section 2 using `egui_ctx.all_styles_mut()`.
Embed Inter + JetBrains Mono fonts.

#### Step 5: Build UI Function

New method: `fn build_egui_ui(&mut self)` containing:
- `TopBottomPanel::top("header")` — header bar
- `SidePanel::left("arm_state")` — joint angles, torques, position, cutting
- `SidePanel::right("properties")` — sim info, workpiece, tool, G-code
- `TopBottomPanel::bottom("timeline")` — progress/transport

All data is read-only display from `&self.sim`.

#### Step 6: Render Integration

In `render()`:
1. Sky gradient pass (existing)
2. PBR pass (existing)
3. Line pass (existing)
4. egui pass (NEW — replaces old overlay pass)

Remove: `OverlayPipeline`, `OverlayBuilder`, `build_diagnostics_overlay()`,
overlay.wgsl, SDF font atlas.

#### Step 7: Cleanup

- Delete `crates/render/src/pipelines/overlay.rs`
- Delete `crates/render/src/shaders/overlay.wgsl`
- Remove overlay module from render crate
- Remove unused SDF/bitmap font code

### Phase 2: Interactive Controls (future)

- Editable joint angle sliders (drag to move arm)
- Speed selector dropdown
- Play/pause/reset buttons in header and timeline
- G-code file picker dialog
- Material property editor (roughness, color)
- Camera preset buttons (top, front, side views)
- Keyboard shortcut overlay (? key)

### Phase 3: Advanced (future)

- Dockable/rearrangeable panels
- Viewport gizmos (orientation cube, axis arrows)
- Console/log panel with filtering
- Undo/redo system
- Viewport split (multi-view)
- Theme editor / user preferences

---

## 8. File Changes Summary

### New Files

```
crates/sim/src/ui.rs           — All egui UI building functions
crates/sim/src/ui/header.rs    — Header bar (optional split)
crates/sim/src/ui/panels.rs    — Left/right/bottom panels
crates/sim/src/ui/theme.rs     — Color palette + style setup
assets/fonts/Inter-Regular.ttf — Embedded font
assets/fonts/JBMono-Regular.ttf— Embedded font
```

### Modified Files

```
Cargo.toml                     — Add egui workspace deps
crates/sim/Cargo.toml          — Add egui deps
crates/render/Cargo.toml       — Add egui dep (for types)
crates/sim/src/main.rs         — egui init, event routing, render pass
crates/render/src/lib.rs       — Remove overlay module
```

### Deleted Files

```
crates/render/src/pipelines/overlay.rs  — Replaced by egui
crates/render/src/shaders/overlay.wgsl  — Replaced by egui
```

---

## 9. Design References

| Reference | What to Take |
|-----------|-------------|
| **Blender 4.x** | Panel layout, orange accent, collapsible sections, viewport-centric |
| **Palantir Foundry** | Data density, monospace values, status indicators, dark precision |
| **Apple Pro Apps** (Logic, Final Cut) | Typography restraint, spacing rhythm, timeline design |
| **Rerun Viewer** | egui-specific patterns, custom dark theme, professional polish |
| **Unreal Engine 5** | Property panel layout, detail levels, toolbar density |

---

## 10. Success Criteria

- [ ] All existing overlay information preserved in egui panels
- [ ] 3D viewport fills remaining space, no fixed-size viewport
- [ ] Mouse input correctly routes (panels vs viewport)
- [ ] Custom dark theme applied (not default egui gray)
- [ ] Inter + JetBrains Mono fonts embedded and used
- [ ] Collapsible sections work for all panel groups
- [ ] Semi-transparent panel backgrounds show 3D scene behind
- [ ] FPS impact < 1ms per frame for egui rendering
- [ ] Zero new dependencies beyond egui/egui-wgpu/egui-winit
- [ ] All 45 existing tests still pass
