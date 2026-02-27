# Multi-Stage Toolpath Strategy — 2026-02-26

## Current State
- `tools/stl_to_gcode/stl_to_gcode.py` generates heightmap-based raster toolpaths
- Single tool (8mm ball-nose) does both roughing and finishing
- Works but slow and unrealistic — ball-nose is fragile for bulk removal

## Real CNC Workflow

### Phase 1: Roughing (flat end mill, 12-25mm)
- Goal: remove 90% of material fast, leave ~1-2mm stock above final surface
- Strategy: waterline raster at large stepdown (3-5mm), wide stepover (50-75% tool dia)
- Toolpath: same heightmap approach but offset surface outward by stock-to-leave
- Tool change: `M6 T1` (tool 1), or just a comment — sim only has one tool

### Phase 2: Semi-finishing (optional, medium end mill)
- Goal: reduce stock-to-leave from 1-2mm down to ~0.3mm
- Removes staircase pattern from roughing so finisher sees even material
- Same raster strategy, tighter stepdown, surface offset = 0.3mm

### Phase 3: Finishing (ball-nose, 4-8mm)
- Goal: final surface quality, follow the actual contour
- Strategy: tight stepover (10-25% tool dia = 1-2mm), single Z pass along surface
- This is where detail comes from — scallop height determines surface quality
- Scallop height ≈ R - sqrt(R² - (stepover/2)²) where R = tool radius

### Phase 4: Rest/pencil machining (optional, 1-3mm ball-nose)
- Goal: clean crevices the larger tool couldn't reach
- Strategy: detect areas where bigger tool left material (tool radius vs surface curvature)
- Eye sockets, nostrils, hair detail on a bust

## How to Implement

### In stl_to_gcode.py
1. Add `--stock-to-leave` parameter (default 1.5mm)
2. Add `--rough-stepover` and `--rough-stepdown` params (defaults: 10mm, 4mm)
3. Roughing pass: offset the heightmap Z by +stock_to_leave before generating waterline passes
4. Finishing pass: use original surface with tight stepover
5. Emit tool change comments between phases (sim ignores M6 but it documents intent)

```
(=== ROUGHING — 25mm flat end mill ===)
(Stock to leave: 1.5mm)
M3 S8000
G1 ... F1200   (aggressive feed, large tool)

(=== FINISHING — 8mm ball-nose ===)
M3 S10000
G1 ... F600    (slow feed, fine detail)
```

### In the simulator (future)
- Support M6 tool change command in gcode.rs parser
- Add tool library (multiple Tool structs with different radii/types)
- Swap active tool on M6 — different SDF shape for material removal
- FlatEnd tool already implemented in cutting/tool.rs

### Adaptive roughing (stretch goal)
- Current raster roughing wastes time cutting air
- Adaptive/trochoidal milling: spiral into material, constant chip load
- Much faster but complex toolpath generation
- Libraries: libarea, opencamlib (C++, Python bindings exist)

## Key Files
- `tools/stl_to_gcode/stl_to_gcode.py` — toolpath generator
- `crates/cutting/src/tool.rs` — Tool struct, already has FlatEnd + BallNose types
- `crates/control/src/gcode.rs` — parser, needs M6 support for tool changes
- `crates/control/src/carving.rs` — execution, needs tool swap logic
