# Meshing Performance Upgrade Plan — 2026-02-26

## Problem
Material removal is choppy when tool moves fast. Root cause: 8-chunk-per-frame meshing cap in `crates/material/src/mesher.rs` can't keep up when many chunks dirty at once.

## Fixes (in order of implementation)

### 1. Increase chunk budget
- `remesh_dirty_capped(max)` in mesher.rs — bump from 8 to 16 or 32
- Profile GPU headroom to find sweet spot
- 1-line change, test immediately

### 2. Predictive chunk loading
- Pre-dirty chunks along the known G-code toolpath ahead of the tool position
- Use `swept_bounds()` from tool.rs to predict which chunks will be hit
- ~1 day effort, eliminates visible pop-in

### 3. LOD meshing
- Coarse mesh (skip every other voxel) for chunks far from tool
- Full resolution only near active cutting zone
- Reduces total mesh vertex count significantly

### 4. GPU compute meshing (stretch goal)
- Move surface nets extraction to a wgpu compute shader
- Big effort but massive throughput gain (10-100x)
- Would remove meshing as a bottleneck entirely

## Key files
- `crates/material/src/mesher.rs` — chunk meshing + dirty cap
- `crates/material/src/octree.rs` — dirty chunk tracking, CSG subtraction
- `crates/material/src/async_mesher.rs` — background meshing
- `crates/cutting/src/tool.rs` — swept_bounds() for predictive loading
- `crates/sim/src/main.rs` — orchestration, remesh calls (~line 2584)
- `crates/control/src/carving.rs` — waypoint lookahead for prediction

## Context
- Octree resolution: 0.5mm, chunk = 32^3 voxels
- Workpiece: 305mm cube (~610^3 potential voxels, sparse)
- Current render: 8-pass PBR pipeline at ~60fps
- Physics: 1kHz fixed timestep, decoupled from render
