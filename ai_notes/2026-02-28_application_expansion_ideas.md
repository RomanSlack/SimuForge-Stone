# Application Expansion Ideas
**Date**: 2026-02-28

The 6DOF robot arm + SDF material system is general-purpose. The carving use case
proved out the full stack (physics, IK, G-code, material sim, rendering). Expanding
to other manufacturing processes is mostly swapping the tool model and CSG operation.

---

## 1. Metal Additive Manufacturing (DED / WAAM)

Directed Energy Deposition or Wire Arc Additive Manufacturing. Same robot arm,
different end effector (welding torch or laser head with powder/wire feed).

**What changes:**
- CSG operation flips from `max()` (subtract) to `min()` (union/deposit)
- OctreeSdf needs a `union()` method that CREATES new chunks (subtract skips empty space)
- Workpiece starts as an empty build plate instead of a solid block
- Tool SDF becomes a deposition bead (swept capsule matching nozzle width)
- G-code is simpler (linear moves + extrusion rate, no complex multi-axis contouring)

**What stays the same:**
- Robot arm, IK, physics, motor models — identical
- Meshing pipeline, rendering, chunk management — all works as-is
- Timelapse mode for visualizing layer-by-layer buildup

**Stretch goals:**
- Thermal visualization (color-code mesh by temperature/time-since-deposit)
- Bead overlap simulation (multiple passes building up layers)
- Hybrid: deposit then machine to final tolerance (additive + subtractive in one sim)

**Difficulty: Low** — core change is ~50 lines for `union()` + new tool SDF

---

## 2. Welding Operations

Robot arc welding along seams/joints. Very similar to DED but the goal is
joining parts rather than building geometry.

**What changes:**
- Load two or more part STLs as separate SDF volumes
- Weld bead deposits material (SDF union) along the seam between parts
- Tool model: welding torch with specific bead cross-section profile
- G-code or teach-pendant style waypoints along seam paths

**What stays the same:**
- Everything from the arm/IK/physics side

**Visual appeal:**
- Could show the weld bead building up in real-time
- Color coding: fresh weld (orange/red) → cooled (silver)

**Difficulty: Low-Medium** — needs multi-body SDF support (multiple workpieces)

---

## 3. Painting / Coating / Spray

Robot arm with spray gun end effector. Instead of adding/removing geometry,
modify a surface property (color, thickness).

**What changes:**
- No CSG operations — instead, per-vertex or per-voxel color/thickness attribute
- Tool model: spray cone with falloff (not a boolean SDF op)
- Rendering: vertex colors or texture splatting instead of uniform material

**What stays the same:**
- Arm, IK, G-code path following

**Difficulty: Medium** — needs surface attribute system (not just SDF sign/distance)

---

## 4. Pick and Place / Assembly

Robot arm picking parts from a tray and assembling them. Different from
manufacturing — no material modification, just rigid body manipulation.

**What changes:**
- Gripper end effector instead of tool
- Collision detection becomes critical (not just self-collision but environment)
- Multiple rigid bodies in the scene
- Grasp planning / placement accuracy

**What stays the same:**
- Arm kinematics, IK, physics, rendering

**Difficulty: Medium-High** — needs rigid body dynamics, grasp simulation

---

## 5. Inspection / Scanning

Robot arm with a probe or laser scanner, measuring a workpiece surface
and comparing to a CAD reference.

**What changes:**
- No material modification
- Tool is a measurement probe (touch or non-contact)
- Visualization: deviation heatmap (measured vs reference surface)
- Path planning for coverage

**What stays the same:**
- Arm, IK, path following

**Difficulty: Medium** — needs comparison/deviation visualization

---

## 6. Multi-Process (The Big One)

Combine subtractive + additive + inspection in a single workflow:
1. Rough-cut a near-net shape (subtractive, current capability)
2. Deposit material where needed (additive, idea #1)
3. Finish-machine to final tolerance (subtractive again)
4. Inspect result (idea #5)

This is what real advanced manufacturing cells do. The simulation already
has the arm and material system — just needs tool-change logic and the
additive CSG operation.

**Difficulty: Medium** — mostly integration, each piece is individually simple

---

## Priority Ranking

| Idea | Effort | Visual Impact | Uniqueness |
|------|--------|--------------|------------|
| Metal 3D printing (DED) | Low | High | High |
| Welding | Low-Med | High | Medium |
| Multi-process hybrid | Medium | Very High | Very High |
| Painting/coating | Medium | Medium | Medium |
| Inspection | Medium | Medium | Low |
| Pick and place | Med-High | Low | Low |

**Recommended next:** Metal additive (DED) — lowest effort, highest impact,
and it directly reuses everything we just built. The timelapse of a metal
part being 3D printed layer by layer by a robot arm would be visually striking.
