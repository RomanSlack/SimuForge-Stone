# SimuForge-Stone

![Rust](https://img.shields.io/badge/Rust-000000?style=flat&logo=rust&logoColor=white)
![wgpu](https://img.shields.io/badge/wgpu-4B8BBE?style=flat)
![nalgebra](https://img.shields.io/badge/nalgebra-f64-blue?style=flat)
![glam](https://img.shields.io/badge/glam-f32-blue?style=flat)
![Vulkan](https://img.shields.io/badge/Vulkan-AC162C?style=flat&logo=vulkan&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-green?style=flat)

A fully custom digital twin platform simulating a 6DOF robot arm carving stone sculptures. Physics engine, material simulation, and renderer are all built from scratch in Rust. No game engine, no ECS, no external physics library.

The first validated use case is a NEMA-stepper-driven articulated arm carving a marble bust of Alexander the Great from a 1ft block. The platform generalizes to any robot arm and any subtractive manufacturing process.

See [`refs/`](refs/) for the original design conversation and research notes that led to this architecture.

## Architecture

```
G-code / Toolpath
       |
  IK Solver --> joint angle targets
       |
  PID Control Loop (1 kHz)
       |
  Motor Model --> torque output
       |
  Featherstone ABA --> joint accelerations --> integrate --> joint state
       |
  Forward Kinematics --> tool tip position
       |
  Material Removal (octree SDF subtract)  <-->  Cutting Force Model
       |
  Dirty chunk flags --> incremental surface nets meshing
       |
  wgpu chunk mesh upload --> PBR render (~60 fps, decoupled)
```

Physics runs at a **1 kHz fixed timestep** on the CPU. Rendering is decoupled at ~60 fps using the classic "Fix Your Timestep" accumulator pattern. Single-threaded — ABA + SDF update completes in <0.1 ms per step.

## Crate Layout

| Crate | Purpose |
|-------|---------|
| `core` | Shared types, DH parameters, coordinate conversions (nalgebra <-> glam) |
| `physics` | Featherstone ABA, spatial vectors, revolute joints, robot arm model |
| `motors` | Stepper motor speed-torque curves, planetary gearboxes, PID controllers |
| `material` | Sparse octree SDF (Sd8 leaves), CSG subtraction, chunk-based surface nets meshing |
| `cutting` | Specific cutting energy model for stone, tool geometry (ball nose, flat, tapered) |
| `control` | Damped least-squares IK, trapezoidal trajectory planner, G-code interpreter |
| `render` | wgpu Vulkan renderer: PBR, shadow maps, SSAO, subsurface scattering, compositing |
| `sim` | Main binary: simulation loop, state orchestration, Cartesian IK control, overlays |

## Dependencies

Six external crates. Everything else is custom.

| Crate | Role |
|-------|------|
| `wgpu` 24 | Rendering + compute (Vulkan backend) |
| `winit` 0.30 | Window creation + input |
| `nalgebra` 0.33 | Physics-side math (Matrix6, Isometry3, spatial vectors) |
| `glam` 0.29 | Render-side math (SIMD f32, required by fast-surface-nets) |
| `bytemuck` | Zero-copy GPU buffer uploads |
| `fast-surface-nets` | Chunk-based isosurface extraction from SDF |

## Robot Arm Specification

Modeled from a real BOM (~$3K build). DH convention with Z-up.

| Joint | a (mm) | d (mm) | alpha | Range | Motor | Gearbox |
|-------|--------|--------|-------|-------|-------|---------|
| J1 | 0 | 300 | -pi/2 | +/-180 deg | NEMA34 12 Nm | 100:1 planetary |
| J2 | 500 | 0 | 0 | -45 to 135 deg | NEMA34 12 Nm | 100:1 planetary |
| J3 | 400 | 0 | 0 | +/-135 deg | NEMA34 8.5 Nm | 100:1 planetary |
| J4 | 0 | 0 | -pi/2 | +/-180 deg | NEMA23 3 Nm | 50:1 planetary |
| J5 | 0 | 300 | pi/2 | +/-120 deg | NEMA23 3 Nm | 50:1 planetary |
| J6 | 0 | 80 | 0 | +/-360 deg | NEMA23 2 Nm | 50:1 planetary |
| RT | -- | -- | -- | +/-360 deg | NEMA34 8.5 Nm | 80:1 worm |

Total reach: ~1200 mm. Reflected motor inertia through gearboxes dominates link inertia by 40-670x and is modeled in the ABA dynamics.

## Key Physics Details

- **Featherstone ABA** -- O(n) forward dynamics with reflected motor inertia (`I_rotor * ratio^2`) added to the diagonal inertia term. Critical for geared systems where motor inertia dominates link inertia.
- **RNEA gravity compensation** -- Recursive Newton-Euler algorithm computes exact gravity torques including inter-joint coupling. Applied as feedforward in the control loop.
- **PID with anti-windup** -- Per-joint position controllers tuned for the high reflected inertia. Motor-side torque commands are amplified through gearbox ratio and efficiency.
- **SDF material removal** -- Swept capsule subtraction on a sparse octree at 0.5 mm resolution. Only dirty 32^3 chunks are remeshed per frame via fast-surface-nets.

## Controls

| Key | Action |
|-----|--------|
| Space | Play / pause simulation |
| W/A/S/D | Move Cartesian target forward/left/back/right (150 mm/s) |
| Q/E | Move target down/up |
| Shift + movement | Fine mode (50 mm/s) |
| X | Toggle spindle on/off |
| R | Reset target to current tool position |
| Shift+R | Full reset: arm pose, velocities, PID state, and target |
| Mouse drag | Orbit camera |
| Scroll | Zoom |
| Middle click drag | Pan |

## Build and Run

```
cargo build
cargo test
cargo run
```

Requires Vulkan-capable GPU. Tested on Ubuntu 24.04 with standard Mesa/NVIDIA drivers.

## Current Status

Phase 1 and Phase 2 are complete. Phase 3 is in progress.

**Done:**
- Featherstone ABA with reflected motor inertia
- RNEA gravity compensation
- PID position control with motor/gearbox chain
- Sparse octree SDF with CSG subtraction and chunk-based surface nets meshing
- Cutting force model (specific cutting energy for marble)
- wgpu PBR renderer with shadow maps, SSAO, SSS, and ACES tonemapping
- Cartesian IK control (WASD/QE moves a target point, arm follows via damped least-squares IK)
- Real-time overlay (joint angles, torques, tool position, IK status, cutting force)
- 41 tests across all crates, all passing

**Remaining:**
- G-code toolpath execution (interpreter exists, not yet wired to sim loop)
- Rotary table integration (A-axis, modeled but not yet driven)
- Auto-toolpath generation from STL (slice + roughing + finishing passes)
- Alexander bust carving end-to-end validation

## Project Vision

This platform is designed as a general-purpose digital twin environment. The stone-carving robot arm is the first scenario, but the architecture supports any articulated manipulator with any end effector and any subtractive (or additive) manufacturing process. The same arm model can drive a physical build via the real BOM components once the simulation is validated.

## License

Apache 2.0
