 Plan to implement                                                                                                                                              │
│                                                                                                                                                                │
│ SimuForge-Stone — Implementation Plan                                                                                                                          │
│                                                                                                                                                                │
│ Context                                                                                                                                                        │
│                                                                                                                                                                │
│ Build a fully custom digital twin platform in Rust that simulates a 6DOF robot arm carving stone sculptures. The primary test case is carving a marble bust of │
│  Alexander the Great. The platform is designed as a general-purpose virtual space — expansive and flexible — with the robot sculpting scenario as the first    │
│ validated use case.                                                                                                                                            │
│                                                                                                                                                                │
│ The entire stack is custom: physics engine, material simulation, renderer. No game engine, no ECS framework, no external physics library. Six total            │
│ dependencies.                                                                                                                                                  │
│                                                                                                                                                                │
│ Tech Stack                                                                                                                                                     │
│ ┌───────────────────┬──────────────────────────────────────────────────────────────┐                                                                           │
│ │       Crate       │                             Role                             │                                                                           │
│ ├───────────────────┼──────────────────────────────────────────────────────────────┤                                                                           │
│ │ wgpu              │ Rendering + compute shaders (Vulkan backend on Ubuntu 24.04) │                                                                           │
│ ├───────────────────┼──────────────────────────────────────────────────────────────┤                                                                           │
│ │ winit             │ Window creation + input handling                             │                                                                           │
│ ├───────────────────┼──────────────────────────────────────────────────────────────┤                                                                           │
│ │ nalgebra          │ Physics-side math (spatial vectors, Matrix6, Isometry3)      │                                                                           │
│ ├───────────────────┼──────────────────────────────────────────────────────────────┤                                                                           │
│ │ glam              │ Render-side math (fast SIMD, used by fast-surface-nets)      │                                                                           │
│ ├───────────────────┼──────────────────────────────────────────────────────────────┤                                                                           │
│ │ bytemuck          │ Zero-copy GPU buffer uploads                                 │                                                                           │
│ ├───────────────────┼──────────────────────────────────────────────────────────────┤                                                                           │
│ │ fast-surface-nets │ Chunk-based isosurface extraction from SDF                   │                                                                           │
│ └───────────────────┴──────────────────────────────────────────────────────────────┘                                                                           │
│ Shaders: WGSL (wgpu's native shader language, no external toolchain).                                                                                          │
│                                                                                                                                                                │
│ Project Structure                                                                                                                                              │
│                                                                                                                                                                │
│ SimuForge-Stone/                                                                                                                                               │
│ ├── Cargo.toml                  # workspace root                                                                                                               │
│ ├── PLAN.md                                                                                                                                                    │
│ ├── crates/                                                                                                                                                    │
│ │   ├── core/                   # shared types, transforms, coordinate conversions                                                                             │
│ │   │   ├── Cargo.toml                                                                                                                                         │
│ │   │   └── src/lib.rs                                                                                                                                         │
│ │   ├── physics/                # Featherstone ABA, joints, rigid body dynamics                                                                                │
│ │   │   ├── Cargo.toml                                                                                                                                         │
│ │   │   └── src/                                                                                                                                               │
│ │   │       ├── lib.rs                                                                                                                                         │
│ │   │       ├── spatial.rs      # spatial vectors, spatial inertia, spatial transforms                                                                         │
│ │   │       ├── joint.rs        # revolute joint type, limits, friction                                                                                        │
│ │   │       ├── aba.rs          # Featherstone articulated body algorithm                                                                                      │
│ │   │       └── arm.rs          # RobotArm struct, DH parameters, link inertias                                                                                │
│ │   ├── motors/                 # stepper models, gearboxes, PID control                                                                                       │
│ │   │   ├── Cargo.toml                                                                                                                                         │
│ │   │   └── src/                                                                                                                                               │
│ │   │       ├── lib.rs                                                                                                                                         │
│ │   │       ├── stepper.rs      # speed-torque curve, closed-loop model                                                                                        │
│ │   │       ├── gearbox.rs      # gear ratio, efficiency, backlash                                                                                             │
│ │   │       └── pid.rs          # PID controller                                                                                                               │
│ │   ├── material/               # sparse octree SDF, material removal, meshing                                                                                 │
│ │   │   ├── Cargo.toml                                                                                                                                         │
│ │   │   └── src/                                                                                                                                               │
│ │   │       ├── lib.rs                                                                                                                                         │
│ │   │       ├── octree.rs       # sparse octree with Sd8 leaves, dirty tracking                                                                                │
│ │   │       ├── sdf_ops.rs      # CSG subtract, swept tool volume                                                                                              │
│ │   │       └── mesher.rs       # chunk-based surface nets (wraps fast-surface-nets)                                                                           │
│ │   ├── cutting/                # cutting force model, tool geometry                                                                                           │
│ │   │   ├── Cargo.toml                                                                                                                                         │
│ │   │   └── src/                                                                                                                                               │
│ │   │       ├── lib.rs                                                                                                                                         │
│ │   │       ├── forces.rs       # specific cutting energy model for stone                                                                                      │
│ │   │       └── tool.rs         # tool geometry (ball nose, flat end, tapered)                                                                                 │
│ │   ├── control/                # IK, trajectory planning, G-code                                                                                              │
│ │   │   ├── Cargo.toml                                                                                                                                         │
│ │   │   └── src/                                                                                                                                               │
│ │   │       ├── lib.rs                                                                                                                                         │
│ │   │       ├── ik.rs           # damped least squares IK solver                                                                                               │
│ │   │       ├── trajectory.rs   # trajectory planner, velocity profiles                                                                                        │
│ │   │       └── gcode.rs        # G-code interpreter (G0, G1, G2/G3, A-axis)                                                                                   │
│ │   ├── render/                 # wgpu renderer, PBR, shaders                                                                                                  │
│ │   │   ├── Cargo.toml                                                                                                                                         │
│ │   │   └── src/                                                                                                                                               │
│ │   │       ├── lib.rs                                                                                                                                         │
│ │   │       ├── context.rs      # wgpu Instance, Device, Queue, Surface                                                                                        │
│ │   │       ├── camera.rs       # orbit camera, projection matrices                                                                                            │
│ │   │       ├── mesh.rs         # dynamic mesh buffer management, chunk uploads                                                                                │
│ │   │       ├── arm_visual.rs   # robot arm link rendering (geometric primitives)                                                                              │
│ │   │       ├── pipelines/                                                                                                                                     │
│ │   │       │   ├── mod.rs                                                                                                                                     │
│ │   │       │   ├── pbr.rs      # PBR metallic-roughness pipeline                                                                                              │
│ │   │       │   ├── shadow.rs   # shadow map depth pass                                                                                                        │
│ │   │       │   ├── ssao.rs     # screen-space ambient occlusion                                                                                               │
│ │   │       │   ├── sss.rs      # subsurface scattering blur passes                                                                                            │
│ │   │       │   └── composite.rs # final compositing + tone mapping                                                                                            │
│ │   │       └── shaders/                                                                                                                                       │
│ │   │           ├── pbr.wgsl                                                                                                                                   │
│ │   │           ├── shadow.wgsl                                                                                                                                │
│ │   │           ├── ssao.wgsl                                                                                                                                  │
│ │   │           ├── sss.wgsl                                                                                                                                   │
│ │   │           └── composite.wgsl                                                                                                                             │
│ │   └── sim/                    # main binary, simulation loop, state orchestration                                                                            │
│ │       ├── Cargo.toml                                                                                                                                         │
│ │       └── src/                                                                                                                                               │
│ │           └── main.rs         # fixed-timestep loop, state management, UI                                                                                    │
│                                                                                                                                                                │
│ Architecture                                                                                                                                                   │
│                                                                                                                                                                │
│ Data Flow                                                                                                                                                      │
│                                                                                                                                                                │
│ G-code / Toolpath                                                                                                                                              │
│        │                                                                                                                                                       │
│   IK Solver → joint angle targets                                                                                                                              │
│        │                                                                                                                                                       │
│   PID Control Loop (1kHz)                                                                                                                                      │
│        │                                                                                                                                                       │
│   Motor Model → torque output                                                                                                                                  │
│        │                                                                                                                                                       │
│   Featherstone ABA → joint accelerations → integrate → joint state                                                                                             │
│        │                                                                                                                                                       │
│   Forward Kinematics → tool tip position/orientation                                                                                                           │
│        │                                                                                                                                                       │
│   Material Removal (octree SDF subtract)  ←→  Cutting Force Model                                                                                              │
│        │                                                                                                                                                       │
│   Dirty chunk flags → incremental surface nets meshing                                                                                                         │
│        │                                                                                                                                                       │
│   wgpu chunk mesh upload → PBR render (60fps, decoupled from physics)                                                                                          │
│                                                                                                                                                                │
│ Physics-Render Separation                                                                                                                                      │
│                                                                                                                                                                │
│ Physics runs at 1kHz fixed timestep on the CPU. Rendering runs at ~60fps, decoupled. The classic "Fix Your Timestep" pattern:                                  │
│                                                                                                                                                                │
│ let physics_dt = 1.0 / 1000.0;                                                                                                                                 │
│ let mut accumulator = 0.0;                                                                                                                                     │
│ let mut prev_state = sim_state.clone();                                                                                                                        │
│                                                                                                                                                                │
│ loop {                                                                                                                                                         │
│     let frame_time = get_frame_time();                                                                                                                         │
│     accumulator += frame_time;                                                                                                                                 │
│                                                                                                                                                                │
│     while accumulator >= physics_dt {                                                                                                                          │
│         prev_state = sim_state.clone();                                                                                                                        │
│         physics_step(&mut sim_state, physics_dt);                                                                                                              │
│         accumulator -= physics_dt;                                                                                                                             │
│     }                                                                                                                                                          │
│                                                                                                                                                                │
│     let alpha = accumulator / physics_dt;                                                                                                                      │
│     let render_state = interpolate(&prev_state, &sim_state, alpha);                                                                                            │
│     render(&render_state);                                                                                                                                     │
│ }                                                                                                                                                              │
│                                                                                                                                                                │
│ Single-threaded for v1. ABA + SDF update takes <0.1ms per step — no threading needed.                                                                          │
│                                                                                                                                                                │
│ Simulation State (plain Rust structs, no ECS)                                                                                                                  │
│                                                                                                                                                                │
│ struct SimState {                                                                                                                                              │
│     arm: RobotArm,             // 7 joints, Featherstone state vectors                                                                                         │
│     motors: [MotorState; 7],   // per-joint stepper + PID state                                                                                                │
│     workpiece: OctreeSdf,      // sparse voxel field                                                                                                           │
│     tool: ToolState,           // position, rpm, wear accumulator                                                                                              │
│     rotary_table: RotaryState, // A-axis angle, velocity                                                                                                       │
│     time: f64,                 // simulation clock                                                                                                             │
│ }                                                                                                                                                              │
│                                                                                                                                                                │
│ SDF Material Removal Pipeline                                                                                                                                  │
│                                                                                                                                                                │
│ 1. Physics step moves tool tip → compute swept volume (capsule SDF for ball-nose cutter)                                                                       │
│ 2. Walk octree, find leaf nodes whose bounding box intersects swept volume                                                                                     │
│ 3. For each leaf: new_sdf = max(existing_sdf, -tool_sdf) (CSG subtraction)                                                                                     │
│ 4. Mark modified 32^3 chunks as dirty                                                                                                                          │
│ 5. Before render: remesh only dirty chunks via fast-surface-nets (~microseconds per chunk)                                                                     │
│ 6. Upload changed chunk vertex/index buffers to GPU via queue.write_buffer()                                                                                   │
│                                                                                                                                                                │
│ Grid: 0.5mm resolution on 1ft block → 600^3 logical → ~19^3 = 6859 chunks of 32^3. Tool touches 1-4 chunks per frame.                                          │
│                                                                                                                                                                │
│ Robot Arm Specification (from BOM)                                                                                                                             │
│                                                                                                                                                                │
│ DH Parameters:                                                                                                                                                 │
│ Joint | a (mm) | d (mm) | α (rad) | θ range      | Motor       | Gearbox                                                                                       │
│ J1    | 0      | 300    | -π/2    | ±180°        | NEMA34 12Nm | 100:1 planetary                                                                               │
│ J2    | 500    | 0      | 0       | -45° to 135° | NEMA34 12Nm | 100:1 planetary                                                                               │
│ J3    | 400    | 0      | 0       | ±135°        | NEMA34 8.5Nm| 100:1 planetary                                                                               │
│ J4    | 0      | 0      | -π/2    | ±180°        | NEMA23 3Nm  | 50:1 planetary                                                                                │
│ J5    | 0      | 300    | π/2     | ±120°        | NEMA23 3Nm  | 50:1 planetary                                                                                │
│ J6    | 0      | 80     | 0       | ±360°        | NEMA23 2Nm  | 50:1 planetary                                                                                │
│ RT    | —      | —      | —       | ±360°        | NEMA34 8.5Nm| 80:1 worm                                                                                     │
│                                                                                                                                                                │
│ Total reach: ~1200mm (~4ft).                                                                                                                                   │
│                                                                                                                                                                │
│ ---                                                                                                                                                            │
│ Implementation Phases                                                                                                                                          │
│                                                                                                                                                                │
│ Phase 1 — Foundation + Physics Core                                                                                                                            │
│                                                                                                                                                                │
│ Goal: Workspace builds, arm simulates correctly in headless mode, basic wgpu window opens.                                                                     │
│                                                                                                                                                                │
│ Steps:                                                                                                                                                         │
│                                                                                                                                                                │
│ 1. Workspace setup — Create root Cargo.toml (workspace), all crate Cargo.toml files, stub lib.rs/main.rs for each crate. Verify cargo build succeeds.          │
│ 2. core crate — Shared types: Transform, coordinate conversion helpers. Thin wrappers around nalgebra types. Keep minimal — only what multiple crates need.    │
│ 3. physics crate — The heart of the simulation.                                                                                                                │
│   - spatial.rs — Spatial vector types (SpatialVector = Vector6<f64>, SpatialInertia = Matrix6<f64>), spatial cross product, spatial transforms from DH params. │
│   - joint.rs — RevoluteJoint struct: angle, velocity, torque, limits, viscous/coulomb friction, backlash deadband.                                             │
│   - aba.rs — Featherstone's Articulated Body Algorithm. Three-pass O(n): (1) outward velocity propagation, (2) inward articulated inertia, (3) outward         │
│ acceleration. ~300 lines.                                                                                                                                      │
│   - arm.rs — RobotArm struct: 7 joints, DH parameter table, link masses/inertias, gravity vector. Methods: forward_dynamics(torques) → accelerations,          │
│ forward_kinematics() → tool_pose, jacobian() → Matrix6xN.                                                                                                      │
│ 4. motors crate — Per-joint motor + control.                                                                                                                   │
│   - stepper.rs — Speed-torque curve (pull-out torque vs RPM lookup), gear ratio application, efficiency loss.                                                  │
│   - gearbox.rs — Ratio, backlash deadband model, efficiency.                                                                                                   │
│   - pid.rs — PID controller with anti-windup. Target position → torque command.                                                                                │
│ 5. render crate (scaffold only) — Get a wgpu window open with a colored background. Just context.rs + camera.rs + basic clear-color render pass. Proves the    │
│ GPU pipeline works.                                                                                                                                            │
│ 6. sim crate (v1) — Headless sim loop. Create arm, apply gravity, verify joints respond correctly. Print joint angles to terminal. Add basic wgpu window from  │
│ render scaffold.                                                                                                                                               │
│                                                                                                                                                                │
│ Validation: Arm falls under gravity with no motor torques. PID holding a target position shows damped oscillation then convergence. Joint torques stay within  │
│ motor limits.                                                                                                                                                  │
│                                                                                                                                                                │
│ Phase 2 — Material System + Cutting                                                                                                                            │
│                                                                                                                                                                │
│ Goal: Stone block exists as SDF, tool carves into it, mesh updates in real-time on screen.                                                                     │
│                                                                                                                                                                │
│ Steps:                                                                                                                                                         │
│                                                                                                                                                                │
│ 1. material crate — octree + SDF                                                                                                                               │
│   - octree.rs — Sparse octree with i8 (Sd8) leaf values. Nodes: Interior([Box<Node>; 8]), Leaf(i8), Solid, Air. Methods: sample(point) → f32,                  │
│ subtract(tool_sdf, bounds), mark_dirty(chunk_coords).                                                                                                          │
│   - sdf_ops.rs — Swept capsule SDF (ball-nose cutter), CSG max operation. fn swept_capsule(p, start, end, radius) -> f32.                                      │
│   - mesher.rs — Chunk-based meshing. For each dirty chunk: extract padded 34^3 sample grid, call fast_surface_nets::surface_nets(), return positions + normals │
│  + indices. Dirty flag management.                                                                                                                             │
│ 2. cutting crate                                                                                                                                               │
│   - tool.rs — Tool geometry params (ball nose radius, shank, length). Swept volume given start/end positions.                                                  │
│   - forces.rs — fn cutting_force(specific_energy: f32, chip_area: f32, doc: f32) -> Vec3. Marble baseline: 20 N/mm². Force vector from voxel intersection      │
│ count + tool direction.                                                                                                                                        │
│ 3. Render integration — Extend render crate with:                                                                                                              │
│   - mesh.rs — DynamicChunkMesh: per-chunk vertex/index GPU buffers. Upload only changed chunks.                                                                │
│   - Basic flat-shaded or simple lit rendering of the workpiece mesh. PBR comes in Phase 3.                                                                     │
│   - Render the arm as colored cylinders/boxes matching link dimensions.                                                                                        │
│ 4. Sim integration — Wire it all together:                                                                                                                     │
│   - Tool tip from FK feeds into SDF subtraction                                                                                                                │
│   - Cutting forces feed back into Featherstone dynamics                                                                                                        │
│   - Dirty chunks remesh and upload each frame                                                                                                                  │
│   - Arm rendered with current joint angles                                                                                                                     │
│                                                                                                                                                                │
│ Validation: Move tool through block via manual joint commands → material visibly removed. Cutting forces cause arm deflection. Mesh updates are smooth with no │
│  gaps between chunks.                                                                                                                                          │
│                                                                                                                                                                │
│ Phase 3 — Renderer Polish + Control + Integration                                                                                                              │
│                                                                                                                                                                │
│ Goal: Beautiful marble rendering. Full toolpath execution. Complete digital twin.                                                                              │
│                                                                                                                                                                │
│ Steps:                                                                                                                                                         │
│                                                                                                                                                                │
│ 1. PBR pipeline (pipelines/pbr.rs + pbr.wgsl)                                                                                                                  │
│   - Metallic-roughness workflow. Marble: roughness ~0.3, metallic 0.0, base color from procedural noise.                                                       │
│   - Directional + point lights. Normal mapping from SDF gradient.                                                                                              │
│ 2. Shadow mapping (pipelines/shadow.rs + shadow.wgsl)                                                                                                          │
│   - Single directional light shadow map. Depth-only pass, PCF filtering.                                                                                       │
│ 3. SSAO (pipelines/ssao.rs + ssao.wgsl)                                                                                                                        │
│   - Screen-space ambient occlusion. Hemisphere kernel sampling from depth + normal buffer. Blur pass.                                                          │
│ 4. SSS for marble (pipelines/sss.rs + sss.wgsl)                                                                                                                │
│   - Screen-space subsurface scattering approximation. Translucency from depth difference. Diffusion blur on irradiance buffer. This is what makes marble look  │
│ like marble.                                                                                                                                                   │
│ 5. Compositing (pipelines/composite.rs + composite.wgsl)                                                                                                       │
│   - Combine PBR color + SSAO + SSS + shadows. Tone mapping (ACES). Gamma correction.                                                                           │
│ 6. Camera — Orbit controls (mouse drag rotate, scroll zoom, middle-click pan). Snap-to-tool and snap-to-workpiece presets.                                     │
│ 7. control crate                                                                                                                                               │
│   - ik.rs — Damped least squares IK. Jacobian from arm.jacobian(), pseudoinverse with damping near singularities. Joint limit clamping.                        │
│   - gcode.rs — G-code interpreter: G0 (rapid), G1 (linear), G2/G3 (arc), A-axis (rotary). Parse file → sequence of tool poses.                                 │
│   - trajectory.rs — Trapezoidal velocity profiles between waypoints. Feed rate control.                                                                        │
│ 8. UI overlays — Real-time text overlay showing: joint angles, joint torques, motor temps (estimated), cutting force magnitude, spindle RPM, simulation time.  │
│ Play/pause/step controls (keyboard).                                                                                                                           │
│ 9. Full integration test — Load Alexander bust STL → generate rough toolpath → execute in simulation → arm carves bust from marble block → visually validate   │
│ result.                                                                                                                                                        │
│                                                                                                                                                                │
│ Validation: Marble looks convincing with SSS. Shadows and SSAO add depth. Toolpath executes smoothly. Joint torques stay within BOM motor limits throughout    │
│ carving. Frame rate >30fps during active carving.                                                                                                              │
│                                                                                                                                                                │
│ ---                                                                                                                                                            │
│ Verification Plan                                                                                                                                              │
│                                                                                                                                                                │
│ 1. cargo build — entire workspace compiles with no warnings                                                                                                    │
│ 2. Physics unit tests — ABA: arm at rest under gravity produces correct reaction torques. FK matches known analytical solutions for simple configurations.     │
│ 3. Motor tests — PID converges to target within expected settling time. Torque limits are respected.                                                           │
│ 4. SDF tests — Subtract a sphere from a filled block → verify volume removed matches analytical sphere volume. Chunk dirty flags propagate correctly.          │
│ 5. Meshing tests — Surface nets produces watertight mesh from test SDF. No gaps at chunk boundaries.                                                           │
│ 6. Visual integration — Run the full sim, visually confirm: arm moves, material is removed, mesh updates, marble looks good, no rendering artifacts.           │
│ 7. Performance — Physics step <0.1ms (1kHz headroom). Chunk remesh <1ms. Render frame <16ms (60fps).                                                           │
╰──────────────────────────────────────────────────────────────────────────────────────────────────────────