//! SimuForge-Stone — Digital twin simulation of a 6DOF robot arm carving stone.
//!
//! Main binary: fixed-timestep physics loop + wgpu rendering.

use std::sync::Arc;
use std::time::Instant;

use glam::{Mat4, Quat, Vec3, Vec4};
use nalgebra::{UnitQuaternion, Vector3};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId};

use simuforge_control::carving::{CarvingSession, CarvingState};
use simuforge_control::ik::IkSolver;
use simuforge_core::{isometry_to_glam, Vertex};
use simuforge_cutting::forces::{self, MaterialProps};
use simuforge_cutting::tool::{Tool, ToolState};
use simuforge_material::async_mesher::AsyncMesher;
use simuforge_material::mesher;
use simuforge_material::octree::OctreeSdf;
use simuforge_motors::gearbox::Gearbox;
use simuforge_motors::pid;
use simuforge_motors::stepper::{MotorState, StepperMotor};
use simuforge_physics::aba::{articulated_body_algorithm, gravity_compensation_rnea};
use simuforge_physics::arm::RobotArm;
use simuforge_physics::joint::RotaryTable;
use simuforge_render::arm_visual::{generate_cylinder, generate_sphere, generate_box};
use simuforge_render::camera::{CameraUniform, OrbitCamera};
use simuforge_render::context::RenderContext;
use simuforge_render::mesh::ChunkMeshManager;
use simuforge_render::pipelines::pbr::{LightUniform, MaterialUniform, PbrPipeline};
use simuforge_render::pipelines::shadow::ShadowPipeline;
use simuforge_render::pipelines::ssao::SsaoPipeline;
use simuforge_render::pipelines::sss::SssPipeline;
use simuforge_render::pipelines::composite::CompositePipeline;
use simuforge_render::pipelines::line::{LinePipeline, LineVertex};
use simuforge_render::pipelines::overlay::{OverlayBuilder, OverlayPipeline};

mod config;
use config::SimConfig;

/// Coordinate swap: DH Z-up physics → Y-up renderer.
fn coord_swap_matrix() -> Mat4 {
    Mat4::from_cols(
        Vec4::new(1.0, 0.0, 0.0, 0.0),  // x → x
        Vec4::new(0.0, 0.0, -1.0, 0.0),  // y → -z
        Vec4::new(0.0, 1.0, 0.0, 0.0),   // z → y
        Vec4::new(0.0, 0.0, 0.0, 1.0),
    )
}

/// Model matrix for a unit cylinder (r=1, h=1) to connect two render-space points with given radius.
fn link_model(from: Vec3, to: Vec3, radius: f32) -> Mat4 {
    let diff = to - from;
    let len = diff.length();
    if len < 0.001 {
        // Coincident frames: draw a small sphere-like shape
        return Mat4::from_scale_rotation_translation(
            Vec3::splat(radius * 2.0),
            Quat::IDENTITY,
            from,
        );
    }
    let mid = (from + to) * 0.5;
    let dir = diff / len;
    let rot = Quat::from_rotation_arc(Vec3::Y, dir);
    Mat4::from_scale_rotation_translation(Vec3::new(radius, len, radius), rot, mid)
}

/// GPU mesh handle (vertex + index buffers).
struct GpuMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    num_indices: u32,
}

/// Material bind group with its own buffer (shares camera/light from pipeline).
struct MaterialBind {
    buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

/// Robot arm link radii for rendering (scaled up for visibility).
const ARM_LINK_RADII: [f32; 6] = [0.08, 0.07, 0.06, 0.05, 0.05, 0.04];

/// Arm link colors [R, G, B, A].
const ARM_LINK_COLORS: [[f32; 4]; 6] = [
    [0.3, 0.3, 0.35, 1.0],  // J1: dark gray (base)
    [0.8, 0.4, 0.1, 1.0],   // J2: orange (upper arm)
    [0.8, 0.4, 0.1, 1.0],   // J3: orange (forearm)
    [0.2, 0.2, 0.25, 1.0],  // J4: dark (wrist 1)
    [0.2, 0.2, 0.25, 1.0],  // J5: dark (wrist 2)
    [0.6, 0.6, 0.65, 1.0],  // J6: light gray (flange)
];

/// Physics timestep: 1kHz.
const PHYSICS_DT: f64 = 1.0 / 1000.0;
/// Maximum accumulated time before clamping (prevent spiral of death).
const MAX_FRAME_TIME: f64 = 0.05;
/// Simulation mode: manual keyboard control or automated G-code carving.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SimMode {
    Manual,
    Carving,
}

/// Speed multiplier presets for carving.
const SPEED_PRESETS: &[f64] = &[0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0, 500.0, 1000.0];

/// Full simulation state.
#[allow(dead_code)]
struct SimState {
    arm: RobotArm,
    motors: Vec<MotorState>,
    gearboxes: Vec<Gearbox>,
    pid_controllers: Vec<simuforge_motors::pid::PidController>,
    joint_targets: Vec<f64>,
    workpiece: OctreeSdf,
    /// Workpiece center in DH world-space (meters).
    /// The SDF is always centered at origin; this offset maps tool→workpiece coords.
    workpiece_center: Vector3<f64>,
    /// Workpiece half-extent (meters). The block is a cube of side = 2 * half_extent.
    workpiece_half_extent: f64,
    tool: Tool,
    tool_state: ToolState,
    rotary_table: RotaryTable,
    material: MaterialProps,
    ik_solver: IkSolver,
    sim_time: f64,
    paused: bool,
    cutting_force_magnitude: f64,
    /// Cartesian target pose in DH world-space (position + orientation).
    /// The arm's IK solver tracks this pose.
    cartesian_target: nalgebra::Isometry3<f64>,
    /// Whether IK converged on last solve.
    ik_converged: bool,
    /// Carving session (loaded G-code program).
    carving: Option<CarvingSession>,
    /// Current mode: Manual or Carving.
    mode: SimMode,
    /// Arm must be within this distance of target before cutting begins (m).
    tracking_threshold: f64,
}

impl SimState {
    fn new(cfg: &SimConfig) -> Self {
        let arm = RobotArm::default_6dof();
        let n = arm.num_joints();

        // Motors matching BOM
        let motors = vec![
            MotorState::new(StepperMotor::nema34(12.0)), // J1
            MotorState::new(StepperMotor::nema34(12.0)), // J2
            MotorState::new(StepperMotor::nema34(8.5)),  // J3
            MotorState::new(StepperMotor::nema23(3.0)),  // J4
            MotorState::new(StepperMotor::nema23(3.0)),  // J5
            MotorState::new(StepperMotor::nema23(2.0)),  // J6
        ];

        let gearboxes = vec![
            Gearbox::planetary(100.0), // J1
            Gearbox::planetary(100.0), // J2
            Gearbox::planetary(100.0), // J3
            Gearbox::planetary(50.0),  // J4
            Gearbox::planetary(50.0),  // J5
            Gearbox::planetary(50.0),  // J6
        ];

        let pid_controllers = vec![
            pid::large_joint_pid(),
            pid::large_joint_pid(),
            pid::large_joint_pid(),
            pid::small_joint_pid(),
            pid::small_joint_pid(),
            pid::small_joint_pid(),
        ];

        // Initial joint targets: arm positioned to reach workpiece from above
        let mut joint_targets = vec![0.0; n];
        joint_targets[1] = 0.6;   // J2: shoulder raised
        joint_targets[2] = -0.6;  // J3: elbow bent down

        // Set initial joint angles to match targets (start at the target pose)
        let mut arm = arm;
        arm.tool_offset = cfg.tool_offset;
        arm.set_joint_angles(&joint_targets);

        let he = cfg.workpiece_half_extent;
        let workpiece = OctreeSdf::new_block(
            [he as f32, he as f32, he as f32],
            cfg.workpiece_resolution as f32,
        );

        let workpiece_center = Vector3::new(
            cfg.workpiece_center[0],
            cfg.workpiece_center[1],
            cfg.workpiece_center[2],
        );

        let tool = match cfg.tool_type.as_str() {
            "ball_nose" => Tool::ball_nose(cfg.tool_radius_mm, cfg.tool_cutting_length_mm),
            _ => Tool::flat_end(cfg.tool_radius_mm, cfg.tool_cutting_length_mm),
        };
        let mut tool_state = ToolState::new();
        // Initialize frame_start_position to the current tool tip
        // so the first frame doesn't create a huge swept path from origin.
        let initial_tool_pos = arm.tool_position();
        tool_state.position = initial_tool_pos;
        tool_state.prev_position = initial_tool_pos;
        tool_state.frame_start_position = initial_tool_pos;

        let rotary_table = RotaryTable::new(5.0); // 5 kg·m² inertia

        // Cartesian target = initial tool tip pose (position + orientation)
        let cartesian_target = arm.forward_kinematics();

        eprintln!(
            "Config: tool={}({:.1}mm), workpiece=({:.2},{:.2},{:.2}), speed={}x",
            cfg.tool_type, cfg.tool_radius_mm,
            cfg.workpiece_center[0], cfg.workpiece_center[1], cfg.workpiece_center[2],
            cfg.default_speed,
        );

        Self {
            arm,
            motors,
            gearboxes,
            pid_controllers,
            joint_targets,
            workpiece,
            workpiece_center,
            workpiece_half_extent: he,
            tool,
            tool_state,
            rotary_table,
            material: MaterialProps::marble(),
            ik_solver: IkSolver {
                damping: 0.05,
                max_iterations: 50,
                position_tolerance: 5e-4,
                orientation_tolerance: 0.005,
                max_step: 0.2,
            },
            sim_time: 0.0,
            paused: true,
            cutting_force_magnitude: 0.0,
            cartesian_target,
            ik_converged: true,
            carving: None,
            mode: SimMode::Manual,
            tracking_threshold: cfg.tracking_threshold,
        }
    }

    /// Solve IK: update joint_targets from cartesian_target.
    /// In carving mode, uses position-only IK (3DOF) — the arm finds its own
    /// natural orientation, avoiding the orientation-fight that causes bobbing.
    /// In manual mode, uses full 6DOF pose IK so the user can control orientation.
    /// Rate-limits joint target changes to prevent flopping at workspace boundaries.
    fn update_ik(&mut self) {
        let mut scratch = self.arm.clone();
        scratch.set_joint_angles(&self.joint_targets);

        let converged = if self.mode == SimMode::Carving {
            // Position-only IK: no orientation constraint → smooth tracking
            let target_pos = self.cartesian_target.translation.vector;
            let (ok, _, _) = self.ik_solver.solve_position(&mut scratch, &target_pos);
            ok
        } else {
            // Full 6DOF IK: user controls orientation via IJKL keys
            let (ok, _, _) = self.ik_solver.solve(&mut scratch, &self.cartesian_target);
            ok
        };
        self.ik_converged = converged;

        // Rate-limit: max joint angle change per frame (~60fps).
        // Prevents oscillation when IK can't fully converge at workspace boundary.
        let max_delta = 0.05; // ~3 deg/frame → ~180 deg/s max slew rate
        let new_angles = scratch.joint_angles();
        for i in 0..self.joint_targets.len() {
            let delta = new_angles[i] - self.joint_targets[i];
            self.joint_targets[i] += delta.clamp(-max_delta, max_delta);
        }
    }

    /// Reset arm, joint targets, velocities, PID state, and cartesian target to initial pose.
    fn reset_to_initial(&mut self) {
        let n = self.arm.num_joints();

        // Initial joint targets: J2=0.6, J3=-0.6, rest 0
        self.joint_targets = vec![0.0; n];
        self.joint_targets[1] = 0.6;
        self.joint_targets[2] = -0.6;

        // Set arm angles to match and zero velocities
        self.arm.set_joint_angles(&self.joint_targets);
        for j in self.arm.joints.iter_mut() {
            j.velocity = 0.0;
            j.torque = 0.0;
        }

        // Reset PID controllers
        for pid in &mut self.pid_controllers {
            pid.reset();
        }

        // Reset motor states
        for motor in &mut self.motors {
            motor.output_torque = 0.0;
            motor.thermal_energy = 0.0;
        }

        // Reset tool state to initial tool position
        let initial_tool_pos = self.arm.tool_position();
        self.tool_state.position = initial_tool_pos;
        self.tool_state.prev_position = initial_tool_pos;
        self.tool_state.frame_start_position = initial_tool_pos;

        // Reset cartesian target (full pose)
        self.cartesian_target = self.arm.forward_kinematics();
        self.ik_converged = true;
        self.cutting_force_magnitude = 0.0;
    }

    /// Run one physics step at 1kHz.
    fn physics_step(&mut self) {
        if self.paused {
            return;
        }

        let n = self.arm.num_joints();
        let gravity = Vector3::new(0.0, 0.0, -9.81); // DH convention: Z-up

        // Exact gravity compensation via RNEA (includes inter-joint coupling)
        let grav_comp = gravity_compensation_rnea(&self.arm, &gravity);

        // PID control: compute motor torque commands
        let mut joint_torques = vec![0.0; n];
        for i in 0..n {
            let pid_output = self.pid_controllers[i].update(
                self.joint_targets[i],
                self.arm.joints[i].angle,
                PHYSICS_DT,
            );

            // Feedforward: grav_comp[i] is the exact joint torque to hold against gravity.
            // Convert to motor-side: divide by gearbox amplification.
            let grav_ff = grav_comp[i] / (self.gearboxes[i].ratio * self.gearboxes[i].efficiency);
            let total_motor_cmd = pid_output + grav_ff;

            // Motor torque clamping (respects speed-torque curve)
            self.motors[i].motor.update_velocity(
                self.arm.joints[i].velocity,
                self.gearboxes[i].ratio,
            );
            let motor_torque = self.motors[i].apply_torque(total_motor_cmd, PHYSICS_DT as f64);

            // Gearbox amplification
            let joint_torque = self.gearboxes[i].motor_to_joint_torque(motor_torque);
            self.arm.joints[i].torque = joint_torque;
            joint_torques[i] = self.arm.joints[i].net_torque();
        }

        // Forward dynamics (Featherstone ABA)
        let accelerations = articulated_body_algorithm(&self.arm, &joint_torques, &gravity);

        // Integrate + hard-clamp joint limits (safety net)
        for i in 0..n {
            self.arm.joints[i].integrate(accelerations[i], PHYSICS_DT);
            self.arm.joints[i].clamp_to_limits();
        }

        // Forward kinematics → tool position
        let tool_pose = self.arm.forward_kinematics();
        let tool_pos = tool_pose.translation.vector;
        self.tool_state.update_position(tool_pos);

        // Rotary table
        self.rotary_table.step(PHYSICS_DT);

        self.sim_time += PHYSICS_DT;
    }

    /// Load a G-code file into a carving session and switch to carving mode.
    fn load_gcode(&mut self, path: &str) -> Result<(), String> {
        // G-code origin = top-center of workpiece (Z=0 is the surface).
        let half_extent = self.workpiece_half_extent;
        let workpiece_top_center = Vector3::new(
            self.workpiece_center.x,
            self.workpiece_center.y,
            self.workpiece_center.z + half_extent,
        );

        // --- Position-only IK pre-positioning ---
        // Use position-only IK with high-J5 seeds. The arm finds its own natural
        // orientation to reach above the workpiece. No orientation fighting.
        let pi2 = std::f64::consts::FRAC_PI_2;

        // Desired position: 15mm above workpiece top center
        let start_pos = Vector3::new(
            workpiece_top_center.x,
            workpiece_top_center.y,
            workpiece_top_center.z + 0.015,
        );

        // IK seeds: various J2/J3/J5 combos. Position-only IK lets the arm
        // find whatever orientation is natural from these starting poses.
        let seeds: &[[f64; 6]] = &[
            [0.0, 0.6, 0.2, 0.0, pi2, 0.0],
            [0.0, 0.5, 0.3, 0.0, pi2, 0.0],
            [0.0, 0.7, 0.1, 0.0, pi2, 0.0],
            [0.0, 0.4, 0.4, 0.0, pi2, 0.0],
            [0.0, 0.8, 0.0, 0.0, pi2, 0.0],
            [0.0, 0.3, 0.5, 0.0, pi2, 0.0],
            [0.0, 0.6, 0.2, 0.0, 1.2, 0.0],
            [0.0, 0.5, 0.3, 0.0, 1.0, 0.0],
        ];

        let mut best_angles: Option<Vec<f64>> = None;
        let mut best_err = f64::MAX;
        for seed in seeds {
            let mut scratch = self.arm.clone();
            scratch.set_joint_angles(&seed.to_vec());
            let (ok, _, final_err) = self.ik_solver.solve_position(&mut scratch, &start_pos);
            if ok && final_err < best_err {
                best_err = final_err;
                best_angles = Some(scratch.joint_angles());
            }
        }

        if let Some(angles) = best_angles {
            self.arm.set_joint_angles(&angles);
            self.joint_targets = angles.clone();
            let fk = self.arm.forward_kinematics();
            let tool_z = fk.rotation * Vector3::z();
            eprintln!(
                "IK positioned (err={:.2}mm): J2={:.1}° J3={:.1}° J4={:.1}° J5={:.1}° tool_Z=({:.2},{:.2},{:.2})",
                best_err * 1000.0,
                angles[1].to_degrees(), angles[2].to_degrees(),
                angles[3].to_degrees(), angles[4].to_degrees(),
                tool_z.x, tool_z.y, tool_z.z,
            );
        } else {
            eprintln!("Warning: IK did not converge from any seed, using current pose");
        }

        // Use the arm's actual FK orientation for the carving session.
        // Position-only IK during carving means orientation doesn't matter much,
        // but the session still needs a consistent orientation for the Isometry3 targets.
        let converged_fk = self.arm.forward_kinematics();
        let tool_orientation = converged_fk.rotation;

        // Snap arm state cleanly (zero velocity, reset PIDs)
        for j in self.arm.joints.iter_mut() {
            j.velocity = 0.0;
        }
        let tool_pos = self.arm.tool_position();
        self.tool_state.position = tool_pos;
        self.tool_state.prev_position = tool_pos;
        self.tool_state.frame_start_position = tool_pos;
        for pid in &mut self.pid_controllers {
            pid.reset();
        }
        // Set cartesian_target to the arm's ACTUAL pose (not the desired one)
        // so there's zero initial error → no flailing.
        self.cartesian_target = converged_fk;

        let mut session = CarvingSession::load_file(path, workpiece_top_center, tool_orientation)?;
        session.set_start_position(self.arm.tool_position());

        eprintln!(
            "Loaded G-code: {} waypoints, estimated {:.1}s",
            session.waypoints.len(),
            session.estimated_total_time
        );
        self.carving = Some(session);
        self.mode = SimMode::Carving;
        Ok(())
    }

    /// Material removal — called once per render frame, NOT per physics step.
    fn material_removal_step(&mut self) {
        // Don't cut when spindle is off
        if !self.tool_state.spindle_on {
            self.cutting_force_magnitude = 0.0;
            self.tool_state.frame_start_position = self.tool_state.position;
            return;
        }

        // Don't cut during rapid moves (G0) — spindle spins but tool is repositioning
        let is_rapid = self.carving.as_ref().map_or(false, |s| s.is_rapid);
        if is_rapid {
            self.cutting_force_magnitude = 0.0;
            self.tool_state.frame_start_position = self.tool_state.position;
            return;
        }

        // Don't cut if the arm is still catching up from a rapid repositioning.
        // At higher speeds the carving session races ahead of arm physics —
        // the session may already be on a cutting segment while the arm is
        // still moving to the start position.
        if self.mode == SimMode::Carving {
            let target_pos = self.cartesian_target.translation.vector;
            let actual_pos = self.tool_state.position;
            let tracking_error = (target_pos - actual_pos).norm();
            if tracking_error > self.tracking_threshold {
                // Arm too far from target — still repositioning, don't cut
                self.cutting_force_magnitude = 0.0;
                self.tool_state.frame_start_position = self.tool_state.position;
                return;
            }
        }

        // Get accumulated tool path for this frame
        let (start_world, end_world) = match self.tool_state.begin_frame() {
            Some(path) => path,
            None => {
                self.cutting_force_magnitude = 0.0;
                return; // tool didn't move enough
            }
        };

        // Transform to workpiece-local coordinates (SDF is centered at origin)
        let wc = self.workpiece_center;
        let start = start_world - wc;
        let end = end_world - wc;

        // Quick AABB check: is the tool anywhere near the workpiece?
        let half = self.workpiece_half_extent + 0.02; // workpiece half-extent + margin
        let tp = end;
        if tp.x.abs() > half || tp.y.abs() > half || tp.z.abs() > half {
            self.cutting_force_magnitude = 0.0;
            return; // tool is outside workpiece region
        }

        let tool_sdf = self.tool.swept_sdf(&start, &end);
        let (bounds_min, bounds_max) = self.tool.swept_bounds(&start, &end);
        self.workpiece.subtract(bounds_min, bounds_max, tool_sdf);

        // Cutting forces (for display)
        let diff = end - start;
        if diff.norm() > 1e-9 {
            let feed_dir = diff.normalize();
            let force = forces::cutting_force(
                &self.material,
                self.tool.radius,
                &feed_dir,
                0.001,
                0.0001,
                2,
            );
            self.cutting_force_magnitude = force.norm();
        }
    }
}

/// Tracks which movement keys are currently held.
#[derive(Default)]
struct HeldKeys {
    w: bool, s: bool, a: bool, d: bool, q: bool, e: bool,
    i: bool, k: bool, j: bool, l: bool,
    shift: bool,
}

/// Application state for winit event loop.
struct App {
    window: Option<Arc<Window>>,
    render_ctx: Option<RenderContext>,
    pbr_pipeline: Option<PbrPipeline>,
    shadow_pipeline: Option<ShadowPipeline>,
    ssao_pipeline: Option<SsaoPipeline>,
    sss_pipeline: Option<SssPipeline>,
    composite_pipeline: Option<CompositePipeline>,
    overlay_pipeline: Option<OverlayPipeline>,
    camera: OrbitCamera,
    chunk_meshes: ChunkMeshManager,
    sim: SimState,
    last_frame: Instant,
    accumulator: f64,
    mouse_pressed: bool,
    middle_pressed: bool,
    last_mouse_pos: Option<(f64, f64)>,
    frame_count: u64,
    fps_timer: Instant,
    fps: f64,
    held_keys: HeldKeys,
    // --- GPU resources for arm, ground, workpiece ---
    arm_cylinder: Option<GpuMesh>,
    arm_materials: Vec<MaterialBind>,
    arm_sphere: Option<GpuMesh>,
    joint_materials: Vec<MaterialBind>,
    base_mesh: Option<GpuMesh>,
    base_material: Option<MaterialBind>,
    ground_mesh: Option<GpuMesh>,
    ground_material: Option<MaterialBind>,
    workpiece_material: Option<MaterialBind>,
    target_material: Option<MaterialBind>,
    // Tool (Dremel) visual
    tool_body_material: Option<MaterialBind>,
    tool_tip_material: Option<MaterialBind>,
    // Carving / toolpath visualization
    line_pipeline: Option<LinePipeline>,
    speed_multiplier: f64,
    max_physics_steps: u32,
    gcode_path: Option<String>,
    // Background meshing
    async_mesher: AsyncMesher,
    // Config-derived visual params
    tool_radius_m: f64,
    tool_cutting_length_m: f64,
}

impl App {
    fn new(cfg: &SimConfig) -> Self {
        Self {
            window: None,
            render_ctx: None,
            pbr_pipeline: None,
            shadow_pipeline: None,
            ssao_pipeline: None,
            sss_pipeline: None,
            composite_pipeline: None,
            overlay_pipeline: None,
            camera: OrbitCamera::new(),
            chunk_meshes: ChunkMeshManager::new(),
            sim: SimState::new(cfg),
            last_frame: Instant::now(),
            accumulator: 0.0,
            mouse_pressed: false,
            middle_pressed: false,
            last_mouse_pos: None,
            frame_count: 0,
            fps_timer: Instant::now(),
            fps: 0.0,
            arm_cylinder: None,
            arm_materials: Vec::new(),
            arm_sphere: None,
            joint_materials: Vec::new(),
            base_mesh: None,
            base_material: None,
            ground_mesh: None,
            ground_material: None,
            workpiece_material: None,
            target_material: None,
            tool_body_material: None,
            tool_tip_material: None,
            line_pipeline: None,
            speed_multiplier: cfg.default_speed,
            max_physics_steps: cfg.max_physics_steps,
            gcode_path: None,
            held_keys: HeldKeys::default(),
            async_mesher: AsyncMesher::new(0),
            tool_radius_m: cfg.tool_radius_mm * 0.001,
            tool_cutting_length_m: cfg.tool_cutting_length_mm * 0.001,
        }
    }

    /// Apply continuous movement from held keys.
    /// Rejects moves that take the target out of the arm's reachable workspace.
    fn apply_held_keys(&mut self, dt: f64) {
        if self.sim.paused {
            return;
        }

        let any_key = self.held_keys.w || self.held_keys.s || self.held_keys.a
            || self.held_keys.d || self.held_keys.q || self.held_keys.e
            || self.held_keys.i || self.held_keys.k || self.held_keys.j || self.held_keys.l;
        if !any_key {
            return;
        }

        // Save current target in case the new one is unreachable
        let prev_target = self.sim.cartesian_target;

        let speed = if self.held_keys.shift { 0.05 } else { 0.15 }; // m/s
        let step = speed * dt;
        let k = &self.held_keys;

        // Position (WASDQE)
        let pos = &mut self.sim.cartesian_target.translation.vector;
        if k.w { pos.x += step; }
        if k.s { pos.x -= step; }
        if k.a { pos.y += step; }
        if k.d { pos.y -= step; }
        if k.q { pos.z += step; }
        if k.e { pos.z -= step; }

        // Orientation (IJKL)
        let rot_speed = if self.held_keys.shift { 0.3 } else { 1.0 }; // rad/s
        let rot_step = rot_speed * dt;
        let rot = &mut self.sim.cartesian_target.rotation;
        if k.i { *rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), rot_step) * *rot; }
        if k.k { *rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -rot_step) * *rot; }
        if k.j { *rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), rot_step) * *rot; }
        if k.l { *rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -rot_step) * *rot; }

        // Test if the new target position is reachable — if not, revert.
        // Uses position-only IK with loose tolerance for a fast go/no-go check.
        let mut test = self.sim.arm.clone();
        test.set_joint_angles(&self.sim.joint_targets);
        let target_pos = self.sim.cartesian_target.translation.vector;
        let (ok, _, _) = self.sim.ik_solver.solve_position(&mut test, &target_pos);
        if !ok {
            self.sim.cartesian_target = prev_target;
        }
    }

    /// Initialize the line pipeline (for toolpath visualization).
    fn init_line_pipeline(&mut self) {
        if self.line_pipeline.is_some() {
            return; // already initialized
        }
        if let (Some(ctx), Some(pbr)) = (&self.render_ctx, &self.pbr_pipeline) {
            self.line_pipeline = Some(LinePipeline::new(ctx, &pbr.camera_buffer));
        }
    }

    /// Build line vertices from the carving session's waypoints.
    /// Colors: yellow=rapid, cyan=pending cut, green=completed.
    fn build_toolpath_lines(&self) -> Vec<LineVertex> {
        let session = match &self.sim.carving {
            Some(s) => s,
            None => return Vec::new(),
        };

        let swap = coord_swap_matrix();
        let completed = session.progress_counts().0;
        let mut vertices = Vec::with_capacity(session.waypoints.len() * 2);

        for i in 0..session.waypoints.len().saturating_sub(1) {
            let wp0 = &session.waypoints[i];
            let wp1 = &session.waypoints[i + 1];

            // Transform from DH to render space
            let p0_dh = glam::Vec3::new(wp0.position.x as f32, wp0.position.y as f32, wp0.position.z as f32);
            let p1_dh = glam::Vec3::new(wp1.position.x as f32, wp1.position.y as f32, wp1.position.z as f32);
            let p0 = swap.transform_point3(p0_dh);
            let p1 = swap.transform_point3(p1_dh);

            let color = if i < completed {
                // Completed
                [0.2, 0.9, 0.3, 1.0]
            } else if wp1.is_rapid {
                // Rapid (pending)
                [1.0, 0.9, 0.2, 0.7]
            } else {
                // Cutting (pending)
                [0.2, 0.7, 1.0, 0.8]
            };

            vertices.push(LineVertex { position: [p0.x, p0.y, p0.z], color });
            vertices.push(LineVertex { position: [p1.x, p1.y, p1.z], color });
        }

        vertices
    }

    fn render(&mut self) {
        let ctx = match &self.render_ctx {
            Some(c) => c,
            None => return,
        };
        let pbr = match &self.pbr_pipeline {
            Some(p) => p,
            None => return,
        };

        let swap = coord_swap_matrix();

        // Update camera uniform
        let cam_uniform = CameraUniform::from_camera(&self.camera, ctx.aspect());
        pbr.update_camera(&ctx.queue, &cam_uniform);

        // Update light (Y-up render space)
        let light = LightUniform {
            direction: [-0.4, -0.8, -0.3, 0.0],
            color: [1.0, 0.98, 0.95, 3.0],
            ambient: [0.6, 0.65, 0.7, 0.35],
            eye_pos: cam_uniform.eye_pos,
        };
        pbr.update_light(&ctx.queue, &light);

        // --- Upload material uniforms ---

        // Workpiece: marble with workpiece offset + coord swap as model matrix.
        // SDF is at origin; model matrix = coord_swap * translate(workpiece_center_dh).
        if let Some(wp) = &self.workpiece_material {
            let wc = &self.sim.workpiece_center;
            let translate_dh = Mat4::from_translation(Vec3::new(
                wc.x as f32, wc.y as f32, wc.z as f32,
            ));
            let model = swap * translate_dh;
            let mat = MaterialUniform::marble().with_model(model.to_cols_array_2d());
            ctx.queue.write_buffer(&wp.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Ground plane: identity model (already in render space)
        if let Some(gnd) = &self.ground_material {
            let mat = MaterialUniform::ground();
            ctx.queue.write_buffer(&gnd.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Arm links: compute per-link model matrices from physics frames
        let frames = self.sim.arm.link_frames();
        let mut render_pts = Vec::with_capacity(7);
        for i in 0..7 {
            let f = isometry_to_glam(&frames[i]);
            render_pts.push(swap.transform_point3(Vec3::new(f.col(3).x, f.col(3).y, f.col(3).z)));
        }

        for i in 0..6 {
            if i >= self.arm_materials.len() {
                break;
            }
            let model = link_model(render_pts[i], render_pts[i + 1], ARM_LINK_RADII[i]);
            let mat = MaterialUniform::metal(ARM_LINK_COLORS[i])
                .with_model(model.to_cols_array_2d());
            ctx.queue
                .write_buffer(&self.arm_materials[i].buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Joint sphere materials (at each frame position)
        let joint_sphere_radii: [f32; 7] = [0.10, 0.09, 0.08, 0.07, 0.06, 0.06, 0.05];
        let joint_color = [0.25, 0.25, 0.30, 1.0]; // dark metallic
        for i in 0..7 {
            if i >= self.joint_materials.len() {
                break;
            }
            let s = joint_sphere_radii[i];
            let model = Mat4::from_scale_rotation_translation(
                Vec3::splat(s),
                Quat::IDENTITY,
                render_pts[i],
            );
            let mat = MaterialUniform::metal(joint_color)
                .with_model(model.to_cols_array_2d());
            ctx.queue
                .write_buffer(&self.joint_materials[i].buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Target point: bright sphere showing where the arm is trying to reach.
        if let Some(tgt_mat) = &self.target_material {
            let ct_pos = &self.sim.cartesian_target.translation.vector;
            let tgt_dh = Vec3::new(ct_pos.x as f32, ct_pos.y as f32, ct_pos.z as f32);
            let tgt_render = swap.transform_point3(tgt_dh);
            let tgt_color = if self.sim.ik_converged {
                [0.1, 1.0, 0.3, 1.0] // green = reachable
            } else {
                [1.0, 0.2, 0.1, 1.0] // red = unreachable
            };
            let tgt_radius = 0.012_f32; // 12mm sphere
            let model = Mat4::from_scale_rotation_translation(
                Vec3::splat(tgt_radius),
                Quat::IDENTITY,
                tgt_render,
            );
            let mat = MaterialUniform::metal(tgt_color)
                .with_model(model.to_cols_array_2d());
            ctx.queue.write_buffer(&tgt_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Tool (Dremel): extends from flange along its local Z-axis.
        // Body: 150mm long, 18mm radius cylinder (gray). Tip: 25mm, 8mm radius (silver).
        {
            let flange_mat = isometry_to_glam(&self.sim.arm.flange_pose());
            let tool_z_dh = Vec3::new(flange_mat.col(2).x, flange_mat.col(2).y, flange_mat.col(2).z);
            let flange_pos_dh = Vec3::new(flange_mat.col(3).x, flange_mat.col(3).y, flange_mat.col(3).z);

            // Tool body: from flange to 150mm along tool Z
            let body_end_dh = flange_pos_dh + tool_z_dh * 0.15;
            let body_from = swap.transform_point3(flange_pos_dh);
            let body_to = swap.transform_point3(body_end_dh);

            if let Some(tb_mat) = &self.tool_body_material {
                let model = link_model(body_from, body_to, 0.018);
                let mat = MaterialUniform::metal([0.35, 0.35, 0.40, 1.0])
                    .with_model(model.to_cols_array_2d());
                ctx.queue.write_buffer(&tb_mat.buffer, 0, bytemuck::bytes_of(&mat));
            }

            // Tool tip (cutting bit): visual matches config
            let tip_end_dh = body_end_dh + tool_z_dh * self.tool_cutting_length_m as f32;
            let tip_to = swap.transform_point3(tip_end_dh);

            if let Some(tt_mat) = &self.tool_tip_material {
                let model = link_model(body_to, tip_to, self.tool_radius_m as f32);
                let mat = MaterialUniform::metal([0.85, 0.85, 0.90, 1.0])
                    .with_model(model.to_cols_array_2d());
                ctx.queue.write_buffer(&tt_mat.buffer, 0, bytemuck::bytes_of(&mat));
            }
        }

        // Base pedestal: tall column from arm base down to ground.
        // Base center at render Y = -0.13 (midpoint of 0 to -0.26).
        if let Some(base_mat) = &self.base_material {
            let base_pos = Vec3::new(render_pts[0].x, render_pts[0].y - 0.13, render_pts[0].z);
            let model = Mat4::from_translation(base_pos);
            let mat = MaterialUniform::metal([0.15, 0.15, 0.18, 1.0])
                .with_model(model.to_cols_array_2d());
            ctx.queue.write_buffer(&base_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Upload completed meshes from background workers.
        // Also do a small sync remesh pass for any remaining dirty chunks
        // (handles initial startup before async has spun up).
        let async_meshes = self.async_mesher.poll_completed();
        let sync_meshes = mesher::remesh_dirty_capped(&mut self.sim.workpiece, 2);
        for mesh in async_meshes.iter().chain(sync_meshes.iter()) {
            let coord = (mesh.coord.x, mesh.coord.y, mesh.coord.z);
            self.chunk_meshes.upload_chunk(
                &ctx.device,
                &ctx.queue,
                coord,
                &mesh.positions,
                &mesh.normals,
                &mesh.indices,
            );
        }

        // Begin render pass
        let output = match ctx.surface.get_current_texture() {
            Ok(t) => t,
            Err(wgpu::SurfaceError::Lost) => return,
            Err(wgpu::SurfaceError::OutOfMemory) => {
                eprintln!("Out of GPU memory!");
                return;
            }
            Err(e) => {
                eprintln!("Surface error: {:?}", e);
                return;
            }
        };

        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        // Main PBR pass
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("PBR Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.12,
                            g: 0.14,
                            b: 0.18,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &ctx.depth_texture,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });

            render_pass.set_pipeline(&pbr.pipeline);

            // 1) Draw ground plane
            if let (Some(gnd_mesh), Some(gnd_mat)) =
                (&self.ground_mesh, &self.ground_material)
            {
                render_pass.set_bind_group(0, &gnd_mat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, gnd_mesh.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    gnd_mesh.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..gnd_mesh.num_indices, 0, 0..1);
            }

            // 2) Draw workpiece chunks
            if let Some(wp) = &self.workpiece_material {
                render_pass.set_bind_group(0, &wp.bind_group, &[]);
                for mesh in self.chunk_meshes.meshes.values() {
                    render_pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    render_pass.set_index_buffer(
                        mesh.index_buffer.slice(..),
                        wgpu::IndexFormat::Uint32,
                    );
                    render_pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }

            // 3) Draw robot arm links (cylinders)
            if let Some(cyl) = &self.arm_cylinder {
                for link_mat in &self.arm_materials {
                    render_pass.set_bind_group(0, &link_mat.bind_group, &[]);
                    render_pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                    render_pass.set_index_buffer(
                        cyl.index_buffer.slice(..),
                        wgpu::IndexFormat::Uint32,
                    );
                    render_pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
                }
            }

            // 4) Draw joint spheres
            if let Some(sph) = &self.arm_sphere {
                for jm in &self.joint_materials {
                    render_pass.set_bind_group(0, &jm.bind_group, &[]);
                    render_pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                    render_pass.set_index_buffer(
                        sph.index_buffer.slice(..),
                        wgpu::IndexFormat::Uint32,
                    );
                    render_pass.draw_indexed(0..sph.num_indices, 0, 0..1);
                }
            }

            // 5) Draw target point sphere
            if let (Some(sph), Some(tgt_mat)) = (&self.arm_sphere, &self.target_material) {
                render_pass.set_bind_group(0, &tgt_mat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    sph.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..sph.num_indices, 0, 0..1);
            }

            // 6) Draw tool body (Dremel)
            if let (Some(cyl), Some(tb_mat)) = (&self.arm_cylinder, &self.tool_body_material) {
                render_pass.set_bind_group(0, &tb_mat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    cyl.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
            }

            // 7) Draw tool tip
            if let (Some(cyl), Some(tt_mat)) = (&self.arm_cylinder, &self.tool_tip_material) {
                render_pass.set_bind_group(0, &tt_mat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    cyl.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
            }

            // 8) Draw base pedestal
            if let (Some(bm), Some(bmat)) = (&self.base_mesh, &self.base_material) {
                render_pass.set_bind_group(0, &bmat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, bm.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    bm.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..bm.num_indices, 0, 0..1);
            }
        }

        // --- Toolpath line pass ---
        // Build line vertices before borrowing the pipeline mutably
        let line_verts = self.build_toolpath_lines();
        if let Some(line_pipe) = &mut self.line_pipeline {
            if !line_verts.is_empty() {
                line_pipe.upload(&ctx.queue, &line_verts);
            }
            if line_pipe.num_vertices > 0 {
                let mut line_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Line Pass"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &view,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &ctx.depth_texture,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
                line_pass.set_pipeline(&line_pipe.pipeline);
                line_pass.set_bind_group(0, &line_pipe.bind_group, &[]);
                line_pass.set_vertex_buffer(0, line_pipe.vertex_buffer.slice(..));
                line_pass.draw(0..line_pipe.num_vertices, 0..1);
            }
        }

        // --- Overlay pass (2D diagnostics panel) ---
        let overlay_idx_count = if let Some(overlay) = &self.overlay_pipeline {
            let size = ctx.config.width as f32;
            let h = ctx.config.height as f32;
            overlay.update_screen_size(&ctx.queue, size, h);

            let mut ob = OverlayBuilder::new();
            self.build_diagnostics_overlay(&mut ob, size, h);
            ob.upload(&ctx.queue, overlay)
        } else {
            0
        };

        if overlay_idx_count > 0 {
            let overlay = self.overlay_pipeline.as_ref().unwrap();
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Overlay Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load, // preserve PBR output
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                ..Default::default()
            });
            pass.set_pipeline(&overlay.pipeline);
            pass.set_bind_group(0, &overlay.bind_group, &[]);
            pass.set_vertex_buffer(0, overlay.vertex_buffer.slice(..));
            pass.set_index_buffer(overlay.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..overlay_idx_count, 0, 0..1);
        }

        ctx.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        // FPS counter
        self.frame_count += 1;
        let elapsed = self.fps_timer.elapsed().as_secs_f64();
        if elapsed >= 1.0 {
            self.fps = self.frame_count as f64 / elapsed;
            self.frame_count = 0;
            self.fps_timer = Instant::now();
        }
    }

    /// Build the left-side diagnostics overlay.
    fn build_diagnostics_overlay(&self, ob: &mut OverlayBuilder, _sw: f32, sh: f32) {
        let panel_w = 220.0f32;
        let margin = 8.0f32;
        let scale = 1.0f32; // text scale (1.0 = 8px native glyphs, crisp)
        let line_h = 8.0 * scale + 3.0; // line height in pixels
        let bar_h = 8.0f32;
        let white = [1.0, 1.0, 1.0, 0.95];
        let gray = [0.7, 0.7, 0.7, 0.9];
        let green = [0.2, 0.9, 0.3, 1.0];
        let red = [0.9, 0.3, 0.2, 1.0];
        let yellow = [0.9, 0.8, 0.2, 1.0];
        let cyan = [0.3, 0.8, 0.9, 1.0];
        let orange = [0.9, 0.6, 0.2, 1.0];
        let bar_bg = [0.15, 0.15, 0.18, 0.6];

        // Panel background
        ob.rect(0.0, 0.0, panel_w, sh, [0.08, 0.09, 0.12, 0.85]);
        // Separator line
        ob.rect(panel_w - 2.0, 0.0, 2.0, sh, [0.3, 0.3, 0.35, 0.5]);

        let mut y = margin;

        // Title
        ob.text(margin, y, 2.0, white, "SimuForge-Stone");
        y += 8.0 * 2.0 + 8.0;

        // Mode indicator
        let mode_color = match self.sim.mode {
            SimMode::Manual => cyan,
            SimMode::Carving => orange,
        };
        let mode_text = match self.sim.mode {
            SimMode::Manual => "MANUAL",
            SimMode::Carving => "CARVING",
        };
        ob.rect(margin, y, 10.0, 10.0, mode_color);
        ob.text(margin + 16.0, y, scale, mode_color, mode_text);
        y += line_h;

        // Status / carving state
        if self.sim.mode == SimMode::Carving {
            if let Some(session) = &self.sim.carving {
                let state_color = match session.state {
                    CarvingState::Idle => gray,
                    CarvingState::Running => green,
                    CarvingState::Paused => yellow,
                    CarvingState::Complete => cyan,
                };
                let state_text = match session.state {
                    CarvingState::Idle => "IDLE",
                    CarvingState::Running => "RUNNING",
                    CarvingState::Paused => "PAUSED",
                    CarvingState::Complete => "COMPLETE",
                };
                ob.text(margin + 16.0, y, scale, state_color, state_text);
                y += line_h;

                // Speed multiplier
                let speed_color = if (self.speed_multiplier - 1.0).abs() < 0.01 { gray } else { yellow };
                ob.text(margin, y, scale, speed_color,
                    &format!("Speed: {}x", self.speed_multiplier));
                y += line_h;

                // Progress bar
                let (done, total) = session.progress_counts();
                let prog = session.progress();
                ob.text(margin, y, scale, white,
                    &format!("Progress: {:.0}% ({}/{})", prog * 100.0, done, total));
                y += line_h;

                // Progress bar visual
                let bar_x = margin;
                let bar_y = y;
                let bar_w = panel_w - margin * 2.0;
                ob.rect(bar_x, bar_y, bar_w, bar_h, bar_bg);
                ob.rect(bar_x, bar_y, bar_w * prog as f32, bar_h, green);
                y += bar_h + 2.0;

                // ETA
                let eta = session.eta();
                if eta > 0.0 && session.state == CarvingState::Running {
                    ob.text(margin, y, scale, gray,
                        &format!("ETA: {:.0}s", eta / self.speed_multiplier));
                    y += line_h;
                }
            }
        } else {
            let status_color = if self.sim.paused { yellow } else { green };
            let status_text = if self.sim.paused { "PAUSED" } else { "RUNNING" };
            ob.text(margin + 16.0, y, scale, status_color, status_text);
            y += line_h;
        }
        y += 4.0;

        // FPS
        ob.text(margin, y, scale, gray, &format!("FPS:  {:.0}", self.fps));
        y += line_h;

        // Sim time
        ob.text(margin, y, scale, gray, &format!("Time: {:.2}s", self.sim.sim_time));
        y += line_h;

        // Chunks / Tris
        ob.text(
            margin, y, scale, gray,
            &format!("Mesh: {} chunks  {}K tri",
                self.chunk_meshes.chunk_count(),
                self.chunk_meshes.total_triangles() / 1000,
            ),
        );
        y += line_h + 8.0;

        // --- Joint angles ---
        ob.text(margin, y, scale, white, "Joint Angles");
        y += line_h + 2.0;

        let link_colors: [[f32; 4]; 6] = [
            orange, orange, orange, cyan, cyan, cyan,
        ];

        for (i, joint) in self.sim.arm.joints.iter().enumerate() {
            let angle_deg = joint.angle.to_degrees();
            let min_deg = joint.angle_min.to_degrees();
            let max_deg = joint.angle_max.to_degrees();
            let range = max_deg - min_deg;

            // Label
            let label = format!("J{}: {:>7.1}", i + 1, angle_deg);
            ob.text(margin, y, scale, link_colors[i], &label);

            // Bar background
            let bar_x = margin;
            let bar_y = y + line_h;
            let bar_w = panel_w - margin * 2.0;
            ob.rect(bar_x, bar_y, bar_w, bar_h, bar_bg);

            // Bar fill: position within range
            if range > 0.0 {
                let frac = ((angle_deg - min_deg) / range).clamp(0.0, 1.0) as f32;
                let fill_w = bar_w * frac;
                ob.rect(bar_x, bar_y, fill_w, bar_h, link_colors[i]);

                // Center mark (0 degree position)
                let zero_frac = ((0.0 - min_deg) / range).clamp(0.0, 1.0) as f32;
                let mark_x = bar_x + bar_w * zero_frac;
                ob.rect(mark_x - 1.0, bar_y, 2.0, bar_h, white);
            }

            y += line_h + bar_h + 4.0;
        }

        y += 4.0;

        // --- Motor torques ---
        ob.text(margin, y, scale, white, "Motor Torques");
        y += line_h + 2.0;

        for (i, joint) in self.sim.arm.joints.iter().enumerate() {
            let torque = joint.torque;
            let max_t = if i < 3 { 1200.0 } else { 150.0 };
            let label = format!("J{}: {:>7.1} Nm", i + 1, torque);
            ob.text(margin, y, scale, gray, &label);

            let bar_x = margin;
            let bar_y = y + line_h;
            let bar_w = panel_w - margin * 2.0;
            ob.rect(bar_x, bar_y, bar_w, bar_h, bar_bg);

            // Torque bar centered at middle (negative goes left, positive right)
            let mid = bar_x + bar_w * 0.5;
            let frac = (torque / max_t).clamp(-1.0, 1.0) as f32;
            if frac >= 0.0 {
                ob.rect(mid, bar_y, bar_w * 0.5 * frac, bar_h, green);
            } else {
                let w = bar_w * 0.5 * (-frac);
                ob.rect(mid - w, bar_y, w, bar_h, red);
            }
            // Center mark
            ob.rect(mid - 1.0, bar_y, 2.0, bar_h, white);

            y += line_h + bar_h + 4.0;
        }

        y += 4.0;

        // --- Cutting force ---
        ob.text(margin, y, scale, white, "Cutting");
        y += line_h;
        ob.text(
            margin, y, scale, gray,
            &format!("Force: {:.1} N", self.sim.cutting_force_magnitude),
        );
        y += line_h;

        let spindle = if self.sim.tool_state.spindle_on {
            "Spindle: ON"
        } else {
            "Spindle: OFF"
        };
        let sp_color = if self.sim.tool_state.spindle_on { green } else { red };
        ob.text(margin, y, scale, sp_color, spindle);
        y += line_h + 8.0;

        // --- Target & Tool ---
        let ct_pos = &self.sim.cartesian_target.translation.vector;
        let ct_rot = &self.sim.cartesian_target.rotation;
        let tp = &self.sim.tool_state.position;
        let ik_color = if self.sim.ik_converged { green } else { red };
        let ik_text = if self.sim.ik_converged { "IK OK" } else { "IK FAIL" };

        ob.text(margin, y, scale, white, "Target");
        y += line_h;
        ob.text(
            margin, y, scale, ik_color,
            &format!("{} X:{:.0} Y:{:.0} Z:{:.0}",
                ik_text, ct_pos.x * 1000.0, ct_pos.y * 1000.0, ct_pos.z * 1000.0),
        );
        y += line_h;
        // Show target orientation as Euler angles (roll/pitch/yaw)
        let (roll, pitch, yaw): (f64, f64, f64) = ct_rot.euler_angles();
        ob.text(
            margin, y, scale, gray,
            &format!("R:{:.1} P:{:.1} Y:{:.1}",
                roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()),
        );
        y += line_h + 2.0;

        ob.text(margin, y, scale, white, "Tool");
        y += line_h;
        ob.text(
            margin, y, scale, gray,
            &format!("X:{:.0} Y:{:.0} Z:{:.0}",
                tp.x * 1000.0, tp.y * 1000.0, tp.z * 1000.0),
        );
        y += line_h + 2.0;

        // Tool position relative to workpiece
        let wc = &self.sim.workpiece_center;
        let local = tp - wc;
        let half = self.sim.workpiece_half_extent;
        let inside = local.x.abs() < half && local.y.abs() < half && local.z.abs() < half;
        let inside_color = if inside { green } else { red };
        let inside_text = if inside { "IN BLOCK" } else { "OUTSIDE" };
        ob.text(margin, y, scale, inside_color, inside_text);
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        let window = Arc::new(
            event_loop
                .create_window(
                    Window::default_attributes()
                        .with_title("SimuForge-Stone — Digital Twin")
                        .with_inner_size(winit::dpi::LogicalSize::new(1280, 720)),
                )
                .expect("Failed to create window"),
        );

        let ctx = pollster::block_on(RenderContext::new(window.clone()));
        let pbr = PbrPipeline::new(&ctx);
        let shadow = ShadowPipeline::new(&ctx);
        let ssao = SsaoPipeline::new(&ctx);
        let sss = SssPipeline::new(&ctx);
        let composite = CompositePipeline::new(&ctx);
        let overlay = OverlayPipeline::new(&ctx);

        // --- Create GPU resources for arm, ground, workpiece ---
        use wgpu::util::DeviceExt;

        // Unit cylinder (radius=1, height=1) reused for all arm links
        let (cyl_verts, cyl_idxs) = generate_cylinder(1.0, 1.0, 16);
        let cyl_vb = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Arm Cylinder VB"),
                contents: bytemuck::cast_slice(&cyl_verts),
                usage: wgpu::BufferUsages::VERTEX,
            });
        let cyl_ib = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Arm Cylinder IB"),
                contents: bytemuck::cast_slice(&cyl_idxs),
                usage: wgpu::BufferUsages::INDEX,
            });
        self.arm_cylinder = Some(GpuMesh {
            vertex_buffer: cyl_vb,
            index_buffer: cyl_ib,
            num_indices: cyl_idxs.len() as u32,
        });

        // Material bind groups for each arm link
        for _ in 0..6 {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.arm_materials.push(MaterialBind {
                buffer: buf,
                bind_group: bg,
            });
        }

        // Unit sphere (radius=1) for joint spheres
        let (sph_verts, sph_idxs) = generate_sphere(1.0, 12, 16);
        let sph_vb = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Joint Sphere VB"),
                contents: bytemuck::cast_slice(&sph_verts),
                usage: wgpu::BufferUsages::VERTEX,
            });
        let sph_ib = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Joint Sphere IB"),
                contents: bytemuck::cast_slice(&sph_idxs),
                usage: wgpu::BufferUsages::INDEX,
            });
        self.arm_sphere = Some(GpuMesh {
            vertex_buffer: sph_vb,
            index_buffer: sph_ib,
            num_indices: sph_idxs.len() as u32,
        });

        // Material bind groups for joint spheres (7: base + 6 joints)
        for _ in 0..7 {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.joint_materials.push(MaterialBind {
                buffer: buf,
                bind_group: bg,
            });
        }

        // Base pedestal: tall box from arm base (Y=0) down to ground (Y=-0.26).
        // Half-height = 0.13, centered at Y = -0.13.
        let (base_verts, base_idxs) = generate_box(0.10, 0.13, 0.10);
        let base_vb = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Base Pedestal VB"),
                contents: bytemuck::cast_slice(&base_verts),
                usage: wgpu::BufferUsages::VERTEX,
            });
        let base_ib = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Base Pedestal IB"),
                contents: bytemuck::cast_slice(&base_idxs),
                usage: wgpu::BufferUsages::INDEX,
            });
        self.base_mesh = Some(GpuMesh {
            vertex_buffer: base_vb,
            index_buffer: base_ib,
            num_indices: base_idxs.len() as u32,
        });
        let (bbuf, bbg) = pbr.create_material_bind_group(&ctx.device);
        self.base_material = Some(MaterialBind {
            buffer: bbuf,
            bind_group: bbg,
        });

        // Ground plane at Y=-0.26 in render space.
        // Cube bottom: DH Z = -0.10 - 0.1525 = -0.2525 → render Y = -0.2525.
        let gs = 3.0f32; // 3m half-extent
        let gy = -0.26f32;
        let gnd_verts = vec![
            Vertex { position: [-gs, gy, -gs], normal: [0.0, 1.0, 0.0] },
            Vertex { position: [ gs, gy, -gs], normal: [0.0, 1.0, 0.0] },
            Vertex { position: [ gs, gy,  gs], normal: [0.0, 1.0, 0.0] },
            Vertex { position: [-gs, gy,  gs], normal: [0.0, 1.0, 0.0] },
        ];
        let gnd_idxs: Vec<u32> = vec![0, 2, 1, 0, 3, 2]; // CCW from above
        let gnd_vb = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Ground VB"),
                contents: bytemuck::cast_slice(&gnd_verts),
                usage: wgpu::BufferUsages::VERTEX,
            });
        let gnd_ib = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Ground IB"),
                contents: bytemuck::cast_slice(&gnd_idxs),
                usage: wgpu::BufferUsages::INDEX,
            });
        self.ground_mesh = Some(GpuMesh {
            vertex_buffer: gnd_vb,
            index_buffer: gnd_ib,
            num_indices: gnd_idxs.len() as u32,
        });
        let (gbuf, gbg) = pbr.create_material_bind_group(&ctx.device);
        self.ground_material = Some(MaterialBind {
            buffer: gbuf,
            bind_group: gbg,
        });

        // Workpiece material bind group
        let (wbuf, wbg) = pbr.create_material_bind_group(&ctx.device);
        self.workpiece_material = Some(MaterialBind {
            buffer: wbuf,
            bind_group: wbg,
        });

        // Target point material bind group (reuses the joint sphere mesh)
        let (tbuf, tbg) = pbr.create_material_bind_group(&ctx.device);
        self.target_material = Some(MaterialBind {
            buffer: tbuf,
            bind_group: tbg,
        });

        // Tool body material (Dremel body — reuses arm cylinder mesh)
        let (tb_buf, tb_bg) = pbr.create_material_bind_group(&ctx.device);
        self.tool_body_material = Some(MaterialBind {
            buffer: tb_buf,
            bind_group: tb_bg,
        });

        // Tool tip material (cutting bit — reuses arm cylinder mesh)
        let (tt_buf, tt_bg) = pbr.create_material_bind_group(&ctx.device);
        self.tool_tip_material = Some(MaterialBind {
            buffer: tt_buf,
            bind_group: tt_bg,
        });

        self.pbr_pipeline = Some(pbr);
        self.shadow_pipeline = Some(shadow);
        self.ssao_pipeline = Some(ssao);
        self.sss_pipeline = Some(sss);
        self.composite_pipeline = Some(composite);
        self.overlay_pipeline = Some(overlay);
        self.render_ctx = Some(ctx);
        self.window = Some(window);

        // Auto-load G-code if specified via CLI
        if let Some(path) = &self.gcode_path.clone() {
            match self.sim.load_gcode(path) {
                Ok(()) => {
                    self.init_line_pipeline();
                    eprintln!("G-code loaded. Press Space to start carving.");
                }
                Err(e) => eprintln!("Failed to load G-code: {}", e),
            }
        }

        eprintln!("SimuForge-Stone initialized. Press SPACE to start.");
        eprintln!("  Move tool: W/S=fwd/back  A/D=left/right  Q/E=up/down");
        eprintln!("  Hold Shift for slow (50mm/s), normal=150mm/s");
        eprintln!("  X=spindle  R=reset target  1/2=camera snap");
        eprintln!("  G=load gcode  M=manual mode  +/-=speed  Shift+R=full reset");
        eprintln!("  Camera: LMB=rotate  Scroll=zoom  MMB=pan");
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }

            WindowEvent::Resized(new_size) => {
                if let Some(ctx) = &mut self.render_ctx {
                    ctx.resize(new_size);
                    if let Some(ssao) = &mut self.ssao_pipeline {
                        ssao.resize(&ctx.device, &ctx.config);
                    }
                    if let Some(sss) = &mut self.sss_pipeline {
                        sss.resize(&ctx.device, &ctx.config);
                    }
                }
            }

            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        logical_key,
                        state,
                        ..
                    },
                ..
            } => {
                let pressed = state == ElementState::Pressed;

                // Continuous movement keys (hold to move)
                match logical_key.as_ref() {
                    Key::Character("w") | Key::Character("W") => self.held_keys.w = pressed,
                    Key::Character("s") | Key::Character("S") => self.held_keys.s = pressed,
                    Key::Character("a") | Key::Character("A") => self.held_keys.a = pressed,
                    Key::Character("d") | Key::Character("D") => self.held_keys.d = pressed,
                    Key::Character("q") | Key::Character("Q") => self.held_keys.q = pressed,
                    Key::Character("e") | Key::Character("E") => self.held_keys.e = pressed,
                    Key::Character("i") | Key::Character("I") => self.held_keys.i = pressed,
                    Key::Character("k") | Key::Character("K") => self.held_keys.k = pressed,
                    Key::Character("j") | Key::Character("J") => self.held_keys.j = pressed,
                    Key::Character("l") | Key::Character("L") => self.held_keys.l = pressed,
                    Key::Named(NamedKey::Shift) => self.held_keys.shift = pressed,
                    _ => {}
                }

                // Single-press actions (only on press, not release)
                if pressed {
                    match logical_key.as_ref() {
                        Key::Named(NamedKey::Space) => {
                            if self.sim.mode == SimMode::Carving {
                                // Toggle carving session start/pause
                                if let Some(session) = &mut self.sim.carving {
                                    session.toggle();
                                    match session.state {
                                        CarvingState::Running => eprintln!("Carving RUNNING"),
                                        CarvingState::Paused => eprintln!("Carving PAUSED"),
                                        CarvingState::Idle => eprintln!("Carving IDLE"),
                                        CarvingState::Complete => eprintln!("Carving COMPLETE"),
                                    }
                                }
                            } else {
                                self.sim.paused = !self.sim.paused;
                                eprintln!(
                                    "Simulation {}",
                                    if self.sim.paused { "PAUSED" } else { "RUNNING" }
                                );
                            }
                        }
                        Key::Named(NamedKey::Escape) => {
                            event_loop.exit();
                        }
                        Key::Character("1") => {
                            self.camera.snap_to_workpiece();
                        }
                        Key::Character("2") => {
                            self.camera.snap_to_arm();
                        }
                        Key::Character("x") | Key::Character("X") => {
                            self.sim.tool_state.spindle_on = !self.sim.tool_state.spindle_on;
                            eprintln!(
                                "Spindle {}",
                                if self.sim.tool_state.spindle_on { "ON" } else { "OFF" }
                            );
                        }
                        Key::Character("r") => {
                            self.sim.cartesian_target = self.sim.arm.forward_kinematics();
                            eprintln!("Target reset to current tool pose");
                        }
                        Key::Character("R") => {
                            // Full reset including carving session
                            self.sim.reset_to_initial();
                            if let Some(session) = &mut self.sim.carving {
                                session.reset();
                            }
                            self.speed_multiplier = 1.0;
                            eprintln!("Full reset: arm, targets, carving session reset");
                        }
                        Key::Character("g") | Key::Character("G") => {
                            // Load G-code file
                            if let Some(path) = &self.gcode_path.clone() {
                                match self.sim.load_gcode(path) {
                                    Ok(()) => {
                                        self.init_line_pipeline();
                                        eprintln!("G-code loaded, press Space to start carving");
                                    }
                                    Err(e) => eprintln!("Failed to load G-code: {}", e),
                                }
                            } else {
                                eprintln!("No G-code file specified. Pass path as CLI argument.");
                            }
                        }
                        Key::Character("m") | Key::Character("M") => {
                            // Switch to manual mode
                            self.sim.mode = SimMode::Manual;
                            self.sim.paused = true;
                            eprintln!("Switched to MANUAL mode");
                        }
                        Key::Character("+") | Key::Character("=") => {
                            // Speed up
                            let current_idx = SPEED_PRESETS.iter()
                                .position(|&s| (s - self.speed_multiplier).abs() < 0.01)
                                .unwrap_or_else(|| {
                                    SPEED_PRESETS.iter()
                                        .position(|&s| s > self.speed_multiplier)
                                        .unwrap_or(SPEED_PRESETS.len() - 1)
                                });
                            let next_idx = (current_idx + 1).min(SPEED_PRESETS.len() - 1);
                            self.speed_multiplier = SPEED_PRESETS[next_idx];
                            eprintln!("Speed: {}x", self.speed_multiplier);
                        }
                        Key::Character("-") => {
                            // Slow down
                            let current_idx = SPEED_PRESETS.iter()
                                .position(|&s| (s - self.speed_multiplier).abs() < 0.01)
                                .unwrap_or_else(|| {
                                    SPEED_PRESETS.iter()
                                        .rposition(|&s| s < self.speed_multiplier)
                                        .unwrap_or(0)
                                });
                            let next_idx = if current_idx > 0 { current_idx - 1 } else { 0 };
                            self.speed_multiplier = SPEED_PRESETS[next_idx];
                            eprintln!("Speed: {}x", self.speed_multiplier);
                        }
                        _ => {}
                    }
                }
            }

            WindowEvent::MouseInput { state, button, .. } => {
                match button {
                    MouseButton::Left => {
                        self.mouse_pressed = state == ElementState::Pressed;
                    }
                    MouseButton::Middle => {
                        self.middle_pressed = state == ElementState::Pressed;
                    }
                    _ => {}
                }
                if state == ElementState::Released {
                    self.last_mouse_pos = None;
                }
            }

            WindowEvent::CursorMoved { position, .. } => {
                if let Some((lx, ly)) = self.last_mouse_pos {
                    let dx = (position.x - lx) as f32;
                    let dy = (position.y - ly) as f32;

                    if self.mouse_pressed {
                        self.camera.rotate(dx * 0.005, -dy * 0.005);
                    }
                    if self.middle_pressed {
                        self.camera.pan(-dx, dy);
                    }
                }
                self.last_mouse_pos = Some((position.x, position.y));
            }

            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    winit::event::MouseScrollDelta::LineDelta(_, y) => y,
                    winit::event::MouseScrollDelta::PixelDelta(pos) => pos.y as f32 * 0.01,
                };
                self.camera.zoom(scroll);
            }

            WindowEvent::RedrawRequested => {
                // Fixed timestep physics
                let now = Instant::now();
                let frame_time = now.duration_since(self.last_frame).as_secs_f64();
                self.last_frame = now;

                let clamped_frame_time = frame_time.min(MAX_FRAME_TIME);

                // Carving mode: advance the carving session and set targets
                if self.sim.mode == SimMode::Carving {
                    if let Some(session) = &mut self.sim.carving {
                        if session.state == CarvingState::Running {
                            let scaled_dt = clamped_frame_time * self.speed_multiplier;
                            if let Some(target) = session.step(scaled_dt) {
                                self.sim.cartesian_target = target;
                            }
                            self.sim.tool_state.spindle_on = session.spindle_on;
                            self.sim.paused = false;
                        }
                    }
                } else {
                    // Manual mode: apply continuous key movement
                    self.apply_held_keys(clamped_frame_time);
                }

                // IK solve: update joint targets from Cartesian target (once per frame)
                if !self.sim.paused {
                    self.sim.update_ik();
                }

                // Scale physics accumulator by speed multiplier in carving mode
                let physics_dt_budget = if self.sim.mode == SimMode::Carving {
                    clamped_frame_time * self.speed_multiplier
                } else {
                    clamped_frame_time
                };
                self.accumulator += physics_dt_budget;

                let mut steps = 0u32;
                while self.accumulator >= PHYSICS_DT && steps < self.max_physics_steps {
                    self.sim.physics_step();
                    self.accumulator -= PHYSICS_DT;
                    steps += 1;
                }
                // Drain excess to prevent spiral of death
                if self.accumulator > PHYSICS_DT * 10.0 {
                    self.accumulator = 0.0;
                }

                // Material removal: once per frame (not per physics step)
                self.sim.material_removal_step();

                // Submit dirty chunks for background meshing
                self.async_mesher.submit_dirty(&mut self.sim.workpiece);

                // Render
                self.render();

                // Request next frame
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }

            _ => {}
        }
    }
}

fn main() {
    eprintln!("Starting SimuForge-Stone...");

    let args: Vec<String> = std::env::args().collect();
    let gcode_path = args.get(1).cloned();

    if let Some(ref path) = gcode_path {
        eprintln!("G-code file: {}", path);
    }

    // Load config from config.toml (falls back to defaults if missing)
    let cfg = SimConfig::from_file(std::path::Path::new("config.toml"));

    let event_loop = EventLoop::new().expect("Failed to create event loop");
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = App::new(&cfg);
    app.gcode_path = gcode_path;
    event_loop.run_app(&mut app).expect("Event loop error");
}
