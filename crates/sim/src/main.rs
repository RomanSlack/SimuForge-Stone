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
use simuforge_material::octree::OctreeSdf;
use simuforge_motors::gearbox::Gearbox;
use simuforge_motors::pid;
use simuforge_motors::stepper::{MotorState, StepperMotor};
use simuforge_physics::aba::{articulated_body_algorithm, gravity_compensation_rnea};
use simuforge_physics::arm::RobotArm;
use simuforge_physics::joint::{LinearTrack, RotaryTable};
use simuforge_render::arm_visual::{generate_cylinder, generate_sphere, generate_box};
use simuforge_render::camera::{CameraUniform, OrbitCamera};
use simuforge_render::context::RenderContext;
use simuforge_render::mesh::ChunkMeshManager;
use simuforge_render::pipelines::pbr::{LightUniform, MaterialUniform, PbrPipeline};
use simuforge_render::pipelines::shadow::ShadowPipeline;
use simuforge_render::pipelines::ssao::{SsaoPipeline, SsaoParams};
use simuforge_render::pipelines::sss::{SssPipeline, SssParams};
use simuforge_render::pipelines::composite::{CompositePipeline, CompositeParams};
use simuforge_render::pipelines::line::{LinePipeline, LineVertex};

mod config;
mod ui;
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

/// Arm link colors [R, G, B, A] — industrial robot palette.
const ARM_LINK_COLORS: [[f32; 4]; 6] = [
    [0.15, 0.15, 0.17, 1.0],  // J1: anthracite (base)
    [0.85, 0.35, 0.05, 1.0],  // J2: industrial orange (upper arm)
    [0.85, 0.35, 0.05, 1.0],  // J3: industrial orange (forearm)
    [0.15, 0.15, 0.17, 1.0],  // J4: anthracite (wrist 1)
    [0.15, 0.15, 0.17, 1.0],  // J5: anthracite (wrist 2)
    [0.55, 0.55, 0.58, 1.0],  // J6: brushed aluminum (flange)
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

/// Optimal arm-local X distance for tool-down orientation.
/// At 0.72m, J3 ≈ -121° (comfortable margin from ±135° limit).
/// The dynamic track auto-positions the arm base so arm-local X stays near this value.
const OPTIMAL_LOCAL_X: f64 = 0.72;

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
    linear_track: LinearTrack,
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
    /// Consecutive frames where IK was collision-rejected.
    ik_consecutive_collisions: u32,
    /// Carving session (loaded G-code program).
    carving: Option<CarvingSession>,
    /// Current mode: Manual or Carving.
    mode: SimMode,
    /// Arm must be within this distance of target before cutting begins (m).
    tracking_threshold: f64,
    /// Orientation weight for IK during carving (0.0=position-only, 1.0=full 6DOF).
    orientation_weight: f64,
    /// Diagnostic: last A-axis angle where cutting happened, to detect face changes.
    diag_last_a: f64,
    /// Diagnostic: counter for cuts on current face.
    diag_cut_count: u32,
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

        // Initial joint targets: arm positioned above and outside workpiece.
        // With tool_offset=0.175 and center_z=-0.2, these angles put the tool
        // at roughly (0.81, 0.0, -0.28) which is safely outside the workpiece
        // bounds (0.50..0.80 in X, -0.35..-0.05 in Z).
        let mut joint_targets = vec![0.0; n];
        joint_targets[1] = 0.5;   // J2: shoulder raised (less than before)
        joint_targets[2] = -0.3;  // J3: elbow slightly bent

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
        let rotary_table = RotaryTable::new(5.0); // 5 kg·m² inertia
        let mut linear_track = LinearTrack::new(50.0); // 50 kg carriage
        linear_track.position = cfg.track_default_position;
        linear_track.target_position = cfg.track_default_position;

        // Initialize frame_start_position to the current tool tip (world space)
        // so the first frame doesn't create a huge swept path from origin.
        let track_offset = Vector3::new(linear_track.position, 0.0, 0.0);
        let initial_tool_pos = arm.tool_position() + track_offset;
        tool_state.position = initial_tool_pos;
        tool_state.prev_position = initial_tool_pos;
        tool_state.frame_start_position = initial_tool_pos;

        // Cartesian target = initial tool tip pose (world space)
        let mut cartesian_target = arm.forward_kinematics();
        cartesian_target.translation.vector += track_offset;

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
            linear_track,
            material: MaterialProps::marble(),
            ik_solver: IkSolver {
                damping: 0.05,
                max_iterations: 50,
                position_tolerance: 5e-4,
                orientation_tolerance: 0.005,
                max_step: 0.2,
                nullspace_gain: 0.5,
            },
            sim_time: 0.0,
            paused: true,
            cutting_force_magnitude: 0.0,
            cartesian_target,
            ik_converged: true,
            ik_consecutive_collisions: 0,
            carving: None,
            mode: SimMode::Manual,
            tracking_threshold: cfg.tracking_threshold,
            orientation_weight: cfg.orientation_weight,
            diag_last_a: 0.0,
            diag_cut_count: 0,
        }
    }

    /// Solve IK: update joint_targets from cartesian_target.
    /// Applies configuration consistency check (Phase 4), self-collision avoidance (Phase 5),
    /// and per-joint velocity limiting (Phase 1).
    fn update_ik(&mut self) {
        let mut scratch = self.arm.clone();
        scratch.set_joint_angles(&self.joint_targets);

        // Transform world target into arm-local frame by subtracting track offset
        let track_offset = Vector3::new(self.linear_track.position, 0.0, 0.0);
        let mut local_target = self.cartesian_target;
        local_target.translation.vector -= track_offset;

        let converged = if self.mode == SimMode::Carving {
            let (ok, _, _) = self.ik_solver.solve_weighted(
                &mut scratch, &local_target, self.orientation_weight,
            );
            ok
        } else {
            let (ok, _, _) = self.ik_solver.solve(&mut scratch, &local_target);
            ok
        };

        let new_angles = scratch.joint_angles();

        // Configuration guard: reject forward wrist solutions (J3 > 0) during carving.
        // Forward wrist makes the arm approach from the side, phasing through the block.
        // Backward wrist (J3 < 0) approaches from above — the only realistic configuration.
        if self.mode == SimMode::Carving && new_angles[2] > 0.0 {
            self.ik_converged = false;
            return;
        }

        // Self-collision avoidance — reject solutions in collision
        if !scratch.is_collision_free() {
            self.ik_consecutive_collisions += 1;
            if self.ik_consecutive_collisions == 30 {
                eprintln!("WARNING: IK collision-rejected 30 consecutive frames!");
                eprintln!("  target=({:.3},{:.3},{:.3})",
                    self.cartesian_target.translation.vector.x,
                    self.cartesian_target.translation.vector.y,
                    self.cartesian_target.translation.vector.z);
                eprintln!("  IK angles: [{:.1},{:.1},{:.1},{:.1},{:.1},{:.1}]°",
                    new_angles[0].to_degrees(), new_angles[1].to_degrees(),
                    new_angles[2].to_degrees(), new_angles[3].to_degrees(),
                    new_angles[4].to_degrees(), new_angles[5].to_degrees());
            }
            self.ik_converged = false;
            return;
        }
        self.ik_consecutive_collisions = 0;

        self.ik_converged = converged;

        // Phase 1: Per-joint velocity limiting instead of flat max_delta.
        // Scale entire delta vector proportionally so no joint exceeds its velocity limit.
        let frame_dt = 1.0 / 60.0; // ~60fps
        let mut deltas: Vec<f64> = new_angles.iter().zip(self.joint_targets.iter())
            .map(|(new, old)| new - old)
            .collect();
        IkSolver::apply_velocity_limits(&mut deltas, &self.arm, frame_dt);

        for i in 0..self.joint_targets.len() {
            self.joint_targets[i] += deltas[i];
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

        // Reset linear track
        self.linear_track.position = 0.0;
        self.linear_track.velocity = 0.0;
        self.linear_track.target_position = 0.0;

        // Reset tool state to initial tool position (world space)
        let track_offset = Vector3::new(self.linear_track.position, 0.0, 0.0);
        let initial_tool_pos = self.arm.tool_position() + track_offset;
        self.tool_state.position = initial_tool_pos;
        self.tool_state.prev_position = initial_tool_pos;
        self.tool_state.frame_start_position = initial_tool_pos;

        // Reset cartesian target (full pose, world space)
        let mut fk = self.arm.forward_kinematics();
        fk.translation.vector += track_offset;
        self.cartesian_target = fk;
        self.ik_converged = true;
        self.cutting_force_magnitude = 0.0;
    }

    /// Run one physics step at 1kHz.
    /// Run one physics step at 1kHz. Does NOT compute FK (caller handles that).
    fn physics_step(&mut self) {
        if self.paused {
            return;
        }

        let n = self.arm.num_joints();
        let gravity = Vector3::new(0.0, 0.0, -9.81); // DH convention: Z-up

        // Exact gravity compensation via RNEA (includes inter-joint coupling)
        let grav_comp = gravity_compensation_rnea(&self.arm, &gravity);

        // PID control: compute motor torque commands (stack array, no heap alloc)
        let mut joint_torques = [0.0; 6];
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
        let accelerations = articulated_body_algorithm(&self.arm, &joint_torques[..n], &gravity);

        // Integrate + hard-clamp joint limits (safety net)
        for i in 0..n {
            self.arm.joints[i].integrate(accelerations[i], PHYSICS_DT);
            self.arm.joints[i].clamp_to_limits();
        }

        // Rotary table
        self.rotary_table.step(PHYSICS_DT);

        // Linear track
        self.linear_track.step(PHYSICS_DT);

        self.sim_time += PHYSICS_DT;
    }

    /// Update tool position from current FK. Called once per frame after all physics steps.
    /// Tool position is in world space (arm-local + track offset).
    fn update_tool_state(&mut self) {
        let tool_pose = self.arm.forward_kinematics();
        let track_offset = Vector3::new(self.linear_track.position, 0.0, 0.0);
        let tool_pos = tool_pose.translation.vector + track_offset;
        self.tool_state.update_position(tool_pos);
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

        // --- Weighted IK pre-positioning with tool-down orientation ---
        // Rx(π) = tool Z pointing downward in DH frame.
        let pi = std::f64::consts::PI;
        let pi2 = std::f64::consts::FRAC_PI_2;
        let tool_down = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pi);

        // Snap track to optimal position for the workpiece center.
        // Dynamic track keeps arm-local X at OPTIMAL_LOCAL_X for best reachability.
        let initial_track = (workpiece_top_center.x - OPTIMAL_LOCAL_X)
            .clamp(self.linear_track.min_position, self.linear_track.max_position);
        self.linear_track.position = initial_track;
        self.linear_track.velocity = 0.0;
        self.linear_track.target_position = initial_track;

        // Desired position: 15mm above workpiece top center (arm-local, subtract track)
        let track_offset = Vector3::new(self.linear_track.position, 0.0, 0.0);
        let start_pos = Vector3::new(
            workpiece_top_center.x,
            workpiece_top_center.y,
            workpiece_top_center.z + 0.015,
        ) - track_offset;
        let start_target = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::from(start_pos),
            tool_down,
        );

        // IK seeds for backward wrist configuration (J3 < 0, J5 = +π/2).
        // At arm-local X ≈ 0.72m, backward wrist converges to J3 ≈ -121° (-2.11 rad).
        // J3 MUST be negative — positive J3 gives forward wrist (tool from side).
        let seeds: &[[f64; 6]] = &[
            [0.0, 0.8, -2.0, 0.0, pi2, 0.0],   // J3=-114.6°
            [0.0, 0.9, -2.1, 0.0, pi2, 0.0],   // J3=-120.3°
            [0.0, 1.0, -2.2, 0.0, pi2, 0.0],   // J3=-126.1°
            [0.0, 0.7, -1.8, 0.0, pi2, 0.0],   // J3=-103.1°
            [0.0, 0.85, -1.9, 0.0, pi2, 0.0],  // J3=-108.9°
        ];

        let mut best_angles: Option<Vec<f64>> = None;
        let mut best_err = f64::MAX;
        for seed in seeds {
            let mut scratch = self.arm.clone();
            scratch.set_joint_angles(&seed.to_vec());
            // Stronger orientation weight for pre-positioning (arm is stationary)
            let (ok, _, final_err) = self.ik_solver.solve_weighted(
                &mut scratch, &start_target, 0.5,
            );
            if !ok { continue; }
            // Reject forward wrist solutions (J3 > 0): arm approaches from the side,
            // phasing through the workpiece. Only accept backward wrist (J3 < 0).
            let j3 = scratch.joints[2].angle;
            if j3 > 0.0 {
                continue;
            }
            if final_err < best_err {
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
        let track_offset = Vector3::new(self.linear_track.position, 0.0, 0.0);
        let tool_pos = self.arm.tool_position() + track_offset;
        self.tool_state.position = tool_pos;
        self.tool_state.prev_position = tool_pos;
        self.tool_state.frame_start_position = tool_pos;
        for pid in &mut self.pid_controllers {
            pid.reset();
        }
        // Set cartesian_target to the arm's ACTUAL pose (not the desired one)
        // so there's zero initial error → no flailing.
        let mut world_fk = converged_fk;
        world_fk.translation.vector += track_offset;
        self.cartesian_target = world_fk;

        let mut session = CarvingSession::load_file(path, self.workpiece_center, half_extent, tool_orientation)?;
        session.set_start_position(tool_pos);

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
                // Arm too far from target — still repositioning, don't cut.
                // IMPORTANT: Do NOT reset frame_start_position here!
                // Let the tool path accumulate so that when the arm catches up,
                // the full swept path is cut in one shot. This prevents
                // "missing" cuts at high speed multipliers.
                self.cutting_force_magnitude = 0.0;
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
        // Apply inverse rotary table rotation so tool path maps to SDF local frame
        let wc = self.workpiece_center;
        let angle = self.rotary_table.angle;
        let rot_inv = nalgebra::Rotation3::from_axis_angle(&Vector3::x_axis(), -angle);
        let start = rot_inv * (start_world - wc);
        let end = rot_inv * (end_world - wc);

        // Diagnostic: detect face changes and log first cuts on each face
        let a_deg = angle.to_degrees();
        if (angle - self.diag_last_a).abs() > 0.1 {
            eprintln!("=== Face change: A={:.1}° → A={:.1}° ===", self.diag_last_a.to_degrees(), a_deg);
            self.diag_last_a = angle;
            self.diag_cut_count = 0;
        }
        if self.diag_cut_count < 5 {
            let he = self.workpiece_half_extent;
            // For A=0: depth from top = he - end.z (top face at z=+he)
            // For A=90: depth from front = he - end.y (front face at y=+he)
            let (face_name, depth) = if angle.abs() < 0.1 {
                ("TOP", he - end.z)
            } else if (angle - std::f64::consts::FRAC_PI_2).abs() < 0.1 {
                ("FRONT", he - end.y)
            } else if (angle + std::f64::consts::FRAC_PI_2).abs() < 0.1 {
                ("BACK", he + end.y) // back face at y=-he
            } else {
                ("BOTTOM", he + end.z) // bottom at z=-he
            };
            eprintln!("  Cut #{} on {} (A={:.1}°): local=({:.1},{:.1},{:.1})mm, depth={:.1}mm, world_end=({:.3},{:.3},{:.3})",
                self.diag_cut_count, face_name, a_deg,
                end.x*1000.0, end.y*1000.0, end.z*1000.0,
                depth*1000.0,
                end_world.x, end_world.y, end_world.z);
            self.diag_cut_count += 1;
        }

        // Quick AABB check: is the tool anywhere near the workpiece?
        // Check if EITHER endpoint is near the workpiece (tool may enter or exit).
        let half = self.workpiece_half_extent + 0.02; // workpiece half-extent + margin
        let start_inside = start.x.abs() <= half && start.y.abs() <= half && start.z.abs() <= half;
        let end_inside = end.x.abs() <= half && end.y.abs() <= half && end.z.abs() <= half;
        if !start_inside && !end_inside {
            self.cutting_force_magnitude = 0.0;
            return; // tool is entirely outside workpiece region
        }

        // Tool axis in workpiece-local coords: tool always points -Z in world,
        // so the body extends +Z (from tip toward shank). Rotate to local frame.
        let tool_axis_world = Vector3::new(0.0, 0.0, 1.0); // tip→shank = +Z in world
        let tool_axis_local = rot_inv * tool_axis_world;
        let tool_sdf = self.tool.swept_sdf(&start, &end, &tool_axis_local);
        let (bounds_min, bounds_max) = self.tool.swept_bounds(&start, &end, &tool_axis_local);
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
    shift: bool, alt: bool,
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
    // egui immediate-mode GUI
    egui_ctx: egui::Context,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
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
    stand_mesh: Option<GpuMesh>,
    stand_material: Option<MaterialBind>,
    ground_mesh: Option<GpuMesh>,
    ground_material: Option<MaterialBind>,
    workpiece_material: Option<MaterialBind>,
    target_material: Option<MaterialBind>,
    // Linear track (rail + carriage)
    track_rail_mesh: Option<GpuMesh>,
    track_rail_material: Option<MaterialBind>,
    track_carriage_mesh: Option<GpuMesh>,
    track_carriage_material: Option<MaterialBind>,
    // Tool (Dremel) visual
    tool_body_material: Option<MaterialBind>,
    tool_tip_material: Option<MaterialBind>,
    // Post-processing resources
    post_sampler: Option<wgpu::Sampler>,
    depth_sampler: Option<wgpu::Sampler>,
    ssao_bind_group: Option<wgpu::BindGroup>,
    sss_bind_group: Option<wgpu::BindGroup>,
    composite_bind_group: Option<wgpu::BindGroup>,
    // Carving / toolpath visualization
    line_pipeline: Option<LinePipeline>,
    speed_multiplier: f64,
    max_physics_steps: u32,
    gcode_path: Option<String>,
    /// Previous A-axis target — detect when rotation is needed.
    prev_a_axis: f64,
    /// Retract phase during A-axis rotation: 0=none, 1=retracting, 2=rotating, 3=approaching.
    retract_phase: u8,
    // Background meshing
    async_mesher: AsyncMesher,
    // Config-derived visual params
    tool_radius_m: f64,
    tool_cutting_length_m: f64,
    // Cached toolpath line vertices (rebuilt only when progress changes)
    cached_line_verts: Vec<LineVertex>,
    cached_line_progress: usize,
    /// Whether to show G-code toolpath lines (Alt+H to toggle).
    show_toolpath: bool,
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
            egui_ctx: egui::Context::default(),
            egui_state: None,
            egui_renderer: None,
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
            stand_mesh: None,
            stand_material: None,
            ground_mesh: None,
            ground_material: None,
            workpiece_material: None,
            target_material: None,
            track_rail_mesh: None,
            track_rail_material: None,
            track_carriage_mesh: None,
            track_carriage_material: None,
            tool_body_material: None,
            tool_tip_material: None,
            post_sampler: None,
            depth_sampler: None,
            ssao_bind_group: None,
            sss_bind_group: None,
            composite_bind_group: None,
            line_pipeline: None,
            speed_multiplier: cfg.default_speed,
            max_physics_steps: cfg.max_physics_steps,
            gcode_path: None,
            prev_a_axis: 0.0,
            retract_phase: 0,
            held_keys: HeldKeys::default(),
            async_mesher: AsyncMesher::new(0),
            tool_radius_m: cfg.tool_radius_mm * 0.001,
            tool_cutting_length_m: cfg.tool_cutting_length_mm * 0.001,
            cached_line_verts: Vec::new(),
            cached_line_progress: usize::MAX, // force rebuild on first use
            show_toolpath: true,
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
        let track_offset = Vector3::new(self.sim.linear_track.position, 0.0, 0.0);
        let target_pos = self.sim.cartesian_target.translation.vector - track_offset;
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

        // Update light — warm key light from upper-right, cool ambient fill
        let light_dir = Vec3::new(-0.5, -0.7, -0.4);
        let light = LightUniform {
            direction: [light_dir.x, light_dir.y, light_dir.z, 0.0],
            color: [1.0, 0.96, 0.90, 3.5],
            ambient: [0.5, 0.55, 0.65, 0.30],
            eye_pos: cam_uniform.eye_pos,
        };
        pbr.update_light(&ctx.queue, &light);

        // --- Compute model matrices for all objects ---

        // Workpiece model matrix: coord_swap * translate(workpiece_center)
        let wc = &self.sim.workpiece_center;
        let translate_dh = Mat4::from_translation(Vec3::new(
            wc.x as f32, wc.y as f32, wc.z as f32,
        ));
        let table_angle = self.sim.rotary_table.angle as f32;
        let rot_a = Mat4::from_axis_angle(Vec3::X, table_angle);
        let workpiece_model = swap * translate_dh * rot_a;

        // Arm link frames → render-space positions (offset by track position)
        let track_x = self.sim.linear_track.position as f32;
        let frames = self.sim.arm.link_frames();
        let mut render_pts = Vec::with_capacity(7);
        for i in 0..7 {
            let f = isometry_to_glam(&frames[i]);
            let dh_pos = Vec3::new(f.col(3).x + track_x, f.col(3).y, f.col(3).z);
            render_pts.push(swap.transform_point3(dh_pos));
        }

        // Arm link model matrices
        let mut arm_models = Vec::with_capacity(6);
        for i in 0..6 {
            arm_models.push(link_model(render_pts[i], render_pts[i + 1], ARM_LINK_RADII[i]));
        }

        // Joint sphere model matrices
        let joint_sphere_radii: [f32; 7] = [0.10, 0.09, 0.08, 0.07, 0.06, 0.06, 0.05];
        let mut joint_models = Vec::with_capacity(7);
        for i in 0..7 {
            joint_models.push(Mat4::from_scale_rotation_translation(
                Vec3::splat(joint_sphere_radii[i]),
                Quat::IDENTITY,
                render_pts[i],
            ));
        }

        // Tool models (offset by track position)
        let flange_mat = isometry_to_glam(&self.sim.arm.flange_pose());
        let tool_z_dh = Vec3::new(flange_mat.col(2).x, flange_mat.col(2).y, flange_mat.col(2).z);
        let flange_pos_dh = Vec3::new(flange_mat.col(3).x + track_x, flange_mat.col(3).y, flange_mat.col(3).z);
        let body_end_dh = flange_pos_dh + tool_z_dh * 0.15;
        let body_from = swap.transform_point3(flange_pos_dh);
        let body_to = swap.transform_point3(body_end_dh);
        let tool_body_model = link_model(body_from, body_to, 0.018);

        let tip_end_dh = body_end_dh + tool_z_dh * self.tool_cutting_length_m as f32;
        let tip_to = swap.transform_point3(tip_end_dh);
        let tool_tip_model = link_model(body_to, tip_to, self.tool_radius_m as f32);

        // Base model — pedestal from arm base down to ground
        let wp_bottom_y = self.sim.workpiece_center.z as f32 - self.sim.workpiece_half_extent as f32;
        let ground_y = wp_bottom_y - 0.01;
        let base_half_h = (-ground_y / 2.0).max(0.01);
        let base_pos = Vec3::new(render_pts[0].x, render_pts[0].y - base_half_h, render_pts[0].z);
        let base_model = Mat4::from_translation(base_pos);

        // Workpiece stand model: centered under the workpiece
        let stand_center_y = (wp_bottom_y + ground_y) / 2.0;
        let stand_pos = Vec3::new(wc.x as f32, stand_center_y, -(wc.y as f32));
        let stand_model = Mat4::from_translation(stand_pos);

        // Track rail: static, centered at midpoint of travel range, on ground.
        // DH coords: X = track center, Y = 0, Z = ground level + rail half-height.
        let track_min = self.sim.linear_track.min_position as f32;
        let track_max = self.sim.linear_track.max_position as f32;
        let rail_center_x = (track_min + track_max) / 2.0;
        let rail_dh = Vec3::new(rail_center_x, 0.0, ground_y + 0.01 + 0.02); // ground + margin + half rail height
        // In render space, DH Z→Y, DH Y→-Z. Rail extends along X.
        // generate_box(hx, hy, hz) → render box is ±hx on X, ±hy on Y, ±hz on Z.
        // We want rail extending along render-X, thin in render-Y (height), and narrow in render-Z (depth).
        // DH X = render X, DH Z = render Y (via coord_swap).
        let rail_render_pos = swap.transform_point3(rail_dh);
        let track_rail_model = Mat4::from_translation(rail_render_pos);

        // Track carriage: slides with arm base, sits on top of rail.
        let carriage_dh = Vec3::new(track_x, 0.0, ground_y + 0.01 + 0.04 + 0.025); // on top of rail
        let carriage_render_pos = swap.transform_point3(carriage_dh);
        let track_carriage_model = Mat4::from_translation(carriage_render_pos);

        // Target sphere model
        let ct_pos = &self.sim.cartesian_target.translation.vector;
        let tgt_dh = Vec3::new(ct_pos.x as f32, ct_pos.y as f32, ct_pos.z as f32);
        let tgt_render = swap.transform_point3(tgt_dh);
        let tgt_radius = 0.012_f32;
        let target_model = Mat4::from_scale_rotation_translation(
            Vec3::splat(tgt_radius),
            Quat::IDENTITY,
            tgt_render,
        );

        // --- Upload material uniforms ---
        if let Some(wp) = &self.workpiece_material {
            let he = self.sim.workpiece_half_extent as f32;
            let mat = MaterialUniform::marble()
                .with_model(workpiece_model.to_cols_array_2d())
                .with_bounds([-he, -he, -he], [he, he, he]);
            ctx.queue.write_buffer(&wp.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(gnd) = &self.ground_material {
            let mat = MaterialUniform::ground();
            ctx.queue.write_buffer(&gnd.buffer, 0, bytemuck::bytes_of(&mat));
        }
        let joint_color = [0.12, 0.12, 0.14, 1.0];
        for i in 0..6 {
            if i < self.arm_materials.len() {
                let mat = MaterialUniform::metal(ARM_LINK_COLORS[i])
                    .with_model(arm_models[i].to_cols_array_2d());
                ctx.queue.write_buffer(&self.arm_materials[i].buffer, 0, bytemuck::bytes_of(&mat));
            }
        }
        for i in 0..7 {
            if i < self.joint_materials.len() {
                let mat = MaterialUniform::metal(joint_color)
                    .with_model(joint_models[i].to_cols_array_2d());
                ctx.queue.write_buffer(&self.joint_materials[i].buffer, 0, bytemuck::bytes_of(&mat));
            }
        }
        if let Some(tgt_mat) = &self.target_material {
            let tgt_color = if self.sim.ik_converged {
                [0.1, 1.0, 0.3, 1.0]
            } else {
                [1.0, 0.2, 0.1, 1.0]
            };
            let mat = MaterialUniform::metal(tgt_color)
                .with_model(target_model.to_cols_array_2d());
            ctx.queue.write_buffer(&tgt_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(tb_mat) = &self.tool_body_material {
            let mat = MaterialUniform::metal([0.35, 0.35, 0.40, 1.0])
                .with_model(tool_body_model.to_cols_array_2d());
            ctx.queue.write_buffer(&tb_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(tt_mat) = &self.tool_tip_material {
            let mat = MaterialUniform::metal([0.85, 0.85, 0.90, 1.0])
                .with_model(tool_tip_model.to_cols_array_2d());
            ctx.queue.write_buffer(&tt_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(base_mat) = &self.base_material {
            let mat = MaterialUniform::metal([0.15, 0.15, 0.18, 1.0])
                .with_model(base_model.to_cols_array_2d());
            ctx.queue.write_buffer(&base_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(stand_mat) = &self.stand_material {
            let mat = MaterialUniform::metal([0.20, 0.20, 0.22, 1.0])
                .with_model(stand_model.to_cols_array_2d());
            ctx.queue.write_buffer(&stand_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(rail_mat) = &self.track_rail_material {
            let mat = MaterialUniform::metal([0.30, 0.30, 0.32, 1.0])
                .with_model(track_rail_model.to_cols_array_2d());
            ctx.queue.write_buffer(&rail_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(car_mat) = &self.track_carriage_material {
            let mat = MaterialUniform::metal([0.18, 0.18, 0.20, 1.0])
                .with_model(track_carriage_model.to_cols_array_2d());
            ctx.queue.write_buffer(&car_mat.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // --- Shadow setup ---
        let light_vp = ShadowPipeline::compute_light_matrix(light_dir.normalize(), 2.0);
        pbr.update_shadow_light_vp(&ctx.queue, &light_vp);

        // Collect shadow matrices: light_vp * model for each shadow caster
        // Order: workpiece, arm links (6), joint spheres (7), tool body, tool tip, base, stand
        let mut shadow_matrices = Vec::with_capacity(18);
        shadow_matrices.push(light_vp * workpiece_model); // 0: workpiece
        for i in 0..6 {
            shadow_matrices.push(light_vp * arm_models[i]); // 1-6: arm links
        }
        for i in 0..7 {
            shadow_matrices.push(light_vp * joint_models[i]); // 7-13: joint spheres
        }
        shadow_matrices.push(light_vp * tool_body_model); // 14: tool body
        shadow_matrices.push(light_vp * tool_tip_model);  // 15: tool tip
        shadow_matrices.push(light_vp * base_model);      // 16: base
        shadow_matrices.push(light_vp * stand_model);     // 17: workpiece stand
        shadow_matrices.push(light_vp * track_rail_model);    // 18: track rail
        shadow_matrices.push(light_vp * track_carriage_model); // 19: track carriage

        if let Some(shadow) = &self.shadow_pipeline {
            shadow.upload_matrices(&ctx.queue, &shadow_matrices);
        }

        // --- Update SSAO params with current projection ---
        if let Some(ssao) = &self.ssao_pipeline {
            ssao.update_params(&ctx.queue, &SsaoParams {
                proj: cam_uniform.proj,
                radius: 0.5,
                bias: 0.025,
                intensity: 1.5,
                _pad: 0.0,
            });
        }

        // Upload completed meshes from background workers.
        let async_meshes = self.async_mesher.poll_completed();
        for mesh in &async_meshes {
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
        // Rebuild merged GPU buffer if any chunks changed (persistent buffer, no alloc).
        self.chunk_meshes.rebuild_if_dirty(&ctx.device, &ctx.queue);

        // --- Begin rendering ---
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

        // ====== Pass 1: Sky clear → swapchain ======
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Sky Clear"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.06, g: 0.07, b: 0.12, a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                ..Default::default()
            });
        }

        // ====== Pass 2: Shadow depth → shadow texture (Clear) ======
        if let Some(shadow) = &self.shadow_pipeline {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Shadow Pass"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &shadow.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            pass.set_pipeline(&shadow.pipeline);

            let mut si = 0usize; // shadow matrix index

            // Workpiece (single merged buffer, 1 draw call)
            pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
            if let (Some(vb), Some(ib)) = (self.chunk_meshes.merged_vertex_buffer(), self.chunk_meshes.merged_index_buffer()) {
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..self.chunk_meshes.merged_num_indices(), 0, 0..1);
            }
            si += 1;

            // Arm links (6)
            if let Some(cyl) = &self.arm_cylinder {
                for i in 0..6 {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si + i)]);
                    pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                    pass.set_index_buffer(cyl.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
                }
            }
            si += 6;

            // Joint spheres (7)
            if let Some(sph) = &self.arm_sphere {
                for i in 0..7 {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si + i)]);
                    pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                    pass.set_index_buffer(sph.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..sph.num_indices, 0, 0..1);
                }
            }
            si += 7;

            // Tool body
            if let Some(cyl) = &self.arm_cylinder {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                pass.set_index_buffer(cyl.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
            }
            si += 1;

            // Tool tip
            if let Some(cyl) = &self.arm_cylinder {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                pass.set_index_buffer(cyl.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
            }
            si += 1;

            // Base pedestal
            if let Some(bm) = &self.base_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, bm.vertex_buffer.slice(..));
                pass.set_index_buffer(bm.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..bm.num_indices, 0, 0..1);
            }
            si += 1;

            // Workpiece stand
            if let Some(sm) = &self.stand_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, sm.vertex_buffer.slice(..));
                pass.set_index_buffer(sm.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..sm.num_indices, 0, 0..1);
            }
            si += 1;

            // Track rail
            if let Some(rm) = &self.track_rail_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, rm.vertex_buffer.slice(..));
                pass.set_index_buffer(rm.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..rm.num_indices, 0, 0..1);
            }
            si += 1;

            // Track carriage
            if let Some(cm) = &self.track_carriage_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, cm.vertex_buffer.slice(..));
                pass.set_index_buffer(cm.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..cm.num_indices, 0, 0..1);
            }
        }

        // ====== Pass 3: PBR → HDR texture (Clear to transparent) + depth (Clear) ======
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("PBR Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &ctx.hdr_texture,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
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
            render_pass.set_bind_group(1, &pbr.shadow_bind_group, &[]);

            // 1) Ground plane
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

            // 2) Workpiece (single merged draw call)
            if let (Some(wp), Some(vb), Some(ib)) = (
                &self.workpiece_material,
                self.chunk_meshes.merged_vertex_buffer(),
                self.chunk_meshes.merged_index_buffer(),
            ) {
                render_pass.set_bind_group(0, &wp.bind_group, &[]);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..self.chunk_meshes.merged_num_indices(), 0, 0..1);
            }

            // 3) Arm links
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

            // 4) Joint spheres
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

            // 5) Target sphere
            if let (Some(sph), Some(tgt_mat)) = (&self.arm_sphere, &self.target_material) {
                render_pass.set_bind_group(0, &tgt_mat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    sph.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..sph.num_indices, 0, 0..1);
            }

            // 6) Tool body
            if let (Some(cyl), Some(tb_mat)) = (&self.arm_cylinder, &self.tool_body_material) {
                render_pass.set_bind_group(0, &tb_mat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    cyl.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
            }

            // 7) Tool tip
            if let (Some(cyl), Some(tt_mat)) = (&self.arm_cylinder, &self.tool_tip_material) {
                render_pass.set_bind_group(0, &tt_mat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    cyl.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
            }

            // 8) Base pedestal
            if let (Some(bm), Some(bmat)) = (&self.base_mesh, &self.base_material) {
                render_pass.set_bind_group(0, &bmat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, bm.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    bm.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..bm.num_indices, 0, 0..1);
            }

            // 9) Workpiece stand
            if let (Some(sm), Some(smat)) = (&self.stand_mesh, &self.stand_material) {
                render_pass.set_bind_group(0, &smat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, sm.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    sm.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..sm.num_indices, 0, 0..1);
            }

            // 10) Track rail
            if let (Some(rm), Some(rmat)) = (&self.track_rail_mesh, &self.track_rail_material) {
                render_pass.set_bind_group(0, &rmat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, rm.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    rm.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..rm.num_indices, 0, 0..1);
            }

            // 11) Track carriage
            if let (Some(cm), Some(cmat)) = (&self.track_carriage_mesh, &self.track_carriage_material) {
                render_pass.set_bind_group(0, &cmat.bind_group, &[]);
                render_pass.set_vertex_buffer(0, cm.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    cm.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint32,
                );
                render_pass.draw_indexed(0..cm.num_indices, 0, 0..1);
            }
        }

        // ====== Pass 4: SSAO → SSAO output texture ======
        if let (Some(ssao), Some(ssao_bg)) = (&self.ssao_pipeline, &self.ssao_bind_group) {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("SSAO Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &ssao.output_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::WHITE),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                ..Default::default()
            });
            pass.set_pipeline(&ssao.pipeline);
            pass.set_bind_group(0, ssao_bg, &[]);
            pass.draw(0..3, 0..1);
        }

        // ====== Pass 5: SSS → SSS output texture ======
        if let (Some(sss), Some(sss_bg)) = (&self.sss_pipeline, &self.sss_bind_group) {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("SSS Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &sss.output_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                ..Default::default()
            });
            pass.set_pipeline(&sss.pipeline);
            pass.set_bind_group(0, sss_bg, &[]);
            pass.draw(0..3, 0..1);
        }

        // ====== Pass 6: Composite → swapchain (Load, preserving sky) ======
        if let (Some(composite), Some(comp_bg)) = (&self.composite_pipeline, &self.composite_bind_group) {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Composite Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                ..Default::default()
            });
            pass.set_pipeline(&composite.pipeline);
            pass.set_bind_group(0, comp_bg, &[]);
            pass.draw(0..3, 0..1);
        }

        // ====== Pass 7: Toolpath lines → swapchain (Load) ======
        let current_progress = self.sim.carving.as_ref().map_or(0, |s| s.progress_counts().0);
        let lines_dirty = current_progress != self.cached_line_progress;
        if lines_dirty {
            self.cached_line_verts = self.build_toolpath_lines();
            self.cached_line_progress = current_progress;
        }
        if self.show_toolpath {
        if let Some(line_pipe) = &mut self.line_pipeline {
            if lines_dirty && !self.cached_line_verts.is_empty() {
                line_pipe.upload(&ctx.queue, &self.cached_line_verts);
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
        } // show_toolpath

        // ====== Pass 8: egui UI → swapchain (Load) ======
        let ui_data = self.collect_ui_data();
        let egui_input = self.egui_state.as_mut().unwrap().take_egui_input(
            self.window.as_ref().unwrap(),
        );
        let mut ui_actions = Vec::new();
        let egui_output = self.egui_ctx.run(egui_input, |ctx| {
            ui_actions = ui::build_ui(ctx, &ui_data);
        });

        // Tessellate
        let paint_jobs = self.egui_ctx.tessellate(
            egui_output.shapes,
            egui_output.pixels_per_point,
        );

        // Update egui textures and buffers
        let screen_descriptor = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [ctx.config.width, ctx.config.height],
            pixels_per_point: egui_output.pixels_per_point,
        };

        let egui_renderer = self.egui_renderer.as_mut().unwrap();
        for (id, image_delta) in &egui_output.textures_delta.set {
            egui_renderer.update_texture(&ctx.device, &ctx.queue, *id, image_delta);
        }
        let egui_cmd_bufs = egui_renderer.update_buffers(
            &ctx.device,
            &ctx.queue,
            &mut encoder,
            &paint_jobs,
            &screen_descriptor,
        );

        // Render egui (forget_lifetime required by egui-wgpu's 'static RenderPass API)
        {
            let pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("egui Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                ..Default::default()
            });
            let mut pass = pass.forget_lifetime();
            egui_renderer.render(&mut pass, &paint_jobs, &screen_descriptor);
        }

        // Free egui textures
        for id in &egui_output.textures_delta.free {
            egui_renderer.free_texture(id);
        }

        // Handle platform output (cursor icon, clipboard, etc.)
        self.egui_state.as_mut().unwrap().handle_platform_output(
            self.window.as_ref().unwrap(),
            egui_output.platform_output,
        );

        // Submit everything
        let mut cmd_bufs: Vec<wgpu::CommandBuffer> = egui_cmd_bufs;
        cmd_bufs.push(encoder.finish());
        ctx.queue.submit(cmd_bufs);
        output.present();

        // Process UI actions from egui buttons
        for action in ui_actions {
            match action {
                ui::UiAction::TogglePause => {
                    if self.sim.mode == SimMode::Carving {
                        if let Some(session) = &mut self.sim.carving {
                            session.toggle();
                        }
                    } else {
                        self.sim.paused = !self.sim.paused;
                    }
                }
                ui::UiAction::SpeedUp => {
                    let current_idx = SPEED_PRESETS.iter()
                        .position(|&s| (s - self.speed_multiplier).abs() < 0.01)
                        .unwrap_or_else(|| {
                            SPEED_PRESETS.iter()
                                .position(|&s| s > self.speed_multiplier)
                                .unwrap_or(SPEED_PRESETS.len() - 1)
                        });
                    let next_idx = (current_idx + 1).min(SPEED_PRESETS.len() - 1);
                    self.speed_multiplier = SPEED_PRESETS[next_idx];
                }
                ui::UiAction::SpeedDown => {
                    let current_idx = SPEED_PRESETS.iter()
                        .position(|&s| (s - self.speed_multiplier).abs() < 0.01)
                        .unwrap_or_else(|| {
                            SPEED_PRESETS.iter()
                                .rposition(|&s| s < self.speed_multiplier)
                                .unwrap_or(0)
                        });
                    let next_idx = if current_idx > 0 { current_idx - 1 } else { 0 };
                    self.speed_multiplier = SPEED_PRESETS[next_idx];
                }
            }
        }

        // FPS counter
        self.frame_count += 1;
        let elapsed = self.fps_timer.elapsed().as_secs_f64();
        if elapsed >= 1.0 {
            self.fps = self.frame_count as f64 / elapsed;
            self.frame_count = 0;
            self.fps_timer = Instant::now();
        }
    }

    /// Collect all UI data from current state into a snapshot for egui rendering.
    fn collect_ui_data(&self) -> ui::UiData {
        let joint_angles: [f64; 6] = std::array::from_fn(|i| {
            self.sim.arm.joints[i].angle.to_degrees()
        });
        let joint_mins: [f64; 6] = std::array::from_fn(|i| {
            self.sim.arm.joints[i].angle_min.to_degrees()
        });
        let joint_maxs: [f64; 6] = std::array::from_fn(|i| {
            self.sim.arm.joints[i].angle_max.to_degrees()
        });
        let joint_torques: [f64; 6] = std::array::from_fn(|i| {
            self.sim.arm.joints[i].torque
        });

        let ct = &self.sim.cartesian_target.translation.vector;
        let tp = &self.sim.tool_state.position;
        let wc = &self.sim.workpiece_center;
        let local = tp - wc;
        let half = self.sim.workpiece_half_extent;
        let inside = local.x.abs() < half && local.y.abs() < half && local.z.abs() < half;

        let tracking_error = (ct - tp).norm() * 1000.0; // mm

        let carving_state = match self.sim.mode {
            SimMode::Carving => self.sim.carving.as_ref().map(|s| match s.state {
                CarvingState::Idle => 0u8,
                CarvingState::Running => 1,
                CarvingState::Paused => 2,
                CarvingState::Complete => 3,
            }),
            SimMode::Manual => None,
        };

        let (carving_done, carving_total) = self.sim.carving.as_ref()
            .map_or((0, 0), |s| s.progress_counts());
        let carving_progress = self.sim.carving.as_ref()
            .map_or(0.0, |s| s.progress() as f32);
        let carving_eta = self.sim.carving.as_ref()
            .map_or(0.0, |s| s.eta());

        let tool_type = match self.sim.tool.tool_type {
            simuforge_cutting::tool::ToolType::BallNose => "Ball Nose",
            simuforge_cutting::tool::ToolType::FlatEnd => "Flat End",
            simuforge_cutting::tool::ToolType::TaperedBall { .. } => "Tapered Ball",
        };

        let gcode_file = self.gcode_path.clone();
        let gcode_waypoints = self.sim.carving.as_ref()
            .map_or(0, |s| s.waypoints.len());
        let gcode_est_time = self.sim.carving.as_ref()
            .map_or(0.0, |s| s.estimated_total_time);

        ui::UiData {
            mode_manual: self.sim.mode == SimMode::Manual,
            paused: self.sim.paused,
            carving_state,
            speed_multiplier: self.speed_multiplier,
            fps: self.fps,
            carving_progress,
            carving_done,
            carving_total,
            carving_eta,
            joint_angles,
            joint_mins,
            joint_maxs,
            joint_torques,
            ik_converged: self.sim.ik_converged,
            target_pos: [ct.x * 1000.0, ct.y * 1000.0, ct.z * 1000.0],
            tool_pos: [tp.x * 1000.0, tp.y * 1000.0, tp.z * 1000.0],
            inside_block: inside,
            tracking_error,
            spindle_on: self.sim.tool_state.spindle_on,
            cutting_force: self.sim.cutting_force_magnitude,
            sim_time: self.sim.sim_time,
            chunk_count: self.chunk_meshes.chunk_count(),
            total_triangles: self.chunk_meshes.total_triangles() as usize,
            tool_type,
            tool_radius_mm: self.sim.tool.radius * 1000.0,
            tool_cutting_length_mm: self.sim.tool.cutting_length * 1000.0,
            gcode_file,
            gcode_waypoints,
            gcode_est_time,
        }
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
        let shadow = ShadowPipeline::new(&ctx);
        let pbr = PbrPipeline::new(&ctx, &shadow);
        let ssao = SsaoPipeline::new(&ctx);
        let sss = SssPipeline::new(&ctx);
        let composite = CompositePipeline::new(&ctx);

        // Initialize egui
        ui::setup_fonts(&self.egui_ctx);
        ui::setup_theme(&self.egui_ctx);
        let viewport_id = self.egui_ctx.viewport_id();
        let egui_state = egui_winit::State::new(
            self.egui_ctx.clone(),
            viewport_id,
            &window,
            None,
            None,
            None,
        );
        let egui_renderer = egui_wgpu::Renderer::new(
            &ctx.device,
            ctx.format(),
            None, // no depth
            1,    // msaa samples
            false, // no dithering
        );
        self.egui_state = Some(egui_state);
        self.egui_renderer = Some(egui_renderer);

        // Post-processing samplers
        let post_sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Post-Process Sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let depth_sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Depth Sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        // Post-processing bind groups
        let ssao_bg = ssao.create_bind_group(&ctx.device, &ctx.depth_texture, &depth_sampler);
        let sss_bg = sss.create_bind_group(&ctx.device, &ctx.hdr_texture, &ctx.depth_texture, &post_sampler);
        let composite_bg = composite.create_bind_group(&ctx.device, &sss.output_view, &ssao.output_view, &post_sampler);

        // Initialize post-processing parameters
        let w = ctx.config.width as f32;
        let h = ctx.config.height as f32;
        ssao.update_params(&ctx.queue, &SsaoParams {
            proj: glam::Mat4::IDENTITY.to_cols_array_2d(),
            radius: 0.5,
            bias: 0.025,
            intensity: 1.5,
            _pad: 0.0,
        });
        sss.update_params(&ctx.queue, &SssParams::marble(w, h));
        composite.update_params(&ctx.queue, &CompositeParams::default());

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

        // Ground level: just below the workpiece bottom.
        let wp_bottom_y_init = self.sim.workpiece_center.z as f32 - self.sim.workpiece_half_extent as f32;
        let ground_y_init = wp_bottom_y_init - 0.01; // 1cm below workpiece bottom

        // Base pedestal: tall box from arm base (Y=0) down to ground.
        let base_half_h = (-ground_y_init / 2.0).max(0.01);
        let (base_verts, base_idxs) = generate_box(0.10, base_half_h, 0.10);
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

        // Linear track rail: long flat box on the ground, extending along X (DH).
        // In render space: X stays X, DH-Z → render-Y, DH-Y → render -Z.
        // Rail extends from min_position to max_position along DH-X.
        let track = &self.sim.linear_track;
        let rail_half_len = ((track.max_position - track.min_position) / 2.0) as f32 + 0.15;
        let rail_half_h = 0.02_f32; // 4cm tall rail
        let rail_half_w = 0.08_f32; // 16cm wide
        let (rail_verts, rail_idxs) = generate_box(rail_half_len, rail_half_h, rail_half_w);
        let rail_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Track Rail VB"),
            contents: bytemuck::cast_slice(&rail_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let rail_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Track Rail IB"),
            contents: bytemuck::cast_slice(&rail_idxs),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.track_rail_mesh = Some(GpuMesh {
            vertex_buffer: rail_vb,
            index_buffer: rail_ib,
            num_indices: rail_idxs.len() as u32,
        });
        let (rbuf, rbg) = pbr.create_material_bind_group(&ctx.device);
        self.track_rail_material = Some(MaterialBind { buffer: rbuf, bind_group: rbg });

        // Track carriage: smaller box that slides with the arm base.
        let carriage_half_len = 0.12_f32; // 24cm long
        let carriage_half_h = 0.025_f32; // 5cm tall
        let carriage_half_w = 0.10_f32; // 20cm wide
        let (car_verts, car_idxs) = generate_box(carriage_half_len, carriage_half_h, carriage_half_w);
        let car_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Track Carriage VB"),
            contents: bytemuck::cast_slice(&car_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let car_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Track Carriage IB"),
            contents: bytemuck::cast_slice(&car_idxs),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.track_carriage_mesh = Some(GpuMesh {
            vertex_buffer: car_vb,
            index_buffer: car_ib,
            num_indices: car_idxs.len() as u32,
        });
        let (cbuf, cbg) = pbr.create_material_bind_group(&ctx.device);
        self.track_carriage_material = Some(MaterialBind { buffer: cbuf, bind_group: cbg });

        // Workpiece stand: box from ground up to workpiece bottom.
        let wp_bottom_y = self.sim.workpiece_center.z as f32 - self.sim.workpiece_half_extent as f32;
        let ground_y = wp_bottom_y - 0.01;
        let stand_half_h = (wp_bottom_y - ground_y) / 2.0;
        if stand_half_h > 0.001 {
            let (stand_verts, stand_idxs) = generate_box(0.06, stand_half_h, 0.06);
            let stand_vb = ctx
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Workpiece Stand VB"),
                    contents: bytemuck::cast_slice(&stand_verts),
                    usage: wgpu::BufferUsages::VERTEX,
                });
            let stand_ib = ctx
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Workpiece Stand IB"),
                    contents: bytemuck::cast_slice(&stand_idxs),
                    usage: wgpu::BufferUsages::INDEX,
                });
            self.stand_mesh = Some(GpuMesh {
                vertex_buffer: stand_vb,
                index_buffer: stand_ib,
                num_indices: stand_idxs.len() as u32,
            });
            let (sbuf, sbg) = pbr.create_material_bind_group(&ctx.device);
            self.stand_material = Some(MaterialBind {
                buffer: sbuf,
                bind_group: sbg,
            });
        }

        // Ground plane just below workpiece bottom.
        let gs = 3.0f32; // 3m half-extent
        let gy = ground_y;
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
        self.post_sampler = Some(post_sampler);
        self.depth_sampler = Some(depth_sampler);
        self.ssao_bind_group = Some(ssao_bg);
        self.sss_bind_group = Some(sss_bg);
        self.composite_bind_group = Some(composite_bg);
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
        // Pass event to egui first
        let _egui_consumed = if let Some(egui_state) = &mut self.egui_state {
            if let Some(window) = &self.window {
                let resp = egui_state.on_window_event(window, &event);
                resp.consumed
            } else {
                false
            }
        } else {
            false
        };

        // Check if egui wants pointer/keyboard input (for suppressing camera/key handling)
        let egui_wants_pointer = self.egui_ctx.wants_pointer_input();
        let egui_wants_keyboard = self.egui_ctx.wants_keyboard_input();

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
                        // Update SSS screen size params
                        let w = ctx.config.width as f32;
                        let h = ctx.config.height as f32;
                        sss.update_params(&ctx.queue, &SssParams::marble(w, h));
                    }
                    // Recreate post-processing bind groups (textures changed)
                    if let (Some(ssao), Some(sss), Some(composite), Some(ds), Some(ps)) = (
                        &self.ssao_pipeline,
                        &self.sss_pipeline,
                        &self.composite_pipeline,
                        &self.depth_sampler,
                        &self.post_sampler,
                    ) {
                        self.ssao_bind_group = Some(ssao.create_bind_group(
                            &ctx.device, &ctx.depth_texture, ds,
                        ));
                        self.sss_bind_group = Some(sss.create_bind_group(
                            &ctx.device, &ctx.hdr_texture, &ctx.depth_texture, ps,
                        ));
                        self.composite_bind_group = Some(composite.create_bind_group(
                            &ctx.device, &sss.output_view, &ssao.output_view, ps,
                        ));
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
            } if !egui_wants_keyboard => {
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
                    Key::Named(NamedKey::Alt) => self.held_keys.alt = pressed,
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
                            let mut fk = self.sim.arm.forward_kinematics();
                            let track_off = Vector3::new(self.sim.linear_track.position, 0.0, 0.0);
                            fk.translation.vector += track_off;
                            self.sim.cartesian_target = fk;
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
                        Key::Character("]") => {
                            self.sim.orientation_weight = (self.sim.orientation_weight + 0.1).min(1.0);
                            eprintln!("Orientation weight: {:.1}", self.sim.orientation_weight);
                        }
                        Key::Character("[") => {
                            self.sim.orientation_weight = (self.sim.orientation_weight - 0.1).max(0.0);
                            eprintln!("Orientation weight: {:.1}", self.sim.orientation_weight);
                        }
                        Key::Character("h") | Key::Character("H") if self.held_keys.alt => {
                            self.show_toolpath = !self.show_toolpath;
                            eprintln!("Toolpath visibility: {}", if self.show_toolpath { "ON" } else { "OFF" });
                        }
                        _ => {}
                    }
                }
            }

            WindowEvent::MouseInput { state, button, .. } if !egui_wants_pointer => {
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

            WindowEvent::CursorMoved { position, .. } if !egui_wants_pointer => {
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

            WindowEvent::MouseWheel { delta, .. } if !egui_wants_pointer => {
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

                // Determine if the simulation needs active processing
                let carving_active = self.sim.mode == SimMode::Carving
                    && self.sim.carving.as_ref().map_or(false, |s| s.state == CarvingState::Running);
                let any_movement_key = self.held_keys.w || self.held_keys.s
                    || self.held_keys.a || self.held_keys.d
                    || self.held_keys.q || self.held_keys.e
                    || self.held_keys.i || self.held_keys.k
                    || self.held_keys.j || self.held_keys.l;
                let sim_active = carving_active || (!self.sim.paused && any_movement_key) || !self.sim.paused;

                // Carving mode: advance the carving session and set targets
                if self.sim.mode == SimMode::Carving {
                    if let Some(session) = &mut self.sim.carving {
                        if session.state == CarvingState::Running {
                            let scaled_dt = clamped_frame_time * self.speed_multiplier;

                            match self.retract_phase {
                                1 => {
                                    // Phase 1: Retracting — wait for arm to clear cube rotation envelope.
                                    // Don't require precise positioning — just ensure the arm is outside
                                    // the sphere swept by cube corners during rotation.
                                    // Keep dynamic track updated so IK solves correctly during retract.
                                    let desired_track = (self.sim.cartesian_target.translation.vector.x - OPTIMAL_LOCAL_X)
                                        .clamp(self.sim.linear_track.min_position, self.sim.linear_track.max_position);
                                    self.sim.linear_track.target_position = desired_track;

                                    let track_off = Vector3::new(self.sim.linear_track.position, 0.0, 0.0);
                                    let arm_pos = self.sim.arm.tool_position() + track_off;
                                    let wc = self.sim.workpiece_center;
                                    let he = self.sim.workpiece_half_extent;
                                    let clearance = he * 1.5 + 0.05; // corner sweep + margin
                                    let dist_from_center = (arm_pos - wc).norm();
                                    if dist_from_center > clearance {
                                        self.retract_phase = 2;
                                        eprintln!("Arm clear ({:.0}mm from center, need {:.0}mm), rotating to A={:.1}°",
                                            dist_from_center * 1000.0, clearance * 1000.0,
                                            self.prev_a_axis.to_degrees());
                                    }
                                }
                                2 => {
                                    // Phase 2: Rotating table — PD runs inside physics_step at 1kHz.
                                    // Keep dynamic track following cartesian_target during rotation.
                                    let desired_track = (self.sim.cartesian_target.translation.vector.x - OPTIMAL_LOCAL_X)
                                        .clamp(self.sim.linear_track.min_position, self.sim.linear_track.max_position);
                                    self.sim.linear_track.target_position = desired_track;

                                    self.sim.rotary_table.target_angle = self.prev_a_axis;
                                    if self.sim.rotary_table.error() < 0.02
                                        && self.sim.rotary_table.velocity.abs() < 0.1
                                    {
                                        self.retract_phase = 0;
                                        // Reset frame_start to prevent accumulated swept path
                                        // from retract leaking into the first cut of the new face.
                                        self.sim.tool_state.frame_start_position = self.sim.tool_state.position;
                                        eprintln!("Table settled at A={:.1}°, resuming carving",
                                            self.prev_a_axis.to_degrees());
                                    }
                                }
                                _ => {
                                    // Phase 0: Normal carving — advance session and check for A changes
                                    self.sim.rotary_table.target_angle = session.a_axis_target;

                                    // Dynamic track: auto-position arm base so arm-local X
                                    // stays near OPTIMAL_LOCAL_X for best reachability.
                                    // This replaces hardcoded G-code U commands — the track
                                    // smoothly follows the carving target in real time.
                                    let desired_track = (self.sim.cartesian_target.translation.vector.x - OPTIMAL_LOCAL_X)
                                        .clamp(self.sim.linear_track.min_position, self.sim.linear_track.max_position);
                                    self.sim.linear_track.target_position = desired_track;

                                    // Gate on rotary table only — it rotates the WORKPIECE,
                                    // so cutting mid-spin would carve wrong positions.
                                    // Track settling is handled implicitly: if the track hasn't
                                    // reached its target, the arm can't reach the IK target either,
                                    // so tracking error exceeds the freeze threshold below.
                                    let table_settled = self.sim.rotary_table.error() < 0.02;

                                    if table_settled {
                                        // Adaptive feedrate (like real CNC servo loops):
                                        // Scale session advancement proportionally to tracking
                                        // error so the arm stays close to the target path.
                                        // CRITICAL: completely freeze when error > 3× threshold
                                        // to prevent the session racing ahead of the arm during
                                        // rapid repositioning (e.g., serif start at X=-50).
                                        let track_off = Vector3::new(self.sim.linear_track.position, 0.0, 0.0);
                                        let arm_pos = self.sim.arm.tool_position() + track_off;
                                        let target_pos = self.sim.cartesian_target.translation.vector;
                                        let tracking_err = (arm_pos - target_pos).norm();
                                        let catchup_threshold = self.sim.tracking_threshold * 3.0;
                                        let effective_dt = if tracking_err > catchup_threshold {
                                            // Arm too far from target — freeze session to let it catch up.
                                            // Without this, the session races ahead during rapids and
                                            // cutting starts before the arm reaches the start position.
                                            0.0
                                        } else if session.is_rapid {
                                            // Rapids: aggressive scaling — halve speed at 10mm lag
                                            let feedrate_scale = 1.0 / (1.0 + tracking_err * 100.0);
                                            scaled_dt * feedrate_scale
                                        } else {
                                            // Cuts: tighter scaling — halve speed at 20mm lag
                                            let feedrate_scale = 1.0 / (1.0 + tracking_err * 50.0);
                                            scaled_dt * feedrate_scale
                                        };

                                        let a_before = session.a_axis_target;
                                        if let Some(target) = session.step(effective_dt) {
                                            self.sim.cartesian_target = target;
                                        }
                                        let a_after = session.a_axis_target;

                                        // Detect A-axis change AFTER stepping the session
                                        if (a_after - a_before).abs() > 0.01 {
                                            self.retract_phase = 1;
                                            self.sim.tool_state.spindle_on = false;
                                            self.prev_a_axis = a_after; // the NEW target angle
                                            // Retract AWAY from cube in +X direction (not up — Z is
                                            // unreachable at safe height with tool-down orientation).
                                            // Cube corner sweep radius = he*sqrt(2) ≈ 0.216m.
                                            // Retract to X = center.x + he*2, Z = center.z (reachable).
                                            let he = self.sim.workpiece_half_extent;
                                            let retract_pos = nalgebra::Vector3::new(
                                                self.sim.workpiece_center.x + he * 2.0 + 0.05,
                                                self.sim.workpiece_center.y,
                                                self.sim.workpiece_center.z,
                                            );
                                            let orient = self.sim.cartesian_target.rotation;
                                            self.sim.cartesian_target = nalgebra::Isometry3::from_parts(
                                                nalgebra::Translation3::from(retract_pos),
                                                orient,
                                            );
                                            eprintln!("A-axis change to {:.1}°: retracting to X={:.3}",
                                                a_after.to_degrees(), retract_pos.x);
                                        }
                                    }
                                    if self.retract_phase == 0 {
                                        self.sim.tool_state.spindle_on = session.spindle_on && table_settled;
                                    }
                                }
                            }
                            self.sim.paused = false;
                        }
                    }
                } else {
                    // Manual mode: apply continuous key movement
                    self.apply_held_keys(clamped_frame_time);
                }

                if sim_active {
                    // IK solve: update joint targets from Cartesian target (once per frame)
                    self.sim.update_ik();

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

                    // FK → tool position: once per frame after all physics steps
                    // (not per physics step — saves N-1 FK computations)
                    self.sim.update_tool_state();

                    // Material removal: once per frame (not per physics step)
                    self.sim.material_removal_step();
                } else {
                    // Idle: don't accumulate physics time
                    self.accumulator = 0.0;
                }

                // Always submit dirty chunks for background meshing
                // (needed during startup even when paused, and after cuts complete)
                self.async_mesher.submit_dirty(&mut self.sim.workpiece);

                // Render (always — camera may have moved via mouse)
                self.render();

                // Request next frame, but throttle when idle
                let has_pending_meshes = !self.sim.workpiece.dirty_chunks.is_empty();
                if let Some(window) = &self.window {
                    if sim_active || has_pending_meshes {
                        window.request_redraw();
                    } else {
                        // Idle: cap at ~30fps to save CPU/GPU
                        std::thread::sleep(std::time::Duration::from_millis(30));
                        window.request_redraw();
                    }
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
