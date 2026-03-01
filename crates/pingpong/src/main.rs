//! SimuForge Ping Pong — Two 5-DOF robot arms playing table tennis.
//!
//! Phase 1: Static scene with arms, table, net, floor. Arms hold ready position.

#![allow(dead_code)]

use std::sync::Arc;
use std::time::Instant;

use glam::{Mat3, Mat4, Quat, Vec3, Vec4};
use nalgebra::Vector3;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId};

use simuforge_core::isometry_to_glam;
use simuforge_motors::gearbox::Gearbox;
use simuforge_motors::pid::PidController;
use simuforge_motors::servo::ServoState;
use simuforge_physics::aba::{articulated_body_algorithm, gravity_compensation_rnea};
use simuforge_physics::arm::RobotArm;
use simuforge_render::arm_visual::{generate_box, generate_cylinder, generate_sphere};
use simuforge_render::camera::{CameraUniform, OrbitCamera};
use simuforge_render::context::RenderContext;
use simuforge_render::pipelines::composite::{CompositePipeline, CompositeParams};
use simuforge_render::pipelines::line::{LinePipeline, LineVertex};
use simuforge_render::pipelines::pbr::{LightUniform, MaterialUniform, PbrPipeline};
use simuforge_render::pipelines::shadow::ShadowPipeline;
use simuforge_render::pipelines::ssao::{SsaoPipeline, SsaoParams};
use simuforge_render::pipelines::sss::{SssPipeline, SssParams};

use simuforge_audio::AudioEngine;
use simuforge_audio::synth::{TableBounceVoice, PaddleHitVoice, NetHitVoice, FloorBounceVoice};
use simuforge_audio::spatial::position_to_pan;

mod ai;
mod arm_config;
mod ball;
mod collision;
mod config;
mod game;
mod hud;
mod table;

use arm_config::{ARM_LINK_RADII, P1_LINK_COLORS, P1_PADDLE_COLOR, P2_LINK_COLORS, P2_PADDLE_COLOR};
use game::GameState;

/// Coordinate swap: DH Z-up physics -> Y-up renderer.
fn coord_swap_matrix() -> Mat4 {
    Mat4::from_cols(
        Vec4::new(1.0, 0.0, 0.0, 0.0),
        Vec4::new(0.0, 0.0, -1.0, 0.0),
        Vec4::new(0.0, 1.0, 0.0, 0.0),
        Vec4::new(0.0, 0.0, 0.0, 1.0),
    )
}

/// Model matrix for a unit cylinder connecting two render-space points.
fn link_model(from: Vec3, to: Vec3, radius: f32) -> Mat4 {
    let diff = to - from;
    let len = diff.length();
    if len < 0.001 {
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

/// Extract paddle state from a player arm for collision detection.
fn paddle_state_from_arm(player: &PlayerArm, player_id: u8) -> collision::PaddleState {
    let fk = player.arm.forward_kinematics();
    let fk_y = fk.rotation * Vector3::y();
    // Face center is offset from end-effector along FK Y (handle sticks out sideways)
    let face_center = fk.translation.vector + player.base_position
        + fk_y * arm_config::PADDLE_FACE_OFFSET;
    // Face normal points along FK X (forward, toward opponent)
    let normal = fk.rotation * Vector3::x();
    // Estimate paddle velocity from end-effector Jacobian * joint velocities
    let jac = player.arm.jacobian();
    let q_dot = nalgebra::DVector::from_iterator(
        player.arm.num_joints(),
        player.arm.joints.iter().map(|j| j.velocity),
    );
    let v_full = &jac * &q_dot;
    let vel = Vector3::new(v_full[3], v_full[4], v_full[5]);
    let ang_vel = Vector3::new(v_full[0], v_full[1], v_full[2]);

    collision::PaddleState {
        position: face_center,
        normal,
        velocity: vel,
        angular_velocity: ang_vel,
        player: player_id,
    }
}

/// Physics timestep: 1kHz.
const PHYSICS_DT: f64 = 1.0 / 1000.0;
/// Maximum accumulated time before clamping.
const MAX_FRAME_TIME: f64 = 0.05;

/// GPU mesh handle.
struct GpuMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    num_indices: u32,
}

/// Material bind group with its own buffer.
struct MaterialBind {
    buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

/// Per-player arm state.
struct PlayerArm {
    arm: RobotArm,
    motors: Vec<ServoState>,
    gearboxes: Vec<Gearbox>,
    pid_controllers: Vec<PidController>,
    joint_targets: Vec<f64>,
    base_position: Vector3<f64>,
    ai_controlled: bool,
    ai: ai::AiController,
    ik_solver: simuforge_control::ik::IkSolver,
}

impl PlayerArm {
    fn new(mirror: bool, base_pos: Vector3<f64>) -> Self {
        let arm = arm_config::create_pingpong_arm(mirror);
        let motors = arm_config::create_servo_motors();
        let gearboxes = arm_config::create_gearboxes();
        let pid_controllers = arm_config::create_pid_controllers();
        let targets = if mirror {
            arm_config::ready_position_p2()
        } else {
            arm_config::ready_position_p1()
        };

        let player_id = if mirror { 2 } else { 1 };
        let mut player = Self {
            arm,
            motors,
            gearboxes,
            pid_controllers,
            joint_targets: targets.clone(),
            base_position: base_pos,
            ai_controlled: true,
            ai: ai::AiController::new(player_id),
            ik_solver: simuforge_control::ik::IkSolver {
                damping: 0.05,
                max_iterations: 30,
                position_tolerance: 0.005,
                orientation_tolerance: 0.02,
                max_step: 0.3,
                nullspace_gain: 0.3,
            },
        };
        player.arm.set_joint_angles(&targets);
        // Zero velocities
        for j in player.arm.joints.iter_mut() {
            j.velocity = 0.0;
        }
        player
    }

    /// Update joint targets to track a world-space position using IK.
    /// The target is where the paddle **face center** should be.
    /// We offset by the paddle face offset along FK Y so IK solves for the wrist.
    fn track_position(&mut self, world_target: Vector3<f64>) {
        // Use current FK Y direction to estimate where the wrist should be
        // so the paddle face center ends up at world_target.
        let mut scratch = self.arm.clone();
        scratch.set_joint_angles(&self.joint_targets);
        let fk = scratch.forward_kinematics();
        let fk_y = fk.rotation * Vector3::y();
        let wrist_target = world_target - fk_y * arm_config::PADDLE_FACE_OFFSET;

        // Convert to arm-local frame
        let local_target = wrist_target - self.base_position;

        let (converged, _, _) = self.ik_solver.solve_position(&mut scratch, &local_target);
        if converged {
            let new_angles = scratch.joint_angles();
            // Rate-limit joint deltas — generous for fast ping pong response
            let max_delta = 0.25; // rad per frame at 60fps = 15 rad/s
            for i in 0..self.joint_targets.len() {
                let delta = (new_angles[i] - self.joint_targets[i]).clamp(-max_delta, max_delta);
                self.joint_targets[i] += delta;
            }
        }
    }

    fn physics_step(&mut self) {
        let n = self.arm.num_joints();
        let gravity = Vector3::new(0.0, 0.0, -9.81);

        let grav_comp = gravity_compensation_rnea(&self.arm, &gravity);

        let mut joint_torques = [0.0_f64; 8]; // MAX_JOINTS in aba is 8
        for i in 0..n {
            let pid_output = self.pid_controllers[i].update(
                self.joint_targets[i],
                self.arm.joints[i].angle,
                PHYSICS_DT,
            );

            let grav_ff =
                grav_comp[i] / (self.gearboxes[i].ratio * self.gearboxes[i].efficiency);
            let total_motor_cmd = pid_output + grav_ff;

            self.motors[i]
                .motor
                .update_velocity(self.arm.joints[i].velocity, self.gearboxes[i].ratio);
            let motor_torque = self.motors[i].apply_torque(total_motor_cmd, PHYSICS_DT);

            let joint_torque = self.gearboxes[i].motor_to_joint_torque(motor_torque);
            self.arm.joints[i].torque = joint_torque;
            joint_torques[i] = self.arm.joints[i].net_torque();
        }

        let accelerations = articulated_body_algorithm(&self.arm, &joint_torques[..n], &gravity);

        for i in 0..n {
            self.arm.joints[i].integrate(accelerations[i], PHYSICS_DT);
            self.arm.joints[i].clamp_to_limits();
        }
    }
}

/// Application state.
struct App {
    window: Option<Arc<Window>>,
    render_ctx: Option<RenderContext>,
    pbr_pipeline: Option<PbrPipeline>,
    shadow_pipeline: Option<ShadowPipeline>,
    ssao_pipeline: Option<SsaoPipeline>,
    sss_pipeline: Option<SssPipeline>,
    composite_pipeline: Option<CompositePipeline>,
    // egui
    egui_ctx: egui::Context,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
    // Camera
    camera: OrbitCamera,
    // Players
    player1: PlayerArm,
    player2: PlayerArm,
    // Game
    game: GameState,
    ball: ball::Ball,
    // Timing
    last_frame: Instant,
    accumulator: f64,
    frame_count: u64,
    fps_timer: Instant,
    fps: f64,
    sim_time: f64,
    paused: bool,
    show_audio_panel: bool,
    // Input
    mouse_pressed: bool,
    middle_pressed: bool,
    last_mouse_pos: Option<(f64, f64)>,
    // GPU meshes
    cylinder_mesh: Option<GpuMesh>,
    sphere_mesh: Option<GpuMesh>,
    box_mesh: Option<GpuMesh>,
    // Table GPU
    table_top_mesh: Option<GpuMesh>,
    table_top_material: Option<MaterialBind>,
    leg_mesh: Option<GpuMesh>,
    leg_materials: Vec<MaterialBind>,
    net_mesh: Option<GpuMesh>,
    net_material: Option<MaterialBind>,
    net_post_mesh: Option<GpuMesh>,
    net_post_materials: Vec<MaterialBind>,
    floor_mesh: Option<GpuMesh>,
    floor_material: Option<MaterialBind>,
    // Arm GPU — Player 1
    p1_link_materials: Vec<MaterialBind>,
    p1_joint_materials: Vec<MaterialBind>,
    p1_paddle_material: Option<MaterialBind>,
    p1_base_mesh: Option<GpuMesh>,
    p1_base_material: Option<MaterialBind>,
    // Arm GPU — Player 2
    p2_link_materials: Vec<MaterialBind>,
    p2_joint_materials: Vec<MaterialBind>,
    p2_paddle_material: Option<MaterialBind>,
    p2_base_mesh: Option<GpuMesh>,
    p2_base_material: Option<MaterialBind>,
    // Paddle mesh
    paddle_mesh: Option<GpuMesh>,
    // Ball GPU
    ball_material: Option<MaterialBind>,
    // Ball trail
    trail_pipeline: Option<LinePipeline>,
    trail_points: Vec<Vec3>,
    // Table markings
    markings_pipeline: Option<LinePipeline>,
    markings_uploaded: bool,
    // Audio
    audio_engine: AudioEngine,
    // Post-processing
    post_sampler: Option<wgpu::Sampler>,
    depth_sampler: Option<wgpu::Sampler>,
    ssao_bind_group: Option<wgpu::BindGroup>,
    sss_bind_group: Option<wgpu::BindGroup>,
    composite_bind_group: Option<wgpu::BindGroup>,
}

impl App {
    fn new() -> Self {
        let p1_base = Vector3::new(
            -(arm_config::ARM_X_OFFSET),
            0.0,
            arm_config::TABLE_HEIGHT,
        );
        let p2_base = Vector3::new(
            arm_config::ARM_X_OFFSET,
            0.0,
            arm_config::TABLE_HEIGHT,
        );

        let mut camera = OrbitCamera::new();
        // Center view on table
        camera.target = Vec3::new(0.0, table::TABLE_HEIGHT / 2.0, 0.0);
        camera.distance = 4.0;
        camera.yaw = 0.8;
        camera.pitch = 0.45;

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
            camera,
            player1: PlayerArm::new(false, p1_base),
            player2: PlayerArm::new(true, p2_base),
            game: GameState::new(),
            ball: ball::Ball::new(),
            last_frame: Instant::now(),
            accumulator: 0.0,
            frame_count: 0,
            fps_timer: Instant::now(),
            fps: 0.0,
            sim_time: 0.0,
            paused: false,
            show_audio_panel: true,
            mouse_pressed: false,
            middle_pressed: false,
            last_mouse_pos: None,
            cylinder_mesh: None,
            sphere_mesh: None,
            box_mesh: None,
            table_top_mesh: None,
            table_top_material: None,
            leg_mesh: None,
            leg_materials: Vec::new(),
            net_mesh: None,
            net_material: None,
            net_post_mesh: None,
            net_post_materials: Vec::new(),
            floor_mesh: None,
            floor_material: None,
            p1_link_materials: Vec::new(),
            p1_joint_materials: Vec::new(),
            p1_paddle_material: None,
            p1_base_mesh: None,
            p1_base_material: None,
            p2_link_materials: Vec::new(),
            p2_joint_materials: Vec::new(),
            p2_paddle_material: None,
            p2_base_mesh: None,
            p2_base_material: None,
            paddle_mesh: None,
            ball_material: None,
            trail_pipeline: None,
            trail_points: Vec::new(),
            markings_pipeline: None,
            markings_uploaded: false,
            audio_engine: AudioEngine::new(),
            post_sampler: None,
            depth_sampler: None,
            ssao_bind_group: None,
            sss_bind_group: None,
            composite_bind_group: None,
        }
    }

    /// Compute arm link render points for a given player.
    fn arm_render_points(&self, player: &PlayerArm) -> Vec<Vec3> {
        let swap = coord_swap_matrix();
        let frames = player.arm.link_frames();
        let base = &player.base_position;
        let n = frames.len();
        let mut pts = Vec::with_capacity(n);
        for f in &frames {
            let m = isometry_to_glam(f);
            let dh_pos = Vec3::new(
                m.col(3).x + base.x as f32,
                m.col(3).y + base.y as f32,
                m.col(3).z + base.z as f32,
            );
            pts.push(swap.transform_point3(dh_pos));
        }
        pts
    }

    /// Compute paddle model matrix for a player.
    /// Uses the full FK rotation so wrist roll controls face direction.
    fn paddle_model(&self, player: &PlayerArm) -> Mat4 {
        let swap = coord_swap_matrix();
        let fk = player.arm.forward_kinematics();
        let fk_mat = isometry_to_glam(&fk);
        let base = &player.base_position;
        let pos_dh = Vec3::new(
            fk_mat.col(3).x + base.x as f32,
            fk_mat.col(3).y + base.y as f32,
            fk_mat.col(3).z + base.z as f32,
        );
        let render_pos = swap.transform_point3(pos_dh);

        // Extract full FK rotation axes (DH space)
        let fk_x = Vec3::new(fk_mat.col(0).x, fk_mat.col(0).y, fk_mat.col(0).z);
        let fk_y = Vec3::new(fk_mat.col(1).x, fk_mat.col(1).y, fk_mat.col(1).z);
        let fk_z = Vec3::new(fk_mat.col(2).x, fk_mat.col(2).y, fk_mat.col(2).z);

        // Transform to render space
        let render_fk_x = swap.transform_vector3(fk_x);
        let render_fk_y = swap.transform_vector3(fk_y);
        let render_fk_z = swap.transform_vector3(fk_z);

        // Paddle mesh: +Y = handle direction, +Z = face normal, +X = face width.
        // At ready position (render space): FK X=forward, FK Y=right, FK Z=down.
        // Desired orientation:
        //   Handle (paddle +Y) → FK Y (sticks out sideways from arm)
        //   Face normal (paddle +Z) → FK X (faces forward toward opponent)
        //   Face width (paddle +X) → -FK Z (upward, perpendicular to both)
        // Right-handed check: (-FK Z) × (FK Y) = FK X ✓
        let rot_mat = Mat3::from_cols(-render_fk_z, render_fk_y, render_fk_x);
        let rot = Quat::from_mat3(&rot_mat);

        Mat4::from_rotation_translation(rot, render_pos)
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

        // Camera
        let cam_uniform = CameraUniform::from_camera(&self.camera, ctx.aspect());
        pbr.update_camera(&ctx.queue, &cam_uniform);

        // Light — overhead gymnasium lighting
        let light_dir = Vec3::new(-0.3, -0.8, -0.3).normalize();
        let light = LightUniform {
            direction: [light_dir.x, light_dir.y, light_dir.z, 0.0],
            color: [1.0, 0.98, 0.95, 3.0],
            ambient: [0.55, 0.55, 0.60, 0.35],
            eye_pos: cam_uniform.eye_pos,
        };
        pbr.update_light(&ctx.queue, &light);

        // --- Compute model matrices ---

        // Table top: centered at origin, height = TABLE_HEIGHT (in render Y-up space)
        let table_model = Mat4::from_translation(Vec3::new(
            0.0,
            table::TABLE_HEIGHT,
            0.0,
        ));

        // Table legs
        let leg_y = table::LEG_HEIGHT / 2.0;
        let leg_positions = table::leg_positions();
        let leg_models: Vec<Mat4> = leg_positions
            .iter()
            .map(|&(x, z)| Mat4::from_translation(Vec3::new(x, leg_y, z)))
            .collect();

        // Net: sits on top of table at center
        let net_model = Mat4::from_translation(Vec3::new(
            0.0,
            table::TABLE_HEIGHT + table::TABLE_THICKNESS / 2.0 + table::NET_HEIGHT / 2.0,
            0.0,
        ));

        // Net posts
        let net_post_positions = table::net_post_positions();
        let net_post_models: Vec<Mat4> = net_post_positions
            .iter()
            .map(|&(x, z)| {
                Mat4::from_translation(Vec3::new(
                    x,
                    table::TABLE_HEIGHT + table::TABLE_THICKNESS / 2.0 + table::NET_POST_HEIGHT / 2.0,
                    z,
                ))
            })
            .collect();

        // Floor
        let floor_model = Mat4::from_translation(Vec3::new(0.0, -table::FLOOR_THICKNESS / 2.0, 0.0));

        // Player 1 arm
        let p1_pts = self.arm_render_points(&self.player1);
        let n_links = self.player1.arm.num_joints();
        let mut p1_link_models = Vec::with_capacity(n_links);
        for i in 0..n_links {
            p1_link_models.push(link_model(p1_pts[i], p1_pts[i + 1], ARM_LINK_RADII[i]));
        }
        let p1_joint_sphere_radii: [f32; 6] = [0.06, 0.05, 0.045, 0.04, 0.035, 0.03];
        let mut p1_joint_models = Vec::with_capacity(n_links + 1);
        for i in 0..=n_links {
            let r = if i < p1_joint_sphere_radii.len() {
                p1_joint_sphere_radii[i]
            } else {
                0.03
            };
            p1_joint_models.push(Mat4::from_scale_rotation_translation(
                Vec3::splat(r),
                Quat::IDENTITY,
                p1_pts[i],
            ));
        }
        let p1_paddle_model = self.paddle_model(&self.player1);

        // Player 1 base pedestal
        let p1_base_render = swap.transform_point3(Vec3::new(
            self.player1.base_position.x as f32,
            self.player1.base_position.y as f32,
            self.player1.base_position.z as f32,
        ));
        let p1_base_half_h = p1_base_render.y / 2.0;
        let p1_base_model = Mat4::from_translation(Vec3::new(
            p1_base_render.x,
            p1_base_half_h,
            p1_base_render.z,
        ));

        // Player 2 arm
        let p2_pts = self.arm_render_points(&self.player2);
        let mut p2_link_models = Vec::with_capacity(n_links);
        for i in 0..n_links {
            p2_link_models.push(link_model(p2_pts[i], p2_pts[i + 1], ARM_LINK_RADII[i]));
        }
        let mut p2_joint_models = Vec::with_capacity(n_links + 1);
        for i in 0..=n_links {
            let r = if i < p1_joint_sphere_radii.len() {
                p1_joint_sphere_radii[i]
            } else {
                0.03
            };
            p2_joint_models.push(Mat4::from_scale_rotation_translation(
                Vec3::splat(r),
                Quat::IDENTITY,
                p2_pts[i],
            ));
        }
        let p2_paddle_model = self.paddle_model(&self.player2);

        // Player 2 base pedestal
        let p2_base_render = swap.transform_point3(Vec3::new(
            self.player2.base_position.x as f32,
            self.player2.base_position.y as f32,
            self.player2.base_position.z as f32,
        ));
        let p2_base_half_h = p2_base_render.y / 2.0;
        let p2_base_model = Mat4::from_translation(Vec3::new(
            p2_base_render.x,
            p2_base_half_h,
            p2_base_render.z,
        ));

        // Ball model
        let ball_render = swap.transform_point3(Vec3::new(
            self.ball.position.x as f32,
            self.ball.position.y as f32,
            self.ball.position.z as f32,
        ));
        let ball_model = Mat4::from_scale_rotation_translation(
            Vec3::splat(ball::BALL_RADIUS as f32),
            Quat::IDENTITY,
            ball_render,
        );

        // --- Upload material uniforms ---

        // Table top
        if let Some(m) = &self.table_top_material {
            let mat = MaterialUniform::metal(table::TABLE_COLOR)
                .with_model(table_model.to_cols_array_2d());
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }
        // Legs
        for (i, lm) in self.leg_materials.iter().enumerate() {
            let mat = MaterialUniform::metal(table::LEG_COLOR)
                .with_model(leg_models[i].to_cols_array_2d());
            ctx.queue.write_buffer(&lm.buffer, 0, bytemuck::bytes_of(&mat));
        }
        // Net
        if let Some(m) = &self.net_material {
            let mat = MaterialUniform::metal(table::NET_COLOR)
                .with_model(net_model.to_cols_array_2d());
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }
        // Net posts
        for (i, pm) in self.net_post_materials.iter().enumerate() {
            let mat = MaterialUniform::metal(table::NET_POST_COLOR)
                .with_model(net_post_models[i].to_cols_array_2d());
            ctx.queue.write_buffer(&pm.buffer, 0, bytemuck::bytes_of(&mat));
        }
        // Floor
        if let Some(m) = &self.floor_material {
            let mat = MaterialUniform::ground()
                .with_model(floor_model.to_cols_array_2d());
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Player 1 arm
        let joint_color = [0.12, 0.12, 0.14, 1.0];
        for (i, lm) in self.p1_link_materials.iter().enumerate() {
            let mat = MaterialUniform::metal(P1_LINK_COLORS[i])
                .with_model(p1_link_models[i].to_cols_array_2d());
            ctx.queue.write_buffer(&lm.buffer, 0, bytemuck::bytes_of(&mat));
        }
        for (i, jm) in self.p1_joint_materials.iter().enumerate() {
            let mat = MaterialUniform::metal(joint_color)
                .with_model(p1_joint_models[i].to_cols_array_2d());
            ctx.queue.write_buffer(&jm.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(pm) = &self.p1_paddle_material {
            let mat = MaterialUniform::metal(P1_PADDLE_COLOR)
                .with_model(p1_paddle_model.to_cols_array_2d());
            ctx.queue.write_buffer(&pm.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(bm) = &self.p1_base_material {
            let mat = MaterialUniform::metal([0.15, 0.15, 0.20, 1.0])
                .with_model(p1_base_model.to_cols_array_2d());
            ctx.queue.write_buffer(&bm.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Player 2 arm
        for (i, lm) in self.p2_link_materials.iter().enumerate() {
            let mat = MaterialUniform::metal(P2_LINK_COLORS[i])
                .with_model(p2_link_models[i].to_cols_array_2d());
            ctx.queue.write_buffer(&lm.buffer, 0, bytemuck::bytes_of(&mat));
        }
        for (i, jm) in self.p2_joint_materials.iter().enumerate() {
            let mat = MaterialUniform::metal(joint_color)
                .with_model(p2_joint_models[i].to_cols_array_2d());
            ctx.queue.write_buffer(&jm.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(pm) = &self.p2_paddle_material {
            let mat = MaterialUniform::metal(P2_PADDLE_COLOR)
                .with_model(p2_paddle_model.to_cols_array_2d());
            ctx.queue.write_buffer(&pm.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(bm) = &self.p2_base_material {
            let mat = MaterialUniform::metal([0.20, 0.15, 0.15, 1.0])
                .with_model(p2_base_model.to_cols_array_2d());
            ctx.queue.write_buffer(&bm.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Ball
        if let Some(bm) = &self.ball_material {
            let ball_color = [0.95, 0.60, 0.10, 1.0]; // orange ball
            let mat = MaterialUniform::metal(ball_color)
                .with_model(ball_model.to_cols_array_2d());
            ctx.queue.write_buffer(&bm.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // --- Shadow setup ---
        let light_vp = ShadowPipeline::compute_light_matrix(light_dir, 4.0);
        pbr.update_shadow_light_vp(&ctx.queue, &light_vp);

        // Collect all shadow matrices
        let mut shadow_matrices: Vec<Mat4> = Vec::with_capacity(32);
        // Table: top + 4 legs + net + 2 posts = 8
        shadow_matrices.push(light_vp * table_model);
        for lm in &leg_models {
            shadow_matrices.push(light_vp * *lm);
        }
        shadow_matrices.push(light_vp * net_model);
        for pm in &net_post_models {
            shadow_matrices.push(light_vp * *pm);
        }
        // Floor
        shadow_matrices.push(light_vp * floor_model);
        // P1: 5 links + 6 joints + paddle + base = 13
        for lm in &p1_link_models {
            shadow_matrices.push(light_vp * *lm);
        }
        for jm in &p1_joint_models {
            shadow_matrices.push(light_vp * *jm);
        }
        shadow_matrices.push(light_vp * p1_paddle_model);
        shadow_matrices.push(light_vp * p1_base_model);
        // P2: same
        // Note: might exceed MAX_SHADOW_OBJECTS (32). If so, skip P2 shadows.
        if shadow_matrices.len() + n_links + n_links + 1 + 2 + 1 <= 32 {
            for lm in &p2_link_models {
                shadow_matrices.push(light_vp * *lm);
            }
            for jm in &p2_joint_models {
                shadow_matrices.push(light_vp * *jm);
            }
            shadow_matrices.push(light_vp * p2_paddle_model);
            shadow_matrices.push(light_vp * p2_base_model);
        }

        if let Some(shadow) = &self.shadow_pipeline {
            shadow.upload_matrices(&ctx.queue, &shadow_matrices);
        }

        // SSAO
        if let Some(ssao) = &self.ssao_pipeline {
            ssao.update_params(
                &ctx.queue,
                &SsaoParams {
                    proj: cam_uniform.proj,
                    radius: 0.5,
                    bias: 0.025,
                    intensity: 1.5,
                    _pad: 0.0,
                },
            );
        }

        // --- Begin rendering ---
        let output = match ctx.surface.get_current_texture() {
            Ok(t) => t,
            Err(_) => return,
        };
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        // Pass 1: Sky clear
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Sky Clear"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.08,
                            g: 0.08,
                            b: 0.12,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                ..Default::default()
            });
        }

        // Pass 2: Shadow depth
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

            let mut si = 0usize;

            // Table top
            if let Some(mesh) = &self.table_top_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
            }
            si += 1;

            // Legs
            if let Some(mesh) = &self.leg_mesh {
                for _ in 0..4 {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                    si += 1;
                }
            } else {
                si += 4;
            }

            // Net
            if let Some(mesh) = &self.net_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
            }
            si += 1;

            // Net posts
            if let Some(mesh) = &self.net_post_mesh {
                for _ in 0..2 {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                    si += 1;
                }
            } else {
                si += 2;
            }

            // Floor
            if let Some(mesh) = &self.floor_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
            }
            si += 1;

            // P1 links
            if let Some(cyl) = &self.cylinder_mesh {
                for _ in 0..n_links {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                    pass.set_index_buffer(cyl.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
                    si += 1;
                }
            } else {
                si += n_links;
            }

            // P1 joints
            if let Some(sph) = &self.sphere_mesh {
                for _ in 0..=n_links {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                    pass.set_index_buffer(sph.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..sph.num_indices, 0, 0..1);
                    si += 1;
                }
            } else {
                si += n_links + 1;
            }

            // P1 paddle
            if let Some(pad) = &self.paddle_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, pad.vertex_buffer.slice(..));
                pass.set_index_buffer(pad.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..pad.num_indices, 0, 0..1);
            }
            si += 1;

            // P1 base
            if let Some(mesh) = &self.p1_base_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
            }
            si += 1;

            // P2 shadows (if room)
            if si + n_links + n_links + 1 + 2 <= 32 {
                if let Some(cyl) = &self.cylinder_mesh {
                    for _ in 0..n_links {
                        pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                        pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                        pass.set_index_buffer(cyl.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
                        si += 1;
                    }
                } else {
                    si += n_links;
                }
                if let Some(sph) = &self.sphere_mesh {
                    for _ in 0..=n_links {
                        pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                        pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                        pass.set_index_buffer(sph.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..sph.num_indices, 0, 0..1);
                        si += 1;
                    }
                } else {
                    si += n_links + 1;
                }
                // P2 paddle
                if let Some(pad) = &self.paddle_mesh {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, pad.vertex_buffer.slice(..));
                    pass.set_index_buffer(pad.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..pad.num_indices, 0, 0..1);
                }
                si += 1;
                // P2 base
                if let Some(mesh) = &self.p2_base_mesh {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }
        }

        // Pass 3: PBR -> HDR
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
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

            pass.set_pipeline(&pbr.pipeline);
            pass.set_bind_group(1, &pbr.shadow_bind_group, &[]);

            // Helper macro for drawing
            macro_rules! draw_mesh {
                ($mesh:expr, $mat:expr) => {
                    if let (Some(mesh), Some(mat)) = ($mesh, $mat) {
                        pass.set_bind_group(0, &mat.bind_group, &[]);
                        pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                        pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                    }
                };
            }

            // Floor
            draw_mesh!(&self.floor_mesh, &self.floor_material);

            // Table top
            draw_mesh!(&self.table_top_mesh, &self.table_top_material);

            // Legs
            for lm in &self.leg_materials {
                if let Some(mesh) = &self.leg_mesh {
                    pass.set_bind_group(0, &lm.bind_group, &[]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }

            // Net
            draw_mesh!(&self.net_mesh, &self.net_material);

            // Net posts
            for pm in &self.net_post_materials {
                if let Some(mesh) = &self.net_post_mesh {
                    pass.set_bind_group(0, &pm.bind_group, &[]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }

            // Player 1 arm links
            if let Some(cyl) = &self.cylinder_mesh {
                for lm in &self.p1_link_materials {
                    pass.set_bind_group(0, &lm.bind_group, &[]);
                    pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                    pass.set_index_buffer(cyl.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
                }
            }
            // Player 1 joint spheres
            if let Some(sph) = &self.sphere_mesh {
                for jm in &self.p1_joint_materials {
                    pass.set_bind_group(0, &jm.bind_group, &[]);
                    pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                    pass.set_index_buffer(sph.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..sph.num_indices, 0, 0..1);
                }
            }
            // Player 1 paddle
            if let Some(pad) = &self.paddle_mesh {
                if let Some(pm) = &self.p1_paddle_material {
                    pass.set_bind_group(0, &pm.bind_group, &[]);
                    pass.set_vertex_buffer(0, pad.vertex_buffer.slice(..));
                    pass.set_index_buffer(pad.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..pad.num_indices, 0, 0..1);
                }
            }
            // Player 1 base
            draw_mesh!(&self.p1_base_mesh, &self.p1_base_material);

            // Player 2 arm links
            if let Some(cyl) = &self.cylinder_mesh {
                for lm in &self.p2_link_materials {
                    pass.set_bind_group(0, &lm.bind_group, &[]);
                    pass.set_vertex_buffer(0, cyl.vertex_buffer.slice(..));
                    pass.set_index_buffer(cyl.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..cyl.num_indices, 0, 0..1);
                }
            }
            // Player 2 joint spheres
            if let Some(sph) = &self.sphere_mesh {
                for jm in &self.p2_joint_materials {
                    pass.set_bind_group(0, &jm.bind_group, &[]);
                    pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                    pass.set_index_buffer(sph.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..sph.num_indices, 0, 0..1);
                }
            }
            // Player 2 paddle
            if let Some(pad) = &self.paddle_mesh {
                if let Some(pm) = &self.p2_paddle_material {
                    pass.set_bind_group(0, &pm.bind_group, &[]);
                    pass.set_vertex_buffer(0, pad.vertex_buffer.slice(..));
                    pass.set_index_buffer(pad.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..pad.num_indices, 0, 0..1);
                }
            }
            // Player 2 base
            draw_mesh!(&self.p2_base_mesh, &self.p2_base_material);

            // Ball
            if self.ball.active {
                if let Some(sph) = &self.sphere_mesh {
                    if let Some(bm) = &self.ball_material {
                        pass.set_bind_group(0, &bm.bind_group, &[]);
                        pass.set_vertex_buffer(0, sph.vertex_buffer.slice(..));
                        pass.set_index_buffer(sph.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..sph.num_indices, 0, 0..1);
                    }
                }
            }
        }

        // Pass 4: SSAO
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

        // Pass 5: SSS (subsurface — minimal effect on metal, still needed for pipeline)
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

        // Pass 6: Composite (tone mapping)
        if let (Some(composite), Some(comp_bg)) =
            (&self.composite_pipeline, &self.composite_bind_group)
        {
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

        // Pass 6b: Table markings (white lines on table surface)
        if let Some(markings) = &mut self.markings_pipeline {
            if !self.markings_uploaded {
                let swap = coord_swap_matrix();
                let marking_lines = table::table_marking_lines();
                let mut verts = Vec::new();
                let white = [1.0, 1.0, 1.0, 0.9];
                for (start, end) in &marking_lines {
                    let s = swap.transform_point3(Vec3::from_array(*start));
                    let e = swap.transform_point3(Vec3::from_array(*end));
                    verts.push(LineVertex { position: s.into(), color: white });
                    verts.push(LineVertex { position: e.into(), color: white });
                }
                markings.upload(&ctx.queue, &verts);
                self.markings_uploaded = true;
            }
            if markings.num_vertices > 0 {
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Table Markings Pass"),
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
                pass.set_pipeline(&markings.pipeline);
                pass.set_bind_group(0, &markings.bind_group, &[]);
                pass.set_vertex_buffer(0, markings.vertex_buffer.slice(..));
                pass.draw(0..markings.num_vertices, 0..1);
            }
        }

        // Pass 6c: Ball trail lines (after composite, before egui)
        if let Some(trail) = &mut self.trail_pipeline {
            if self.trail_points.len() >= 2 {
                let n = self.trail_points.len();
                let mut verts = Vec::with_capacity(n * 2);
                for i in 0..n - 1 {
                    let t = i as f32 / n as f32;
                    let alpha = t * 0.8 + 0.2; // fade older points
                    let color = [1.0, 0.6 + t * 0.4, 0.1, alpha];
                    verts.push(LineVertex {
                        position: self.trail_points[i].into(),
                        color,
                    });
                    verts.push(LineVertex {
                        position: self.trail_points[i + 1].into(),
                        color,
                    });
                }
                trail.upload(&ctx.queue, &verts);
            } else {
                trail.num_vertices = 0;
            }

            if trail.num_vertices > 0 {
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Trail Pass"),
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
                pass.set_pipeline(&trail.pipeline);
                pass.set_bind_group(0, &trail.bind_group, &[]);
                pass.set_vertex_buffer(0, trail.vertex_buffer.slice(..));
                pass.draw(0..trail.num_vertices, 0..1);
            }
        }

        // Pass 7: egui overlay
        let egui_input = self
            .egui_state
            .as_mut()
            .unwrap()
            .take_egui_input(self.window.as_ref().unwrap());
        self.egui_ctx.begin_pass(egui_input);

        hud::draw_hud(
            &self.egui_ctx,
            &self.game,
            self.fps,
            self.player1.ai_controlled,
            self.player2.ai_controlled,
            &self.audio_engine,
            self.show_audio_panel,
        );

        let egui_output = self.egui_ctx.end_pass();
        let egui_prims = self
            .egui_ctx
            .tessellate(egui_output.shapes, egui_output.pixels_per_point);
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [ctx.config.width, ctx.config.height],
            pixels_per_point: egui_output.pixels_per_point,
        };
        let egui_renderer = self.egui_renderer.as_mut().unwrap();
        for (id, delta) in &egui_output.textures_delta.set {
            egui_renderer.update_texture(&ctx.device, &ctx.queue, *id, delta);
        }
        let egui_cmd_bufs = egui_renderer.update_buffers(
            &ctx.device,
            &ctx.queue,
            &mut encoder,
            &egui_prims,
            &screen,
        );
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
            egui_renderer.render(&mut pass, &egui_prims, &screen);
        }
        for id in &egui_output.textures_delta.free {
            egui_renderer.free_texture(id);
        }
        self.egui_state.as_mut().unwrap().handle_platform_output(
            self.window.as_ref().unwrap(),
            egui_output.platform_output,
        );

        let mut cmd_bufs: Vec<wgpu::CommandBuffer> = egui_cmd_bufs;
        cmd_bufs.push(encoder.finish());
        ctx.queue.submit(cmd_bufs);
        output.present();
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
                        .with_title("SimuForge — Ping Pong")
                        .with_inner_size(winit::dpi::LogicalSize::new(1440, 810)),
                )
                .expect("Failed to create window"),
        );

        let ctx = pollster::block_on(RenderContext::new(window.clone()));
        let shadow = ShadowPipeline::new(&ctx);
        let pbr = PbrPipeline::new(&ctx, &shadow);
        let ssao = SsaoPipeline::new(&ctx);
        let sss = SssPipeline::new(&ctx);
        let composite = CompositePipeline::new(&ctx);

        // egui init
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
            None,
            1,
            false,
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

        let ssao_bg = ssao.create_bind_group(&ctx.device, &ctx.depth_texture, &depth_sampler);
        let sss_bg = sss.create_bind_group(
            &ctx.device,
            &ctx.hdr_texture,
            &ctx.depth_texture,
            &post_sampler,
        );
        let composite_bg = composite.create_bind_group(
            &ctx.device,
            &sss.output_view,
            &ssao.output_view,
            &post_sampler,
        );

        let w = ctx.config.width as f32;
        let h = ctx.config.height as f32;
        ssao.update_params(
            &ctx.queue,
            &SsaoParams {
                proj: glam::Mat4::IDENTITY.to_cols_array_2d(),
                radius: 0.5,
                bias: 0.025,
                intensity: 1.5,
                _pad: 0.0,
            },
        );
        sss.update_params(&ctx.queue, &SssParams::marble(w, h));
        composite.update_params(&ctx.queue, &CompositeParams::default());

        // --- Create GPU meshes ---
        use wgpu::util::DeviceExt;

        // Shared unit cylinder
        let (cyl_v, cyl_i) = generate_cylinder(1.0, 1.0, 16);
        let cyl_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Cylinder VB"),
            contents: bytemuck::cast_slice(&cyl_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let cyl_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Cylinder IB"),
            contents: bytemuck::cast_slice(&cyl_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.cylinder_mesh = Some(GpuMesh {
            vertex_buffer: cyl_vb,
            index_buffer: cyl_ib,
            num_indices: cyl_i.len() as u32,
        });

        // Shared unit sphere
        let (sph_v, sph_i) = generate_sphere(1.0, 12, 16);
        let sph_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Sphere VB"),
            contents: bytemuck::cast_slice(&sph_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let sph_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Sphere IB"),
            contents: bytemuck::cast_slice(&sph_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.sphere_mesh = Some(GpuMesh {
            vertex_buffer: sph_vb,
            index_buffer: sph_ib,
            num_indices: sph_i.len() as u32,
        });

        // Paddle mesh: circular head + handle (real paddle shape)
        let (pad_v, pad_i) = arm_config::generate_paddle_mesh();
        let pad_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Paddle VB"),
            contents: bytemuck::cast_slice(&pad_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let pad_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Paddle IB"),
            contents: bytemuck::cast_slice(&pad_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.paddle_mesh = Some(GpuMesh {
            vertex_buffer: pad_vb,
            index_buffer: pad_ib,
            num_indices: pad_i.len() as u32,
        });

        // Table top
        let (tt_v, tt_i) = table::generate_table_top();
        let tt_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Table Top VB"),
            contents: bytemuck::cast_slice(&tt_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let tt_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Table Top IB"),
            contents: bytemuck::cast_slice(&tt_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.table_top_mesh = Some(GpuMesh {
            vertex_buffer: tt_vb,
            index_buffer: tt_ib,
            num_indices: tt_i.len() as u32,
        });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.table_top_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Table legs
        let (leg_v, leg_i) = table::generate_leg();
        let leg_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Leg VB"),
            contents: bytemuck::cast_slice(&leg_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let leg_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Leg IB"),
            contents: bytemuck::cast_slice(&leg_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.leg_mesh = Some(GpuMesh {
            vertex_buffer: leg_vb,
            index_buffer: leg_ib,
            num_indices: leg_i.len() as u32,
        });
        for _ in 0..4 {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.leg_materials.push(MaterialBind { buffer: buf, bind_group: bg });
        }

        // Net
        let (net_v, net_i) = table::generate_net();
        let net_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Net VB"),
            contents: bytemuck::cast_slice(&net_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let net_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Net IB"),
            contents: bytemuck::cast_slice(&net_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.net_mesh = Some(GpuMesh {
            vertex_buffer: net_vb,
            index_buffer: net_ib,
            num_indices: net_i.len() as u32,
        });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.net_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Net posts
        let (np_v, np_i) = table::generate_net_post();
        let np_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Net Post VB"),
            contents: bytemuck::cast_slice(&np_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let np_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Net Post IB"),
            contents: bytemuck::cast_slice(&np_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.net_post_mesh = Some(GpuMesh {
            vertex_buffer: np_vb,
            index_buffer: np_ib,
            num_indices: np_i.len() as u32,
        });
        for _ in 0..2 {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.net_post_materials.push(MaterialBind { buffer: buf, bind_group: bg });
        }

        // Floor
        let (fl_v, fl_i) = table::generate_floor();
        let fl_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Floor VB"),
            contents: bytemuck::cast_slice(&fl_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let fl_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Floor IB"),
            contents: bytemuck::cast_slice(&fl_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.floor_mesh = Some(GpuMesh {
            vertex_buffer: fl_vb,
            index_buffer: fl_ib,
            num_indices: fl_i.len() as u32,
        });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.floor_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Player 1 arm materials (5 links + 6 joints + paddle + base)
        let n = self.player1.arm.num_joints();
        for _ in 0..n {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.p1_link_materials.push(MaterialBind { buffer: buf, bind_group: bg });
        }
        for _ in 0..=n {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.p1_joint_materials.push(MaterialBind { buffer: buf, bind_group: bg });
        }
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.p1_paddle_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Player 1 base pedestal
        let base_half_h = (table::TABLE_HEIGHT / 2.0).max(0.01);
        let (base_v, base_i) = generate_box(0.08, base_half_h, 0.08);
        let base_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("P1 Base VB"),
            contents: bytemuck::cast_slice(&base_v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let base_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("P1 Base IB"),
            contents: bytemuck::cast_slice(&base_i),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.p1_base_mesh = Some(GpuMesh {
            vertex_buffer: base_vb,
            index_buffer: base_ib,
            num_indices: base_i.len() as u32,
        });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.p1_base_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Player 2 arm materials
        for _ in 0..n {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.p2_link_materials.push(MaterialBind { buffer: buf, bind_group: bg });
        }
        for _ in 0..=n {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.p2_joint_materials.push(MaterialBind { buffer: buf, bind_group: bg });
        }
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.p2_paddle_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Player 2 base pedestal (reuse same dimensions)
        let (base_v2, base_i2) = generate_box(0.08, base_half_h, 0.08);
        let base_vb2 = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("P2 Base VB"),
            contents: bytemuck::cast_slice(&base_v2),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let base_ib2 = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("P2 Base IB"),
            contents: bytemuck::cast_slice(&base_i2),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.p2_base_mesh = Some(GpuMesh {
            vertex_buffer: base_vb2,
            index_buffer: base_ib2,
            num_indices: base_i2.len() as u32,
        });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.p2_base_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Ball material
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.ball_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Ball trail line pipeline
        let trail = LinePipeline::new(&ctx, &pbr.camera_buffer);
        self.trail_pipeline = Some(trail);

        // Table markings line pipeline
        let markings = LinePipeline::new(&ctx, &pbr.camera_buffer);
        self.markings_pipeline = Some(markings);
        self.markings_uploaded = false;

        self.post_sampler = Some(post_sampler);
        self.depth_sampler = Some(depth_sampler);
        self.ssao_bind_group = Some(ssao_bg);
        self.sss_bind_group = Some(sss_bg);
        self.composite_bind_group = Some(composite_bg);

        self.shadow_pipeline = Some(shadow);
        self.pbr_pipeline = Some(pbr);
        self.ssao_pipeline = Some(ssao);
        self.sss_pipeline = Some(sss);
        self.composite_pipeline = Some(composite);
        self.render_ctx = Some(ctx);
        self.window = Some(window);
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        // Let egui handle events first
        if let Some(state) = &mut self.egui_state {
            let _ = state.on_window_event(self.window.as_ref().unwrap(), &event);
        }

        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::Resized(new_size) => {
                if let Some(ctx) = &mut self.render_ctx {
                    ctx.resize(new_size);

                    // Recreate post-processing bind groups
                    if let (Some(ssao), Some(ds)) = (&self.ssao_pipeline, &self.depth_sampler) {
                        self.ssao_bind_group =
                            Some(ssao.create_bind_group(&ctx.device, &ctx.depth_texture, ds));
                    }
                    if let (Some(sss), Some(ps)) = (&self.sss_pipeline, &self.post_sampler) {
                        self.sss_bind_group = Some(sss.create_bind_group(
                            &ctx.device,
                            &ctx.hdr_texture,
                            &ctx.depth_texture,
                            ps,
                        ));
                    }
                    if let (Some(composite), Some(sss), Some(ssao), Some(ps)) = (
                        &self.composite_pipeline,
                        &self.sss_pipeline,
                        &self.ssao_pipeline,
                        &self.post_sampler,
                    ) {
                        self.composite_bind_group = Some(composite.create_bind_group(
                            &ctx.device,
                            &sss.output_view,
                            &ssao.output_view,
                            ps,
                        ));
                    }

                    // Update SSS params with new dimensions
                    if let Some(sss) = &self.sss_pipeline {
                        let w = ctx.config.width as f32;
                        let h = ctx.config.height as f32;
                        sss.update_params(&ctx.queue, &SssParams::marble(w, h));
                    }
                }
            }
            WindowEvent::KeyboardInput {
                event:
                    winit::event::KeyEvent {
                        logical_key,
                        state: ElementState::Pressed,
                        ..
                    },
                ..
            } => {
                match logical_key {
                    Key::Named(NamedKey::Escape) => event_loop.exit(),
                    Key::Named(NamedKey::Space) => {
                        // Serve ball
                        if !self.ball.active {
                            let serve_pos = Vector3::new(-1.0, 0.0, table::TABLE_HEIGHT as f64 + 0.3);
                            let serve_vel = Vector3::new(3.0, 0.2, 1.5);
                            let topspin = Vector3::new(0.0, -50.0, 0.0);
                            self.ball.serve(serve_pos, serve_vel, topspin);
                            self.trail_points.clear();
                            self.game.phase = game::GamePhase::Rally;
                        }
                    }
                    Key::Character(ref c) if c.as_str() == "p" => {
                        self.paused = !self.paused;
                    }
                    Key::Character(ref c) if c.as_str() == "a" => {
                        self.show_audio_panel = !self.show_audio_panel;
                    }
                    Key::Named(NamedKey::Tab) => {
                        self.player1.ai_controlled = !self.player1.ai_controlled;
                        eprintln!(
                            "Player 1 AI: {}",
                            if self.player1.ai_controlled { "ON" } else { "OFF" }
                        );
                    }
                    Key::Character(ref c) if c.as_str() == "r" => {
                        // Reset game
                        self.game = GameState::new();
                        self.ball = ball::Ball::new();
                        self.trail_points.clear();
                        // Reset arms to ready position
                        let p1_targets = arm_config::ready_position_p1();
                        self.player1.arm.set_joint_angles(&p1_targets);
                        self.player1.joint_targets = p1_targets;
                        for j in self.player1.arm.joints.iter_mut() {
                            j.velocity = 0.0;
                        }
                        for pid in &mut self.player1.pid_controllers {
                            pid.reset();
                        }
                        let p2_targets = arm_config::ready_position_p2();
                        self.player2.arm.set_joint_angles(&p2_targets);
                        self.player2.joint_targets = p2_targets;
                        for j in self.player2.arm.joints.iter_mut() {
                            j.velocity = 0.0;
                        }
                        for pid in &mut self.player2.pid_controllers {
                            pid.reset();
                        }
                    }
                    // Camera presets
                    Key::Character(ref c) if c.as_str() == "1" => {
                        // Side view (default)
                        self.camera.target = Vec3::new(0.0, table::TABLE_HEIGHT / 2.0, 0.0);
                        self.camera.distance = 4.0;
                        self.camera.yaw = 0.8;
                        self.camera.pitch = 0.45;
                    }
                    Key::Character(ref c) if c.as_str() == "2" => {
                        // Player 1 view (behind P1)
                        self.camera.target = Vec3::new(0.0, table::TABLE_HEIGHT / 2.0, 0.0);
                        self.camera.distance = 3.0;
                        self.camera.yaw = std::f32::consts::PI;
                        self.camera.pitch = 0.35;
                    }
                    Key::Character(ref c) if c.as_str() == "3" => {
                        // Top-down view
                        self.camera.target = Vec3::new(0.0, table::TABLE_HEIGHT / 2.0, 0.0);
                        self.camera.distance = 5.0;
                        self.camera.yaw = 0.0;
                        self.camera.pitch = 1.4; // near vertical
                    }
                    Key::Character(ref c) if c.as_str() == "4" => {
                        // Close-up on net
                        self.camera.target = Vec3::new(0.0, table::TABLE_HEIGHT / 2.0 + 0.1, 0.0);
                        self.camera.distance = 1.8;
                        self.camera.yaw = 0.4;
                        self.camera.pitch = 0.25;
                    }
                    _ => {}
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
                    winit::event::MouseScrollDelta::PixelDelta(p) => p.y as f32 * 0.01,
                };
                self.camera.zoom(scroll);
            }
            WindowEvent::RedrawRequested => {
                // Timing
                let now = Instant::now();
                let raw_dt = now.duration_since(self.last_frame).as_secs_f64();
                let frame_dt = raw_dt.min(MAX_FRAME_TIME);
                self.last_frame = now;

                // AI update (once per frame, before physics)
                if !self.paused {
                    // Player 1 AI
                    if self.player1.ai_controlled {
                        let target = self.player1.ai.update(&self.ball, frame_dt);
                        self.player1.track_position(target);
                    }
                    // Player 2 AI
                    if self.player2.ai_controlled {
                        let target = self.player2.ai.update(&self.ball, frame_dt);
                        self.player2.track_position(target);
                    }
                }

                // Physics accumulator
                if !self.paused {
                    self.accumulator += frame_dt;
                    let max_steps = 200; // safety limit
                    let mut steps = 0;
                    while self.accumulator >= PHYSICS_DT && steps < max_steps {
                        self.player1.physics_step();
                        self.player2.physics_step();
                        self.ball.step(PHYSICS_DT);
                        if self.ball.active {
                            // Build paddle states for collision
                            let paddles = vec![
                                paddle_state_from_arm(&self.player1, 1),
                                paddle_state_from_arm(&self.player2, 2),
                            ];
                            let events = collision::resolve_collisions(&mut self.ball, &paddles);
                            // Process collision events for scoring and audio
                            let ball_x = self.ball.position.x as f32;
                            let ball_speed = self.ball.velocity.norm() as f32;
                            let pan = position_to_pan(ball_x, 2.0);
                            let intensity = (ball_speed / 8.0).clamp(0.2, 1.0);
                            for event in &events {
                                match event {
                                    collision::CollisionEvent::Table => {
                                        self.audio_engine.play(Box::new(
                                            TableBounceVoice::new(pan, intensity),
                                        ));
                                    }
                                    collision::CollisionEvent::Paddle { .. } => {
                                        self.audio_engine.play(Box::new(
                                            PaddleHitVoice::new(pan, intensity),
                                        ));
                                    }
                                    collision::CollisionEvent::Net => {
                                        self.audio_engine.play(Box::new(
                                            NetHitVoice::new(pan),
                                        ));
                                    }
                                    collision::CollisionEvent::Floor => {
                                        self.audio_engine.play(Box::new(
                                            FloorBounceVoice::new(pan),
                                        ));
                                        if self.game.phase == game::GamePhase::Rally {
                                            if self.ball.position.x < 0.0 {
                                                self.game.score_point(game::Player::Player2);
                                            } else {
                                                self.game.score_point(game::Player::Player1);
                                            }
                                        }
                                    }
                                }
                            }
                            // Record trail point every ~5ms (every 5th step)
                            if steps % 5 == 0 {
                                let swap = coord_swap_matrix();
                                let render_pos = swap.transform_point3(Vec3::new(
                                    self.ball.position.x as f32,
                                    self.ball.position.y as f32,
                                    self.ball.position.z as f32,
                                ));
                                self.trail_points.push(render_pos);
                                // Keep last 200 points
                                if self.trail_points.len() > 200 {
                                    self.trail_points.remove(0);
                                }
                            }
                        }
                        self.accumulator -= PHYSICS_DT;
                        self.sim_time += PHYSICS_DT;
                        steps += 1;
                    }
                }

                // Auto-transition PointScored -> WaitingForServe when ball dies
                if !self.ball.active && self.game.phase == game::GamePhase::PointScored {
                    self.game.reset_for_serve();
                }

                // Render
                self.render();

                // FPS
                self.frame_count += 1;
                let fps_elapsed = now.duration_since(self.fps_timer).as_secs_f64();
                if fps_elapsed >= 1.0 {
                    self.fps = self.frame_count as f64 / fps_elapsed;
                    self.frame_count = 0;
                    self.fps_timer = now;
                }
            }
            _ => {}
        }
    }
}

fn main() {
    let event_loop = EventLoop::new().expect("Failed to create event loop");
    event_loop.set_control_flow(ControlFlow::Poll);
    let mut app = App::new();
    event_loop.run_app(&mut app).expect("Event loop failed");
}
