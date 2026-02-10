//! SimuForge-Stone — Digital twin simulation of a 6DOF robot arm carving stone.
//!
//! Main binary: fixed-timestep physics loop + wgpu rendering.

use std::sync::Arc;
use std::time::Instant;

use glam::{Mat4, Quat, Vec3, Vec4};
use nalgebra::Vector3;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId};

use simuforge_control::ik::IkSolver;
use simuforge_core::{isometry_to_glam, Vertex};
use simuforge_cutting::forces::{self, MaterialProps};
use simuforge_cutting::tool::{Tool, ToolState};
use simuforge_material::mesher;
use simuforge_material::octree::OctreeSdf;
use simuforge_motors::gearbox::Gearbox;
use simuforge_motors::pid;
use simuforge_motors::stepper::{MotorState, StepperMotor};
use simuforge_physics::aba::articulated_body_algorithm;
use simuforge_physics::arm::RobotArm;
use simuforge_physics::joint::RotaryTable;
use simuforge_render::arm_visual::generate_cylinder;
use simuforge_render::camera::{CameraUniform, OrbitCamera};
use simuforge_render::context::RenderContext;
use simuforge_render::mesh::ChunkMeshManager;
use simuforge_render::pipelines::pbr::{LightUniform, MaterialUniform, PbrPipeline};
use simuforge_render::pipelines::shadow::ShadowPipeline;
use simuforge_render::pipelines::ssao::SsaoPipeline;
use simuforge_render::pipelines::sss::SssPipeline;
use simuforge_render::pipelines::composite::CompositePipeline;
use simuforge_render::pipelines::overlay::{OverlayBuilder, OverlayPipeline};

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

/// Full simulation state.
#[allow(dead_code)]
struct SimState {
    arm: RobotArm,
    motors: Vec<MotorState>,
    gearboxes: Vec<Gearbox>,
    pid_controllers: Vec<simuforge_motors::pid::PidController>,
    joint_targets: Vec<f64>,
    workpiece: OctreeSdf,
    tool: Tool,
    tool_state: ToolState,
    rotary_table: RotaryTable,
    material: MaterialProps,
    ik_solver: IkSolver,
    sim_time: f64,
    paused: bool,
    cutting_force_magnitude: f64,
}

impl SimState {
    fn new() -> Self {
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

        let joint_targets = vec![0.0; n];

        // 1ft (305mm) marble block, 0.5mm resolution
        let half_extent = 0.1525; // ~6 inches = 152.5mm
        let workpiece = OctreeSdf::new_block(
            [half_extent, half_extent, half_extent],
            0.0005, // 0.5mm
        );

        let tool = Tool::ball_nose(3.0, 20.0); // 3mm radius ball nose
        let tool_state = ToolState::new();
        let rotary_table = RotaryTable::new(5.0); // 5 kg·m² inertia

        Self {
            arm,
            motors,
            gearboxes,
            pid_controllers,
            joint_targets,
            workpiece,
            tool,
            tool_state,
            rotary_table,
            material: MaterialProps::marble(),
            ik_solver: IkSolver::default(),
            sim_time: 0.0,
            paused: true,
            cutting_force_magnitude: 0.0,
        }
    }

    /// Run one physics step at 1kHz.
    fn physics_step(&mut self) {
        if self.paused {
            return;
        }

        let n = self.arm.num_joints();
        let gravity = Vector3::new(0.0, 0.0, -9.81); // DH convention: Z-up

        // PID control: compute motor torque commands
        let mut joint_torques = vec![0.0; n];
        for i in 0..n {
            let pid_output = self.pid_controllers[i].update(
                self.joint_targets[i],
                self.arm.joints[i].angle,
                PHYSICS_DT,
            );

            // Motor torque clamping
            self.motors[i].motor.update_velocity(
                self.arm.joints[i].velocity,
                self.gearboxes[i].ratio,
            );
            let motor_torque = self.motors[i].apply_torque(pid_output, PHYSICS_DT as f64);

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

        // Material removal (if tool is in the workpiece region and spindle is on)
        if self.tool_state.spindle_on && self.tool_state.displacement() > 1e-6 {
            let tool_sdf = self
                .tool
                .swept_sdf(&self.tool_state.prev_position, &self.tool_state.position);
            let (bounds_min, bounds_max) = self
                .tool
                .swept_bounds(&self.tool_state.prev_position, &self.tool_state.position);
            self.workpiece.subtract(bounds_min, bounds_max, tool_sdf);

            // Cutting forces
            let feed_dir = (self.tool_state.position - self.tool_state.prev_position).normalize();
            let force = forces::cutting_force(
                &self.material,
                self.tool.radius,
                &feed_dir,
                0.001,  // 1mm depth of cut estimate
                0.0001, // 0.1mm feed per tooth
                2,      // 2 flutes
            );
            self.cutting_force_magnitude = force.norm();

            // Apply cutting force reaction to arm joints
            // (simplified: distribute force to last 3 joints)
            let force_torque = force.norm() * 0.01; // rough estimate
            if n >= 3 {
                for i in (n - 3)..n {
                    self.arm.joints[i].torque -= force_torque;
                }
            }
        }

        // Rotary table
        self.rotary_table.step(PHYSICS_DT);

        self.sim_time += PHYSICS_DT;
    }
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
    // --- GPU resources for arm, ground, workpiece ---
    arm_cylinder: Option<GpuMesh>,
    arm_materials: Vec<MaterialBind>,
    ground_mesh: Option<GpuMesh>,
    ground_material: Option<MaterialBind>,
    workpiece_material: Option<MaterialBind>,
}

impl App {
    fn new() -> Self {
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
            sim: SimState::new(),
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
            ground_mesh: None,
            ground_material: None,
            workpiece_material: None,
        }
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

        // Workpiece: marble with coord swap as model matrix
        if let Some(wp) = &self.workpiece_material {
            let mat = MaterialUniform::marble().with_model(swap.to_cols_array_2d());
            ctx.queue.write_buffer(&wp.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Ground plane: identity model (already in render space)
        if let Some(gnd) = &self.ground_material {
            let mat = MaterialUniform::ground();
            ctx.queue.write_buffer(&gnd.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Arm links: compute per-link model matrices from physics frames
        let frames = self.sim.arm.link_frames();
        for i in 0..6 {
            if i >= self.arm_materials.len() {
                break;
            }
            // Convert physics frame positions to render space
            let f0 = isometry_to_glam(&frames[i]);
            let f1 = isometry_to_glam(&frames[i + 1]);
            let p0 = swap.transform_point3(Vec3::new(f0.col(3).x, f0.col(3).y, f0.col(3).z));
            let p1 = swap.transform_point3(Vec3::new(f1.col(3).x, f1.col(3).y, f1.col(3).z));
            let model = link_model(p0, p1, ARM_LINK_RADII[i]);
            let mat = MaterialUniform::metal(ARM_LINK_COLORS[i])
                .with_model(model.to_cols_array_2d());
            ctx.queue
                .write_buffer(&self.arm_materials[i].buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Remesh dirty chunks
        let meshes = mesher::remesh_dirty(&mut self.sim.workpiece);
        for mesh in &meshes {
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

            // 3) Draw robot arm links
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
        let panel_w = 260.0f32;
        let margin = 10.0f32;
        let scale = 1.5f32; // text scale (1.0 = 8px glyphs)
        let line_h = 8.0 * scale + 4.0; // line height in pixels
        let bar_h = 12.0f32;
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

        // Status
        let status_color = if self.sim.paused { yellow } else { green };
        let status_text = if self.sim.paused { "PAUSED" } else { "RUNNING" };
        ob.rect(margin, y, 10.0, 10.0, status_color);
        ob.text(margin + 16.0, y, scale, status_color, status_text);
        y += line_h + 4.0;

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

        // --- Tool position ---
        let tp = &self.sim.tool_state.position;
        ob.text(margin, y, scale, white, "Tool Position");
        y += line_h;
        ob.text(
            margin, y, scale, gray,
            &format!("X:{:>6.1}mm", tp.x * 1000.0),
        );
        y += line_h;
        ob.text(
            margin, y, scale, gray,
            &format!("Y:{:>6.1}mm", tp.y * 1000.0),
        );
        y += line_h;
        ob.text(
            margin, y, scale, gray,
            &format!("Z:{:>6.1}mm", tp.z * 1000.0),
        );
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

        // Ground plane (large quad at Y=0 in render space)
        let gs = 3.0f32; // 3m half-extent
        let gy = -0.002f32;
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

        self.pbr_pipeline = Some(pbr);
        self.shadow_pipeline = Some(shadow);
        self.ssao_pipeline = Some(ssao);
        self.sss_pipeline = Some(sss);
        self.composite_pipeline = Some(composite);
        self.overlay_pipeline = Some(overlay);
        self.render_ctx = Some(ctx);
        self.window = Some(window);

        eprintln!("SimuForge-Stone initialized.");
        eprintln!("Controls: SPACE=pause/resume, 1=snap workpiece, 2=snap arm, LMB=rotate, RMB=zoom, MMB=pan");
        eprintln!("          Arrow keys=move joint targets, S=toggle spindle");
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
                        state: ElementState::Pressed,
                        ..
                    },
                ..
            } => {
                match logical_key.as_ref() {
                    Key::Named(NamedKey::Space) => {
                        self.sim.paused = !self.sim.paused;
                        eprintln!(
                            "Simulation {}",
                            if self.sim.paused { "PAUSED" } else { "RUNNING" }
                        );
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
                    Key::Character("s") => {
                        self.sim.tool_state.spindle_on = !self.sim.tool_state.spindle_on;
                        eprintln!(
                            "Spindle {}",
                            if self.sim.tool_state.spindle_on { "ON" } else { "OFF" }
                        );
                    }
                    Key::Named(NamedKey::ArrowUp) => {
                        // Move J2 target up
                        self.sim.joint_targets[1] += 0.05;
                    }
                    Key::Named(NamedKey::ArrowDown) => {
                        self.sim.joint_targets[1] -= 0.05;
                    }
                    Key::Named(NamedKey::ArrowLeft) => {
                        self.sim.joint_targets[0] += 0.05;
                    }
                    Key::Named(NamedKey::ArrowRight) => {
                        self.sim.joint_targets[0] -= 0.05;
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
                    winit::event::MouseScrollDelta::PixelDelta(pos) => pos.y as f32 * 0.01,
                };
                self.camera.zoom(scroll);
            }

            WindowEvent::RedrawRequested => {
                // Fixed timestep physics
                let now = Instant::now();
                let frame_time = now.duration_since(self.last_frame).as_secs_f64();
                self.last_frame = now;

                self.accumulator += frame_time.min(MAX_FRAME_TIME);

                while self.accumulator >= PHYSICS_DT {
                    self.sim.physics_step();
                    self.accumulator -= PHYSICS_DT;
                }

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

    let event_loop = EventLoop::new().expect("Failed to create event loop");
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = App::new();
    event_loop.run_app(&mut app).expect("Event loop error");
}
