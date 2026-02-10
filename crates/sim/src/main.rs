//! SimuForge-Stone — Digital twin simulation of a 6DOF robot arm carving stone.
//!
//! Main binary: fixed-timestep physics loop + wgpu rendering.

use std::sync::Arc;
use std::time::Instant;

use nalgebra::Vector3;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId};

use simuforge_control::ik::IkSolver;
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
use simuforge_render::camera::{CameraUniform, OrbitCamera};
use simuforge_render::context::RenderContext;
use simuforge_render::mesh::ChunkMeshManager;
use simuforge_render::pipelines::pbr::{LightUniform, MaterialUniform, PbrPipeline};
use simuforge_render::pipelines::shadow::ShadowPipeline;
use simuforge_render::pipelines::ssao::SsaoPipeline;
use simuforge_render::pipelines::sss::SssPipeline;
use simuforge_render::pipelines::composite::CompositePipeline;

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

        // Integrate
        for i in 0..n {
            self.arm.joints[i].integrate(accelerations[i], PHYSICS_DT);
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

        // Update camera uniform
        let cam_uniform = CameraUniform::from_camera(&self.camera, ctx.aspect());
        pbr.update_camera(&ctx.queue, &cam_uniform);

        // Update light
        let light = LightUniform {
            direction: [-0.5, -1.0, -0.3, 0.0],
            color: [1.0, 0.98, 0.95, 3.0],
            ambient: [0.6, 0.65, 0.7, 0.3],
            eye_pos: cam_uniform.eye_pos,
        };
        pbr.update_light(&ctx.queue, &light);

        // Update material (marble for workpiece)
        pbr.update_material(&ctx.queue, &MaterialUniform::marble());

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
                            r: 0.15,
                            g: 0.17,
                            b: 0.22,
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
            render_pass.set_bind_group(0, &pbr.bind_group, &[]);

            // Draw all chunk meshes (workpiece)
            for mesh in self.chunk_meshes.meshes.values() {
                render_pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                render_pass
                    .set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
            }
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

            // Print status
            let angles: Vec<f64> = self.sim.arm.joints.iter().map(|j| j.angle.to_degrees()).collect();
            eprintln!(
                "FPS: {:.0} | Sim: {:.2}s | Chunks: {} | Tris: {} | Joints: [{:.1}, {:.1}, {:.1}, {:.1}, {:.1}, {:.1}]° | Force: {:.1}N | {}",
                self.fps,
                self.sim.sim_time,
                self.chunk_meshes.chunk_count(),
                self.chunk_meshes.total_triangles(),
                angles.first().unwrap_or(&0.0),
                angles.get(1).unwrap_or(&0.0),
                angles.get(2).unwrap_or(&0.0),
                angles.get(3).unwrap_or(&0.0),
                angles.get(4).unwrap_or(&0.0),
                angles.get(5).unwrap_or(&0.0),
                self.sim.cutting_force_magnitude,
                if self.sim.paused { "PAUSED" } else { "RUNNING" },
            );
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
        let pbr = PbrPipeline::new(&ctx);
        let shadow = ShadowPipeline::new(&ctx);
        let ssao = SsaoPipeline::new(&ctx);
        let sss = SssPipeline::new(&ctx);
        let composite = CompositePipeline::new(&ctx);

        self.pbr_pipeline = Some(pbr);
        self.shadow_pipeline = Some(shadow);
        self.ssao_pipeline = Some(ssao);
        self.sss_pipeline = Some(sss);
        self.composite_pipeline = Some(composite);
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
