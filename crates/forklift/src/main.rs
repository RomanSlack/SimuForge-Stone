mod forklift;
mod mesh;
mod warehouse;

use forklift::ForkliftState;
use warehouse::Warehouse;

use glam::{Mat4, Vec3};
use simuforge_core::Vertex;
use simuforge_render::camera::{CameraUniform, OrbitCamera};
use simuforge_render::context::RenderContext;
use simuforge_render::pipelines::composite::{CompositePipeline, CompositeParams};
use simuforge_render::pipelines::pbr::{LightUniform, MaterialUniform, PbrPipeline};
use simuforge_render::pipelines::shadow::ShadowPipeline;
use simuforge_render::pipelines::ssao::{SsaoPipeline, SsaoParams};
use std::sync::Arc;
use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::Window;

const PHYSICS_DT: f64 = 1.0 / 200.0;
const MAX_FRAME_TIME: f64 = 0.1;
const MAX_STEPS_PER_FRAME: u32 = 40;

/// Convert DH Z-up position to render Y-up position.
/// DH: X=forward, Y=left, Z=up → Render: X=right, Y=up, Z=backward
/// We choose: DH X → render -Z, DH Y → render -X, DH Z → render Y
fn to_render(dh_pos: &nalgebra::Vector3<f64>) -> Vec3 {
    Vec3::new(-(dh_pos.y as f32), dh_pos.z as f32, -(dh_pos.x as f32))
}

/// Convert DH heading (rotation around Z-up) to render Y-rotation.
/// In DH, heading=0 means facing +X. In render, we want heading=0 facing -Z.
/// DH heading rotates CCW around Z. Render heading rotates around Y.
fn heading_to_render_y_rot(heading: f64) -> f32 {
    // DH forward (+X) maps to render -Z.
    // A positive DH heading (turning left) should rotate CCW when viewed from above = negative Y rotation.
    -(heading as f32)
}

struct GpuMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    num_indices: u32,
}

struct MaterialBind {
    buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

struct HeldKeys {
    w: bool,
    s: bool,
    a: bool,
    d: bool,
    r: bool,
    f: bool,
    t: bool,
    g: bool,
    space: bool,
}

impl HeldKeys {
    fn new() -> Self {
        Self {
            w: false,
            s: false,
            a: false,
            d: false,
            r: false,
            f: false,
            t: false,
            g: false,
            space: false,
        }
    }
}

/// Index of the pallet currently carried by the forklift (if any).
type CarriedPalletIdx = Option<usize>;

#[derive(Clone, Copy, PartialEq)]
enum CameraMode {
    Orbit,
    Chase,
    Operator,
}

struct App {
    window: Option<Arc<Window>>,
    render_ctx: Option<RenderContext>,

    // Pipelines
    pbr_pipeline: Option<PbrPipeline>,
    shadow_pipeline: Option<ShadowPipeline>,
    ssao_pipeline: Option<SsaoPipeline>,
    composite_pipeline: Option<CompositePipeline>,

    // Bind groups
    ssao_bind_group: Option<wgpu::BindGroup>,
    composite_bind_group: Option<wgpu::BindGroup>,
    sampler: Option<wgpu::Sampler>,
    non_filtering_sampler: Option<wgpu::Sampler>,

    // Meshes
    chassis_mesh: Option<GpuMesh>,
    guard_mesh: Option<GpuMesh>,
    outer_mast_mesh: Option<GpuMesh>,
    inner_mast_mesh: Option<GpuMesh>,
    carriage_mesh: Option<GpuMesh>,
    forks_mesh: Option<GpuMesh>,
    front_wheel_mesh: Option<GpuMesh>,
    rear_wheel_mesh: Option<GpuMesh>,
    seat_mesh: Option<GpuMesh>,
    floor_mesh: Option<GpuMesh>,
    pallet_mesh: Option<GpuMesh>,
    cargo_box_mesh: Option<GpuMesh>,
    wall_mesh: Option<GpuMesh>,

    // Materials (one per visual element)
    chassis_mat: Option<MaterialBind>,
    guard_mat: Option<MaterialBind>,
    outer_mast_mat: Option<MaterialBind>,
    inner_mast_mat: Option<MaterialBind>,
    carriage_mat: Option<MaterialBind>,
    forks_mat: Option<MaterialBind>,
    lf_wheel_mat: Option<MaterialBind>,
    rf_wheel_mat: Option<MaterialBind>,
    lr_wheel_mat: Option<MaterialBind>,
    rr_wheel_mat: Option<MaterialBind>,
    seat_mat: Option<MaterialBind>,
    floor_mat: Option<MaterialBind>,
    pallet_mats: Vec<MaterialBind>,
    cargo_mats: Vec<MaterialBind>,
    wall_mats: Vec<MaterialBind>,

    // egui
    egui_ctx: egui::Context,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,

    // Simulation
    forklift: ForkliftState,
    warehouse: Warehouse,
    carried_pallet: CarriedPalletIdx,

    // Camera
    camera: OrbitCamera,
    camera_mode: CameraMode,
    chase_cam_pos: Vec3,

    // Timing
    last_frame: Instant,
    accumulator: f64,
    paused: bool,
    time_scale: f64,

    // Input
    held_keys: HeldKeys,
    mouse_pressed: bool,
    middle_pressed: bool,
    last_mouse_pos: Option<(f64, f64)>,
}

impl App {
    fn new() -> Self {
        let mut camera = OrbitCamera::new();
        camera.target = Vec3::new(0.0, 1.0, 0.0);
        camera.distance = 8.0;
        camera.yaw = 0.8;
        camera.pitch = 0.4;
        camera.far = 200.0;

        Self {
            window: None,
            render_ctx: None,
            pbr_pipeline: None,
            shadow_pipeline: None,
            ssao_pipeline: None,
            composite_pipeline: None,
            ssao_bind_group: None,
            composite_bind_group: None,
            sampler: None,
            non_filtering_sampler: None,
            chassis_mesh: None,
            guard_mesh: None,
            outer_mast_mesh: None,
            inner_mast_mesh: None,
            carriage_mesh: None,
            forks_mesh: None,
            front_wheel_mesh: None,
            rear_wheel_mesh: None,
            seat_mesh: None,
            floor_mesh: None,
            pallet_mesh: None,
            cargo_box_mesh: None,
            wall_mesh: None,
            chassis_mat: None,
            guard_mat: None,
            outer_mast_mat: None,
            inner_mast_mat: None,
            carriage_mat: None,
            forks_mat: None,
            lf_wheel_mat: None,
            rf_wheel_mat: None,
            lr_wheel_mat: None,
            rr_wheel_mat: None,
            seat_mat: None,
            floor_mat: None,
            pallet_mats: Vec::new(),
            cargo_mats: Vec::new(),
            wall_mats: Vec::new(),
            egui_ctx: egui::Context::default(),
            egui_state: None,
            egui_renderer: None,
            forklift: ForkliftState::new(),
            warehouse: Warehouse::new(),
            carried_pallet: None,
            camera,
            camera_mode: CameraMode::Orbit,
            chase_cam_pos: Vec3::new(0.0, 3.0, 5.0),
            last_frame: Instant::now(),
            accumulator: 0.0,
            paused: false,
            time_scale: 1.0,
            held_keys: HeldKeys::new(),
            mouse_pressed: false,
            middle_pressed: false,
            last_mouse_pos: None,
        }
    }

    fn create_gpu_mesh(device: &wgpu::Device, verts: &[Vertex], idxs: &[u32]) -> GpuMesh {
        use wgpu::util::DeviceExt;
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Mesh VB"),
            contents: bytemuck::cast_slice(verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Mesh IB"),
            contents: bytemuck::cast_slice(idxs),
            usage: wgpu::BufferUsages::INDEX,
        });
        GpuMesh {
            vertex_buffer,
            index_buffer,
            num_indices: idxs.len() as u32,
        }
    }

    /// Try to pick up the nearest pallet within range of the forks.
    /// Uses the front of the forklift (mast position) as the reference point.
    fn try_pickup_pallet(&mut self) {
        if self.carried_pallet.is_some() {
            return; // already carrying
        }
        // Front of the forklift in DH frame: mast is at WHEELBASE + 0.175 forward
        let cos_h = self.forklift.heading.cos();
        let sin_h = self.forklift.heading.sin();
        let front_dist = 1.525 + 0.175 + 0.5; // roughly mid-fork
        let front_x = self.forklift.position.x + cos_h * front_dist;
        let front_y = self.forklift.position.y + sin_h * front_dist;

        let mut best_idx = None;
        let mut best_dist = 3.0_f64; // generous pickup range

        for (i, pallet) in self.warehouse.pallets.iter().enumerate() {
            if pallet.picked_up {
                continue;
            }
            let dx = pallet.position.x - front_x;
            let dy = pallet.position.y - front_y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist < best_dist {
                best_dist = dist;
                best_idx = Some(i);
            }
        }

        if let Some(idx) = best_idx {
            self.warehouse.pallets[idx].picked_up = true;
            self.carried_pallet = Some(idx);
            self.forklift.carried_load = Some(forklift::LoadState {
                mass: self.warehouse.pallets[idx].mass,
                cg_height: 0.3,
            });
        }
    }

    /// Drop the currently carried pallet at the fork position.
    fn drop_pallet(&mut self) {
        if let Some(idx) = self.carried_pallet.take() {
            let fork_tip = self.forklift.fork_tip_center();
            let fork_base = self.forklift.fork_base_center();
            let drop_pos = (fork_tip + fork_base) * 0.5;
            self.warehouse.pallets[idx].position.x = drop_pos.x;
            self.warehouse.pallets[idx].position.y = drop_pos.y;
            self.warehouse.pallets[idx].position.z = 0.0; // on ground
            self.warehouse.pallets[idx].heading = self.forklift.heading;
            self.warehouse.pallets[idx].picked_up = false;
            self.forklift.carried_load = None;
        }
    }

    fn rebuild_bind_groups(&mut self) {
        let ctx = self.render_ctx.as_ref().unwrap();
        let ssao = self.ssao_pipeline.as_ref().unwrap();
        let composite = self.composite_pipeline.as_ref().unwrap();
        let sampler = self.sampler.as_ref().unwrap();
        let nf_sampler = self.non_filtering_sampler.as_ref().unwrap();

        self.ssao_bind_group =
            Some(ssao.create_bind_group(&ctx.device, &ctx.depth_texture, nf_sampler));
        self.composite_bind_group = Some(composite.create_bind_group(
            &ctx.device,
            &ctx.hdr_texture,
            &ssao.output_view,
            sampler,
        ));
    }

    fn apply_held_keys(&mut self) {
        // Throttle
        if self.held_keys.w {
            self.forklift.throttle = 1.0;
        } else if self.held_keys.s {
            self.forklift.throttle = -0.5;
        } else {
            self.forklift.throttle = 0.0;
        }

        // Steering (A/D = left/right, which is negative/positive steer for rear-steer)
        if self.held_keys.a {
            self.forklift.steer_cmd = -1.0; // steer left → rear wheels go right
        } else if self.held_keys.d {
            self.forklift.steer_cmd = 1.0;
        } else {
            self.forklift.steer_cmd = 0.0;
        }

        // Brake
        self.forklift.brake = if self.held_keys.space { 1.0 } else { 0.0 };

        // Fork height
        if self.held_keys.r {
            self.forklift.fork_height_cmd = 3.0; // lift to max
        } else if self.held_keys.f {
            self.forklift.fork_height_cmd = 0.15; // lower to min
        }
        // If neither R nor F, hold current height
        if !self.held_keys.r && !self.held_keys.f {
            self.forklift.fork_height_cmd = self.forklift.fork_height;
        }

        // Mast tilt
        if self.held_keys.t {
            self.forklift.mast_tilt_cmd = 0.2094; // tilt back max
        } else if self.held_keys.g {
            self.forklift.mast_tilt_cmd = -0.1047; // tilt forward max
        }
        if !self.held_keys.t && !self.held_keys.g {
            self.forklift.mast_tilt_cmd = self.forklift.mast_tilt;
        }
    }

    /// Forward direction of forklift in render space.
    fn forklift_forward_render(&self) -> Vec3 {
        let yr = heading_to_render_y_rot(self.forklift.heading);
        // heading=0 → forward is -Z in render space
        Vec3::new(yr.sin(), 0.0, -yr.cos())
    }

    fn update_camera(&mut self) {
        let fl_render = to_render(&self.forklift.position);
        let center = fl_render + Vec3::new(0.0, 0.8, 0.0);

        match self.camera_mode {
            CameraMode::Orbit => {
                self.camera.target = center;
            }
            CameraMode::Chase => {
                let fwd = self.forklift_forward_render();
                let ideal = center - fwd * 6.0 + Vec3::Y * 3.0;
                self.chase_cam_pos += (ideal - self.chase_cam_pos) * 0.05;
                self.camera.target = center;
                self.camera.distance = (self.chase_cam_pos - center).length();
                let diff = self.chase_cam_pos - center;
                self.camera.yaw = diff.x.atan2(diff.z);
                self.camera.pitch = (diff.y / self.camera.distance.max(0.01)).asin();
            }
            CameraMode::Operator => {
                let fwd = self.forklift_forward_render();
                let seat_pos = fl_render + Vec3::new(0.0, 1.5, 0.0);
                self.camera.target = seat_pos + fwd * 5.0;
                self.camera.distance = 0.01;
                let diff = seat_pos - self.camera.target;
                self.camera.yaw = diff.x.atan2(diff.z);
                self.camera.pitch = 0.0;
            }
        }
    }

    fn forklift_model_matrices(&self) -> ForkliftMatrices {
        // ALL coordinates in render space: X=right, Y=up, Z=backward
        // Forklift origin at rear axle center on ground (Y=0).
        // Forward direction is -Z. Wheelbase extends toward -Z (front axle).
        let pos = to_render(&self.forklift.position);
        let yr = heading_to_render_y_rot(self.forklift.heading);

        // Chassis: translate to world position, rotate around Y for heading
        let chassis = Mat4::from_translation(pos) * Mat4::from_rotation_y(yr);

        // --- Wheels ---
        // Wheel meshes are pre-oriented along X-axis (axle direction).
        // Spin is rotation around X. No extra orient matrix needed.
        let fwa = self.forklift.front_wheel_angle as f32;
        let rwa = self.forklift.rear_wheel_angle as f32;
        let steer = self.forklift.steer_angle as f32;

        let front_z = -1.525_f32;
        let fw_y = 0.265_f32;
        let front_half_track = 0.50_f32;

        let lf_wheel = chassis
            * Mat4::from_translation(Vec3::new(-front_half_track, fw_y, front_z))
            * Mat4::from_rotation_x(fwa);

        let rf_wheel = chassis
            * Mat4::from_translation(Vec3::new(front_half_track, fw_y, front_z))
            * Mat4::from_rotation_x(fwa);

        let rw_y = 0.20_f32;
        let rear_half_track = 0.35_f32;

        let lr_wheel = chassis
            * Mat4::from_translation(Vec3::new(-rear_half_track, rw_y, 0.0))
            * Mat4::from_rotation_y(-steer)
            * Mat4::from_rotation_x(rwa);

        let rr_wheel = chassis
            * Mat4::from_translation(Vec3::new(rear_half_track, rw_y, 0.0))
            * Mat4::from_rotation_y(-steer)
            * Mat4::from_rotation_x(rwa);

        // --- Guard, Seat (fixed to chassis) ---
        let guard = chassis;
        let seat = chassis * Mat4::from_translation(Vec3::new(0.0, 0.80, 0.30));

        // --- Mast assembly ---
        // Mast pivot is near ground level at front of chassis.
        // The tilt pivot is at the bottom of the outer mast.
        let tilt = self.forklift.mast_tilt as f32;
        let outer_mast = chassis
            * Mat4::from_translation(Vec3::new(0.0, 0.0, -1.70))
            * Mat4::from_rotation_x(-tilt);

        // fork_height is the actual height of fork tips above ground.
        // fork_height directly drives carriage Y position.
        // Chain coupling: inner mast gets half, carriage gets the other half.
        let fork_h = self.forklift.fork_height as f32;
        let inner_lift = fork_h * 0.5;
        let inner_mast = outer_mast * Mat4::from_translation(Vec3::new(0.0, inner_lift, 0.0));

        let carriage_lift = fork_h * 0.5;
        let carriage = inner_mast * Mat4::from_translation(Vec3::new(0.0, carriage_lift, 0.0));

        // Forks: fixed to carriage. Fork mesh has tine bottom at Y=0.
        let forks = carriage;

        ForkliftMatrices {
            chassis,
            guard,
            outer_mast,
            inner_mast,
            carriage,
            forks,
            lf_wheel,
            rf_wheel,
            lr_wheel,
            rr_wheel,
            seat,
        }
    }

    fn render(&mut self) {
        let ctx = self.render_ctx.as_ref().unwrap();
        let pbr = self.pbr_pipeline.as_ref().unwrap();
        let shadow = self.shadow_pipeline.as_ref().unwrap();
        let ssao = self.ssao_pipeline.as_ref().unwrap();
        let composite = self.composite_pipeline.as_ref().unwrap();

        let surface_texture = match ctx.surface.get_current_texture() {
            Ok(t) => t,
            Err(_) => return,
        };
        let surface_view = surface_texture
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let aspect = ctx.aspect();
        let cam_uniform = CameraUniform::from_camera(&self.camera, aspect);
        pbr.update_camera(&ctx.queue, &cam_uniform);

        // Light — outdoor daylight (strong directional + good ambient fill)
        let light_dir = Vec3::new(-0.4, -0.8, -0.3).normalize();
        let eye = self.camera.eye();
        let light = LightUniform {
            direction: [light_dir.x, light_dir.y, light_dir.z, 0.0],
            color: [1.0, 0.95, 0.85, 3.0],
            ambient: [0.5, 0.48, 0.42, 0.5],
            eye_pos: [eye.x, eye.y, eye.z, 0.0],
        };
        pbr.update_light(&ctx.queue, &light);

        // Shadow setup
        let fl_render = to_render(&self.forklift.position);
        let scene_center = fl_render + Vec3::new(0.0, 1.0, 0.0);
        let scene_radius = 15.0_f32;
        let light_pos = scene_center - light_dir * scene_radius * 2.0;
        let shadow_view = Mat4::look_at_rh(light_pos, scene_center, Vec3::Y);
        let shadow_proj = Mat4::orthographic_rh(
            -scene_radius, scene_radius,
            -scene_radius, scene_radius,
            0.1, scene_radius * 4.0,
        );
        let light_vp = shadow_proj * shadow_view;
        pbr.update_shadow_light_vp(&ctx.queue, &light_vp);

        // SSAO params
        let proj = self.camera.projection_matrix(aspect);
        ssao.update_params(
            &ctx.queue,
            &SsaoParams {
                proj: proj.to_cols_array_2d(),
                radius: 0.5,
                bias: 0.025,
                intensity: 1.5,
                _pad: 0.0,
            },
        );

        // Composite params
        composite.update_params(
            &ctx.queue,
            &CompositeParams {
                exposure: 1.0,
                gamma: 2.2,
                ssao_strength: 0.6,
                sss_strength: 0.0,
                thermal_mode: 0.0,
                flir_mode: 0.0,
                _pad: [0.0; 2],
            },
        );

        // Compute model matrices
        let matrices = self.forklift_model_matrices();

        // Upload shadow matrices
        let mut shadow_matrices = Vec::new();
        let model_list = [
            &matrices.chassis,
            &matrices.guard,
            &matrices.outer_mast,
            &matrices.inner_mast,
            &matrices.carriage,
            &matrices.forks,
            &matrices.lf_wheel,
            &matrices.rf_wheel,
            &matrices.lr_wheel,
            &matrices.rr_wheel,
            &matrices.seat,
        ];
        // Floor
        let floor_model = Mat4::from_translation(Vec3::new(0.0, -0.01, 0.0));
        shadow_matrices.push(light_vp * floor_model);
        for m in &model_list {
            shadow_matrices.push(light_vp * **m);
        }
        // Walls (already in render space)
        for wall in &self.warehouse.walls {
            let wm = Mat4::from_translation(Vec3::from(wall.render_center))
                * Mat4::from_scale(Vec3::from(wall.render_half_extents));
            shadow_matrices.push(light_vp * wm);
        }
        // Pallets
        for pallet in &self.warehouse.pallets {
            if pallet.picked_up {
                continue;
            }
            let pr = to_render(&pallet.position);
            let pm = Mat4::from_translation(pr) * Mat4::from_rotation_y(heading_to_render_y_rot(pallet.heading));
            shadow_matrices.push(light_vp * pm);
        }
        shadow_matrices.truncate(32); // MAX_SHADOW_OBJECTS
        shadow.upload_matrices(&ctx.queue, &shadow_matrices);

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        // === Shadow Pass ===
        {
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

            let mut shadow_idx = 0;
            // Floor
            if let Some(m) = &self.floor_mesh {
                pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(shadow_idx)]);
                pass.set_vertex_buffer(0, m.vertex_buffer.slice(..));
                pass.set_index_buffer(m.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.num_indices, 0, 0..1);
                shadow_idx += 1;
            }

            // Forklift parts
            let meshes_and_matrices: Vec<(&Option<GpuMesh>, &Mat4)> = vec![
                (&self.chassis_mesh, &matrices.chassis),
                (&self.guard_mesh, &matrices.guard),
                (&self.outer_mast_mesh, &matrices.outer_mast),
                (&self.inner_mast_mesh, &matrices.inner_mast),
                (&self.carriage_mesh, &matrices.carriage),
                (&self.forks_mesh, &matrices.forks),
                (&self.front_wheel_mesh, &matrices.lf_wheel),
                (&self.front_wheel_mesh, &matrices.rf_wheel),
                (&self.rear_wheel_mesh, &matrices.lr_wheel),
                (&self.rear_wheel_mesh, &matrices.rr_wheel),
                (&self.seat_mesh, &matrices.seat),
            ];

            for (mesh_opt, _) in &meshes_and_matrices {
                if let Some(m) = mesh_opt {
                    if shadow_idx < 32 {
                        pass.set_bind_group(
                            0,
                            &shadow.bind_group,
                            &[ShadowPipeline::dynamic_offset(shadow_idx)],
                        );
                        pass.set_vertex_buffer(0, m.vertex_buffer.slice(..));
                        pass.set_index_buffer(m.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..m.num_indices, 0, 0..1);
                        shadow_idx += 1;
                    }
                }
            }
        }

        // === PBR Pass (HDR) ===
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("PBR Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &ctx.hdr_texture,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.529,
                            g: 0.808,
                            b: 0.922,
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
            pass.set_pipeline(&pbr.pipeline);

            // Floor
            if let (Some(m), Some(mat)) = (&self.floor_mesh, &self.floor_mat) {
                let mut floor_uniform = MaterialUniform::ground()
                    .with_model(floor_model.to_cols_array_2d());
                // Large grid fade distance for warehouse (1m grid lines, visible to 40m)
                floor_uniform.params[3] = -40.0;
                ctx.queue
                    .write_buffer(&mat.buffer, 0, bytemuck::bytes_of(&floor_uniform));
                pass.set_bind_group(0, &mat.bind_group, &[]);
                pass.set_bind_group(1, &pbr.shadow_bind_group, &[]);
                pass.set_vertex_buffer(0, m.vertex_buffer.slice(..));
                pass.set_index_buffer(m.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.num_indices, 0, 0..1);
            }

            // Forklift parts with materials
            let parts: Vec<(&Option<GpuMesh>, &Option<MaterialBind>, [f32; 4], [f32; 4], Mat4)> = vec![
                (&self.chassis_mesh, &self.chassis_mat, [0.95, 0.75, 0.05, 1.0], [0.4, 0.9, 0.0, 0.0], matrices.chassis),
                (&self.guard_mesh, &self.guard_mat, [0.1, 0.1, 0.1, 1.0], [0.5, 0.8, 0.0, 0.0], matrices.guard),
                (&self.outer_mast_mesh, &self.outer_mast_mat, [0.3, 0.3, 0.3, 1.0], [0.4, 0.9, 0.0, 0.0], matrices.outer_mast),
                (&self.inner_mast_mesh, &self.inner_mast_mat, [0.35, 0.35, 0.35, 1.0], [0.4, 0.9, 0.0, 0.0], matrices.inner_mast),
                (&self.carriage_mesh, &self.carriage_mat, [0.3, 0.3, 0.3, 1.0], [0.5, 0.8, 0.0, 0.0], matrices.carriage),
                (&self.forks_mesh, &self.forks_mat, [0.5, 0.5, 0.5, 1.0], [0.3, 0.9, 0.0, 0.0], matrices.forks),
                (&self.front_wheel_mesh, &self.lf_wheel_mat, [0.25, 0.25, 0.25, 1.0], [0.85, 0.05, 0.0, 0.0], matrices.lf_wheel),
                (&self.front_wheel_mesh, &self.rf_wheel_mat, [0.25, 0.25, 0.25, 1.0], [0.85, 0.05, 0.0, 0.0], matrices.rf_wheel),
                (&self.rear_wheel_mesh, &self.lr_wheel_mat, [0.25, 0.25, 0.25, 1.0], [0.85, 0.05, 0.0, 0.0], matrices.lr_wheel),
                (&self.rear_wheel_mesh, &self.rr_wheel_mat, [0.25, 0.25, 0.25, 1.0], [0.85, 0.05, 0.0, 0.0], matrices.rr_wheel),
                (&self.seat_mesh, &self.seat_mat, [0.1, 0.1, 0.3, 1.0], [0.7, 0.1, 0.0, 0.0], matrices.seat),
            ];

            for (mesh_opt, mat_opt, color, params, model) in &parts {
                if let (Some(m), Some(mat)) = (mesh_opt, mat_opt) {
                    let uniform = MaterialUniform {
                        base_color: *color,
                        params: *params,
                        model: model.to_cols_array_2d(),
                        bounds_min: [0.0; 4],
                        bounds_max: [0.0; 4],
                    };
                    ctx.queue
                        .write_buffer(&mat.buffer, 0, bytemuck::bytes_of(&uniform));
                    pass.set_bind_group(0, &mat.bind_group, &[]);
                    pass.set_bind_group(1, &pbr.shadow_bind_group, &[]);
                    pass.set_vertex_buffer(0, m.vertex_buffer.slice(..));
                    pass.set_index_buffer(m.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..m.num_indices, 0, 0..1);
                }
            }

            // Walls
            if let Some(wall_mesh) = &self.wall_mesh {
                for (i, wall) in self.warehouse.walls.iter().enumerate() {
                    if let Some(mat) = self.wall_mats.get(i) {
                        // Wall mesh is a unit box; scale to wall dimensions (render space)
                        let wm = Mat4::from_translation(Vec3::from(wall.render_center))
                            * Mat4::from_scale(Vec3::from(wall.render_half_extents));
                        let uniform = MaterialUniform {
                            base_color: [0.7, 0.7, 0.65, 1.0],
                            params: [0.95, 0.0, 0.0, 0.0],
                            model: wm.to_cols_array_2d(),
                            bounds_min: [0.0; 4],
                            bounds_max: [0.0; 4],
                        };
                        ctx.queue
                            .write_buffer(&mat.buffer, 0, bytemuck::bytes_of(&uniform));
                        pass.set_bind_group(0, &mat.bind_group, &[]);
                        pass.set_bind_group(1, &pbr.shadow_bind_group, &[]);
                        pass.set_vertex_buffer(0, wall_mesh.vertex_buffer.slice(..));
                        pass.set_index_buffer(
                            wall_mesh.index_buffer.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        pass.draw_indexed(0..wall_mesh.num_indices, 0, 0..1);
                    }
                }
            }

            // Pallets (on ground + carried on forks)
            if let Some(pallet_mesh) = &self.pallet_mesh {
                for (i, pallet) in self.warehouse.pallets.iter().enumerate() {
                    if let Some(mat) = self.pallet_mats.get(i) {
                        let pm = if pallet.picked_up {
                            // Carried pallet: position on the forks
                            // Forks mesh origin is at carriage, forks extend -Z by 1.07m
                            // Place pallet center at mid-fork, sitting on fork surface
                            matrices.forks
                                * Mat4::from_translation(Vec3::new(0.0, 0.035, -0.53))
                        } else {
                            // Ground pallet
                            let pr = to_render(&pallet.position);
                            Mat4::from_translation(pr)
                                * Mat4::from_rotation_y(heading_to_render_y_rot(pallet.heading))
                        };
                        let uniform = MaterialUniform {
                            base_color: [0.65, 0.50, 0.30, 1.0],
                            params: [0.85, 0.0, 0.0, 0.0],
                            model: pm.to_cols_array_2d(),
                            bounds_min: [0.0; 4],
                            bounds_max: [0.0; 4],
                        };
                        ctx.queue
                            .write_buffer(&mat.buffer, 0, bytemuck::bytes_of(&uniform));
                        pass.set_bind_group(0, &mat.bind_group, &[]);
                        pass.set_bind_group(1, &pbr.shadow_bind_group, &[]);
                        pass.set_vertex_buffer(0, pallet_mesh.vertex_buffer.slice(..));
                        pass.set_index_buffer(
                            pallet_mesh.index_buffer.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        pass.draw_indexed(0..pallet_mesh.num_indices, 0, 0..1);
                    }
                }
            }

            // Cargo boxes on pallets (including carried)
            if let Some(cargo_mesh) = &self.cargo_box_mesh {
                let mut cargo_idx = 0;
                for (_pi, pallet) in self.warehouse.pallets.iter().enumerate() {
                    for &ch in &pallet.cargo_heights {
                        if let Some(mat) = self.cargo_mats.get(cargo_idx) {
                            let pm = if pallet.picked_up {
                                // Cargo on carried pallet: relative to fork transform
                                matrices.forks
                                    * Mat4::from_translation(Vec3::new(0.0, 0.035 + ch as f32 + 0.20, -0.53))
                            } else {
                                let pr = to_render(&pallet.position);
                                let box_pos = pr + Vec3::new(0.0, ch as f32 + 0.20, 0.0);
                                Mat4::from_translation(box_pos)
                                    * Mat4::from_rotation_y(heading_to_render_y_rot(pallet.heading))
                            };
                            // Alternate box colors
                            let colors = [
                                [0.7, 0.3, 0.2, 1.0],
                                [0.2, 0.5, 0.7, 1.0],
                                [0.3, 0.6, 0.3, 1.0],
                                [0.8, 0.7, 0.2, 1.0],
                            ];
                            let color = colors[cargo_idx % colors.len()];
                            let uniform = MaterialUniform {
                                base_color: color,
                                params: [0.8, 0.0, 0.0, 0.0],
                                model: pm.to_cols_array_2d(),
                                bounds_min: [0.0; 4],
                                bounds_max: [0.0; 4],
                            };
                            ctx.queue
                                .write_buffer(&mat.buffer, 0, bytemuck::bytes_of(&uniform));
                            pass.set_bind_group(0, &mat.bind_group, &[]);
                            pass.set_bind_group(1, &pbr.shadow_bind_group, &[]);
                            pass.set_vertex_buffer(0, cargo_mesh.vertex_buffer.slice(..));
                            pass.set_index_buffer(
                                cargo_mesh.index_buffer.slice(..),
                                wgpu::IndexFormat::Uint32,
                            );
                            pass.draw_indexed(0..cargo_mesh.num_indices, 0, 0..1);
                            cargo_idx += 1;
                        }
                    }
                }
            }
        }

        // === SSAO Pass ===
        {
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
            if let Some(bg) = &self.ssao_bind_group {
                pass.set_bind_group(0, bg, &[]);
                pass.draw(0..3, 0..1);
            }
        }

        // === Composite Pass ===
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Composite Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &surface_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                ..Default::default()
            });
            pass.set_pipeline(&composite.pipeline);
            if let Some(bg) = &self.composite_bind_group {
                pass.set_bind_group(0, bg, &[]);
                pass.draw(0..3, 0..1);
            }
        }

        // === egui Pass ===
        let egui_input = self.egui_state.as_mut().unwrap().take_egui_input(self.window.as_ref().unwrap());
        self.egui_ctx.begin_pass(egui_input);

        egui::Window::new("Forklift")
            .default_pos([10.0, 10.0])
            .show(&self.egui_ctx, |ui| {
                ui.label(format!(
                    "Speed: {:.1} km/h",
                    self.forklift.speed_kmh()
                ));
                ui.label(format!(
                    "Fork Height: {:.2} m",
                    self.forklift.fork_height
                ));
                ui.label(format!(
                    "Mast Tilt: {:.1}°",
                    self.forklift.mast_tilt.to_degrees()
                ));
                ui.label(format!(
                    "Steer: {:.1}°",
                    self.forklift.steer_angle.to_degrees()
                ));
                ui.separator();
                ui.label(if self.forklift.carried_load.is_some() {
                    "Load: CARRYING"
                } else {
                    "Load: EMPTY"
                });
                ui.separator();
                ui.label("W/S: Drive | A/D: Steer");
                ui.label("R/F: Lift | T/G: Tilt");
                ui.label("Space: Brake | C: Camera");
                ui.label("P: Pause");
            });

        let egui_output = self.egui_ctx.end_pass();
        let egui_prims = self.egui_ctx.tessellate(egui_output.shapes, egui_output.pixels_per_point);
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
                    view: &surface_view,
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

        let mut cmd_bufs: Vec<wgpu::CommandBuffer> = egui_cmd_bufs;
        cmd_bufs.push(encoder.finish());
        ctx.queue.submit(cmd_bufs);
        surface_texture.present();
    }
}

struct ForkliftMatrices {
    chassis: Mat4,
    guard: Mat4,
    outer_mast: Mat4,
    inner_mast: Mat4,
    carriage: Mat4,
    forks: Mat4,
    lf_wheel: Mat4,
    rf_wheel: Mat4,
    lr_wheel: Mat4,
    rr_wheel: Mat4,
    seat: Mat4,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        let attrs = Window::default_attributes()
            .with_title("SimuForge — Forklift Digital Twin")
            .with_inner_size(winit::dpi::LogicalSize::new(1600, 900));
        let window = Arc::new(event_loop.create_window(attrs).unwrap());
        self.window = Some(window.clone());

        let ctx = pollster::block_on(RenderContext::new(window.clone()));

        // Pipelines
        let shadow = ShadowPipeline::new(&ctx);
        let pbr = PbrPipeline::new(&ctx, &shadow);
        let ssao = SsaoPipeline::new(&ctx);
        let composite = CompositePipeline::new(&ctx);

        // Samplers
        let sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Linear Sampler"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let non_filtering_sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Non-Filtering Sampler"),
            ..Default::default()
        });

        // Bind groups
        let ssao_bg = ssao.create_bind_group(&ctx.device, &ctx.depth_texture, &non_filtering_sampler);
        let composite_bg =
            composite.create_bind_group(&ctx.device, &ctx.hdr_texture, &ssao.output_view, &sampler);

        // egui
        let egui_state = egui_winit::State::new(
            self.egui_ctx.clone(),
            egui::ViewportId::ROOT,
            &window,
            None,
            None,
            None,
        );
        let egui_renderer =
            egui_wgpu::Renderer::new(&ctx.device, ctx.format(), None, 1, false);

        // === Create meshes ===
        let (cv, ci) = mesh::generate_chassis();
        self.chassis_mesh = Some(Self::create_gpu_mesh(&ctx.device, &cv, &ci));

        let (gv, gi) = mesh::generate_overhead_guard();
        self.guard_mesh = Some(Self::create_gpu_mesh(&ctx.device, &gv, &gi));

        let (omv, omi) = mesh::generate_outer_mast();
        self.outer_mast_mesh = Some(Self::create_gpu_mesh(&ctx.device, &omv, &omi));

        let (imv, imi) = mesh::generate_inner_mast();
        self.inner_mast_mesh = Some(Self::create_gpu_mesh(&ctx.device, &imv, &imi));

        let (cav, cai) = mesh::generate_carriage();
        self.carriage_mesh = Some(Self::create_gpu_mesh(&ctx.device, &cav, &cai));

        let (fv, fi) = mesh::generate_forks();
        self.forks_mesh = Some(Self::create_gpu_mesh(&ctx.device, &fv, &fi));

        let (fwv, fwi) = mesh::generate_front_wheel();
        self.front_wheel_mesh = Some(Self::create_gpu_mesh(&ctx.device, &fwv, &fwi));

        let (rwv, rwi) = mesh::generate_rear_wheel();
        self.rear_wheel_mesh = Some(Self::create_gpu_mesh(&ctx.device, &rwv, &rwi));

        let (sv, si) = mesh::generate_seat();
        self.seat_mesh = Some(Self::create_gpu_mesh(&ctx.device, &sv, &si));

        let (flv, fli) = mesh::generate_floor(25.0);
        self.floor_mesh = Some(Self::create_gpu_mesh(&ctx.device, &flv, &fli));

        let (pv, pi) = mesh::generate_pallet();
        self.pallet_mesh = Some(Self::create_gpu_mesh(&ctx.device, &pv, &pi));

        let (bv, bi) = mesh::generate_cargo_box(0.30, 0.20, 0.25);
        self.cargo_box_mesh = Some(Self::create_gpu_mesh(&ctx.device, &bv, &bi));

        // Wall mesh (unit box, scaled per-wall)
        let (wv, wi) = simuforge_render::arm_visual::generate_box(1.0, 1.0, 1.0);
        self.wall_mesh = Some(Self::create_gpu_mesh(&ctx.device, &wv, &wi));

        // === Create material bind groups ===
        let make_mat = |pbr: &PbrPipeline| -> MaterialBind {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            MaterialBind {
                buffer: buf,
                bind_group: bg,
            }
        };

        self.chassis_mat = Some(make_mat(&pbr));
        self.guard_mat = Some(make_mat(&pbr));
        self.outer_mast_mat = Some(make_mat(&pbr));
        self.inner_mast_mat = Some(make_mat(&pbr));
        self.carriage_mat = Some(make_mat(&pbr));
        self.forks_mat = Some(make_mat(&pbr));
        self.lf_wheel_mat = Some(make_mat(&pbr));
        self.rf_wheel_mat = Some(make_mat(&pbr));
        self.lr_wheel_mat = Some(make_mat(&pbr));
        self.rr_wheel_mat = Some(make_mat(&pbr));
        self.seat_mat = Some(make_mat(&pbr));
        self.floor_mat = Some(make_mat(&pbr));

        // Pallet materials
        self.pallet_mats.clear();
        for _ in &self.warehouse.pallets {
            self.pallet_mats.push(make_mat(&pbr));
        }

        // Cargo box materials
        self.cargo_mats.clear();
        let total_cargo: usize = self.warehouse.pallets.iter().map(|p| p.cargo_heights.len()).sum();
        for _ in 0..total_cargo {
            self.cargo_mats.push(make_mat(&pbr));
        }

        // Wall materials
        self.wall_mats.clear();
        for _ in &self.warehouse.walls {
            self.wall_mats.push(make_mat(&pbr));
        }

        // Store everything
        self.render_ctx = Some(ctx);
        self.shadow_pipeline = Some(shadow);
        self.pbr_pipeline = Some(pbr);
        self.ssao_pipeline = Some(ssao);
        self.composite_pipeline = Some(composite);
        self.ssao_bind_group = Some(ssao_bg);
        self.composite_bind_group = Some(composite_bg);
        self.sampler = Some(sampler);
        self.non_filtering_sampler = Some(non_filtering_sampler);
        self.egui_state = Some(egui_state);
        self.egui_renderer = Some(egui_renderer);

        self.last_frame = Instant::now();
    }

    fn window_event(
        &mut self,
        event_loop: &winit::event_loop::ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        // egui gets first crack at events
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
                    if let Some(ssao) = &mut self.ssao_pipeline {
                        ssao.resize(&ctx.device, &ctx.config);
                    }
                    self.rebuild_bind_groups();
                }
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        logical_key,
                        state: key_state,
                        ..
                    },
                ..
            } => {
                let pressed = key_state == ElementState::Pressed;
                match &logical_key {
                    Key::Character(c) => match c.as_str() {
                        "w" => self.held_keys.w = pressed,
                        "s" => self.held_keys.s = pressed,
                        "a" => self.held_keys.a = pressed,
                        "d" => self.held_keys.d = pressed,
                        "r" => self.held_keys.r = pressed,
                        "f" => self.held_keys.f = pressed,
                        "t" => self.held_keys.t = pressed,
                        "g" => self.held_keys.g = pressed,
                        "p" if pressed => self.paused = !self.paused,
                        "e" if pressed => self.try_pickup_pallet(),
                        "q" if pressed => self.drop_pallet(),
                        "c" if pressed => {
                            self.camera_mode = match self.camera_mode {
                                CameraMode::Orbit => CameraMode::Chase,
                                CameraMode::Chase => CameraMode::Operator,
                                CameraMode::Operator => CameraMode::Orbit,
                            };
                        }
                        _ => {}
                    },
                    Key::Named(NamedKey::Space) => self.held_keys.space = pressed,
                    Key::Named(NamedKey::Escape) if pressed => event_loop.exit(),
                    _ => {}
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                let pressed = state == ElementState::Pressed;
                match button {
                    MouseButton::Left => self.mouse_pressed = pressed,
                    MouseButton::Middle => self.middle_pressed = pressed,
                    _ => {}
                }
                if !pressed {
                    self.last_mouse_pos = None;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let (x, y) = (position.x, position.y);
                if let Some((lx, ly)) = self.last_mouse_pos {
                    let dx = (x - lx) as f32;
                    let dy = (y - ly) as f32;
                    if self.mouse_pressed && self.camera_mode == CameraMode::Orbit {
                        self.camera.rotate(-dx * 0.005, -dy * 0.005);
                    }
                    if self.middle_pressed {
                        self.camera.pan(-dx, dy);
                    }
                }
                self.last_mouse_pos = Some((x, y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    winit::event::MouseScrollDelta::LineDelta(_, y) => y,
                    winit::event::MouseScrollDelta::PixelDelta(p) => p.y as f32 * 0.01,
                };
                self.camera.zoom(scroll);
            }
            WindowEvent::RedrawRequested => {
                let now = Instant::now();
                let raw_dt = now.duration_since(self.last_frame).as_secs_f64();
                let frame_dt = raw_dt.min(MAX_FRAME_TIME);
                self.last_frame = now;

                // Apply held keys to forklift commands
                self.apply_held_keys();

                // Physics loop
                if !self.paused {
                    self.accumulator += frame_dt * self.time_scale;
                    let mut steps = 0;
                    while self.accumulator >= PHYSICS_DT && steps < MAX_STEPS_PER_FRAME {
                        self.forklift.step(PHYSICS_DT);
                        self.accumulator -= PHYSICS_DT;
                        steps += 1;
                    }
                }

                // Camera
                self.update_camera();

                // Render
                self.render();

                if let Some(w) = &self.window {
                    w.request_redraw();
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &winit::event_loop::ActiveEventLoop) {
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }
}

fn main() {
    let event_loop = EventLoop::new().expect("Failed to create event loop");
    event_loop.set_control_flow(ControlFlow::Poll);
    let mut app = App::new();
    event_loop.run_app(&mut app).expect("Event loop failed");
}
