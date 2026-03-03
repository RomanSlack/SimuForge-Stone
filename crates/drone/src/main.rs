//! SimuForge Drone — Shahed-136 digital twin flight simulation.
//!
//! Educational delta-wing loitering munition flying a 50km mission over
//! procedural desert terrain with HDR skybox, physics-based flight dynamics,
//! GPS/INS guidance, time-lapse controls, and multiple camera modes.

#![allow(dead_code)]

use std::sync::{Arc, Mutex};
use std::time::Instant;

use glam::{Mat4, Quat, Vec3, Vec4};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId};

use simuforge_render::camera::{CameraUniform, OrbitCamera};
use simuforge_render::context::RenderContext;
use simuforge_render::pipelines::composite::{CompositePipeline, CompositeParams};
use simuforge_render::pipelines::line::{LinePipeline, LineVertex};
use simuforge_render::pipelines::pbr::{LightUniform, MaterialUniform, PbrPipeline};
use simuforge_render::pipelines::shadow::ShadowPipeline;
use simuforge_render::pipelines::ssao::{SsaoPipeline, SsaoParams};
use simuforge_render::pipelines::sss::{SssPipeline, SssParams};

use simuforge_audio::AudioEngine;

mod drone;
mod flight;
mod guidance;
mod skybox;
mod sound;
mod terrain;

use flight::FlightState;
use guidance::{FlightPhase, Guidance};
use skybox::SkyboxPipeline;

/// Physics timestep for flight model.
const PHYSICS_DT: f64 = flight::PHYSICS_DT;
/// Maximum accumulated time before clamping (seconds).
const MAX_FRAME_TIME: f64 = 0.1;
/// Maximum physics steps per frame.
const MAX_STEPS_PER_FRAME: u64 = 4000;

/// Path to the HDR skybox.
const HDR_PATH: &str = "assets/hdrs/kloppenheim_06_puresky_4k.hdr";

/// Coordinate swap: DH Z-up physics -> Y-up renderer.
fn coord_swap_matrix() -> Mat4 {
    Mat4::from_cols(
        Vec4::new(1.0, 0.0, 0.0, 0.0),
        Vec4::new(0.0, 0.0, -1.0, 0.0),
        Vec4::new(0.0, 1.0, 0.0, 0.0),
        Vec4::new(0.0, 0.0, 0.0, 1.0),
    )
}

/// Convert DH Z-up position to render Y-up position.
fn to_render(pos: &nalgebra::Vector3<f64>) -> Vec3 {
    Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32)
}

/// Camera mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CameraMode {
    Orbit,
    Chase,
    Side,
}

impl CameraMode {
    fn next(self) -> Self {
        match self {
            Self::Orbit => Self::Chase,
            Self::Chase => Self::Side,
            Self::Side => Self::Orbit,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Orbit => "Orbit",
            Self::Chase => "Chase",
            Self::Side => "Side",
        }
    }
}

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

/// Application state.
struct App {
    window: Option<Arc<Window>>,
    render_ctx: Option<RenderContext>,
    pbr_pipeline: Option<PbrPipeline>,
    shadow_pipeline: Option<ShadowPipeline>,
    ssao_pipeline: Option<SsaoPipeline>,
    sss_pipeline: Option<SssPipeline>,
    composite_pipeline: Option<CompositePipeline>,
    skybox_pipeline: Option<SkyboxPipeline>,
    // egui
    egui_ctx: egui::Context,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
    // Camera
    camera: OrbitCamera,
    camera_mode: CameraMode,
    chase_cam_pos: Vec3,
    // Flight state
    flight: FlightState,
    guidance: Guidance,
    // Timing
    last_frame: Instant,
    accumulator: f64,
    frame_count: u64,
    fps_timer: Instant,
    fps: f64,
    sim_time: f64,
    time_scale: f64,
    paused: bool,
    // Input
    mouse_pressed: bool,
    middle_pressed: bool,
    last_mouse_pos: Option<(f64, f64)>,
    // Skybox exposure
    sky_exposure: f32,
    // GPU meshes — terrain (single dynamic buffer for all tiles)
    terrain_vb: Option<wgpu::Buffer>,
    terrain_ib: Option<wgpu::Buffer>,
    terrain_num_indices: u32,
    terrain_material: Option<MaterialBind>,
    last_terrain_snap: (i32, i32), // cached snap position to avoid regen every frame
    building_mesh: Option<GpuMesh>,
    building_material: Option<MaterialBind>,
    rail_mesh: Option<GpuMesh>,
    rail_material: Option<MaterialBind>,
    ref_building_meshes: Vec<GpuMesh>,
    ref_building_materials: Vec<MaterialBind>,
    // GPU meshes — drone
    drone_mesh: Option<GpuMesh>,
    drone_material: Option<MaterialBind>,
    prop_mesh: Option<GpuMesh>,
    prop_material: Option<MaterialBind>,
    // Target bullseye
    target_outer_mesh: Option<GpuMesh>,
    target_outer_material: Option<MaterialBind>,
    target_inner_mesh: Option<GpuMesh>,
    target_inner_material: Option<MaterialBind>,
    // Trail
    trail_pipeline: Option<LinePipeline>,
    trail_points: Vec<(Vec3, [f32; 4])>,
    trail_distance_accum: f64,
    // Audio — persistent voices wrapped in SharedVoice for per-frame updates
    audio_engine: AudioEngine,
    audio_started: bool,
    engine_voice: Option<Arc<Mutex<sound::EngineVoice>>>,
    wind_voice: Option<Arc<Mutex<sound::WindVoice>>>,
    // Wind
    wind: nalgebra::Vector3<f64>,
    // Radar
    show_radar: bool,
    // Post-processing
    post_sampler: Option<wgpu::Sampler>,
    depth_sampler: Option<wgpu::Sampler>,
    ssao_bind_group: Option<wgpu::BindGroup>,
    sss_bind_group: Option<wgpu::BindGroup>,
    composite_bind_group: Option<wgpu::BindGroup>,
}

impl App {
    fn new() -> Self {
        let mut camera = OrbitCamera::new();
        camera.target = Vec3::new(0.0, 2.0, 0.0);
        camera.distance = 25.0;
        camera.yaw = -0.3;
        camera.pitch = 0.15;
        camera.far = 100_000.0; // 100km far plane for flight sim
        camera.near = 0.5;

        Self {
            window: None,
            render_ctx: None,
            pbr_pipeline: None,
            shadow_pipeline: None,
            ssao_pipeline: None,
            sss_pipeline: None,
            composite_pipeline: None,
            skybox_pipeline: None,
            egui_ctx: egui::Context::default(),
            egui_state: None,
            egui_renderer: None,
            camera,
            camera_mode: CameraMode::Orbit,
            chase_cam_pos: Vec3::new(-20.0, 10.0, 0.0),
            flight: FlightState::new(),
            guidance: Guidance::new(),
            last_frame: Instant::now(),
            accumulator: 0.0,
            frame_count: 0,
            fps_timer: Instant::now(),
            fps: 0.0,
            sim_time: 0.0,
            time_scale: 1.0,
            paused: false,
            mouse_pressed: false,
            middle_pressed: false,
            last_mouse_pos: None,
            sky_exposure: 0.4,
            terrain_vb: None,
            terrain_ib: None,
            terrain_num_indices: 0,
            terrain_material: None,
            last_terrain_snap: (i32::MAX, i32::MAX),
            building_mesh: None,
            building_material: None,
            rail_mesh: None,
            rail_material: None,
            ref_building_meshes: Vec::new(),
            ref_building_materials: Vec::new(),
            drone_mesh: None,
            drone_material: None,
            prop_mesh: None,
            prop_material: None,
            target_outer_mesh: None,
            target_outer_material: None,
            target_inner_mesh: None,
            target_inner_material: None,
            trail_pipeline: None,
            trail_points: Vec::new(),
            trail_distance_accum: 0.0,
            audio_engine: AudioEngine::new(),
            audio_started: false,
            engine_voice: None,
            wind_voice: None,
            wind: nalgebra::Vector3::new(8.0, 0.0, 0.0), // 8 m/s tailwind
            show_radar: true,
            post_sampler: None,
            depth_sampler: None,
            ssao_bind_group: None,
            sss_bind_group: None,
            composite_bind_group: None,
        }
    }

    /// Reset everything for a new mission.
    fn reset(&mut self) {
        self.flight = FlightState::new();
        self.guidance = Guidance::new();
        self.sim_time = 0.0;
        self.accumulator = 0.0;
        self.trail_points.clear();
        self.trail_distance_accum = 0.0;
        self.time_scale = 1.0;
        self.paused = false;
        // Clear audio voices and drop refs so they can be recreated on next launch
        self.audio_engine.clear_voices();
        self.audio_started = false;
        self.engine_voice = None;
        self.wind_voice = None;
        self.last_terrain_snap = (i32::MAX, i32::MAX);
        // Reset camera to launch view
        self.camera.target = Vec3::new(0.0, 2.0, 0.0);
        self.camera.distance = 25.0;
        self.camera.yaw = -0.3;
        self.camera.pitch = 0.15;
        self.camera_mode = CameraMode::Orbit;
    }

    /// Update camera based on current mode and drone position.
    fn update_camera(&mut self) {
        let drone_render = to_render(&self.flight.position);

        match self.camera_mode {
            CameraMode::Orbit => {
                self.camera.target = drone_render;
            }
            CameraMode::Chase => {
                let fwd = self.flight.forward_dir();
                let fwd_render = Vec3::new(fwd.x as f32, fwd.z as f32, -fwd.y as f32);
                let ideal_pos = drone_render - fwd_render * 30.0 + Vec3::Y * 10.0;
                let lerp = 0.03_f32;
                self.chase_cam_pos += (ideal_pos - self.chase_cam_pos) * lerp;
                self.camera.target = drone_render;
                let diff = self.chase_cam_pos - drone_render;
                self.camera.distance = diff.length();
                if self.camera.distance > 0.1 {
                    self.camera.yaw = diff.z.atan2(diff.x);
                    self.camera.pitch = (diff.y / self.camera.distance).asin();
                }
            }
            CameraMode::Side => {
                let fwd = self.flight.forward_dir();
                let fwd_render = Vec3::new(fwd.x as f32, fwd.z as f32, -fwd.y as f32);
                let side = Vec3::new(-fwd_render.z, 0.0, fwd_render.x).normalize_or_zero();
                let ideal_pos = drone_render + side * 50.0 + Vec3::Y * 15.0;
                let lerp = 0.03_f32;
                self.chase_cam_pos += (ideal_pos - self.chase_cam_pos) * lerp;
                self.camera.target = drone_render;
                let diff = self.chase_cam_pos - drone_render;
                self.camera.distance = diff.length();
                if self.camera.distance > 0.1 {
                    self.camera.yaw = diff.z.atan2(diff.x);
                    self.camera.pitch = (diff.y / self.camera.distance).asin();
                }
            }
        }
    }

    /// Compute the drone model matrix.
    fn drone_model_matrix(&self) -> Mat4 {
        let pos = to_render(&self.flight.position);
        let heading = self.flight.heading as f32;
        let pitch = self.flight.pitch as f32;
        let bank = self.flight.bank as f32;
        let swap = coord_swap_matrix();
        let rot_heading = Mat4::from_rotation_z(heading);
        let rot_pitch = Mat4::from_rotation_y(-pitch);
        let rot_bank = Mat4::from_rotation_x(bank);
        let dh_rotation = rot_heading * rot_pitch * rot_bank;
        let render_rotation = swap * dh_rotation;
        let rot_quat = Quat::from_mat4(&render_rotation);
        Mat4::from_scale_rotation_translation(Vec3::ONE, rot_quat, pos)
    }

    /// Compute the prop model matrix.
    fn prop_model_matrix(&self) -> Mat4 {
        let drone_mat = self.drone_model_matrix();
        let prop_offset = Mat4::from_translation(Vec3::new(-1.75, 0.0, 0.0));
        let prop_spin = Mat4::from_rotation_x(self.flight.prop_angle as f32);
        drone_mat * prop_offset * prop_spin
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

        // Light — desert sunlight (toned down to avoid washout)
        let light_dir = Vec3::new(-0.3, -0.8, -0.5).normalize();
        let light = LightUniform {
            direction: [light_dir.x, light_dir.y, light_dir.z, 0.0],
            color: [1.0, 0.95, 0.85, 2.5],
            ambient: [0.45, 0.42, 0.35, 0.3],
            eye_pos: cam_uniform.eye_pos,
        };
        pbr.update_light(&ctx.queue, &light);

        // --- Model matrices ---

        // Terrain material (identity model — verts are in world space)
        if let Some(m) = &self.terrain_material {
            let mut mat = MaterialUniform::metal(terrain::SAND_COLOR)
                .with_model(Mat4::IDENTITY.to_cols_array_2d());
            mat.params = [0.95, 0.0, 0.0, 0.0]; // very rough desert sand
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Update skybox: compute inverse VP on CPU and upload with exposure
        if let Some(sky) = &self.skybox_pipeline {
            let view = self.camera.view_matrix();
            let proj = self.camera.projection_matrix(ctx.aspect());
            let vp = proj * view;
            let inv_vp = vp.inverse();
            sky.update(&ctx.queue, inv_vp, self.sky_exposure);
        }

        // Target building offset 15m beside the bullseye at (50000, 15, 1.5) in DH space
        let building_dh = Vec3::new(50_000.0, 15.0, 1.5);
        let building_render = swap.transform_point3(building_dh);
        let building_model = Mat4::from_translation(building_render);

        // Target bullseye at (50000, 0, 0) in DH space
        let target_dh = Vec3::new(50_000.0, 0.0, 0.0);
        let target_render = swap.transform_point3(target_dh);
        let target_model = Mat4::from_translation(target_render);

        // Launch rail
        let rail_render = swap.transform_point3(Vec3::new(2.5, 0.0, 1.0));
        let rail_rot = Quat::from_rotation_z(10.0_f32.to_radians());
        let rail_model = Mat4::from_rotation_translation(rail_rot, rail_render);

        // Reference buildings
        let ref_buildings = terrain::reference_buildings();
        let ref_models: Vec<Mat4> = ref_buildings.iter().map(|(x, _, hy, _)| {
            let dh = Vec3::new(*x as f32, 0.0, *hy);
            Mat4::from_translation(swap.transform_point3(dh))
        }).collect();

        // Drone
        let drone_model = self.drone_model_matrix();
        let prop_model = self.prop_model_matrix();

        // Upload non-tile materials
        if let Some(m) = &self.building_material {
            let mut mat = MaterialUniform::metal(terrain::BUILDING_COLOR)
                .with_model(building_model.to_cols_array_2d());
            mat.params = [0.8, 0.1, 0.0, 0.0];
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(m) = &self.rail_material {
            let mat = MaterialUniform::metal(terrain::RAIL_COLOR)
                .with_model(rail_model.to_cols_array_2d());
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }
        for (i, m) in self.ref_building_materials.iter().enumerate() {
            if i < ref_models.len() {
                let mut mat = MaterialUniform::metal(terrain::REF_BUILDING_COLOR)
                    .with_model(ref_models[i].to_cols_array_2d());
                mat.params = [0.85, 0.05, 0.0, 0.0];
                ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
            }
        }
        if let Some(m) = &self.drone_material {
            let mut mat = MaterialUniform::metal(drone::DRONE_COLOR)
                .with_model(drone_model.to_cols_array_2d());
            mat.params = [0.6, 0.2, 0.0, 0.0];
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(m) = &self.prop_material {
            let mat = MaterialUniform::metal(drone::PROP_COLOR)
                .with_model(prop_model.to_cols_array_2d());
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(m) = &self.target_outer_material {
            let mut mat = MaterialUniform::metal(terrain::TARGET_RED)
                .with_model(target_model.to_cols_array_2d());
            mat.params = [0.95, 0.0, 0.0, 0.0];
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }
        if let Some(m) = &self.target_inner_material {
            let mut mat = MaterialUniform::metal(terrain::TARGET_INNER)
                .with_model(target_model.to_cols_array_2d());
            mat.params = [0.95, 0.0, 0.0, 0.0];
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // --- Shadow setup (centered on drone for nearby detail) ---
        let drone_render = to_render(&self.flight.position);
        let scene_radius = 60.0_f32;
        let light_pos = drone_render - light_dir * scene_radius * 2.0;
        let shadow_view = Mat4::look_at_rh(light_pos, drone_render, Vec3::Y);
        let shadow_proj = Mat4::orthographic_rh(
            -scene_radius, scene_radius, -scene_radius, scene_radius,
            0.1, scene_radius * 4.0,
        );
        let light_vp = shadow_proj * shadow_view;
        pbr.update_shadow_light_vp(&ctx.queue, &light_vp);

        // Shadow matrices: terrain (identity), buildings, drone
        let mut shadow_matrices: Vec<Mat4> = Vec::with_capacity(16);
        shadow_matrices.push(light_vp * Mat4::IDENTITY); // terrain is in world space
        shadow_matrices.push(light_vp * building_model);
        shadow_matrices.push(light_vp * rail_model);
        for rm in &ref_models {
            if shadow_matrices.len() < 28 {
                shadow_matrices.push(light_vp * *rm);
            }
        }
        shadow_matrices.push(light_vp * target_model); // outer ring
        shadow_matrices.push(light_vp * target_model); // inner disc
        shadow_matrices.push(light_vp * drone_model);
        shadow_matrices.push(light_vp * prop_model);

        if let Some(shadow) = &self.shadow_pipeline {
            shadow.upload_matrices(&ctx.queue, &shadow_matrices);
        }

        // SSAO
        if let Some(ssao) = &self.ssao_pipeline {
            ssao.update_params(&ctx.queue, &SsaoParams {
                proj: cam_uniform.proj,
                radius: 0.5,
                bias: 0.025,
                intensity: 1.5,
                _pad: 0.0,
            });
        }

        // --- Begin rendering ---
        let output = match ctx.surface.get_current_texture() {
            Ok(t) => t,
            Err(_) => return,
        };
        let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = ctx.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });

        // Pass 1: HDR Skybox (replaces flat sky clear)
        if let Some(sky) = &self.skybox_pipeline {
            sky.render(&mut encoder, &view);
        } else {
            // Fallback flat sky
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Sky Clear"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.53, g: 0.71, b: 0.90, a: 1.0 }),
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

            // Shadow for terrain
            if let (Some(vb), Some(ib)) = (&self.terrain_vb, &self.terrain_ib) {
                if si < shadow_matrices.len() && self.terrain_num_indices > 0 {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, vb.slice(..));
                    pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..self.terrain_num_indices, 0, 0..1);
                }
            }
            si += 1;

            // Building
            if let Some(mesh) = &self.building_mesh {
                if si < shadow_matrices.len() {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }
            si += 1;

            // Rail
            if let Some(mesh) = &self.rail_mesh {
                if si < shadow_matrices.len() {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }
            si += 1;

            // Ref buildings
            for mesh in &self.ref_building_meshes {
                if si < shadow_matrices.len() {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
                si += 1;
            }

            // Target outer ring
            if let Some(mesh) = &self.target_outer_mesh {
                if si < shadow_matrices.len() {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }
            si += 1;

            // Target inner disc
            if let Some(mesh) = &self.target_inner_mesh {
                if si < shadow_matrices.len() {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }
            si += 1;

            // Drone
            if let Some(mesh) = &self.drone_mesh {
                if si < shadow_matrices.len() {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }
            si += 1;

            // Prop
            if let Some(mesh) = &self.prop_mesh {
                if si < shadow_matrices.len() {
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

            // Terrain (single draw call for all tiles)
            if let (Some(vb), Some(ib), Some(mat)) = (&self.terrain_vb, &self.terrain_ib, &self.terrain_material) {
                if self.terrain_num_indices > 0 {
                    pass.set_bind_group(0, &mat.bind_group, &[]);
                    pass.set_vertex_buffer(0, vb.slice(..));
                    pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..self.terrain_num_indices, 0, 0..1);
                }
            }

            // Buildings
            draw_mesh!(&self.building_mesh, &self.building_material);
            draw_mesh!(&self.rail_mesh, &self.rail_material);
            for (mesh, mat) in self.ref_building_meshes.iter().zip(self.ref_building_materials.iter()) {
                pass.set_bind_group(0, &mat.bind_group, &[]);
                pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
            }

            // Target bullseye
            draw_mesh!(&self.target_outer_mesh, &self.target_outer_material);
            draw_mesh!(&self.target_inner_mesh, &self.target_inner_material);

            // Drone
            draw_mesh!(&self.drone_mesh, &self.drone_material);
            draw_mesh!(&self.prop_mesh, &self.prop_material);
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

        // Pass 5: SSS
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

        // Pass 6: Composite
        if let (Some(composite), Some(comp_bg)) = (&self.composite_pipeline, &self.composite_bind_group) {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Composite Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load, // preserve skybox
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

        // Pass 6b: Trail lines
        if let Some(trail) = &mut self.trail_pipeline {
            if self.trail_points.len() >= 2 {
                let n = self.trail_points.len();
                let mut verts = Vec::with_capacity(n * 2);
                for i in 0..n - 1 {
                    verts.push(LineVertex { position: self.trail_points[i].0.into(), color: self.trail_points[i].1 });
                    verts.push(LineVertex { position: self.trail_points[i + 1].0.into(), color: self.trail_points[i + 1].1 });
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
                        ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &ctx.depth_texture,
                        depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store }),
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

        // Pass 7: egui HUD
        let egui_input = self.egui_state.as_mut().unwrap()
            .take_egui_input(self.window.as_ref().unwrap());
        self.egui_ctx.begin_pass(egui_input);
        let mut new_exposure = self.sky_exposure;
        draw_hud(&self.egui_ctx, &self.flight, &self.guidance, self.time_scale, self.fps, self.camera_mode, &mut new_exposure, &self.wind);
        if self.show_radar {
            draw_radar(&self.egui_ctx, &self.flight, &self.guidance);
        }
        self.sky_exposure = new_exposure;
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
        let egui_cmd_bufs = egui_renderer.update_buffers(&ctx.device, &ctx.queue, &mut encoder, &egui_prims, &screen);
        {
            let pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("egui Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
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
            self.window.as_ref().unwrap(), egui_output.platform_output,
        );

        let mut cmd_bufs: Vec<wgpu::CommandBuffer> = egui_cmd_bufs;
        cmd_bufs.push(encoder.finish());
        ctx.queue.submit(cmd_bufs);
        output.present();
    }
}

// ── HUD ──────────────────────────────────────────────────────────────────────

fn draw_hud(
    ctx: &egui::Context,
    flight: &FlightState,
    guidance: &Guidance,
    time_scale: f64,
    fps: f64,
    camera_mode: CameraMode,
    sky_exposure: &mut f32,
    wind: &nalgebra::Vector3<f64>,
) {
    egui::Area::new(egui::Id::new("telemetry"))
        .fixed_pos(egui::pos2(10.0, 10.0))
        .show(ctx, |ui| {
            ui.visuals_mut().override_text_color = Some(egui::Color32::WHITE);

            let phase_color = guidance.phase.color();
            let ec = egui::Color32::from_rgba_unmultiplied(
                (phase_color[0] * 255.0) as u8, (phase_color[1] * 255.0) as u8,
                (phase_color[2] * 255.0) as u8, 255,
            );
            ui.label(egui::RichText::new(guidance.phase.label()).size(24.0).strong().color(ec));
            ui.add_space(4.0);
            ui.label(egui::RichText::new(format!("Speed: {:.0} km/h", flight.airspeed_kmh())).size(16.0));
            ui.label(egui::RichText::new(format!("Alt: {:.0} m AGL", flight.altitude())).size(16.0));
            ui.label(egui::RichText::new(format!("Hdg: {:.0}\u{00b0}", flight.heading_deg())).size(16.0));
            let dist = guidance.distance_to_target(&flight.position);
            let dist_label = if dist > 1000.0 { format!("Target: {:.1} km", dist / 1000.0) }
                else { format!("Target: {:.0} m", dist) };
            ui.label(egui::RichText::new(dist_label).size(16.0));
            let fuel_pct = flight.fuel_mass / flight::FUEL_INIT * 100.0;
            ui.label(egui::RichText::new(format!("Fuel: {:.1} kg ({:.0}%)", flight.fuel_mass, fuel_pct)).size(16.0));
            let wind_speed = wind.norm();
            let wind_dir = wind.y.atan2(wind.x).to_degrees().rem_euclid(360.0);
            ui.label(egui::RichText::new(format!("Wind: {:.0} m/s @ {:.0}\u{00b0}", wind_speed, wind_dir)).size(16.0).color(egui::Color32::from_rgb(140, 200, 255)));
            ui.label(egui::RichText::new(format!("T+{:.1}s", guidance.mission_time)).size(14.0).color(egui::Color32::LIGHT_GRAY));
            ui.add_space(4.0);
            ui.label(egui::RichText::new(format!("Time: {:.0}x", time_scale)).size(14.0).color(egui::Color32::YELLOW));
            ui.label(egui::RichText::new(format!("Cam: {}", camera_mode.label())).size(14.0).color(egui::Color32::LIGHT_GRAY));
            ui.label(egui::RichText::new(format!("{:.0} FPS", fps)).size(13.0).color(egui::Color32::LIGHT_GRAY));
            let lat = 33.5 + flight.position.y * 0.000009;
            let lon = 45.0 + flight.position.x * 0.000009;
            ui.label(egui::RichText::new(format!("GPS: {:.4}\u{00b0}N  {:.4}\u{00b0}E", lat, lon)).size(13.0).color(egui::Color32::from_rgb(100, 200, 100)));
        });

    if guidance.phase == FlightPhase::PreLaunch || guidance.phase == FlightPhase::Impact {
        egui::Area::new(egui::Id::new("instructions"))
            .fixed_pos(egui::pos2(10.0, ctx.screen_rect().height() - 100.0))
            .show(ctx, |ui| {
                if guidance.phase == FlightPhase::PreLaunch {
                    ui.label(egui::RichText::new("Press SPACE to launch").size(20.0).color(egui::Color32::YELLOW).strong());
                } else {
                    ui.label(egui::RichText::new("IMPACT - Mission Complete").size(20.0).color(egui::Color32::RED).strong());
                }
                ui.label(egui::RichText::new("R: Reset  |  1-5: Time scale  |  C: Camera  |  P: Pause").size(14.0).color(egui::Color32::LIGHT_GRAY));
            });
    }

    // Settings panel (top-right)
    egui::Window::new("Settings")
        .default_pos(egui::pos2(ctx.screen_rect().width() - 220.0, 10.0))
        .default_width(200.0)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Sky Exposure");
            ui.add(egui::Slider::new(sky_exposure, 0.05..=3.0).logarithmic(true).text(""));
        });
}

// ── Radar ─────────────────────────────────────────────────────────────────────

fn draw_radar(ctx: &egui::Context, flight: &FlightState, guidance: &Guidance) {
    egui::Window::new("Radar")
        .default_pos(egui::pos2(ctx.screen_rect().width() - 220.0, 80.0))
        .default_width(200.0)
        .resizable(false)
        .show(ctx, |ui| {
            let size = egui::vec2(180.0, 180.0);
            let (response, painter) = ui.allocate_painter(size, egui::Sense::hover());
            let rect = response.rect;
            let center = rect.center();
            let radius = rect.width().min(rect.height()) * 0.45;

            // Dark green background
            painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(10, 30, 10));

            // Range circles
            let dist_to_target = guidance.distance_to_target(&flight.position);
            let scale_km = if dist_to_target > 20_000.0 { 50.0 }
                else if dist_to_target > 5_000.0 { 10.0 }
                else { 2.0 };
            let ring_color = egui::Color32::from_rgba_unmultiplied(40, 100, 40, 120);
            for ring in 1..=3 {
                let r = radius * ring as f32 / 3.0;
                painter.circle_stroke(center, r, egui::Stroke::new(1.0, ring_color));
            }

            // Target at center (red dot)
            painter.circle_filled(center, 4.0, egui::Color32::from_rgb(200, 40, 40));

            // Drone position relative to target
            let dx = flight.position.x - guidance::TARGET_POS.x;
            let dy = flight.position.y - guidance::TARGET_POS.y;
            let max_range = scale_km * 1000.0; // meters
            let px = (dx / max_range) as f32 * radius;
            let py = -(dy / max_range) as f32 * radius; // flip Y for screen coords
            let drone_screen = egui::pos2(center.x + px, center.y + py);

            // Clamp drone dot to circle
            let d = ((drone_screen.x - center.x).powi(2) + (drone_screen.y - center.y).powi(2)).sqrt();
            let drone_pos = if d > radius {
                let scale = radius / d;
                egui::pos2(
                    center.x + (drone_screen.x - center.x) * scale,
                    center.y + (drone_screen.y - center.y) * scale,
                )
            } else {
                drone_screen
            };

            painter.circle_filled(drone_pos, 3.0, egui::Color32::from_rgb(40, 200, 40));

            // Range text
            let range_text = if dist_to_target > 1000.0 {
                format!("{:.1} km", dist_to_target / 1000.0)
            } else {
                format!("{:.0} m", dist_to_target)
            };
            painter.text(
                egui::pos2(center.x, rect.max.y - 8.0),
                egui::Align2::CENTER_BOTTOM,
                range_text,
                egui::FontId::proportional(12.0),
                egui::Color32::from_rgb(80, 180, 80),
            );

            // Scale label
            painter.text(
                egui::pos2(rect.min.x + 4.0, rect.min.y + 4.0),
                egui::Align2::LEFT_TOP,
                format!("{:.0} km", scale_km),
                egui::FontId::proportional(10.0),
                egui::Color32::from_rgb(60, 140, 60),
            );
        });
}

// ── ApplicationHandler ───────────────────────────────────────────────────────

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() { return; }

        let window = Arc::new(
            event_loop.create_window(
                Window::default_attributes()
                    .with_title("SimuForge \u{2014} Shahed-136 Flight Simulation")
                    .with_inner_size(winit::dpi::LogicalSize::new(1920, 1080))
                    .with_maximized(true),
            ).expect("Failed to create window"),
        );

        let ctx = pollster::block_on(RenderContext::new(window.clone()));
        let shadow = ShadowPipeline::new(&ctx);
        let pbr = PbrPipeline::new(&ctx, &shadow);
        let ssao = SsaoPipeline::new(&ctx);
        let sss = SssPipeline::new(&ctx);
        let composite = CompositePipeline::new(&ctx);

        // Skybox (self-contained — has its own uniform buffer for inv_vp + exposure)
        let skybox = SkyboxPipeline::new(&ctx, HDR_PATH);

        // egui
        let viewport_id = self.egui_ctx.viewport_id();
        let egui_state = egui_winit::State::new(self.egui_ctx.clone(), viewport_id, &window, None, None, None);
        let egui_renderer = egui_wgpu::Renderer::new(&ctx.device, ctx.format(), None, 1, false);
        self.egui_state = Some(egui_state);
        self.egui_renderer = Some(egui_renderer);

        // Samplers
        let post_sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Post Sampler"), mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear, ..Default::default()
        });
        let depth_sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Depth Sampler"), mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest, ..Default::default()
        });
        let ssao_bg = ssao.create_bind_group(&ctx.device, &ctx.depth_texture, &depth_sampler);
        let sss_bg = sss.create_bind_group(&ctx.device, &ctx.hdr_texture, &ctx.depth_texture, &post_sampler);
        let composite_bg = composite.create_bind_group(&ctx.device, &sss.output_view, &ssao.output_view, &post_sampler);

        let w = ctx.config.width as f32;
        let h = ctx.config.height as f32;
        ssao.update_params(&ctx.queue, &SsaoParams { proj: glam::Mat4::IDENTITY.to_cols_array_2d(), radius: 0.5, bias: 0.025, intensity: 1.5, _pad: 0.0 });
        // Disable SSS — marble scatter washes out desert sand
        let mut sss_params = SssParams::marble(w, h);
        sss_params.strength = 0.0;
        sss.update_params(&ctx.queue, &sss_params);
        composite.update_params(&ctx.queue, &CompositeParams::default());

        // --- GPU meshes ---
        use wgpu::util::DeviceExt;

        // Terrain: single large dynamic buffer for all tiles
        // Initial generation centered at origin
        let (tv, ti) = terrain::generate_all_tiles(0.0, 0.0);
        let terrain_vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Terrain VB"), contents: bytemuck::cast_slice(&tv),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });
        let terrain_ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Terrain IB"), contents: bytemuck::cast_slice(&ti),
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
        });
        self.terrain_num_indices = ti.len() as u32;
        self.terrain_vb = Some(terrain_vb);
        self.terrain_ib = Some(terrain_ib);
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.terrain_material = Some(MaterialBind { buffer: buf, bind_group: bg });
        self.last_terrain_snap = (0, 0);

        // Target building
        let (bv, bi) = terrain::generate_target_building();
        let bvb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Building VB"), contents: bytemuck::cast_slice(&bv), usage: wgpu::BufferUsages::VERTEX });
        let bib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Building IB"), contents: bytemuck::cast_slice(&bi), usage: wgpu::BufferUsages::INDEX });
        self.building_mesh = Some(GpuMesh { vertex_buffer: bvb, index_buffer: bib, num_indices: bi.len() as u32 });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.building_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Launch rail
        let (rv, ri) = terrain::generate_launch_rail();
        let rvb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Rail VB"), contents: bytemuck::cast_slice(&rv), usage: wgpu::BufferUsages::VERTEX });
        let rib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Rail IB"), contents: bytemuck::cast_slice(&ri), usage: wgpu::BufferUsages::INDEX });
        self.rail_mesh = Some(GpuMesh { vertex_buffer: rvb, index_buffer: rib, num_indices: ri.len() as u32 });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.rail_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Reference buildings
        self.ref_building_meshes.clear();
        self.ref_building_materials.clear();
        for (_, hx, hy, hz) in terrain::reference_buildings() {
            let (v, i) = terrain::generate_ref_building(hx, hy, hz);
            let vb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("RefBuilding VB"), contents: bytemuck::cast_slice(&v), usage: wgpu::BufferUsages::VERTEX });
            let ib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("RefBuilding IB"), contents: bytemuck::cast_slice(&i), usage: wgpu::BufferUsages::INDEX });
            self.ref_building_meshes.push(GpuMesh { vertex_buffer: vb, index_buffer: ib, num_indices: i.len() as u32 });
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.ref_building_materials.push(MaterialBind { buffer: buf, bind_group: bg });
        }

        // Drone
        let (dv, di) = drone::generate_drone_mesh();
        let dvb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Drone VB"), contents: bytemuck::cast_slice(&dv), usage: wgpu::BufferUsages::VERTEX });
        let dib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Drone IB"), contents: bytemuck::cast_slice(&di), usage: wgpu::BufferUsages::INDEX });
        self.drone_mesh = Some(GpuMesh { vertex_buffer: dvb, index_buffer: dib, num_indices: di.len() as u32 });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.drone_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Prop
        let (pv, pi) = drone::generate_prop_disc();
        let pvb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Prop VB"), contents: bytemuck::cast_slice(&pv), usage: wgpu::BufferUsages::VERTEX });
        let pib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Prop IB"), contents: bytemuck::cast_slice(&pi), usage: wgpu::BufferUsages::INDEX });
        self.prop_mesh = Some(GpuMesh { vertex_buffer: pvb, index_buffer: pib, num_indices: pi.len() as u32 });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.prop_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Target bullseye outer ring
        let (tov, toi) = terrain::generate_target_outer_ring();
        let tovb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("TargetOuter VB"), contents: bytemuck::cast_slice(&tov), usage: wgpu::BufferUsages::VERTEX });
        let toib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("TargetOuter IB"), contents: bytemuck::cast_slice(&toi), usage: wgpu::BufferUsages::INDEX });
        self.target_outer_mesh = Some(GpuMesh { vertex_buffer: tovb, index_buffer: toib, num_indices: toi.len() as u32 });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.target_outer_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Target bullseye inner disc
        let (tiv, tii) = terrain::generate_target_inner_disc();
        let tivb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("TargetInner VB"), contents: bytemuck::cast_slice(&tiv), usage: wgpu::BufferUsages::VERTEX });
        let tiib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("TargetInner IB"), contents: bytemuck::cast_slice(&tii), usage: wgpu::BufferUsages::INDEX });
        self.target_inner_mesh = Some(GpuMesh { vertex_buffer: tivb, index_buffer: tiib, num_indices: tii.len() as u32 });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.target_inner_material = Some(MaterialBind { buffer: buf, bind_group: bg });

        // Trail pipeline
        self.trail_pipeline = Some(LinePipeline::new(&ctx, &pbr.camera_buffer));

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
        self.skybox_pipeline = Some(skybox);

        self.render_ctx = Some(ctx);
        self.window = Some(window);
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _window_id: WindowId, event: WindowEvent) {
        if let Some(state) = &mut self.egui_state {
            let _ = state.on_window_event(self.window.as_ref().unwrap(), &event);
        }

        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(new_size) => {
                if let Some(ctx) = &mut self.render_ctx {
                    ctx.resize(new_size);
                    if let (Some(ssao), Some(ds)) = (&self.ssao_pipeline, &self.depth_sampler) {
                        self.ssao_bind_group = Some(ssao.create_bind_group(&ctx.device, &ctx.depth_texture, ds));
                    }
                    if let (Some(sss), Some(ps)) = (&self.sss_pipeline, &self.post_sampler) {
                        self.sss_bind_group = Some(sss.create_bind_group(&ctx.device, &ctx.hdr_texture, &ctx.depth_texture, ps));
                    }
                    if let (Some(composite), Some(sss), Some(ssao), Some(ps)) = (&self.composite_pipeline, &self.sss_pipeline, &self.ssao_pipeline, &self.post_sampler) {
                        self.composite_bind_group = Some(composite.create_bind_group(&ctx.device, &sss.output_view, &ssao.output_view, ps));
                    }
                    if let Some(sss) = &self.sss_pipeline {
                        let mut sp = SssParams::marble(ctx.config.width as f32, ctx.config.height as f32);
                        sp.strength = 0.0;
                        sss.update_params(&ctx.queue, &sp);
                    }
                }
            }
            WindowEvent::KeyboardInput { event: winit::event::KeyEvent { logical_key, state: ElementState::Pressed, .. }, .. } => {
                match logical_key {
                    Key::Named(NamedKey::Escape) => event_loop.exit(),
                    Key::Named(NamedKey::Space) => {
                        if self.guidance.phase == FlightPhase::PreLaunch {
                            self.guidance.launch();
                            // Start persistent audio voices via SharedVoice wrappers
                            if !self.audio_started {
                                let ev = Arc::new(Mutex::new(sound::EngineVoice::new()));
                                if let Ok(mut e) = ev.lock() { e.set_running(true); }
                                self.audio_engine.play(Box::new(sound::SharedVoice::new(ev.clone())));
                                self.engine_voice = Some(ev);

                                let wv = Arc::new(Mutex::new(sound::WindVoice::new()));
                                self.audio_engine.play(Box::new(sound::SharedVoice::new(wv.clone())));
                                self.wind_voice = Some(wv);

                                self.audio_engine.play(Box::new(sound::BoosterVoice::new(0.0)));
                                self.audio_started = true;
                            }
                        }
                    }
                    Key::Character(ref c) if c.as_str() == "r" => self.reset(),
                    Key::Character(ref c) if c.as_str() == "p" => self.paused = !self.paused,
                    Key::Character(ref c) if c.as_str() == "c" => self.camera_mode = self.camera_mode.next(),
                    Key::Character(ref c) if c.as_str() == "1" => self.time_scale = 1.0,
                    Key::Character(ref c) if c.as_str() == "2" => self.time_scale = 10.0,
                    Key::Character(ref c) if c.as_str() == "3" => self.time_scale = 50.0,
                    Key::Character(ref c) if c.as_str() == "4" => self.time_scale = 100.0,
                    Key::Character(ref c) if c.as_str() == "5" => self.time_scale = 200.0,
                    _ => {}
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                match button {
                    MouseButton::Left => self.mouse_pressed = state == ElementState::Pressed,
                    MouseButton::Middle => self.middle_pressed = state == ElementState::Pressed,
                    _ => {}
                }
                if state == ElementState::Released { self.last_mouse_pos = None; }
            }
            WindowEvent::CursorMoved { position, .. } => {
                if let Some((lx, ly)) = self.last_mouse_pos {
                    let dx = (position.x - lx) as f32;
                    let dy = (position.y - ly) as f32;
                    if self.mouse_pressed { self.camera.rotate(dx * 0.005, -dy * 0.005); }
                    if self.middle_pressed { self.camera.pan(-dx, dy); }
                }
                self.last_mouse_pos = Some((position.x, position.y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    winit::event::MouseScrollDelta::LineDelta(_, y) => y,
                    winit::event::MouseScrollDelta::PixelDelta(p) => p.y as f32 * 0.01,
                };
                // Override default zoom limits for flight sim
                self.camera.distance = (self.camera.distance * (1.0 - scroll * 0.1)).clamp(5.0, 500.0);
            }
            WindowEvent::RedrawRequested => {
                let now = Instant::now();
                let raw_dt = now.duration_since(self.last_frame).as_secs_f64();
                let frame_dt = raw_dt.min(MAX_FRAME_TIME);
                self.last_frame = now;

                let effective_scale = if self.guidance.should_auto_slow() {
                    self.time_scale.min(1.0)
                } else {
                    self.time_scale
                };

                // Physics loop
                if !self.paused && self.guidance.phase != FlightPhase::Impact
                    && self.guidance.phase != FlightPhase::PreLaunch
                {
                    self.accumulator += frame_dt * effective_scale;
                    let mut steps = 0u64;
                    while self.accumulator >= PHYSICS_DT && steps < MAX_STEPS_PER_FRAME {
                        let (thrust, pitch_cmd, bank_cmd) = self.guidance.update(&self.flight);
                        flight::step(&mut self.flight, thrust, pitch_cmd, bank_cmd, &self.wind);

                        if self.guidance.phase == FlightPhase::Impact {
                            self.flight.velocity = nalgebra::Vector3::zeros();
                            break;
                        }

                        self.accumulator -= PHYSICS_DT;
                        self.sim_time += PHYSICS_DT;
                        steps += 1;

                        // Trail
                        let speed = self.flight.airspeed();
                        self.trail_distance_accum += speed * PHYSICS_DT;
                        if self.trail_distance_accum >= 50.0 {
                            self.trail_distance_accum = 0.0;
                            self.trail_points.push((to_render(&self.flight.position), self.guidance.phase.trail_color()));
                            if self.trail_points.len() > 10_000 { self.trail_points.remove(0); }
                        }
                    }

                    // Update persistent voice params once per frame
                    let airspeed = self.flight.true_airspeed(&self.wind);
                    let cam_eye = self.camera.eye();
                    let drone_render = to_render(&self.flight.position);
                    let dist = (cam_eye - drone_render).length() as f64;
                    let pan = (drone_render.x - cam_eye.x).atan2((drone_render.z - cam_eye.z).abs() + 1.0);
                    if let Some(ev) = &self.engine_voice {
                        if let Ok(mut e) = ev.lock() {
                            e.update_params(airspeed, dist, pan.clamp(-1.0, 1.0));
                        }
                    }
                    if let Some(wv) = &self.wind_voice {
                        if let Ok(mut w) = wv.lock() {
                            w.update_params(airspeed, dist, pan.clamp(-1.0, 1.0));
                        }
                    }
                }

                // Camera
                self.update_camera();

                // Regenerate terrain tiles if camera moved to new snap position
                let cam = self.camera.eye();
                let snap_x = (cam.x / terrain::TILE_SIZE).round() as i32;
                let snap_z = (cam.z / terrain::TILE_SIZE).round() as i32;
                if (snap_x, snap_z) != self.last_terrain_snap {
                    self.last_terrain_snap = (snap_x, snap_z);
                    let (tv, ti) = terrain::generate_all_tiles(cam.x, cam.z);
                    if let (Some(ctx), Some(vb), Some(ib)) = (&self.render_ctx, &self.terrain_vb, &self.terrain_ib) {
                        // Check if buffers are large enough; if not, we'd need to recreate.
                        // Since NUM_TILES is constant, size is always the same.
                        ctx.queue.write_buffer(vb, 0, bytemuck::cast_slice(&tv));
                        ctx.queue.write_buffer(ib, 0, bytemuck::cast_slice(&ti));
                        self.terrain_num_indices = ti.len() as u32;
                    }
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
