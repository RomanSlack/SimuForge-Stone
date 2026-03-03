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
    Ground,
}

impl CameraMode {
    fn next(self) -> Self {
        match self {
            Self::Orbit => Self::Chase,
            Self::Chase => Self::Side,
            Self::Side => Self::Ground,
            Self::Ground => Self::Orbit,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Orbit => "Orbit",
            Self::Chase => "Chase",
            Self::Side => "Side",
            Self::Ground => "Ground",
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

/// Maximum fleet size.
const MAX_FLEET_SIZE: usize = 100;

/// Generate a drone body tint from index using golden-angle hue distribution.
fn fleet_tint(i: usize) -> [f32; 4] {
    let hue = (i as f32 * 137.508) % 360.0; // golden angle
    let s = 0.7_f32;
    let v = 0.8_f32;
    let (r, g, b) = hsv_to_rgb(hue, s, v);
    [r, g, b, 1.0]
}

/// Generate a trail color (brighter version of tint).
fn fleet_trail_color(i: usize) -> [f32; 4] {
    let hue = (i as f32 * 137.508) % 360.0;
    let s = 0.5_f32;
    let v = 1.0_f32;
    let (r, g, b) = hsv_to_rgb(hue, s, v);
    [r, g, b, 1.0]
}

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    let c = v * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;
    let (r, g, b) = if h < 60.0 { (c, x, 0.0) }
        else if h < 120.0 { (x, c, 0.0) }
        else if h < 180.0 { (0.0, c, x) }
        else if h < 240.0 { (0.0, x, c) }
        else if h < 300.0 { (x, 0.0, c) }
        else { (c, 0.0, x) };
    (r + m, g + m, b + m)
}

/// Per-drone instance state.
struct DroneInstance {
    flight: FlightState,
    guidance: Guidance,
    drone_material: Option<MaterialBind>,
    prop_material: Option<MaterialBind>,
    drone_outline_material: Option<MaterialBind>,
    trail_points: Vec<(Vec3, [f32; 4])>,
    trail_distance_accum: f64,
    minimap_trail: Vec<[f64; 2]>,
    trail_color: [f32; 4],
    drone_tint: [f32; 4],
    launch_pos: [f64; 2],
}

/// Fleet planning state.
struct FleetPlanState {
    active: bool,
    launch_positions: Vec<[f64; 2]>,
    /// Map center in DH coords for pan/zoom.
    map_center: [f64; 2],
    /// Half-range in meters (zoom level).
    map_half_range: f64,
}

/// Compute the drone model matrix from flight state.
fn drone_model_matrix_for(flight: &FlightState) -> Mat4 {
    let pos = to_render(&flight.position);
    let heading = flight.heading as f32;
    let pitch = flight.pitch as f32;
    let bank = flight.bank as f32;
    let swap = coord_swap_matrix();
    let rot_heading = Mat4::from_rotation_z(heading);
    let rot_pitch = Mat4::from_rotation_y(-pitch);
    let rot_bank = Mat4::from_rotation_x(bank);
    let dh_rotation = rot_heading * rot_pitch * rot_bank;
    let render_rotation = swap * dh_rotation;
    let rot_quat = Quat::from_mat4(&render_rotation);
    Mat4::from_scale_rotation_translation(Vec3::ONE, rot_quat, pos)
}

/// Compute the prop model matrix from flight state and drone model matrix.
fn prop_model_matrix_for(flight: &FlightState, drone_mat: &Mat4) -> Mat4 {
    let prop_offset = Mat4::from_translation(Vec3::new(-1.75, 0.0, 0.0));
    let prop_spin = Mat4::from_rotation_x(flight.prop_angle as f32);
    *drone_mat * prop_offset * prop_spin
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
    // Fleet
    drones: Vec<DroneInstance>,
    primary_drone: usize,
    fleet_plan: FleetPlanState,
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
    /// CPU-side terrain vertex cache for incremental updates.
    terrain_verts: Vec<simuforge_core::Vertex>,
    /// CPU-side terrain index cache for incremental updates.
    terrain_idxs: Vec<u32>,
    /// Queue of tile slots pending regeneration (slot_index, tile_cx, tile_cz).
    terrain_regen_queue: Vec<(usize, f32, f32)>,
    building_mesh: Option<GpuMesh>,
    building_material: Option<MaterialBind>,
    rail_mesh: Option<GpuMesh>,
    rail_material: Option<MaterialBind>,
    flag_mesh: Option<GpuMesh>,
    flag_material: Option<MaterialBind>,
    ref_building_meshes: Vec<GpuMesh>,
    ref_building_materials: Vec<MaterialBind>,
    // GPU meshes — drone (shared geometry, per-drone materials in DroneInstance)
    drone_mesh: Option<GpuMesh>,
    prop_mesh: Option<GpuMesh>,
    // Target bullseye
    target_outer_mesh: Option<GpuMesh>,
    target_outer_material: Option<MaterialBind>,
    target_inner_mesh: Option<GpuMesh>,
    target_inner_material: Option<MaterialBind>,
    // Rocks
    rock_mesh: Option<GpuMesh>,
    rock_material: Option<MaterialBind>,
    // Horizon dust
    horizon_dust: f32,
    // Ground camera
    ground_cam_pos: Vec3,
    ground_cam_yaw: f32,
    ground_cam_pitch: f32,
    ground_cam_fov: f32,
    // Trail
    trail_pipeline: Option<LinePipeline>,
    // Audio — persistent voices wrapped in SharedVoice for per-frame updates
    audio_engine: AudioEngine,
    audio_started: bool,
    engine_voice: Option<Arc<Mutex<sound::EngineVoice>>>,
    wind_voice: Option<Arc<Mutex<sound::WindVoice>>>,
    // Wind
    wind: nalgebra::Vector3<f64>,
    wind_speed: f64,
    wind_direction: f64,
    // Audio controls
    master_volume: f32,
    engine_muted: bool,
    // Thermal IR + YOLO
    thermal_mode: bool,
    yolo_tracking: bool,
    pending_fleet_launch: bool,
    // Fullscreen map
    fullscreen_map: bool,
    map_center: [f64; 2],
    map_half_range: f64,
    // Display toggles
    show_trails: bool,
    show_drone_outlines: bool,
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
            drones: vec![DroneInstance {
                flight: FlightState::new(),
                guidance: Guidance::new(),
                drone_material: None,
                prop_material: None,
                drone_outline_material: None,
                trail_points: Vec::new(),
                trail_distance_accum: 0.0,
                minimap_trail: Vec::new(),
                trail_color: fleet_trail_color(0),
                drone_tint: drone::DRONE_COLOR,
                launch_pos: [0.0, 0.0],
            }],
            primary_drone: 0,
            fleet_plan: FleetPlanState { active: false, launch_positions: Vec::new(), map_center: [25_000.0, 0.0], map_half_range: 28_000.0 },
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
            terrain_verts: Vec::new(),
            terrain_idxs: Vec::new(),
            terrain_regen_queue: Vec::new(),
            building_mesh: None,
            building_material: None,
            rail_mesh: None,
            rail_material: None,
            flag_mesh: None,
            flag_material: None,
            ref_building_meshes: Vec::new(),
            ref_building_materials: Vec::new(),
            drone_mesh: None,
            prop_mesh: None,
            ground_cam_pos: Vec3::new(0.0, 0.0, 0.0),
            ground_cam_yaw: 0.0,
            ground_cam_pitch: 0.2,
            ground_cam_fov: std::f32::consts::FRAC_PI_4, // 45° default
            target_outer_mesh: None,
            target_outer_material: None,
            target_inner_mesh: None,
            target_inner_material: None,
            rock_mesh: None,
            rock_material: None,
            horizon_dust: 0.5,
            trail_pipeline: None,
            audio_engine: AudioEngine::new(),
            audio_started: false,
            engine_voice: None,
            wind_voice: None,
            wind: nalgebra::Vector3::new(8.0, 0.0, 0.0), // 8 m/s tailwind
            wind_speed: 8.0,
            wind_direction: 0.0,
            master_volume: 0.5,
            engine_muted: false,
            thermal_mode: false,
            yolo_tracking: false,
            pending_fleet_launch: false,
            fullscreen_map: false,
            map_center: [25_000.0, 0.0],
            map_half_range: 28_000.0,
            show_trails: true,
            show_drone_outlines: true,
            post_sampler: None,
            depth_sampler: None,
            ssao_bind_group: None,
            sss_bind_group: None,
            composite_bind_group: None,
        }
    }

    /// Reset everything for a new mission.
    fn reset(&mut self) {
        // Keep materials from existing drones, just reset state
        for d in &mut self.drones {
            d.flight = FlightState::new();
            d.guidance = Guidance::new();
            d.trail_points.clear();
            d.trail_distance_accum = 0.0;
            d.minimap_trail.clear();
        }
        // Trim to single default drone
        self.drones.truncate(1);
        if let Some(d) = self.drones.first_mut() {
            d.drone_tint = drone::DRONE_COLOR;
            d.trail_color = fleet_trail_color(0);
            d.launch_pos = [0.0, 0.0];
        }
        self.primary_drone = 0;
        self.fleet_plan = FleetPlanState { active: false, launch_positions: Vec::new(), map_center: [25_000.0, 0.0], map_half_range: 28_000.0 };
        self.sim_time = 0.0;
        self.accumulator = 0.0;
        self.time_scale = 1.0;
        self.paused = false;
        self.audio_engine.clear_voices();
        self.audio_started = false;
        self.engine_voice = None;
        self.wind_voice = None;
        self.last_terrain_snap = (i32::MAX, i32::MAX);
        self.terrain_regen_queue.clear();
        self.horizon_dust = 0.5;
        self.camera.target = Vec3::new(0.0, 2.0, 0.0);
        self.camera.distance = 25.0;
        self.camera.yaw = -0.3;
        self.camera.pitch = 0.15;
        self.camera_mode = CameraMode::Orbit;
        self.ground_cam_pos = Vec3::ZERO;
        self.ground_cam_yaw = 0.0;
        self.ground_cam_pitch = 0.2;
        self.ground_cam_fov = std::f32::consts::FRAC_PI_4;
    }

    /// Update camera based on current mode and drone position.
    fn update_camera(&mut self) {
        let pi = self.primary_drone.min(self.drones.len().saturating_sub(1));
        let drone_render = to_render(&self.drones[pi].flight.position);

        // Restore default FOV when not in ground mode
        if self.camera_mode != CameraMode::Ground {
            self.camera.fov = std::f32::consts::FRAC_PI_4;
        }

        match self.camera_mode {
            CameraMode::Orbit => {
                self.camera.target = drone_render;
            }
            CameraMode::Chase => {
                let fwd = self.drones[pi].flight.forward_dir();
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
                let fwd = self.drones[pi].flight.forward_dir();
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
            CameraMode::Ground => {
                // Ground observer: first-person, always above terrain surface
                let ground_y = terrain::terrain_height(self.ground_cam_pos.x, self.ground_cam_pos.z);
                let eye_height = (ground_y + 1.7).max(1.7); // always at least 1.7m above sea level
                let eye_pos = Vec3::new(self.ground_cam_pos.x, eye_height, self.ground_cam_pos.z);
                // Look direction from yaw/pitch
                let look_dir = Vec3::new(
                    self.ground_cam_yaw.cos() * self.ground_cam_pitch.cos(),
                    self.ground_cam_pitch.sin(),
                    self.ground_cam_yaw.sin() * self.ground_cam_pitch.cos(),
                );
                self.camera.target = eye_pos + look_dir * 100.0;
                self.camera.distance = 100.0;
                self.camera.fov = self.ground_cam_fov;
                let diff = eye_pos - self.camera.target;
                self.camera.yaw = diff.z.atan2(diff.x);
                self.camera.pitch = (diff.y / self.camera.distance).asin();
            }
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
            mat.params = [0.95, 0.0, 0.0, 1.0]; // very rough desert sand, params.w=1 = terrain flag
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Update skybox: compute inverse VP on CPU and upload with exposure
        if let Some(sky) = &self.skybox_pipeline {
            let view = self.camera.view_matrix();
            let proj = self.camera.projection_matrix(ctx.aspect());
            let vp = proj * view;
            let inv_vp = vp.inverse();
            sky.update(&ctx.queue, inv_vp, self.sky_exposure, self.horizon_dust);
        }

        // Target building offset 15m beside the bullseye at (50000, 15, 1.5) in DH space
        let building_dh = Vec3::new(50_000.0, 15.0, 1.5);
        let building_render = swap.transform_point3(building_dh);
        let building_model = Mat4::from_translation(building_render);

        // Target bullseye — vertices are in world space (terrain-conforming)
        let target_model = Mat4::IDENTITY;

        // Launch rail
        let rail_render = swap.transform_point3(Vec3::new(2.5, 0.0, 1.0));
        let rail_rot = Quat::from_rotation_z(10.0_f32.to_radians());
        let rail_model = Mat4::from_rotation_translation(rail_rot, rail_render);

        // Flagpole: stands vertical beside the rail, base sunk below ground
        let flag_dh = Vec3::new(0.0, 2.0, -0.8);
        let flag_render = swap.transform_point3(flag_dh);
        let flag_model = Mat4::from_translation(flag_render);

        // Reference buildings
        let ref_buildings = terrain::reference_buildings();
        let ref_models: Vec<Mat4> = ref_buildings.iter().map(|(x, _, hy, _)| {
            let dh = Vec3::new(*x as f32, 0.0, *hy);
            Mat4::from_translation(swap.transform_point3(dh))
        }).collect();

        // Compute per-drone model matrices
        let pi = self.primary_drone.min(self.drones.len().saturating_sub(1));
        let mut drone_models: Vec<Mat4> = Vec::with_capacity(self.drones.len());
        let mut prop_models: Vec<Mat4> = Vec::with_capacity(self.drones.len());
        for d in &self.drones {
            let dm = drone_model_matrix_for(&d.flight);
            let pm = prop_model_matrix_for(&d.flight, &dm);
            drone_models.push(dm);
            prop_models.push(pm);
        }

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
        if let Some(m) = &self.flag_material {
            let mut mat = MaterialUniform::metal(terrain::FLAG_COLOR)
                .with_model(flag_model.to_cols_array_2d());
            mat.params = [0.9, 0.0, 0.0, 0.0]; // rough cloth, non-metallic
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
        // Upload per-drone materials
        let thermal_emission = if self.thermal_mode { 1.0_f32 } else { 0.0 };
        let prop_thermal = if self.thermal_mode { 1.5_f32 } else { 0.0 };
        for (i, d) in self.drones.iter().enumerate() {
            if let Some(m) = &d.drone_material {
                let mut mat = MaterialUniform::metal(d.drone_tint)
                    .with_model(drone_models[i].to_cols_array_2d());
                mat.params = [0.6, 0.2, thermal_emission, 0.0];
                ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
            }
            if self.camera_mode == CameraMode::Ground {
                if let Some(m) = &d.drone_outline_material {
                    let outline_model = drone_models[i] * Mat4::from_scale(Vec3::splat(1.08));
                    let mut mat = MaterialUniform::metal([1.0, 0.1, 0.1, 1.0])
                        .with_model(outline_model.to_cols_array_2d());
                    mat.params = [1.0, 0.0, 0.0, 0.0];
                    ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
                }
            }
            if let Some(m) = &d.prop_material {
                let mut mat = MaterialUniform::metal(drone::PROP_COLOR)
                    .with_model(prop_models[i].to_cols_array_2d());
                mat.params = [0.6, 0.2, prop_thermal, 0.0];
                ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
            }
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
        // Rocks (world-space vertices, identity model)
        if let Some(m) = &self.rock_material {
            let mut mat = MaterialUniform::metal(terrain::ROCK_COLOR)
                .with_model(Mat4::IDENTITY.to_cols_array_2d());
            mat.params = [0.95, 0.0, 0.0, 0.0]; // very rough, non-metallic
            ctx.queue.write_buffer(&m.buffer, 0, bytemuck::bytes_of(&mat));
        }

        // Composite params (thermal mode)
        if let Some(composite) = &self.composite_pipeline {
            let cp = CompositeParams {
                thermal_mode: if self.thermal_mode { 1.0 } else { 0.0 },
                ..CompositeParams::default()
            };
            composite.update_params(&ctx.queue, &cp);
        }

        // --- Shadow setup (centered on primary drone, radius scales with camera distance) ---
        let primary_render = to_render(&self.drones[pi].flight.position);
        let cam_dist = (self.camera.eye() - primary_render).length();
        let scene_radius = cam_dist.clamp(60.0, 800.0);
        let light_pos = primary_render - light_dir * scene_radius * 2.0;
        let shadow_view = Mat4::look_at_rh(light_pos, primary_render, Vec3::Y);
        let shadow_proj = Mat4::orthographic_rh(
            -scene_radius, scene_radius, -scene_radius, scene_radius,
            0.1, scene_radius * 4.0,
        );
        let light_vp = shadow_proj * shadow_view;
        pbr.update_shadow_light_vp(&ctx.queue, &light_vp);

        // Shadow matrices: terrain (identity), buildings, all drones
        let mut shadow_matrices: Vec<Mat4> = Vec::with_capacity(32);
        shadow_matrices.push(light_vp * Mat4::IDENTITY); // terrain
        shadow_matrices.push(light_vp * building_model);
        shadow_matrices.push(light_vp * rail_model);
        shadow_matrices.push(light_vp * flag_model);
        for rm in &ref_models {
            if shadow_matrices.len() < 28 {
                shadow_matrices.push(light_vp * *rm);
            }
        }
        shadow_matrices.push(light_vp * target_model); // outer ring
        shadow_matrices.push(light_vp * target_model); // inner disc
        shadow_matrices.push(light_vp * Mat4::IDENTITY); // rocks
        // Shadow only nearest drones (limited by MAX_SHADOW_OBJECTS=32)
        let max_shadow_drones = ((32 - shadow_matrices.len()) / 2).min(self.drones.len());
        // Sort drone indices by distance to camera for shadow priority
        let mut drone_shadow_order: Vec<usize> = (0..self.drones.len()).collect();
        drone_shadow_order.sort_by(|&a, &b| {
            let da = (to_render(&self.drones[a].flight.position) - primary_render).length();
            let db = (to_render(&self.drones[b].flight.position) - primary_render).length();
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        });
        let shadow_drone_set: Vec<usize> = drone_shadow_order.into_iter().take(max_shadow_drones).collect();
        for &i in &shadow_drone_set {
            shadow_matrices.push(light_vp * drone_models[i]);
            shadow_matrices.push(light_vp * prop_models[i]);
        }

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

            // Flag
            if let Some(mesh) = &self.flag_mesh {
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

            // Rocks
            if let Some(mesh) = &self.rock_mesh {
                if si < shadow_matrices.len() {
                    pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                }
            }
            si += 1;

            // Shadowed drones + props (only nearest few)
            for _di in 0..shadow_drone_set.len() {
                if let Some(mesh) = &self.drone_mesh {
                    if si < shadow_matrices.len() {
                        pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                        pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                        pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                    }
                }
                si += 1;
                if let Some(mesh) = &self.prop_mesh {
                    if si < shadow_matrices.len() {
                        pass.set_bind_group(0, &shadow.bind_group, &[ShadowPipeline::dynamic_offset(si)]);
                        pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                        pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
                    }
                }
                si += 1;
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
            draw_mesh!(&self.flag_mesh, &self.flag_material);
            for (mesh, mat) in self.ref_building_meshes.iter().zip(self.ref_building_materials.iter()) {
                pass.set_bind_group(0, &mat.bind_group, &[]);
                pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.num_indices, 0, 0..1);
            }

            // Target bullseye
            draw_mesh!(&self.target_outer_mesh, &self.target_outer_material);
            draw_mesh!(&self.target_inner_mesh, &self.target_inner_material);

            // Rocks
            draw_mesh!(&self.rock_mesh, &self.rock_material);

            // All drones
            for d in &self.drones {
                if self.camera_mode == CameraMode::Ground && self.show_drone_outlines {
                    draw_mesh!(&self.drone_mesh, &d.drone_outline_material);
                }
                draw_mesh!(&self.drone_mesh, &d.drone_material);
                draw_mesh!(&self.prop_mesh, &d.prop_material);
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

        // Pass 6b: Trail lines (combined from all drones)
        if let Some(trail) = &mut self.trail_pipeline {
            let mut verts: Vec<LineVertex> = Vec::new();
            if self.show_trails {
                for d in &self.drones {
                    let n = d.trail_points.len();
                    if n >= 2 {
                        for i in 0..n - 1 {
                            verts.push(LineVertex { position: d.trail_points[i].0.into(), color: d.trail_points[i].1 });
                            verts.push(LineVertex { position: d.trail_points[i + 1].0.into(), color: d.trail_points[i + 1].1 });
                        }
                        let drone_pos = to_render(&d.flight.position);
                        if let Some(last) = d.trail_points.last() {
                            verts.push(LineVertex { position: last.0.into(), color: last.1 });
                            verts.push(LineVertex { position: drone_pos.into(), color: d.trail_color });
                        }
                    }
                }
            }
            if !verts.is_empty() {
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
        let mut new_horizon_dust = self.horizon_dust;
        let mut new_wind_speed = self.wind_speed;
        let mut new_wind_dir = self.wind_direction;
        let mut new_master_vol = self.master_volume;
        let mut new_engine_muted = self.engine_muted;
        let mut cam_mode = self.camera_mode;
        let mut thermal = self.thermal_mode;
        let mut yolo = self.yolo_tracking;
        let mut trails = self.show_trails;
        let mut outlines = self.show_drone_outlines;
        let primary_flight = &self.drones[pi].flight;
        let primary_guidance = &self.drones[pi].guidance;
        draw_mission_control(
            &self.egui_ctx, primary_flight, primary_guidance,
            self.time_scale, self.fps, &mut cam_mode,
            &mut new_exposure, &mut new_horizon_dust, &self.wind,
            &mut new_wind_speed, &mut new_wind_dir,
            &mut new_master_vol, &mut new_engine_muted,
            &mut thermal, &mut yolo, &mut trails, &mut outlines,
            self.drones.len(), pi,
        );
        self.camera_mode = cam_mode;
        self.thermal_mode = thermal;
        self.yolo_tracking = yolo;
        self.show_trails = trails;
        self.show_drone_outlines = outlines;
        let minimap_click = draw_minimap(
            &self.egui_ctx, &self.drones,
            &self.wind, self.camera_mode, &self.ground_cam_pos, self.ground_cam_yaw,
            pi,
        );
        if let Some([dh_x, dh_y]) = minimap_click {
            self.ground_cam_pos = Vec3::new(dh_x as f32, 0.0, -(dh_y as f32));
            self.ground_cam_yaw = 0.0;
            self.ground_cam_pitch = 0.2;
            self.camera_mode = CameraMode::Ground;
        }

        // Fullscreen map overlay (M key)
        if self.fullscreen_map {
            let fs_click = draw_fullscreen_map(
                &self.egui_ctx, &self.drones,
                &self.wind, self.camera_mode, &self.ground_cam_pos, self.ground_cam_yaw,
                pi, &mut self.map_center, &mut self.map_half_range,
                self.show_trails,
            );
            if let Some([dh_x, dh_y]) = fs_click {
                self.ground_cam_pos = Vec3::new(dh_x as f32, 0.0, -(dh_y as f32));
                self.ground_cam_yaw = 0.0;
                self.ground_cam_pitch = 0.2;
                self.camera_mode = CameraMode::Ground;
            }
        }

        self.sky_exposure = new_exposure;
        self.horizon_dust = new_horizon_dust;
        self.wind_speed = new_wind_speed;
        self.wind_direction = new_wind_dir;
        self.master_volume = new_master_vol;
        self.engine_muted = new_engine_muted;

        // Fleet planner window
        let mut fleet_launch_clicked = false;
        if self.drones[pi].guidance.phase == FlightPhase::PreLaunch {
            draw_fleet_planner(&self.egui_ctx, &mut self.fleet_plan, &mut fleet_launch_clicked);
        }
        if fleet_launch_clicked {
            self.pending_fleet_launch = true;
        }

        // YOLO bounding box overlay (Ground mode only)
        if self.yolo_tracking && self.camera_mode == CameraMode::Ground {
            let view_mat = self.camera.view_matrix();
            let proj_mat = self.camera.projection_matrix(ctx.aspect());
            let vp = proj_mat * view_mat;
            let screen_w = ctx.config.width as f32;
            let screen_h = ctx.config.height as f32;
            draw_yolo_overlay(&self.egui_ctx, &self.drones, &vp, screen_w, screen_h, self.camera.fov, pi);
        }

        // Thermal HUD overlay
        if self.thermal_mode && self.camera_mode == CameraMode::Ground {
            draw_thermal_hud(&self.egui_ctx, ctx.config.width as f32, ctx.config.height as f32);
        }

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

// ── Mission Control ──────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
fn draw_mission_control(
    ctx: &egui::Context,
    flight: &FlightState,
    guidance: &Guidance,
    time_scale: f64,
    fps: f64,
    camera_mode: &mut CameraMode,
    sky_exposure: &mut f32,
    horizon_dust: &mut f32,
    wind: &nalgebra::Vector3<f64>,
    wind_speed: &mut f64,
    wind_direction: &mut f64,
    master_volume: &mut f32,
    engine_muted: &mut bool,
    thermal_mode: &mut bool,
    yolo_tracking: &mut bool,
    show_trails: &mut bool,
    show_drone_outlines: &mut bool,
    fleet_size: usize,
    primary_idx: usize,
) {
    egui::Window::new("Mission Control")
        .default_pos(egui::pos2(10.0, 10.0))
        .default_width(240.0)
        .resizable(false)
        .show(ctx, |ui| {
            ui.visuals_mut().override_text_color = Some(egui::Color32::WHITE);

            // ── Telemetry ──
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
            let w_speed = wind.norm();
            let w_dir = wind.y.atan2(wind.x).to_degrees().rem_euclid(360.0);
            ui.label(egui::RichText::new(format!("Wind: {:.0} m/s @ {:.0}\u{00b0}", w_speed, w_dir)).size(16.0).color(egui::Color32::from_rgb(140, 200, 255)));
            ui.label(egui::RichText::new(format!("T+{:.1}s", guidance.mission_time)).size(14.0).color(egui::Color32::LIGHT_GRAY));
            ui.add_space(2.0);
            ui.label(egui::RichText::new(format!("Time: {:.0}x", time_scale)).size(14.0).color(egui::Color32::YELLOW));
            ui.label(egui::RichText::new(format!("Cam: {}", camera_mode.label())).size(14.0).color(egui::Color32::LIGHT_GRAY));
            // Ground mode: Back to Drone button
            if *camera_mode == CameraMode::Ground {
                if ui.button(egui::RichText::new("Back to Drone").size(14.0).color(egui::Color32::YELLOW)).clicked() {
                    *camera_mode = CameraMode::Chase;
                }
            }
            ui.label(egui::RichText::new(format!("{:.0} FPS", fps)).size(13.0).color(egui::Color32::LIGHT_GRAY));
            let lat = 33.5 + flight.position.y * 0.000009;
            let lon = 45.0 + flight.position.x * 0.000009;
            ui.label(egui::RichText::new(format!("GPS: {:.4}\u{00b0}N  {:.4}\u{00b0}E", lat, lon)).size(13.0).color(egui::Color32::from_rgb(100, 200, 100)));

            // ── Settings ──
            ui.separator();
            ui.label(egui::RichText::new("Sky Exposure").size(13.0));
            ui.add(egui::Slider::new(sky_exposure, 0.05..=3.0).logarithmic(true).text(""));

            ui.label(egui::RichText::new("Horizon Dust").size(13.0));
            ui.add(egui::Slider::new(horizon_dust, 0.0..=1.0).text(""));

            ui.label(egui::RichText::new("Wind Speed (m/s)").size(13.0));
            let mut ws = *wind_speed as f32;
            if ui.add(egui::Slider::new(&mut ws, 0.0..=25.0).text("")).changed() {
                *wind_speed = ws as f64;
            }

            ui.label(egui::RichText::new("Wind Direction (\u{00b0})").size(13.0));
            let mut wd = *wind_direction as f32;
            if ui.add(egui::Slider::new(&mut wd, 0.0..=359.0).text("")).changed() {
                *wind_direction = wd as f64;
            }

            ui.label(egui::RichText::new("Volume").size(13.0));
            ui.add(egui::Slider::new(master_volume, 0.0..=1.0).text(""));

            ui.checkbox(engine_muted, "Mute Engine");

            ui.separator();
            ui.checkbox(show_trails, "Contrails");
            ui.checkbox(show_drone_outlines, "Drone Outlines");

            // Fleet info
            if fleet_size > 1 {
                ui.separator();
                ui.label(egui::RichText::new(format!("Fleet: {}/{} | Primary: #{}", fleet_size, MAX_FLEET_SIZE, primary_idx + 1)).size(13.0).color(egui::Color32::from_rgb(200, 200, 100)));
                ui.label(egui::RichText::new("Tab: cycle drone").size(11.0).color(egui::Color32::LIGHT_GRAY));
            }

            // Ground mode sensors
            if *camera_mode == CameraMode::Ground {
                ui.separator();
                ui.checkbox(thermal_mode, "Thermal IR");
                ui.checkbox(yolo_tracking, "YOLO Tracking");
            }
        });

    // Instructions at bottom (PreLaunch/Impact only)
    if guidance.phase == FlightPhase::PreLaunch || guidance.phase == FlightPhase::Impact {
        egui::Area::new(egui::Id::new("instructions"))
            .fixed_pos(egui::pos2(10.0, ctx.screen_rect().height() - 100.0))
            .show(ctx, |ui| {
                if guidance.phase == FlightPhase::PreLaunch {
                    ui.label(egui::RichText::new("Press SPACE to launch").size(20.0).color(egui::Color32::YELLOW).strong());
                } else {
                    ui.label(egui::RichText::new("IMPACT - Mission Complete").size(20.0).color(egui::Color32::RED).strong());
                }
                ui.label(egui::RichText::new("R: Reset | 1-5: Time | C: Camera | P: Pause | Tab: Cycle | M: Map | Click map: Ground cam").size(14.0).color(egui::Color32::LIGHT_GRAY));
            });
    }
}

// ── GIS Minimap ──────────────────────────────────────────────────────────────

fn draw_minimap(
    ctx: &egui::Context,
    drones: &[DroneInstance],
    wind: &nalgebra::Vector3<f64>,
    camera_mode: CameraMode,
    ground_cam_pos: &Vec3,
    ground_cam_yaw: f32,
    primary_idx: usize,
) -> Option<[f64; 2]> {
    let flight = &drones[primary_idx].flight;
    let guidance = &drones[primary_idx].guidance;
    let mut clicked_pos: Option<[f64; 2]> = None;
    egui::Window::new("Map")
        .default_pos(egui::pos2(ctx.screen_rect().width() - 320.0, 10.0))
        .default_width(300.0)
        .resizable(false)
        .show(ctx, |ui| {
            let map_size = egui::vec2(280.0, 280.0);
            let (response, painter) = ui.allocate_painter(map_size, egui::Sense::click());
            let rect = response.rect;
            let center = rect.center();

            // Map projection: center (25000, 0), half-range 28km
            let map_cx = 25_000.0_f64;
            let map_cy = 0.0_f64;
            let half_range = 28_000.0_f64;
            let map_half = rect.width().min(rect.height()) * 0.5;

            let to_screen = |dh_x: f64, dh_y: f64| -> egui::Pos2 {
                let sx = ((dh_x - map_cx) / half_range) as f32 * map_half + center.x;
                let sy = -((dh_y - map_cy) / half_range) as f32 * map_half + center.y;
                egui::pos2(sx, sy)
            };

            // Sand background
            painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(194, 178, 128));

            // Grid lines every 10km
            let grid_color = egui::Color32::from_rgba_unmultiplied(139, 119, 80, 80);
            let grid_step = 10_000.0_f64;
            let grid_min_x = ((map_cx - half_range) / grid_step).ceil() as i32;
            let grid_max_x = ((map_cx + half_range) / grid_step).floor() as i32;
            let grid_min_y = ((map_cy - half_range) / grid_step).ceil() as i32;
            let grid_max_y = ((map_cy + half_range) / grid_step).floor() as i32;
            for gx in grid_min_x..=grid_max_x {
                let x = gx as f64 * grid_step;
                let p0 = to_screen(x, map_cy - half_range);
                let p1 = to_screen(x, map_cy + half_range);
                painter.line_segment([p0, p1], egui::Stroke::new(1.0, grid_color));
            }
            for gy in grid_min_y..=grid_max_y {
                let y = gy as f64 * grid_step;
                let p0 = to_screen(map_cx - half_range, y);
                let p1 = to_screen(map_cx + half_range, y);
                painter.line_segment([p0, p1], egui::Stroke::new(1.0, grid_color));
            }

            // Flight trails (per-drone color)
            for d in drones {
                let tc = d.trail_color;
                let trail_color = egui::Color32::from_rgb(
                    (tc[0] * 255.0) as u8, (tc[1] * 255.0) as u8, (tc[2] * 255.0) as u8,
                );
                let mt = &d.minimap_trail;
                if mt.len() >= 2 {
                    for i in 0..mt.len() - 1 {
                        let p0 = to_screen(mt[i][0], mt[i][1]);
                        let p1 = to_screen(mt[i + 1][0], mt[i + 1][1]);
                        painter.line_segment([p0, p1], egui::Stroke::new(1.5, trail_color));
                    }
                }
            }

            // Launch marker (green dot + label)
            let launch_pos = to_screen(0.0, 0.0);
            painter.circle_filled(launch_pos, 4.0, egui::Color32::from_rgb(40, 200, 40));
            painter.text(
                egui::pos2(launch_pos.x + 8.0, launch_pos.y),
                egui::Align2::LEFT_CENTER,
                "LAUNCH",
                egui::FontId::proportional(10.0),
                egui::Color32::from_rgb(40, 200, 40),
            );

            // Target marker (red X + label)
            let tgt_pos = to_screen(guidance::TARGET_POS.x, guidance::TARGET_POS.y);
            let x_size = 5.0_f32;
            let x_color = egui::Color32::from_rgb(220, 40, 40);
            painter.line_segment(
                [egui::pos2(tgt_pos.x - x_size, tgt_pos.y - x_size), egui::pos2(tgt_pos.x + x_size, tgt_pos.y + x_size)],
                egui::Stroke::new(2.0, x_color),
            );
            painter.line_segment(
                [egui::pos2(tgt_pos.x + x_size, tgt_pos.y - x_size), egui::pos2(tgt_pos.x - x_size, tgt_pos.y + x_size)],
                egui::Stroke::new(2.0, x_color),
            );
            painter.text(
                egui::pos2(tgt_pos.x + 8.0, tgt_pos.y),
                egui::Align2::LEFT_CENTER,
                "TGT",
                egui::FontId::proportional(10.0),
                x_color,
            );

            // Waypoints (yellow dots + labels)
            let wp_color = egui::Color32::from_rgb(60, 140, 255);
            // Skip last waypoint (it's the target)
            let num_wps = if guidance.waypoints.len() > 1 { guidance.waypoints.len() - 1 } else { 0 };
            for (i, wp) in guidance.waypoints.iter().take(num_wps).enumerate() {
                let wp_pos = to_screen(wp.x, wp.y);
                painter.circle_filled(wp_pos, 3.0, wp_color);
                painter.text(
                    egui::pos2(wp_pos.x + 6.0, wp_pos.y),
                    egui::Align2::LEFT_CENTER,
                    format!("W{}", i + 1),
                    egui::FontId::proportional(9.0),
                    wp_color,
                );
            }

            // Drone icons (all drones, primary is larger)
            for (di, d) in drones.iter().enumerate() {
                let dp = to_screen(d.flight.position.x, d.flight.position.y);
                let h = d.flight.heading as f32;
                let is_primary = di == primary_idx;
                let tri_size = if is_primary { 8.0_f32 } else { 5.0_f32 };
                let tc = d.trail_color;
                let tri_color = egui::Color32::from_rgb(
                    (tc[0] * 255.0) as u8, (tc[1] * 255.0) as u8, (tc[2] * 255.0) as u8,
                );
                let fwd_x = h.cos();
                let fwd_y = -h.sin();
                let tip = egui::pos2(dp.x + fwd_x * tri_size, dp.y + fwd_y * tri_size);
                let left = egui::pos2(
                    dp.x + (-fwd_x * 0.5 + fwd_y * 0.5) * tri_size,
                    dp.y + (-fwd_y * 0.5 - fwd_x * 0.5) * tri_size,
                );
                let right = egui::pos2(
                    dp.x + (-fwd_x * 0.5 - fwd_y * 0.5) * tri_size,
                    dp.y + (-fwd_y * 0.5 + fwd_x * 0.5) * tri_size,
                );
                painter.add(egui::Shape::convex_polygon(
                    vec![tip, left, right],
                    tri_color,
                    egui::Stroke::NONE,
                ));
                // Number label
                painter.text(
                    egui::pos2(dp.x + 10.0, dp.y),
                    egui::Align2::LEFT_CENTER,
                    format!("#{}", di + 1),
                    egui::FontId::proportional(if is_primary { 11.0 } else { 9.0 }),
                    tri_color,
                );
            }

            // Distance readout at bottom
            let dist_to_target = guidance.distance_to_target(&flight.position);
            let range_text = if dist_to_target > 1000.0 {
                format!("{:.1} km to target", dist_to_target / 1000.0)
            } else {
                format!("{:.0} m to target", dist_to_target)
            };
            painter.text(
                egui::pos2(center.x, rect.max.y - 6.0),
                egui::Align2::CENTER_BOTTOM,
                range_text,
                egui::FontId::proportional(12.0),
                egui::Color32::from_rgb(60, 60, 40),
            );

            // Scale bar at top-left (10km reference)
            let bar_start = to_screen(map_cx - half_range + 2000.0, map_cy + half_range - 2000.0);
            let bar_len_px = (10_000.0 / half_range) as f32 * map_half;
            let bar_end = egui::pos2(bar_start.x + bar_len_px, bar_start.y);
            let bar_color = egui::Color32::from_rgb(80, 70, 50);
            painter.line_segment([bar_start, bar_end], egui::Stroke::new(2.0, bar_color));
            // End ticks
            painter.line_segment(
                [egui::pos2(bar_start.x, bar_start.y - 3.0), egui::pos2(bar_start.x, bar_start.y + 3.0)],
                egui::Stroke::new(1.5, bar_color),
            );
            painter.line_segment(
                [egui::pos2(bar_end.x, bar_end.y - 3.0), egui::pos2(bar_end.x, bar_end.y + 3.0)],
                egui::Stroke::new(1.5, bar_color),
            );
            painter.text(
                egui::pos2((bar_start.x + bar_end.x) * 0.5, bar_start.y - 6.0),
                egui::Align2::CENTER_BOTTOM,
                "10 km",
                egui::FontId::proportional(10.0),
                bar_color,
            );

            // ── Compass rose (bottom-left corner) ──
            let compass_center = egui::pos2(rect.min.x + 30.0, rect.max.y - 30.0);
            let compass_r = 18.0_f32;
            painter.circle_stroke(compass_center, compass_r, egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(80, 70, 50, 120)));

            // Cardinal direction ticks and labels
            let directions = [
                ("N", 0.0_f32, -1.0_f32, egui::Color32::from_rgb(220, 40, 40)),
                ("E", 1.0, 0.0, egui::Color32::WHITE),
                ("S", 0.0, 1.0, egui::Color32::WHITE),
                ("W", -1.0, 0.0, egui::Color32::WHITE),
            ];
            for (label, dx, dy, color) in &directions {
                let tick_inner = egui::pos2(compass_center.x + dx * (compass_r - 4.0), compass_center.y + dy * (compass_r - 4.0));
                let tick_outer = egui::pos2(compass_center.x + dx * compass_r, compass_center.y + dy * compass_r);
                painter.line_segment([tick_inner, tick_outer], egui::Stroke::new(1.5, *color));
                let label_pos = egui::pos2(compass_center.x + dx * (compass_r + 8.0), compass_center.y + dy * (compass_r + 8.0));
                painter.text(label_pos, egui::Align2::CENTER_CENTER, *label, egui::FontId::proportional(9.0), *color);
            }

            // ── Wind direction arrow ──
            let wind_speed = wind.norm();
            if wind_speed > 0.5 {
                // Wind arrow shows direction wind blows TO (downwind)
                let wind_screen_x = wind.x as f32;  // east = right
                let wind_screen_y = -(wind.y as f32); // north = up on screen
                let wind_len = (wind_screen_x * wind_screen_x + wind_screen_y * wind_screen_y).sqrt();
                if wind_len > 0.01 {
                    let wx = wind_screen_x / wind_len;
                    let wy = wind_screen_y / wind_len;
                    let arrow_len = (wind_speed as f32 / 25.0 * 14.0).min(14.0).max(4.0);
                    let arrow_tip = egui::pos2(compass_center.x + wx * arrow_len, compass_center.y + wy * arrow_len);
                    let wind_color = egui::Color32::from_rgb(140, 200, 255);
                    painter.line_segment([compass_center, arrow_tip], egui::Stroke::new(2.0, wind_color));
                    // Arrowhead
                    let perp_x = -wy;
                    let perp_y = wx;
                    let head_size = 4.0_f32;
                    let head_back = egui::pos2(arrow_tip.x - wx * head_size, arrow_tip.y - wy * head_size);
                    let head_l = egui::pos2(head_back.x + perp_x * head_size * 0.5, head_back.y + perp_y * head_size * 0.5);
                    let head_r = egui::pos2(head_back.x - perp_x * head_size * 0.5, head_back.y - perp_y * head_size * 0.5);
                    painter.add(egui::Shape::convex_polygon(vec![arrow_tip, head_l, head_r], wind_color, egui::Stroke::NONE));
                }
            }

            // ── Ground camera position + FOV wedge (only in Ground mode) ──
            if camera_mode == CameraMode::Ground {
                // Convert render-space ground_cam_pos back to DH for minimap
                let dh_x = ground_cam_pos.x as f64;
                let dh_y = -(ground_cam_pos.z as f64);
                let cam_screen = to_screen(dh_x, dh_y);
                painter.circle_filled(cam_screen, 4.0, egui::Color32::from_rgb(255, 200, 40));
                painter.text(
                    egui::pos2(cam_screen.x + 8.0, cam_screen.y),
                    egui::Align2::LEFT_CENTER,
                    "CAM",
                    egui::FontId::proportional(9.0),
                    egui::Color32::from_rgb(255, 200, 40),
                );
                // FOV wedge lines (±30° from look direction)
                let wedge_len = 20.0_f32;
                let fov_half = 30.0_f32.to_radians();
                // ground_cam_yaw is in render space: need to convert to map angle
                // In render space: yaw 0 = +X (east), but the z axis is -DH_y
                // On map: east = +screen_x, north = -screen_y
                let map_angle = ground_cam_yaw; // render yaw matches map x-axis convention
                for sign in [-1.0_f32, 1.0] {
                    let angle = map_angle + sign * fov_half;
                    let dx = angle.cos();
                    let dy = angle.sin(); // render z → screen y is flipped
                    let end = egui::pos2(cam_screen.x + dx * wedge_len, cam_screen.y + dy * wedge_len);
                    painter.line_segment(
                        [cam_screen, end],
                        egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 200, 40, 100)),
                    );
                }
            }

            // ── Click to place ground camera ──
            if response.clicked() {
                if let Some(click_pos) = response.interact_pointer_pos() {
                    let dh_x = ((click_pos.x - center.x) / map_half) as f64 * half_range + map_cx;
                    let dh_y = -((click_pos.y - center.y) / map_half) as f64 * half_range + map_cy;
                    clicked_pos = Some([dh_x, dh_y]);
                }
            }
        });
    clicked_pos
}

// ── YOLO Bounding Box Overlay ────────────────────────────────────────────────

fn draw_yolo_overlay(
    ctx: &egui::Context,
    drones: &[DroneInstance],
    view_proj: &Mat4,
    screen_w: f32,
    screen_h: f32,
    fov: f32,
    primary_idx: usize,
) {
    egui::Area::new(egui::Id::new("yolo_overlay"))
        .fixed_pos(egui::pos2(0.0, 0.0))
        .interactable(false)
        .show(ctx, |ui| {
            let painter = ui.painter();
            for (di, d) in drones.iter().enumerate() {
                let world_pos = to_render(&d.flight.position);
                let clip = *view_proj * Vec4::new(world_pos.x, world_pos.y, world_pos.z, 1.0);
                if clip.w <= 0.0 { continue; }
                let ndc_x = clip.x / clip.w;
                let ndc_y = clip.y / clip.w;
                if ndc_x < -1.0 || ndc_x > 1.0 || ndc_y < -1.0 || ndc_y > 1.0 { continue; }
                let sx = (ndc_x + 1.0) * 0.5 * screen_w;
                let sy = (1.0 - ndc_y) * 0.5 * screen_h; // flip Y
                let distance = clip.w;
                let box_px = (3.5 / distance) * (screen_h / (2.0 * (fov / 2.0).tan()));
                let box_px = box_px.clamp(20.0, 400.0);
                let half = box_px * 0.5;
                let r = egui::Rect::from_center_size(
                    egui::pos2(sx, sy),
                    egui::vec2(box_px, box_px),
                );
                let color = egui::Color32::from_rgb(0, 255, 0);
                painter.rect_stroke(r, 0.0, egui::Stroke::new(2.0, color), egui::StrokeKind::Outside);
                let label = if di == primary_idx { format!("UAV-{} 87%", di + 1) } else { format!("UAV-{} 84%", di + 1) };
                painter.text(
                    egui::pos2(sx - half, sy - half - 16.0),
                    egui::Align2::LEFT_BOTTOM,
                    label,
                    egui::FontId::monospace(13.0),
                    color,
                );
            }
        });
}

// ── Thermal IR HUD ──────────────────────────────────────────────────────────

fn draw_thermal_hud(ctx: &egui::Context, screen_w: f32, screen_h: f32) {
    // FLIR watermark top-left
    egui::Area::new(egui::Id::new("flir_label"))
        .fixed_pos(egui::pos2(10.0, screen_h - 40.0))
        .interactable(false)
        .show(ctx, |ui| {
            ui.label(egui::RichText::new("FLIR").size(20.0).color(egui::Color32::from_rgba_unmultiplied(200, 200, 200, 180)).strong());
        });

    // Vertical color scale bar (right side)
    egui::Area::new(egui::Id::new("thermal_scale"))
        .fixed_pos(egui::pos2(screen_w - 40.0, screen_h * 0.3))
        .interactable(false)
        .show(ctx, |ui| {
            let bar_h = screen_h * 0.4;
            let bar_w = 15.0_f32;
            let (response, painter) = ui.allocate_painter(egui::vec2(bar_w + 30.0, bar_h + 20.0), egui::Sense::hover());
            let rect = response.rect;
            let steps = 32;
            let step_h = bar_h / steps as f32;
            for i in 0..steps {
                let t = 1.0 - i as f32 / steps as f32;
                let (r, g, b) = thermal_ramp_cpu(t);
                let color = egui::Color32::from_rgb(r, g, b);
                let y = rect.min.y + i as f32 * step_h;
                painter.rect_filled(
                    egui::Rect::from_min_size(egui::pos2(rect.min.x, y), egui::vec2(bar_w, step_h + 1.0)),
                    0.0,
                    color,
                );
            }
            // Labels
            painter.text(egui::pos2(rect.min.x + bar_w + 3.0, rect.min.y), egui::Align2::LEFT_TOP, "HOT", egui::FontId::proportional(10.0), egui::Color32::WHITE);
            painter.text(egui::pos2(rect.min.x + bar_w + 3.0, rect.min.y + bar_h), egui::Align2::LEFT_BOTTOM, "COLD", egui::FontId::proportional(10.0), egui::Color32::WHITE);
        });
}

/// CPU-side thermal ramp matching the WGSL shader.
fn thermal_ramp_cpu(t: f32) -> (u8, u8, u8) {
    let tc = t.clamp(0.0, 1.0);
    let (r, g, b) = if tc < 0.2 {
        let f = tc / 0.2;
        (0.0, 0.0, 0.8 * f)
    } else if tc < 0.4 {
        let f = (tc - 0.2) / 0.2;
        (0.6 * f, 0.0, 0.8)
    } else if tc < 0.6 {
        let f = (tc - 0.4) / 0.2;
        (0.6 + 0.4 * f, 0.0 * (1.0 - f), 0.8 * (1.0 - f))
    } else if tc < 0.8 {
        let f = (tc - 0.6) / 0.2;
        (1.0, f, 0.0)
    } else {
        let f = (tc - 0.8) / 0.2;
        (1.0, 1.0, f)
    };
    ((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8)
}

// ── Fleet Planner ───────────────────────────────────────────────────────────

fn draw_fleet_planner(ctx: &egui::Context, plan: &mut FleetPlanState, launch_all: &mut bool) {
    if !plan.active {
        // Show "Fleet Plan" button in corner
        egui::Area::new(egui::Id::new("fleet_btn"))
            .fixed_pos(egui::pos2(10.0, ctx.screen_rect().height() - 140.0))
            .show(ctx, |ui| {
                if ui.button(egui::RichText::new("Fleet Plan").size(16.0).color(egui::Color32::from_rgb(100, 200, 255))).clicked() {
                    plan.active = true;
                }
            });
        return;
    }

    egui::Window::new("Fleet Planner")
        .default_pos(egui::pos2(300.0, 100.0))
        .default_width(600.0)
        .resizable(false)
        .show(ctx, |ui| {
            let range_km = plan.map_half_range / 1000.0;
            ui.label(egui::RichText::new(format!(
                "Click to place drones ({}/{}) | Scroll to zoom, right-drag to pan | View: {:.1} km",
                plan.launch_positions.len(), MAX_FLEET_SIZE, range_km * 2.0
            )).size(13.0).color(egui::Color32::WHITE));

            let map_size = egui::vec2(560.0, 560.0);
            let (response, painter) = ui.allocate_painter(
                map_size,
                egui::Sense::click_and_drag(),
            );
            let rect = response.rect;
            let center = rect.center();

            let map_cx = plan.map_center[0];
            let map_cy = plan.map_center[1];
            let half_range = plan.map_half_range;
            let map_half = rect.width().min(rect.height()) * 0.5;

            let to_screen = |dh_x: f64, dh_y: f64| -> egui::Pos2 {
                let sx = ((dh_x - map_cx) / half_range) as f32 * map_half + center.x;
                let sy = -((dh_y - map_cy) / half_range) as f32 * map_half + center.y;
                egui::pos2(sx, sy)
            };

            // Background
            painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(30, 30, 40));

            // Grid — adaptive spacing based on zoom level
            let grid_step = if half_range > 20_000.0 { 10_000.0 }
                else if half_range > 5_000.0 { 5_000.0 }
                else if half_range > 2_000.0 { 1_000.0 }
                else if half_range > 500.0 { 500.0 }
                else if half_range > 100.0 { 100.0 }
                else { 50.0 };
            let grid_color = egui::Color32::from_rgba_unmultiplied(80, 80, 100, 80);
            let grid_min_x = ((map_cx - half_range) / grid_step).ceil() as i64;
            let grid_max_x = ((map_cx + half_range) / grid_step).floor() as i64;
            let grid_min_y = ((map_cy - half_range) / grid_step).ceil() as i64;
            let grid_max_y = ((map_cy + half_range) / grid_step).floor() as i64;
            for gx in grid_min_x..=grid_max_x {
                let x = gx as f64 * grid_step;
                let p0 = to_screen(x, map_cy - half_range);
                let p1 = to_screen(x, map_cy + half_range);
                painter.line_segment([p0, p1], egui::Stroke::new(1.0, grid_color));
            }
            for gy in grid_min_y..=grid_max_y {
                let y = gy as f64 * grid_step;
                let p0 = to_screen(map_cx - half_range, y);
                let p1 = to_screen(map_cx + half_range, y);
                painter.line_segment([p0, p1], egui::Stroke::new(1.0, grid_color));
            }

            // Grid scale label
            let scale_text = if grid_step >= 1000.0 { format!("{:.0} km grid", grid_step / 1000.0) }
                else { format!("{:.0} m grid", grid_step) };
            painter.text(
                egui::pos2(rect.min.x + 6.0, rect.max.y - 6.0),
                egui::Align2::LEFT_BOTTOM,
                scale_text,
                egui::FontId::proportional(10.0),
                egui::Color32::from_rgba_unmultiplied(150, 150, 170, 180),
            );

            // Target marker
            let tgt = to_screen(guidance::TARGET_POS.x, guidance::TARGET_POS.y);
            painter.circle_filled(tgt, 6.0, egui::Color32::from_rgb(220, 40, 40));
            painter.text(egui::pos2(tgt.x + 10.0, tgt.y), egui::Align2::LEFT_CENTER, "TARGET", egui::FontId::proportional(11.0), egui::Color32::from_rgb(220, 40, 40));

            // Launch origin — the original drone spawn at (0, 0)
            let origin = to_screen(0.0, 0.0);
            // Crosshair + circle
            let origin_color = egui::Color32::from_rgb(40, 220, 40);
            painter.circle_stroke(origin, 8.0, egui::Stroke::new(2.0, origin_color));
            painter.line_segment(
                [egui::pos2(origin.x - 12.0, origin.y), egui::pos2(origin.x + 12.0, origin.y)],
                egui::Stroke::new(1.5, origin_color),
            );
            painter.line_segment(
                [egui::pos2(origin.x, origin.y - 12.0), egui::pos2(origin.x, origin.y + 12.0)],
                egui::Stroke::new(1.5, origin_color),
            );
            painter.text(
                egui::pos2(origin.x + 14.0, origin.y),
                egui::Align2::LEFT_CENTER,
                "LAUNCH SITE",
                egui::FontId::proportional(11.0),
                origin_color,
            );

            // Placed drones
            for (i, pos) in plan.launch_positions.iter().enumerate() {
                let sp = to_screen(pos[0], pos[1]);
                let tc = fleet_trail_color(i);
                let color = egui::Color32::from_rgb((tc[0] * 255.0) as u8, (tc[1] * 255.0) as u8, (tc[2] * 255.0) as u8);
                painter.circle_filled(sp, 6.0, color);
                painter.text(egui::pos2(sp.x + 8.0, sp.y), egui::Align2::LEFT_CENTER, format!("#{}", i + 1), egui::FontId::proportional(12.0), color);
            }

            // Scroll to zoom
            let scroll = ui.input(|i| {
                i.events.iter().filter_map(|e| match e {
                    egui::Event::MouseWheel { delta, .. } => Some(delta.y),
                    _ => None,
                }).sum::<f32>()
            });
            if response.hovered() && scroll.abs() > 0.01 {
                let zoom_factor = if scroll > 0.0 { 0.85 } else { 1.0 / 0.85 };
                // Zoom toward cursor
                if let Some(hover_pos) = response.hover_pos() {
                    let before_x = ((hover_pos.x - center.x) / map_half) as f64 * half_range + map_cx;
                    let before_y = -((hover_pos.y - center.y) / map_half) as f64 * half_range + map_cy;
                    plan.map_half_range = (plan.map_half_range * zoom_factor as f64).clamp(50.0, 60_000.0);
                    let new_hr = plan.map_half_range;
                    let after_x = ((hover_pos.x - center.x) / map_half) as f64 * new_hr + plan.map_center[0];
                    let after_y = -((hover_pos.y - center.y) / map_half) as f64 * new_hr + plan.map_center[1];
                    plan.map_center[0] += before_x - after_x;
                    plan.map_center[1] += before_y - after_y;
                } else {
                    plan.map_half_range = (plan.map_half_range * zoom_factor as f64).clamp(50.0, 60_000.0);
                }
            }

            // Right-drag to pan
            if response.dragged_by(egui::PointerButton::Secondary) {
                let drag = response.drag_delta();
                plan.map_center[0] -= (drag.x as f64 / map_half as f64) * half_range;
                plan.map_center[1] += (drag.y as f64 / map_half as f64) * half_range;
            }

            // Left-click to place drone
            if response.clicked_by(egui::PointerButton::Primary) && plan.launch_positions.len() < MAX_FLEET_SIZE {
                if let Some(click_pos) = response.interact_pointer_pos() {
                    let dh_x = ((click_pos.x - center.x) / map_half) as f64 * half_range + map_cx;
                    let dh_y = -((click_pos.y - center.y) / map_half) as f64 * half_range + map_cy;
                    plan.launch_positions.push([dh_x, dh_y]);
                }
            }

            ui.horizontal(|ui| {
                if ui.button(egui::RichText::new("Clear").size(14.0)).clicked() {
                    plan.launch_positions.clear();
                }
                if !plan.launch_positions.is_empty() {
                    if ui.button(egui::RichText::new("Launch All").size(14.0).color(egui::Color32::from_rgb(255, 100, 100)).strong()).clicked() {
                        *launch_all = true;
                    }
                }
                if ui.button(egui::RichText::new("Reset View").size(14.0)).clicked() {
                    plan.map_center = [25_000.0, 0.0];
                    plan.map_half_range = 28_000.0;
                }
                if ui.button(egui::RichText::new("Close").size(14.0)).clicked() {
                    plan.active = false;
                }
            });
        });
}

// ── Fullscreen Map ──────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
fn draw_fullscreen_map(
    ctx: &egui::Context,
    drones: &[DroneInstance],
    wind: &nalgebra::Vector3<f64>,
    camera_mode: CameraMode,
    ground_cam_pos: &Vec3,
    ground_cam_yaw: f32,
    primary_idx: usize,
    map_center: &mut [f64; 2],
    map_half_range: &mut f64,
    show_trails: bool,
) -> Option<[f64; 2]> {
    let mut clicked_pos: Option<[f64; 2]> = None;

    let screen = ctx.screen_rect();
    let margin = 10.0;
    let map_w = screen.width() - margin * 2.0;
    let map_h = screen.height() - margin * 2.0 - 30.0; // leave room for status bar

    egui::Area::new(egui::Id::new("fullscreen_map"))
        .fixed_pos(egui::pos2(margin, margin))
        .order(egui::Order::Background)
        .interactable(true)
        .show(ctx, |ui| {
            let map_size = egui::vec2(map_w, map_h);
            let (response, painter) = ui.allocate_painter(map_size, egui::Sense::click_and_drag());
            let rect = response.rect;
            let center = rect.center();
            let map_cx = map_center[0];
            let map_cy = map_center[1];
            let half_range = *map_half_range;
            let map_half = rect.width().min(rect.height()) * 0.5;

            let to_screen = |dh_x: f64, dh_y: f64| -> egui::Pos2 {
                let sx = ((dh_x - map_cx) / half_range) as f32 * map_half + center.x;
                let sy = -((dh_y - map_cy) / half_range) as f32 * map_half + center.y;
                egui::pos2(sx, sy)
            };

            // Background
            painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 22, 30));

            // Adaptive grid
            let grid_step = if half_range > 20_000.0 { 10_000.0 }
                else if half_range > 5_000.0 { 5_000.0 }
                else if half_range > 2_000.0 { 1_000.0 }
                else if half_range > 500.0 { 500.0 }
                else if half_range > 100.0 { 100.0 }
                else { 50.0 };
            let grid_color = egui::Color32::from_rgba_unmultiplied(60, 60, 80, 80);
            let grid_min_x = ((map_cx - half_range) / grid_step).ceil() as i64;
            let grid_max_x = ((map_cx + half_range) / grid_step).floor() as i64;
            let grid_min_y = ((map_cy - half_range) / grid_step).ceil() as i64;
            let grid_max_y = ((map_cy + half_range) / grid_step).floor() as i64;
            for gx in grid_min_x..=grid_max_x {
                let x = gx as f64 * grid_step;
                let p0 = to_screen(x, map_cy - half_range);
                let p1 = to_screen(x, map_cy + half_range);
                painter.line_segment([p0, p1], egui::Stroke::new(1.0, grid_color));
            }
            for gy in grid_min_y..=grid_max_y {
                let y = gy as f64 * grid_step;
                let p0 = to_screen(map_cx - half_range, y);
                let p1 = to_screen(map_cx + half_range, y);
                painter.line_segment([p0, p1], egui::Stroke::new(1.0, grid_color));
            }

            // Scale label
            let scale_text = if grid_step >= 1000.0 { format!("{:.0} km grid", grid_step / 1000.0) }
                else { format!("{:.0} m grid", grid_step) };
            painter.text(
                egui::pos2(rect.min.x + 8.0, rect.max.y - 6.0),
                egui::Align2::LEFT_BOTTOM, scale_text,
                egui::FontId::proportional(12.0),
                egui::Color32::from_rgba_unmultiplied(150, 150, 180, 200),
            );

            // Launch site
            let origin = to_screen(0.0, 0.0);
            let origin_color = egui::Color32::from_rgb(40, 220, 40);
            painter.circle_stroke(origin, 6.0, egui::Stroke::new(2.0, origin_color));
            painter.line_segment([egui::pos2(origin.x - 10.0, origin.y), egui::pos2(origin.x + 10.0, origin.y)], egui::Stroke::new(1.5, origin_color));
            painter.line_segment([egui::pos2(origin.x, origin.y - 10.0), egui::pos2(origin.x, origin.y + 10.0)], egui::Stroke::new(1.5, origin_color));
            painter.text(egui::pos2(origin.x + 12.0, origin.y), egui::Align2::LEFT_CENTER, "LAUNCH", egui::FontId::proportional(11.0), origin_color);

            // Target
            let tgt = to_screen(guidance::TARGET_POS.x, guidance::TARGET_POS.y);
            let x_size = 6.0_f32;
            let x_color = egui::Color32::from_rgb(220, 40, 40);
            painter.line_segment([egui::pos2(tgt.x - x_size, tgt.y - x_size), egui::pos2(tgt.x + x_size, tgt.y + x_size)], egui::Stroke::new(2.0, x_color));
            painter.line_segment([egui::pos2(tgt.x + x_size, tgt.y - x_size), egui::pos2(tgt.x - x_size, tgt.y + x_size)], egui::Stroke::new(2.0, x_color));
            painter.text(egui::pos2(tgt.x + 10.0, tgt.y), egui::Align2::LEFT_CENTER, "TGT", egui::FontId::proportional(11.0), x_color);

            // Waypoints
            let primary_guidance = &drones[primary_idx].guidance;
            let wp_color = egui::Color32::from_rgb(60, 140, 255);
            let num_wps = if primary_guidance.waypoints.len() > 1 { primary_guidance.waypoints.len() - 1 } else { 0 };
            for (i, wp) in primary_guidance.waypoints.iter().take(num_wps).enumerate() {
                let wp_pos = to_screen(wp.x, wp.y);
                painter.circle_filled(wp_pos, 3.0, wp_color);
                painter.text(egui::pos2(wp_pos.x + 6.0, wp_pos.y), egui::Align2::LEFT_CENTER, format!("W{}", i + 1), egui::FontId::proportional(9.0), wp_color);
            }

            // Trails
            if show_trails {
                for d in drones {
                    let tc = d.trail_color;
                    let trail_color = egui::Color32::from_rgba_unmultiplied(
                        (tc[0] * 255.0) as u8, (tc[1] * 255.0) as u8, (tc[2] * 255.0) as u8, 180,
                    );
                    let mt = &d.minimap_trail;
                    if mt.len() >= 2 {
                        for i in 0..mt.len() - 1 {
                            let p0 = to_screen(mt[i][0], mt[i][1]);
                            let p1 = to_screen(mt[i + 1][0], mt[i + 1][1]);
                            painter.line_segment([p0, p1], egui::Stroke::new(1.5, trail_color));
                        }
                    }
                }
            }

            // Drone icons
            for (di, d) in drones.iter().enumerate() {
                let dp = to_screen(d.flight.position.x, d.flight.position.y);
                let h = d.flight.heading as f32;
                let is_primary = di == primary_idx;
                let tri_size = if is_primary { 10.0_f32 } else { 6.0_f32 };
                let tc = d.trail_color;
                let tri_color = egui::Color32::from_rgb(
                    (tc[0] * 255.0) as u8, (tc[1] * 255.0) as u8, (tc[2] * 255.0) as u8,
                );
                let fwd_x = h.cos();
                let fwd_y = -h.sin();
                let tip = egui::pos2(dp.x + fwd_x * tri_size, dp.y + fwd_y * tri_size);
                let left = egui::pos2(dp.x + (-fwd_x * 0.5 + fwd_y * 0.5) * tri_size, dp.y + (-fwd_y * 0.5 - fwd_x * 0.5) * tri_size);
                let right = egui::pos2(dp.x + (-fwd_x * 0.5 - fwd_y * 0.5) * tri_size, dp.y + (-fwd_y * 0.5 + fwd_x * 0.5) * tri_size);
                painter.add(egui::Shape::convex_polygon(vec![tip, left, right], tri_color, egui::Stroke::NONE));
                if is_primary || half_range < 5_000.0 {
                    painter.text(egui::pos2(dp.x + tri_size + 4.0, dp.y), egui::Align2::LEFT_CENTER,
                        format!("#{}", di + 1), egui::FontId::proportional(if is_primary { 12.0 } else { 9.0 }), tri_color);
                }
            }

            // Ground camera
            if camera_mode == CameraMode::Ground {
                let dh_x = ground_cam_pos.x as f64;
                let dh_y = -(ground_cam_pos.z as f64);
                let cam_screen = to_screen(dh_x, dh_y);
                let cam_color = egui::Color32::from_rgb(255, 200, 40);
                painter.circle_filled(cam_screen, 5.0, cam_color);
                painter.text(egui::pos2(cam_screen.x + 8.0, cam_screen.y), egui::Align2::LEFT_CENTER, "CAM", egui::FontId::proportional(10.0), cam_color);
                let wedge_len = 25.0_f32;
                let fov_half = 30.0_f32.to_radians();
                for sign in [-1.0_f32, 1.0] {
                    let angle = ground_cam_yaw + sign * fov_half;
                    let dx = angle.cos();
                    let dy = angle.sin();
                    let end = egui::pos2(cam_screen.x + dx * wedge_len, cam_screen.y + dy * wedge_len);
                    painter.line_segment([cam_screen, end], egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 200, 40, 100)));
                }
            }

            // Wind arrow (top-right corner)
            let wind_speed = wind.norm();
            if wind_speed > 0.5 {
                let compass_center = egui::pos2(rect.max.x - 35.0, rect.min.y + 35.0);
                let wx = wind.x as f32;
                let wy = -(wind.y as f32);
                let wlen = (wx * wx + wy * wy).sqrt();
                if wlen > 0.01 {
                    let wnx = wx / wlen;
                    let wny = wy / wlen;
                    let arrow_len = (wind_speed as f32 / 25.0 * 18.0).clamp(5.0, 18.0);
                    let tip = egui::pos2(compass_center.x + wnx * arrow_len, compass_center.y + wny * arrow_len);
                    let wc = egui::Color32::from_rgb(140, 200, 255);
                    painter.line_segment([compass_center, tip], egui::Stroke::new(2.0, wc));
                    painter.text(egui::pos2(compass_center.x, compass_center.y + 22.0), egui::Align2::CENTER_TOP,
                        format!("{:.0} m/s", wind_speed), egui::FontId::proportional(10.0), wc);
                }
            }

            // Status bar
            let primary = &drones[primary_idx];
            let dist = primary.guidance.distance_to_target(&primary.flight.position);
            let info = format!(
                "M: close map | Scroll: zoom | Right-drag: pan | Click: place ground cam | Drones: {} | Primary #{} | Alt: {:.0}m | Dist: {:.1}km",
                drones.len(), primary_idx + 1, primary.flight.altitude(), dist / 1000.0
            );
            painter.text(
                egui::pos2(center.x, rect.max.y + 4.0),
                egui::Align2::CENTER_TOP, info,
                egui::FontId::proportional(13.0),
                egui::Color32::from_rgb(180, 180, 200),
            );

            // Scroll to zoom
            let scroll = ui.input(|i| {
                i.events.iter().filter_map(|e| match e {
                    egui::Event::MouseWheel { delta, .. } => Some(delta.y),
                    _ => None,
                }).sum::<f32>()
            });
            if response.hovered() && scroll.abs() > 0.01 {
                let zoom_factor = if scroll > 0.0 { 0.85 } else { 1.0 / 0.85 };
                if let Some(hover_pos) = response.hover_pos() {
                    let before_x = ((hover_pos.x - center.x) / map_half) as f64 * half_range + map_cx;
                    let before_y = -((hover_pos.y - center.y) / map_half) as f64 * half_range + map_cy;
                    *map_half_range = (*map_half_range * zoom_factor as f64).clamp(50.0, 60_000.0);
                    let new_hr = *map_half_range;
                    let after_x = ((hover_pos.x - center.x) / map_half) as f64 * new_hr + map_center[0];
                    let after_y = -((hover_pos.y - center.y) / map_half) as f64 * new_hr + map_center[1];
                    map_center[0] += before_x - after_x;
                    map_center[1] += before_y - after_y;
                } else {
                    *map_half_range = (*map_half_range * zoom_factor as f64).clamp(50.0, 60_000.0);
                }
            }

            // Right-drag to pan
            if response.dragged_by(egui::PointerButton::Secondary) {
                let drag = response.drag_delta();
                map_center[0] -= (drag.x as f64 / map_half as f64) * half_range;
                map_center[1] += (drag.y as f64 / map_half as f64) * half_range;
            }

            // Left-click to place ground camera
            if response.clicked_by(egui::PointerButton::Primary) {
                if let Some(click_pos) = response.interact_pointer_pos() {
                    let dh_x = ((click_pos.x - center.x) / map_half) as f64 * half_range + map_cx;
                    let dh_y = -((click_pos.y - center.y) / map_half) as f64 * half_range + map_cy;
                    clicked_pos = Some([dh_x, dh_y]);
                }
            }
        });

    clicked_pos
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
        self.terrain_verts = tv;
        self.terrain_idxs = ti;
        self.terrain_regen_queue.clear();
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

        // Flagpole beside launch rail
        let (fv, fi) = terrain::generate_flagpole();
        let fvb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Flag VB"), contents: bytemuck::cast_slice(&fv), usage: wgpu::BufferUsages::VERTEX });
        let fib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Flag IB"), contents: bytemuck::cast_slice(&fi), usage: wgpu::BufferUsages::INDEX });
        self.flag_mesh = Some(GpuMesh { vertex_buffer: fvb, index_buffer: fib, num_indices: fi.len() as u32 });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.flag_material = Some(MaterialBind { buffer: buf, bind_group: bg });

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

        // Drone mesh (shared geometry)
        let (dv, di) = drone::generate_drone_mesh();
        let dvb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Drone VB"), contents: bytemuck::cast_slice(&dv), usage: wgpu::BufferUsages::VERTEX });
        let dib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Drone IB"), contents: bytemuck::cast_slice(&di), usage: wgpu::BufferUsages::INDEX });
        self.drone_mesh = Some(GpuMesh { vertex_buffer: dvb, index_buffer: dib, num_indices: di.len() as u32 });

        // Prop mesh (shared geometry)
        let (pv, pi) = drone::generate_prop_disc();
        let pvb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Prop VB"), contents: bytemuck::cast_slice(&pv), usage: wgpu::BufferUsages::VERTEX });
        let pib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Prop IB"), contents: bytemuck::cast_slice(&pi), usage: wgpu::BufferUsages::INDEX });
        self.prop_mesh = Some(GpuMesh { vertex_buffer: pvb, index_buffer: pib, num_indices: pi.len() as u32 });

        // Create materials for the default drone (drones[0])
        {
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.drones[0].drone_material = Some(MaterialBind { buffer: buf, bind_group: bg });
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.drones[0].drone_outline_material = Some(MaterialBind { buffer: buf, bind_group: bg });
            let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
            self.drones[0].prop_material = Some(MaterialBind { buffer: buf, bind_group: bg });
        }

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

        // Rocks (single combined mesh, world-space vertices)
        let (rov, roi) = terrain::generate_rocks();
        let rovb = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Rock VB"), contents: bytemuck::cast_slice(&rov), usage: wgpu::BufferUsages::VERTEX });
        let roib = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("Rock IB"), contents: bytemuck::cast_slice(&roi), usage: wgpu::BufferUsages::INDEX });
        self.rock_mesh = Some(GpuMesh { vertex_buffer: rovb, index_buffer: roib, num_indices: roi.len() as u32 });
        let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
        self.rock_material = Some(MaterialBind { buffer: buf, bind_group: bg });

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
                        // Launch all drones
                        let all_prelaunch = self.drones.iter().all(|d| d.guidance.phase == FlightPhase::PreLaunch);
                        if all_prelaunch {
                            // If fleet is planned but not yet spawned, spawn them now
                            if !self.fleet_plan.launch_positions.is_empty() {
                                self.pending_fleet_launch = true;
                            } else {
                                // Launch default single drone
                                for d in &mut self.drones {
                                    d.guidance.launch();
                                }
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
                    }
                    Key::Named(NamedKey::Tab) => {
                        if self.drones.len() > 1 {
                            self.primary_drone = (self.primary_drone + 1) % self.drones.len();
                        }
                    }
                    Key::Character(ref c) if c.as_str() == "r" => self.reset(),
                    Key::Character(ref c) if c.as_str() == "p" => self.paused = !self.paused,
                    Key::Character(ref c) if c.as_str() == "c" => self.camera_mode = self.camera_mode.next(),
                    Key::Character(ref c) if c.as_str() == "b" => {
                        if self.camera_mode == CameraMode::Ground {
                            self.camera_mode = CameraMode::Chase;
                        }
                    }
                    Key::Character(ref c) if c.as_str() == "m" => {
                        self.fullscreen_map = !self.fullscreen_map;
                    }
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
                    if self.mouse_pressed {
                        if self.camera_mode == CameraMode::Ground {
                            self.ground_cam_yaw += dx * 0.005;
                            self.ground_cam_pitch = (self.ground_cam_pitch + dy * 0.005).clamp(-1.2, 1.2);
                        } else {
                            self.camera.rotate(dx * 0.005, -dy * 0.005);
                        }
                    }
                    if self.middle_pressed { self.camera.pan(-dx, dy); }
                }
                self.last_mouse_pos = Some((position.x, position.y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    winit::event::MouseScrollDelta::LineDelta(_, y) => y,
                    winit::event::MouseScrollDelta::PixelDelta(p) => p.y as f32 * 0.01,
                };
                if self.camera_mode == CameraMode::Ground {
                    // Scroll = FOV zoom (scroll up = zoom in = narrower FOV)
                    self.ground_cam_fov = (self.ground_cam_fov * (1.0 - scroll * 0.08))
                        .clamp(5.0_f32.to_radians(), 120.0_f32.to_radians());
                } else {
                    // Override default zoom limits for flight sim
                    self.camera.distance = (self.camera.distance * (1.0 - scroll * 0.1)).clamp(5.0, 500.0);
                }
            }
            WindowEvent::RedrawRequested => {
                let now = Instant::now();
                let raw_dt = now.duration_since(self.last_frame).as_secs_f64();
                let frame_dt = raw_dt.min(MAX_FRAME_TIME);
                self.last_frame = now;

                let pi = self.primary_drone.min(self.drones.len().saturating_sub(1));
                let effective_scale = if self.drones[pi].guidance.should_auto_slow() {
                    self.time_scale.min(1.0)
                } else {
                    self.time_scale
                };

                // Reconstruct wind vector from sliders
                let wd_rad = self.wind_direction.to_radians();
                self.wind = nalgebra::Vector3::new(
                    self.wind_speed * wd_rad.cos(),
                    self.wind_speed * wd_rad.sin(),
                    0.0,
                );

                // Master volume
                self.audio_engine.set_volume(self.master_volume);

                // Handle pending fleet launch (spawn drones from plan)
                if self.pending_fleet_launch {
                    self.pending_fleet_launch = false;
                    let positions = self.fleet_plan.launch_positions.clone();
                    if !positions.is_empty() {
                        self.drones.clear();
                        for (i, pos) in positions.iter().enumerate() {
                            let launch = nalgebra::Vector3::new(pos[0], pos[1], 1.0);
                            let heading = (guidance::TARGET_POS.x - pos[0]).atan2(guidance::TARGET_POS.y - pos[1]);
                            let mut di = DroneInstance {
                                flight: FlightState::new_at(launch, heading),
                                guidance: Guidance::new_from(launch),
                                drone_material: None,
                                prop_material: None,
                                drone_outline_material: None,
                                trail_points: Vec::new(),
                                trail_distance_accum: 0.0,
                                minimap_trail: Vec::new(),
                                trail_color: fleet_trail_color(i),
                                drone_tint: fleet_tint(i),
                                launch_pos: *pos,
                            };
                            // Create materials
                            if let Some(pbr) = &self.pbr_pipeline {
                                if let Some(ctx) = &self.render_ctx {
                                    let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
                                    di.drone_material = Some(MaterialBind { buffer: buf, bind_group: bg });
                                    let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
                                    di.drone_outline_material = Some(MaterialBind { buffer: buf, bind_group: bg });
                                    let (buf, bg) = pbr.create_material_bind_group(&ctx.device);
                                    di.prop_material = Some(MaterialBind { buffer: buf, bind_group: bg });
                                }
                            }
                            self.drones.push(di);
                        }
                        self.primary_drone = 0;
                        self.fleet_plan.active = false;
                        // Launch all
                        for d in &mut self.drones {
                            d.guidance.launch();
                        }
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

                // Physics loop — all drones
                let any_active = self.drones.iter().any(|d| {
                    d.guidance.phase != FlightPhase::Impact && d.guidance.phase != FlightPhase::PreLaunch
                });
                if !self.paused && any_active {
                    self.accumulator += frame_dt * effective_scale;
                    let mut steps = 0u64;
                    while self.accumulator >= PHYSICS_DT && steps < MAX_STEPS_PER_FRAME {
                        let mut all_done = true;
                        let mut new_impacts = Vec::new();
                        for (di, d) in self.drones.iter_mut().enumerate() {
                            if d.guidance.phase == FlightPhase::Impact || d.guidance.phase == FlightPhase::PreLaunch {
                                continue;
                            }
                            all_done = false;
                            let (thrust, pitch_cmd, bank_cmd) = d.guidance.update(&d.flight);
                            flight::step(&mut d.flight, thrust, pitch_cmd, bank_cmd, &self.wind);

                            // Terrain-aware ground check using visual mesh interpolation
                            // DH (x,y,z) → render (x,z,-y)
                            let ground_z = terrain::terrain_height_visual(
                                d.flight.position.x as f32,
                                -(d.flight.position.y as f32),
                            ) as f64;
                            if d.flight.position.z <= ground_z + 0.1 {
                                // Clamp to ground surface
                                d.flight.position.z = ground_z;
                                d.flight.velocity = nalgebra::Vector3::zeros();
                                d.guidance.phase = FlightPhase::Impact;
                                new_impacts.push(di);
                            } else if d.guidance.phase == FlightPhase::Impact {
                                d.flight.velocity = nalgebra::Vector3::zeros();
                            }
                            // Trail
                            let speed = d.flight.airspeed();
                            d.trail_distance_accum += speed * PHYSICS_DT;
                            if d.trail_distance_accum >= 50.0 {
                                d.trail_distance_accum = 0.0;
                                d.trail_points.push((to_render(&d.flight.position), d.trail_color));
                                if d.trail_points.len() > 10_000 { d.trail_points.remove(0); }
                                d.minimap_trail.push([d.flight.position.x, d.flight.position.y]);
                                if d.minimap_trail.len() > 10_000 { d.minimap_trail.remove(0); }
                            }
                        }
                        // Play impact sound for new impacts
                        for _di in &new_impacts {
                            self.audio_engine.play(Box::new(sound::ImpactVoice::new()));
                        }
                        if all_done { break; }
                        self.accumulator -= PHYSICS_DT;
                        self.sim_time += PHYSICS_DT;
                        steps += 1;
                    }

                    // Update persistent voice params from primary drone
                    let pi = self.primary_drone.min(self.drones.len().saturating_sub(1));
                    let all_impacted = self.drones.iter().all(|d| d.guidance.phase == FlightPhase::Impact);
                    if all_impacted {
                        // Stop engine and wind sounds on impact
                        if let Some(ev) = &self.engine_voice {
                            if let Ok(mut e) = ev.lock() { e.set_running(false); e.volume = 0.0; }
                        }
                        if let Some(wv) = &self.wind_voice {
                            if let Ok(mut w) = wv.lock() { w.volume = 0.0; }
                        }
                    } else {
                        let airspeed = self.drones[pi].flight.true_airspeed(&self.wind);
                        let cam_eye = self.camera.eye();
                        let drone_render = to_render(&self.drones[pi].flight.position);
                        let dist = (cam_eye - drone_render).length() as f64;
                        let pan = (drone_render.x - cam_eye.x).atan2((drone_render.z - cam_eye.z).abs() + 1.0);
                        if let Some(ev) = &self.engine_voice {
                            if let Ok(mut e) = ev.lock() {
                                e.update_params(airspeed, dist, pan.clamp(-1.0, 1.0));
                                if self.engine_muted { e.volume = 0.0; }
                            }
                        }
                        if let Some(wv) = &self.wind_voice {
                            if let Ok(mut w) = wv.lock() {
                                w.update_params(airspeed, dist, pan.clamp(-1.0, 1.0));
                            }
                        }
                    }
                }

                // Camera
                self.update_camera();

                // Incremental terrain tile regeneration
                let cam = self.camera.eye();
                let snap_x = (cam.x / terrain::TILE_SIZE).round() as i32;
                let snap_z = (cam.z / terrain::TILE_SIZE).round() as i32;
                if (snap_x, snap_z) != self.last_terrain_snap {
                    let snap_wx = snap_x as f32 * terrain::TILE_SIZE;
                    let snap_wz = snap_z as f32 * terrain::TILE_SIZE;
                    // Queue all tiles for regeneration
                    self.terrain_regen_queue.clear();
                    for tdx in -terrain::TILE_RADIUS..=terrain::TILE_RADIUS {
                        for tdz in -terrain::TILE_RADIUS..=terrain::TILE_RADIUS {
                            let slot = terrain::tile_slot(tdx, tdz);
                            let (cx, cz) = terrain::tile_world_center(snap_wx, snap_wz, tdx, tdz);
                            self.terrain_regen_queue.push((slot, cx, cz));
                        }
                    }
                    // Sort: nearest tiles first so the visible area updates immediately
                    self.terrain_regen_queue.sort_by(|a, b| {
                        let da = (a.1 - cam.x) * (a.1 - cam.x) + (a.2 - cam.z) * (a.2 - cam.z);
                        let db = (b.1 - cam.x) * (b.1 - cam.x) + (b.2 - cam.z) * (b.2 - cam.z);
                        da.partial_cmp(&db).unwrap()
                    });
                    self.last_terrain_snap = (snap_x, snap_z);
                }

                // Process a batch of queued tiles per frame (max 60 → spreads 441 tiles over ~8 frames)
                if !self.terrain_regen_queue.is_empty() {
                    let batch_size = 60.min(self.terrain_regen_queue.len());
                    let batch: Vec<_> = self.terrain_regen_queue.drain(..batch_size).collect();
                    let vert_stride = terrain::VERTS_PER_TILE * std::mem::size_of::<simuforge_core::Vertex>();
                    let idx_stride = terrain::IDXS_PER_TILE * std::mem::size_of::<u32>();

                    for (slot, cx, cz) in &batch {
                        terrain::generate_single_tile(*cx, *cz, *slot, &mut self.terrain_verts, &mut self.terrain_idxs);
                    }

                    // Upload only the changed tile regions
                    if let (Some(ctx), Some(vb), Some(ib)) = (&self.render_ctx, &self.terrain_vb, &self.terrain_ib) {
                        for (slot, _, _) in &batch {
                            ctx.queue.write_buffer(vb, (slot * vert_stride) as u64,
                                bytemuck::cast_slice(&self.terrain_verts[slot * terrain::VERTS_PER_TILE..(slot + 1) * terrain::VERTS_PER_TILE]));
                            ctx.queue.write_buffer(ib, (slot * idx_stride) as u64,
                                bytemuck::cast_slice(&self.terrain_idxs[slot * terrain::IDXS_PER_TILE..(slot + 1) * terrain::IDXS_PER_TILE]));
                        }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Headless flight simulation: runs a full mission from launch to impact and
    /// verifies the drone actually reaches the visual terrain surface.
    #[test]
    fn test_headless_flight_to_impact() {
        let mut state = FlightState::new();
        let mut guide = Guidance::new();
        let wind = nalgebra::Vector3::new(8.0, 0.0, 0.0);

        guide.launch();

        let max_steps = 200 * 60 * 30; // 30 minutes at 200 Hz
        let mut hit_terrain = false;
        let mut _final_ground_z = 0.0_f64;
        let mut final_visual_z = 0.0_f64;
        let mut prev_phase = guide.phase;
        let mut terminal_print_counter = 0_u32;

        for step in 0..max_steps {
            if guide.phase == FlightPhase::Impact {
                break;
            }

            let (thrust, pitch_cmd, bank_cmd) = guide.update(&state);
            flight::step(&mut state, thrust, pitch_cmd, bank_cmd, &wind);

            // Terrain check FIRST (before guidance phase-break), same as main loop
            let render_wx = state.position.x as f32;
            let render_wz = -(state.position.y as f32);
            let ground_z_visual = terrain::terrain_height_visual(render_wx, render_wz) as f64;

            if state.position.z <= ground_z_visual + 0.1 {
                state.position.z = ground_z_visual;
                state.velocity = nalgebra::Vector3::zeros();
                guide.phase = FlightPhase::Impact;
                hit_terrain = true;
                final_visual_z = ground_z_visual;

                let t = step as f64 / 200.0;
                println!("IMPACT at T+{:.1}s step {step}:", t);
                println!("  DH position: ({:.1}, {:.1}, {:.2})", state.position.x, state.position.y, state.position.z);
                println!("  terrain_height_visual:   {:.2}", ground_z_visual);
                println!("  drone z vs visual:       {:.2} m", state.position.z - ground_z_visual);
                break;
            }

            // Log phase transitions
            if guide.phase != prev_phase {
                let t = step as f64 / 200.0;
                println!("T+{:.1}s: Phase changed to {:?} at pos=({:.0},{:.0},{:.1}) dist_tgt={:.0}",
                    t, guide.phase, state.position.x, state.position.y, state.position.z,
                    guide.distance_to_target(&state.position));
                prev_phase = guide.phase;
            }

            // Fine-grained terminal phase logging
            if guide.phase == FlightPhase::Terminal {
                terminal_print_counter += 1;
                if terminal_print_counter % 200 == 0 {
                    let gap = state.position.z - ground_z_visual;
                    let dist = guide.distance_to_target(&state.position);
                    println!("  TERMINAL: z={:.1} ground={:.1} gap={:.1}m pitch={:.1}deg vel_z={:.1} dist={:.0}",
                        state.position.z, ground_z_visual, gap,
                        state.pitch.to_degrees(), state.velocity.z, dist);
                }
            }

            // Coarse logging during cruise
            if step % (200 * 60) == 0 && step > 0 && guide.phase != FlightPhase::Terminal {
                let t = step as f64 / 200.0;
                println!("T+{:.0}s: {:?} pos=({:.0},{:.0},{:.1}) dist={:.0} ground={:.1}",
                    t, guide.phase, state.position.x, state.position.y, state.position.z,
                    guide.distance_to_target(&state.position), ground_z_visual);
            }
        }

        assert!(hit_terrain, "Drone never hit terrain within 30 min sim time");
        let gap = (state.position.z - final_visual_z).abs();
        println!("\nFinal gap between drone and visual terrain: {:.3} m", gap);
        assert!(gap < 0.2, "Drone should be within 0.2m of visual terrain, got {:.3}m", gap);
    }

    /// Test that terrain_height_visual matches terrain_height at grid points
    /// and interpolates smoothly between them.
    #[test]
    fn test_terrain_visual_matches_at_grid() {
        let step = terrain::GRID_STEP;
        // Sample at a grid point
        let gx = 1000.0_f32;
        let gz = 500.0_f32;
        let exact = terrain::terrain_height(gx * step, gz * step);
        let visual = terrain::terrain_height_visual(gx * step, gz * step);
        let diff = (exact - visual).abs();
        println!("Grid point ({}, {}): exact={:.4}, visual={:.4}, diff={:.6}",
            gx * step, gz * step, exact, visual, diff);
        assert!(diff < 0.01, "At grid points, visual should match exact, got diff={}", diff);
    }

    /// Test that terrain_height vs terrain_height_visual divergence is bounded.
    #[test]
    fn test_terrain_visual_divergence() {
        let mut max_diff = 0.0_f32;
        let mut sum_diff = 0.0_f32;
        let n = 1000;
        for i in 0..n {
            // Sample at non-grid-aligned points across the terrain
            let wx = 100.0 + i as f32 * 53.7; // arbitrary stride, not grid-aligned
            let wz = -200.0 + i as f32 * 37.3;
            let exact = terrain::terrain_height(wx, wz);
            let visual = terrain::terrain_height_visual(wx, wz);
            let diff = (exact - visual).abs();
            max_diff = max_diff.max(diff);
            sum_diff += diff;
        }
        let avg_diff = sum_diff / n as f32;
        println!("Terrain exact vs visual over {} samples: max={:.2}m, avg={:.2}m", n, max_diff, avg_diff);
        // If max_diff is large, the drone would stop visibly above/below the mesh
        assert!(max_diff < 20.0, "Max divergence too large: {}m", max_diff);
    }
}

