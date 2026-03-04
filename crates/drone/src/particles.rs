//! GPU particle system for explosion effects.
//!
//! CPU-side simulation with camera-billboarded quads uploaded each frame.
//! Five particle layers:
//! - **Fireball**: bright expanding core, gravity-pulled
//! - **Sparks**: tiny fast streaking embers shooting outward
//! - **Debris**: dark heavy chunks arcing with full gravity
//! - **Shockwave**: fast outward ring of dust
//! - **Smoke**: dark billowing clouds (alpha-blended, not additive)
//!
//! Two render pipelines: additive for fire/sparks/debris, alpha for smoke.

use glam::Vec3;
use simuforge_render::context::{RenderContext, DEPTH_FORMAT};

/// Particle vertex for GPU upload.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ParticleVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
    pub uv: [f32; 2],
}

/// Maximum particle vertices per pipeline (each particle = 6 vertices).
const MAX_PARTICLE_VERTICES: usize = 120_000;

#[derive(Debug, Clone, Copy, PartialEq)]
enum ParticleKind {
    Fireball,
    Sparks,
    Debris,
    Shockwave,
    Smoke,
}

impl ParticleKind {
    fn is_smoke(self) -> bool {
        self == Self::Smoke
    }
}

#[derive(Debug, Clone)]
struct Particle {
    position: Vec3,
    velocity: Vec3,
    size: f32,
    age: f32,
    lifetime: f32,
    kind: ParticleKind,
    base_color: [f32; 3],
}

struct Explosion {
    particles: Vec<Particle>,
}

/// Particle system with dual pipelines: additive (fire) + alpha (smoke).
pub struct ParticleSystem {
    explosions: Vec<Explosion>,
    // Additive pipeline (fire, sparks, debris, shockwave)
    pub fire_pipeline: wgpu::RenderPipeline,
    pub fire_vertex_buffer: wgpu::Buffer,
    pub fire_bind_group: wgpu::BindGroup,
    pub fire_num_vertices: u32,
    // Alpha pipeline (smoke)
    pub smoke_pipeline: wgpu::RenderPipeline,
    pub smoke_vertex_buffer: wgpu::Buffer,
    pub smoke_bind_group: wgpu::BindGroup,
    pub smoke_num_vertices: u32,
    rng_state: u32,
}

fn create_particle_pipeline(
    ctx: &RenderContext,
    camera_buffer: &wgpu::Buffer,
    label: &str,
    additive: bool,
) -> (wgpu::RenderPipeline, wgpu::Buffer, wgpu::BindGroup) {
    let shader = ctx.device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(&format!("{label} Shader")),
        source: wgpu::ShaderSource::Wgsl(include_str!("shaders/particle.wgsl").into()),
    });

    let bind_group_layout = ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(&format!("{label} BGL")),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });

    let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(&format!("{label} BG")),
        layout: &bind_group_layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: camera_buffer.as_entire_binding(),
        }],
    });

    let pipeline_layout = ctx.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(&format!("{label} Layout")),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[],
    });

    let vertex_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(&format!("{label} VB")),
        size: (MAX_PARTICLE_VERTICES * std::mem::size_of::<ParticleVertex>()) as u64,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let blend = if additive {
        wgpu::BlendState {
            color: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::SrcAlpha,
                dst_factor: wgpu::BlendFactor::One,
                operation: wgpu::BlendOperation::Add,
            },
            alpha: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::One,
                dst_factor: wgpu::BlendFactor::One,
                operation: wgpu::BlendOperation::Add,
            },
        }
    } else {
        wgpu::BlendState::ALPHA_BLENDING
    };

    let pipeline = ctx.device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(&pipeline_layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: &[wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<ParticleVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &[
                    wgpu::VertexAttribute { offset: 0, shader_location: 0, format: wgpu::VertexFormat::Float32x3 },
                    wgpu::VertexAttribute { offset: 12, shader_location: 1, format: wgpu::VertexFormat::Float32x4 },
                    wgpu::VertexAttribute { offset: 28, shader_location: 2, format: wgpu::VertexFormat::Float32x2 },
                ],
            }],
            compilation_options: Default::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(wgpu::ColorTargetState {
                format: ctx.format(),
                blend: Some(blend),
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: false,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        multiview: None,
        cache: None,
    });

    (pipeline, vertex_buffer, bind_group)
}

impl ParticleSystem {
    pub fn new(ctx: &RenderContext, camera_buffer: &wgpu::Buffer) -> Self {
        let (fire_pipeline, fire_vb, fire_bg) =
            create_particle_pipeline(ctx, camera_buffer, "Particle Fire", true);
        let (smoke_pipeline, smoke_vb, smoke_bg) =
            create_particle_pipeline(ctx, camera_buffer, "Particle Smoke", false);

        Self {
            explosions: Vec::new(),
            fire_pipeline,
            fire_vertex_buffer: fire_vb,
            fire_bind_group: fire_bg,
            fire_num_vertices: 0,
            smoke_pipeline,
            smoke_vertex_buffer: smoke_vb,
            smoke_bind_group: smoke_bg,
            smoke_num_vertices: 0,
            rng_state: 42,
        }
    }

    fn rand(&mut self) -> f32 {
        self.rng_state = self.rng_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((self.rng_state >> 16) & 0x7FFF) as f32 / 32767.0
    }

    fn rand_range(&mut self, min: f32, max: f32) -> f32 {
        min + self.rand() * (max - min)
    }

    fn rand_sphere(&mut self) -> Vec3 {
        loop {
            let x = self.rand() * 2.0 - 1.0;
            let y = self.rand() * 2.0 - 1.0;
            let z = self.rand() * 2.0 - 1.0;
            let v = Vec3::new(x, y, z);
            if v.length_squared() <= 1.0 && v.length_squared() > 0.001 {
                return v.normalize();
            }
        }
    }

    /// Random direction biased upward (hemisphere above ground).
    fn rand_hemisphere_up(&mut self) -> Vec3 {
        let mut v = self.rand_sphere();
        v.y = v.y.abs(); // force upward
        v.normalize()
    }

    pub fn spawn_explosion(&mut self, pos: Vec3) {
        let mut particles = Vec::with_capacity(1200);

        // ── Fireball core: bright, violent outward, asymmetric ──
        for _ in 0..250 {
            // Non-uniform: pick a random axis to bias toward for lopsided blast
            let dir = self.rand_sphere();
            let bias_strength = self.rand_range(0.0, 0.6);
            let biased = (dir + Vec3::Y * self.rand_range(0.0, 0.8)
                + Vec3::X * self.rand_range(-bias_strength, bias_strength)
                + Vec3::Z * self.rand_range(-bias_strength, bias_strength))
                .normalize();
            let speed = self.rand_range(20.0, 80.0);
            particles.push(Particle {
                position: pos + dir * self.rand_range(0.0, 1.5),
                velocity: biased * speed,
                size: self.rand_range(2.0, 8.0),
                age: 0.0,
                lifetime: self.rand_range(0.3, 1.2),
                kind: ParticleKind::Fireball,
                base_color: [
                    self.rand_range(0.9, 1.0),
                    self.rand_range(0.5, 0.9),
                    self.rand_range(0.05, 0.3),
                ],
            });
        }

        // ── Secondary fire: delayed, asymmetric clumps ──
        for _ in 0..100 {
            let dir = self.rand_sphere();
            let biased = (dir + Vec3::Y * self.rand_range(0.2, 1.0)).normalize();
            let speed = self.rand_range(8.0, 30.0);
            let delay = self.rand_range(0.0, 0.3);
            particles.push(Particle {
                position: pos + dir * self.rand_range(0.0, 2.0),
                velocity: biased * speed,
                size: self.rand_range(3.0, 10.0),
                age: -delay,
                lifetime: self.rand_range(0.5, 1.5),
                kind: ParticleKind::Fireball,
                base_color: [
                    self.rand_range(0.8, 1.0),
                    self.rand_range(0.2, 0.5),
                    self.rand_range(0.0, 0.1),
                ],
            });
        }

        // ── Sparks: tiny, VERY fast, streaking outward like shrapnel ──
        for _ in 0..300 {
            let dir = self.rand_sphere();
            let biased = (dir + Vec3::Y * 0.3).normalize();
            let speed = self.rand_range(60.0, 200.0);
            particles.push(Particle {
                position: pos + dir * self.rand_range(0.0, 0.5),
                velocity: biased * speed,
                size: self.rand_range(0.05, 0.2),
                age: 0.0,
                lifetime: self.rand_range(0.5, 2.5),
                kind: ParticleKind::Sparks,
                base_color: [
                    self.rand_range(1.0, 1.0),
                    self.rand_range(0.6, 1.0),
                    self.rand_range(0.1, 0.5),
                ],
            });
        }

        // ── Debris: heavy dark chunks, full gravity, explosive throw ──
        for _ in 0..200 {
            let dir = self.rand_sphere();
            let biased = (dir + Vec3::Y * 0.5).normalize();
            let speed = self.rand_range(40.0, 150.0);
            particles.push(Particle {
                position: pos + dir * self.rand_range(0.0, 1.0),
                velocity: biased * speed,
                size: self.rand_range(0.15, 0.6),
                age: 0.0,
                lifetime: self.rand_range(2.0, 5.0),
                kind: ParticleKind::Debris,
                base_color: [
                    self.rand_range(0.08, 0.2),
                    self.rand_range(0.04, 0.1),
                    self.rand_range(0.0, 0.04),
                ],
            });
        }

        // ── Shockwave ring: fast horizontal burst of dust ──
        for _ in 0..100 {
            let angle = self.rand_range(0.0, std::f32::consts::TAU);
            let dir = Vec3::new(angle.cos(), self.rand_range(0.0, 0.15), angle.sin());
            let speed = self.rand_range(30.0, 80.0);
            particles.push(Particle {
                position: pos + dir * self.rand_range(0.0, 2.0),
                velocity: dir * speed,
                size: self.rand_range(2.0, 5.0),
                age: 0.0,
                lifetime: self.rand_range(0.5, 1.5),
                kind: ParticleKind::Shockwave,
                base_color: [0.6, 0.55, 0.45],
            });
        }

        // ── Smoke: fewer, asymmetric clumps, shorter lived ──
        // Pick 3-4 random "clump" directions so smoke isn't a perfect sphere
        let num_clumps = 3 + (self.rand() * 2.0) as usize;
        let mut clump_dirs = Vec::with_capacity(num_clumps);
        for _ in 0..num_clumps {
            clump_dirs.push((self.rand_sphere() + Vec3::Y * 0.5).normalize());
        }
        for _ in 0..150 {
            // Pick a random clump to spawn near
            let clump_idx = (self.rand() * clump_dirs.len() as f32) as usize % clump_dirs.len();
            let clump_dir = clump_dirs[clump_idx];
            // Scatter around the clump direction
            let jitter = Vec3::new(
                self.rand_range(-0.4, 0.4),
                self.rand_range(-0.2, 0.4),
                self.rand_range(-0.4, 0.4),
            );
            let dir = (clump_dir + jitter).normalize();
            let speed = self.rand_range(5.0, 20.0);
            let vel = dir * speed + Vec3::Y * self.rand_range(3.0, 10.0);
            let grey = self.rand_range(0.05, 0.2);
            particles.push(Particle {
                position: pos + dir * self.rand_range(0.0, 2.0),
                velocity: vel,
                size: self.rand_range(3.0, 10.0),
                age: 0.0,
                lifetime: self.rand_range(1.5, 4.0),
                kind: ParticleKind::Smoke,
                base_color: [grey, grey * 0.9, grey * 0.8],
            });
        }

        // ── Lingering smoke wisps: small staggered column, not a blob ──
        for _ in 0..80 {
            let spread = self.rand_range(0.0, 2.0);
            let offset = Vec3::new(
                self.rand_range(-spread, spread),
                self.rand_range(0.0, 1.0),
                self.rand_range(-spread, spread),
            );
            let vel = Vec3::Y * self.rand_range(6.0, 14.0)
                + Vec3::new(self.rand_range(-1.5, 1.5), 0.0, self.rand_range(-1.5, 1.5));
            let grey = self.rand_range(0.03, 0.12);
            let delay = self.rand_range(0.0, 1.5);
            particles.push(Particle {
                position: pos + offset,
                velocity: vel,
                size: self.rand_range(3.0, 8.0),
                age: -delay,
                lifetime: self.rand_range(2.0, 5.0),
                kind: ParticleKind::Smoke,
                base_color: [grey, grey * 0.85, grey * 0.75],
            });
        }

        self.explosions.push(Explosion { particles });
    }

    /// Update all particles. `dt` is wall-clock seconds.
    /// `wind` is in render-space (Y-up): convert DH wind (x,y,z) → render (x,z,-y).
    /// `ground_y` returns the terrain height at a render-space (x, z) position.
    pub fn update(&mut self, dt: f32, wind: Vec3, ground_y: impl Fn(f32, f32) -> f32) {
        const GRAVITY: f32 = 9.81;

        for explosion in &mut self.explosions {
            for p in &mut explosion.particles {
                p.age += dt;
                if p.age < 0.0 { continue; }

                // Wind effect — stronger on light particles, weaker on heavy
                let wind_factor = match p.kind {
                    ParticleKind::Smoke => 1.0,     // smoke catches wind fully
                    ParticleKind::Shockwave => 0.5,  // dust picks up some
                    ParticleKind::Fireball => 0.3,   // hot gas resists a bit
                    ParticleKind::Sparks => 0.1,     // too fast to care much
                    ParticleKind::Debris => 0.05,    // heavy, barely affected
                };
                // Accelerate toward wind velocity (drag-like — approaches wind speed over time)
                let wind_accel = (wind * wind_factor - p.velocity * wind_factor) * 0.5;
                p.velocity += wind_accel * dt;

                match p.kind {
                    ParticleKind::Fireball => {
                        p.velocity *= (1.0 - 2.0 * dt).max(0.0);
                        p.velocity.y -= GRAVITY * 0.5 * dt;
                        p.size += dt * 5.0;
                    }
                    ParticleKind::Sparks => {
                        p.velocity.y -= GRAVITY * dt;
                        p.velocity *= (1.0 - 0.3 * dt).max(0.0);
                        let gy = ground_y(p.position.x, p.position.z);
                        if p.position.y < gy {
                            p.position.y = gy;
                            p.velocity.y = p.velocity.y.abs() * 0.3;
                            p.velocity *= 0.5;
                        }
                    }
                    ParticleKind::Debris => {
                        p.velocity.y -= GRAVITY * 1.5 * dt;
                        p.velocity *= (1.0 - 0.2 * dt).max(0.0);
                        let gy = ground_y(p.position.x, p.position.z);
                        if p.position.y < gy {
                            p.position.y = gy;
                            p.velocity = Vec3::ZERO;
                        }
                    }
                    ParticleKind::Shockwave => {
                        p.velocity *= (1.0 - 2.5 * dt).max(0.0);
                        p.velocity.y -= GRAVITY * 0.3 * dt;
                        p.size += dt * 8.0;
                        let gy = ground_y(p.position.x, p.position.z);
                        if p.position.y < gy { p.position.y = gy; }
                    }
                    ParticleKind::Smoke => {
                        p.velocity *= (1.0 - 1.0 * dt).max(0.0);
                        p.velocity.y += 2.0 * dt; // buoyancy
                        p.size += dt * 3.0;
                    }
                }

                p.position += p.velocity * dt;
            }

            explosion.particles.retain(|p| p.age < p.lifetime);
        }

        self.explosions.retain(|e| !e.particles.is_empty());
    }

    /// Generate billboard vertices and upload to GPU (split by blend mode).
    pub fn upload(&mut self, queue: &wgpu::Queue, camera_right: Vec3, camera_up: Vec3) {
        let mut fire_verts: Vec<ParticleVertex> = Vec::new();
        let mut smoke_verts: Vec<ParticleVertex> = Vec::new();

        for explosion in &self.explosions {
            for p in &explosion.particles {
                if p.age < 0.0 { continue; }
                let t = (p.age / p.lifetime).min(1.0);

                let (r, g, b, a) = match p.kind {
                    ParticleKind::Fireball => {
                        let brightness = (1.0 - t).powi(2);
                        (
                            p.base_color[0] * brightness * 4.0,
                            p.base_color[1] * brightness * 4.0,
                            p.base_color[2] * brightness * 4.0,
                            (1.0 - t).powi(2) * 0.95,
                        )
                    }
                    ParticleKind::Sparks => {
                        let brightness = (1.0 - t * 0.7).max(0.0);
                        (
                            p.base_color[0] * brightness * 5.0,
                            p.base_color[1] * brightness * 5.0,
                            p.base_color[2] * brightness * 3.0,
                            (1.0 - t).max(0.0) * 0.9,
                        )
                    }
                    ParticleKind::Debris => {
                        (
                            p.base_color[0],
                            p.base_color[1],
                            p.base_color[2],
                            (1.0 - t) * 0.7,
                        )
                    }
                    ParticleKind::Shockwave => {
                        let fade = (1.0 - t).powi(3);
                        (
                            p.base_color[0] * fade * 1.5,
                            p.base_color[1] * fade * 1.5,
                            p.base_color[2] * fade * 1.5,
                            fade * 0.4,
                        )
                    }
                    ParticleKind::Smoke => {
                        let fade_in = (t * 4.0).min(1.0);
                        let fade_out = (1.0 - t).powi(2);
                        (
                            p.base_color[0],
                            p.base_color[1],
                            p.base_color[2],
                            fade_in * fade_out * 0.7, // higher alpha — actually opaque dark smoke
                        )
                    }
                };

                if a < 0.001 { continue; }

                // For sparks, stretch the quad along velocity direction for streak effect
                let (right_vec, up_vec) = if p.kind == ParticleKind::Sparks && p.velocity.length_squared() > 1.0 {
                    let vel_dir = p.velocity.normalize();
                    let streak_len = p.size * 0.5 + p.velocity.length() * 0.015;
                    let streak_width = p.size * 0.3;
                    // Align quad long axis with velocity, short axis with camera
                    let side = vel_dir.cross(camera_right).normalize_or_zero();
                    let side = if side.length_squared() < 0.5 {
                        vel_dir.cross(camera_up).normalize_or_zero()
                    } else {
                        side
                    };
                    (vel_dir * streak_len, side * streak_width)
                } else {
                    let half = p.size * 0.5;
                    (camera_right * half, camera_up * half)
                };

                let color = [r, g, b, a];
                let pos = p.position;

                let tl = pos - right_vec + up_vec;
                let tr = pos + right_vec + up_vec;
                let bl = pos - right_vec - up_vec;
                let br = pos + right_vec - up_vec;

                let quad = [
                    ParticleVertex { position: tl.into(), color, uv: [-1.0, 1.0] },
                    ParticleVertex { position: tr.into(), color, uv: [1.0, 1.0] },
                    ParticleVertex { position: bl.into(), color, uv: [-1.0, -1.0] },
                    ParticleVertex { position: tr.into(), color, uv: [1.0, 1.0] },
                    ParticleVertex { position: br.into(), color, uv: [1.0, -1.0] },
                    ParticleVertex { position: bl.into(), color, uv: [-1.0, -1.0] },
                ];

                if p.kind.is_smoke() {
                    smoke_verts.extend_from_slice(&quad);
                } else {
                    fire_verts.extend_from_slice(&quad);
                }
            }
        }

        let fire_count = fire_verts.len().min(MAX_PARTICLE_VERTICES);
        if fire_count > 0 {
            queue.write_buffer(&self.fire_vertex_buffer, 0, bytemuck::cast_slice(&fire_verts[..fire_count]));
        }
        self.fire_num_vertices = fire_count as u32;

        let smoke_count = smoke_verts.len().min(MAX_PARTICLE_VERTICES);
        if smoke_count > 0 {
            queue.write_buffer(&self.smoke_vertex_buffer, 0, bytemuck::cast_slice(&smoke_verts[..smoke_count]));
        }
        self.smoke_num_vertices = smoke_count as u32;
    }

    pub fn has_particles(&self) -> bool {
        self.fire_num_vertices > 0 || self.smoke_num_vertices > 0
    }

    pub fn clear(&mut self) {
        self.explosions.clear();
        self.fire_num_vertices = 0;
        self.smoke_num_vertices = 0;
    }
}
