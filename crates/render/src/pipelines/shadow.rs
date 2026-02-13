//! Shadow map depth pass pipeline.

use crate::context::RenderContext;
use crate::mesh::vertex_buffer_layout;

/// Shadow map resolution.
pub const SHADOW_MAP_SIZE: u32 = 2048;

/// Maximum number of shadow-casting objects per frame.
pub const MAX_SHADOW_OBJECTS: usize = 32;

/// Uniform buffer offset alignment (conservative value matching wgpu default limits).
const UNIFORM_ALIGNMENT: usize = 256;

/// Shadow map pipeline and resources.
pub struct ShadowPipeline {
    pub pipeline: wgpu::RenderPipeline,
    pub depth_texture: wgpu::Texture,
    pub depth_view: wgpu::TextureView,
    pub sampler: wgpu::Sampler,
    pub light_matrix_buffer: wgpu::Buffer,
    pub bind_group_layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
}

/// Light-space matrix uniform.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LightMatrixUniform {
    pub light_vp: [[f32; 4]; 4],
}

impl ShadowPipeline {
    pub fn new(ctx: &RenderContext) -> Self {
        let shader =
            ctx.device
                .create_shader_module(wgpu::ShaderModuleDescriptor {
                    label: Some("Shadow Shader"),
                    source: wgpu::ShaderSource::Wgsl(
                        include_str!("../shaders/shadow.wgsl").into(),
                    ),
                });

        let depth_texture = ctx.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Shadow Depth Texture"),
            size: wgpu::Extent3d {
                width: SHADOW_MAP_SIZE,
                height: SHADOW_MAP_SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&Default::default());

        let sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Shadow Sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            compare: Some(wgpu::CompareFunction::LessEqual),
            ..Default::default()
        });

        // Dynamic uniform buffer: holds up to MAX_SHADOW_OBJECTS matrices, each
        // aligned to UNIFORM_ALIGNMENT bytes. This allows per-object shadow matrices
        // within a single render pass using dynamic offsets.
        let light_matrix_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Shadow Light Matrices"),
            size: (MAX_SHADOW_OBJECTS * UNIFORM_ALIGNMENT) as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("Shadow Bind Group Layout"),
                    entries: &[wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::VERTEX,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: true,
                            min_binding_size: wgpu::BufferSize::new(64), // mat4x4<f32>
                        },
                        count: None,
                    }],
                });

        let bind_group = ctx
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Shadow Bind Group"),
                layout: &bind_group_layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                        buffer: &light_matrix_buffer,
                        offset: 0,
                        size: wgpu::BufferSize::new(64), // one mat4x4<f32>
                    }),
                }],
            });

        let pipeline_layout =
            ctx.device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("Shadow Pipeline Layout"),
                    bind_group_layouts: &[&bind_group_layout],
                    push_constant_ranges: &[],
                });

        let pipeline =
            ctx.device
                .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("Shadow Pipeline"),
                    layout: Some(&pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &shader,
                        entry_point: Some("vs_main"),
                        buffers: &[vertex_buffer_layout()],
                        compilation_options: Default::default(),
                    },
                    fragment: None, // Depth-only pass
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::TriangleList,
                        front_face: wgpu::FrontFace::Ccw,
                        cull_mode: Some(wgpu::Face::Back),
                        ..Default::default()
                    },
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: wgpu::TextureFormat::Depth32Float,
                        depth_write_enabled: true,
                        depth_compare: wgpu::CompareFunction::Less,
                        stencil: wgpu::StencilState::default(),
                        bias: wgpu::DepthBiasState {
                            constant: 2,
                            slope_scale: 2.0,
                            clamp: 0.0,
                        },
                    }),
                    multisample: wgpu::MultisampleState::default(),
                    multiview: None,
                    cache: None,
                });

        Self {
            pipeline,
            depth_texture,
            depth_view,
            sampler,
            light_matrix_buffer,
            bind_group_layout,
            bind_group,
        }
    }

    /// Compute orthographic light-space VP matrix for directional light.
    pub fn compute_light_matrix(light_dir: glam::Vec3, scene_radius: f32) -> glam::Mat4 {
        let light_pos = -light_dir.normalize() * scene_radius * 2.0;
        let view = glam::Mat4::look_at_rh(light_pos, glam::Vec3::ZERO, glam::Vec3::Y);
        let proj = glam::Mat4::orthographic_rh(
            -scene_radius,
            scene_radius,
            -scene_radius,
            scene_radius,
            0.1,
            scene_radius * 4.0,
        );
        proj * view
    }

    /// Upload an array of pre-multiplied `light_vp * model` matrices for shadow rendering.
    /// Each matrix is placed at UNIFORM_ALIGNMENT intervals for dynamic offset access.
    pub fn upload_matrices(&self, queue: &wgpu::Queue, matrices: &[glam::Mat4]) {
        assert!(matrices.len() <= MAX_SHADOW_OBJECTS);
        let mut data = vec![0u8; matrices.len() * UNIFORM_ALIGNMENT];
        for (i, m) in matrices.iter().enumerate() {
            let cols = m.to_cols_array_2d();
            let bytes = bytemuck::bytes_of(&cols);
            let offset = i * UNIFORM_ALIGNMENT;
            data[offset..offset + 64].copy_from_slice(bytes);
        }
        queue.write_buffer(&self.light_matrix_buffer, 0, &data);
    }

    /// Dynamic offset for the i-th shadow object.
    pub fn dynamic_offset(index: usize) -> u32 {
        (index * UNIFORM_ALIGNMENT) as u32
    }
}
