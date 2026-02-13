//! PBR metallic-roughness rendering pipeline.

use crate::camera::CameraUniform;
use crate::context::{RenderContext, DEPTH_FORMAT, HDR_FORMAT};
use crate::mesh::vertex_buffer_layout;
use crate::pipelines::shadow::ShadowPipeline;

/// Light uniform data.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LightUniform {
    /// Directional light direction (world space, normalized).
    pub direction: [f32; 4],
    /// Directional light color and intensity.
    pub color: [f32; 4],
    /// Ambient light color and intensity.
    pub ambient: [f32; 4],
    /// Camera/eye position (world space).
    pub eye_pos: [f32; 4],
}

const IDENTITY_MAT4: [[f32; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// Material properties for PBR (must match MaterialUniform in pbr.wgsl).
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MaterialUniform {
    /// Base color (RGB) + alpha.
    pub base_color: [f32; 4],
    /// Roughness, metallic, subsurface, grid_flag.
    pub params: [f32; 4],
    /// Model matrix (object → world).
    pub model: [[f32; 4]; 4],
}

impl MaterialUniform {
    /// White marble material.
    pub fn marble() -> Self {
        Self {
            base_color: [0.92, 0.91, 0.88, 1.0],
            params: [0.3, 0.0, 0.5, 0.0],
            model: IDENTITY_MAT4,
        }
    }

    /// Metal material (for arm links).
    pub fn metal(color: [f32; 4]) -> Self {
        Self {
            base_color: color,
            params: [0.4, 0.9, 0.0, 0.0],
            model: IDENTITY_MAT4,
        }
    }

    /// Ground plane material with grid flag.
    pub fn ground() -> Self {
        Self {
            base_color: [0.25, 0.27, 0.30, 1.0],
            params: [0.9, 0.0, 0.0, -1.0], // params.w < 0 triggers grid in shader
            model: IDENTITY_MAT4,
        }
    }

    /// Set model matrix, returning modified copy.
    pub fn with_model(mut self, model: [[f32; 4]; 4]) -> Self {
        self.model = model;
        self
    }
}

/// PBR render pipeline and associated resources.
pub struct PbrPipeline {
    pub pipeline: wgpu::RenderPipeline,
    pub camera_buffer: wgpu::Buffer,
    pub light_buffer: wgpu::Buffer,
    pub material_buffer: wgpu::Buffer,
    pub bind_group_layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
    // Shadow map resources (group 1)
    pub shadow_bind_group_layout: wgpu::BindGroupLayout,
    pub shadow_bind_group: wgpu::BindGroup,
    pub shadow_light_vp_buffer: wgpu::Buffer,
}

impl PbrPipeline {
    pub fn new(ctx: &RenderContext, shadow: &ShadowPipeline) -> Self {
        let shader =
            ctx.device
                .create_shader_module(wgpu::ShaderModuleDescriptor {
                    label: Some("PBR Shader"),
                    source: wgpu::ShaderSource::Wgsl(
                        include_str!("../shaders/pbr.wgsl").into(),
                    ),
                });

        let camera_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Camera Uniform"),
            size: std::mem::size_of::<CameraUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let light_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Light Uniform"),
            size: std::mem::size_of::<LightUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let material_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Material Uniform"),
            size: std::mem::size_of::<MaterialUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Group 0: per-material (camera, light, material)
        let bind_group_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("PBR Bind Group Layout"),
                    entries: &[
                        wgpu::BindGroupLayoutEntry {
                            binding: 0,
                            visibility: wgpu::ShaderStages::VERTEX
                                | wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Uniform,
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                        wgpu::BindGroupLayoutEntry {
                            binding: 1,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Uniform,
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                        wgpu::BindGroupLayoutEntry {
                            binding: 2,
                            visibility: wgpu::ShaderStages::VERTEX
                                | wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Uniform,
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                    ],
                });

        let bind_group = ctx
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("PBR Bind Group"),
                layout: &bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: camera_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: light_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: material_buffer.as_entire_binding(),
                    },
                ],
            });

        // Group 1: shadow map (depth texture, comparison sampler, light VP matrix)
        let shadow_light_vp_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Shadow Light VP (PBR)"),
            size: 64, // mat4x4<f32>
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let shadow_bind_group_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("PBR Shadow Bind Group Layout"),
                    entries: &[
                        // Shadow depth texture
                        wgpu::BindGroupLayoutEntry {
                            binding: 0,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Texture {
                                sample_type: wgpu::TextureSampleType::Depth,
                                view_dimension: wgpu::TextureViewDimension::D2,
                                multisampled: false,
                            },
                            count: None,
                        },
                        // Shadow comparison sampler
                        wgpu::BindGroupLayoutEntry {
                            binding: 1,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Sampler(
                                wgpu::SamplerBindingType::Comparison,
                            ),
                            count: None,
                        },
                        // Light VP matrix
                        wgpu::BindGroupLayoutEntry {
                            binding: 2,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Buffer {
                                ty: wgpu::BufferBindingType::Uniform,
                                has_dynamic_offset: false,
                                min_binding_size: None,
                            },
                            count: None,
                        },
                    ],
                });

        let shadow_bind_group = ctx
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("PBR Shadow Bind Group"),
                layout: &shadow_bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&shadow.depth_view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Sampler(&shadow.sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: shadow_light_vp_buffer.as_entire_binding(),
                    },
                ],
            });

        let pipeline_layout =
            ctx.device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("PBR Pipeline Layout"),
                    bind_group_layouts: &[&bind_group_layout, &shadow_bind_group_layout],
                    push_constant_ranges: &[],
                });

        let pipeline =
            ctx.device
                .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("PBR Pipeline"),
                    layout: Some(&pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &shader,
                        entry_point: Some("vs_main"),
                        buffers: &[vertex_buffer_layout()],
                        compilation_options: Default::default(),
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &shader,
                        entry_point: Some("fs_main"),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: HDR_FORMAT,
                            blend: Some(wgpu::BlendState::REPLACE),
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                        compilation_options: Default::default(),
                    }),
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::TriangleList,
                        front_face: wgpu::FrontFace::Ccw,
                        cull_mode: Some(wgpu::Face::Back),
                        ..Default::default()
                    },
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: DEPTH_FORMAT,
                        depth_write_enabled: true,
                        depth_compare: wgpu::CompareFunction::Less,
                        stencil: wgpu::StencilState::default(),
                        bias: wgpu::DepthBiasState::default(),
                    }),
                    multisample: wgpu::MultisampleState::default(),
                    multiview: None,
                    cache: None,
                });

        Self {
            pipeline,
            camera_buffer,
            light_buffer,
            material_buffer,
            bind_group_layout,
            bind_group,
            shadow_bind_group_layout,
            shadow_bind_group,
            shadow_light_vp_buffer,
        }
    }

    /// Update camera uniform.
    pub fn update_camera(&self, queue: &wgpu::Queue, uniform: &CameraUniform) {
        queue.write_buffer(&self.camera_buffer, 0, bytemuck::bytes_of(uniform));
    }

    /// Update light uniform.
    pub fn update_light(&self, queue: &wgpu::Queue, uniform: &LightUniform) {
        queue.write_buffer(&self.light_buffer, 0, bytemuck::bytes_of(uniform));
    }

    /// Update material uniform.
    pub fn update_material(&self, queue: &wgpu::Queue, uniform: &MaterialUniform) {
        queue.write_buffer(&self.material_buffer, 0, bytemuck::bytes_of(uniform));
    }

    /// Update the shadow light VP matrix (raw, without model — for fragment shader shadow coords).
    pub fn update_shadow_light_vp(&self, queue: &wgpu::Queue, light_vp: &glam::Mat4) {
        queue.write_buffer(
            &self.shadow_light_vp_buffer,
            0,
            bytemuck::bytes_of(&light_vp.to_cols_array_2d()),
        );
    }

    /// Create a separate material bind group (shares camera/light buffers).
    /// Returns (material_buffer, bind_group) so caller can update material per-frame.
    pub fn create_material_bind_group(
        &self,
        device: &wgpu::Device,
    ) -> (wgpu::Buffer, wgpu::BindGroup) {
        let material_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Material Uniform (extra)"),
            size: std::mem::size_of::<MaterialUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("PBR Bind Group (extra)"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.camera_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: self.light_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: material_buffer.as_entire_binding(),
                },
            ],
        });
        (material_buffer, bind_group)
    }
}
