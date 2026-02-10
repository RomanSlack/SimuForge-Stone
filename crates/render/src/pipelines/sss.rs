//! Subsurface scattering approximation for marble rendering.
//!
//! Uses a screen-space diffusion blur on the irradiance buffer to simulate
//! light transport through translucent marble.

use crate::context::RenderContext;

/// SSS parameters.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct SssParams {
    /// Subsurface scattering radius in screen pixels.
    pub scatter_radius: f32,
    /// Scatter intensity/strength.
    pub strength: f32,
    /// Subsurface color tint (warm for marble).
    pub scatter_color: [f32; 3],
    /// Screen dimensions for texel calculations.
    pub _pad: f32,
    pub screen_size: [f32; 2],
    pub _pad2: [f32; 2],
}

impl SssParams {
    pub fn marble(width: f32, height: f32) -> Self {
        Self {
            scatter_radius: 8.0,
            strength: 0.5,
            scatter_color: [1.0, 0.85, 0.75], // warm tint for marble
            _pad: 0.0,
            screen_size: [width, height],
            _pad2: [0.0, 0.0],
        }
    }
}

/// SSS blur pipeline.
pub struct SssPipeline {
    pub pipeline: wgpu::RenderPipeline,
    pub params_buffer: wgpu::Buffer,
    pub bind_group_layout: wgpu::BindGroupLayout,
    pub output_texture: wgpu::Texture,
    pub output_view: wgpu::TextureView,
}

impl SssPipeline {
    pub fn new(ctx: &RenderContext) -> Self {
        let shader =
            ctx.device
                .create_shader_module(wgpu::ShaderModuleDescriptor {
                    label: Some("SSS Shader"),
                    source: wgpu::ShaderSource::Wgsl(
                        include_str!("../shaders/sss.wgsl").into(),
                    ),
                });

        let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("SSS Params"),
            size: std::mem::size_of::<SssParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("SSS Bind Group Layout"),
                    entries: &[
                        // Input color texture
                        wgpu::BindGroupLayoutEntry {
                            binding: 0,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Texture {
                                sample_type: wgpu::TextureSampleType::Float { filterable: true },
                                view_dimension: wgpu::TextureViewDimension::D2,
                                multisampled: false,
                            },
                            count: None,
                        },
                        // Depth texture
                        wgpu::BindGroupLayoutEntry {
                            binding: 1,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Texture {
                                sample_type: wgpu::TextureSampleType::Depth,
                                view_dimension: wgpu::TextureViewDimension::D2,
                                multisampled: false,
                            },
                            count: None,
                        },
                        // Sampler
                        wgpu::BindGroupLayoutEntry {
                            binding: 2,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Sampler(
                                wgpu::SamplerBindingType::Filtering,
                            ),
                            count: None,
                        },
                        // Params
                        wgpu::BindGroupLayoutEntry {
                            binding: 3,
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

        let output_texture = create_sss_texture(&ctx.device, &ctx.config);
        let output_view = output_texture.create_view(&Default::default());

        let pipeline_layout =
            ctx.device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("SSS Pipeline Layout"),
                    bind_group_layouts: &[&bind_group_layout],
                    push_constant_ranges: &[],
                });

        let pipeline =
            ctx.device
                .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("SSS Pipeline"),
                    layout: Some(&pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &shader,
                        entry_point: Some("vs_main"),
                        buffers: &[],
                        compilation_options: Default::default(),
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &shader,
                        entry_point: Some("fs_main"),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: ctx.format(),
                            blend: None,
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                        compilation_options: Default::default(),
                    }),
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::TriangleList,
                        ..Default::default()
                    },
                    depth_stencil: None,
                    multisample: wgpu::MultisampleState::default(),
                    multiview: None,
                    cache: None,
                });

        Self {
            pipeline,
            params_buffer,
            bind_group_layout,
            output_texture,
            output_view,
        }
    }

    pub fn resize(&mut self, device: &wgpu::Device, config: &wgpu::SurfaceConfiguration) {
        self.output_texture = create_sss_texture(device, config);
        self.output_view = self.output_texture.create_view(&Default::default());
    }
}

fn create_sss_texture(
    device: &wgpu::Device,
    config: &wgpu::SurfaceConfiguration,
) -> wgpu::Texture {
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some("SSS Output"),
        size: wgpu::Extent3d {
            width: config.width.max(1),
            height: config.height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: config.format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    })
}
