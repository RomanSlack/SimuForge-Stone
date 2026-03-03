//! Equirectangular HDR skybox with adjustable exposure.
//!
//! Inverse view-projection matrix computed on CPU (no shader matrix inverse).
//! Uses FLOAT32_FILTERABLE for bilinear-filtered HDR sampling.

use simuforge_render::context::RenderContext;
use wgpu::util::DeviceExt;

/// GPU sky uniforms: inv_vp + exposure + horizon_dust.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct SkyUniforms {
    pub inv_view_proj: [[f32; 4]; 4],
    pub exposure: f32,
    pub horizon_dust: f32,
    pub _pad: [f32; 2],
}

pub struct SkyboxPipeline {
    pub pipeline: wgpu::RenderPipeline,
    pub bind_group: wgpu::BindGroup,
    pub uniform_buffer: wgpu::Buffer,
}

impl SkyboxPipeline {
    pub fn new(ctx: &RenderContext, hdr_path: &str) -> Self {
        // Load HDR
        let file = std::fs::File::open(hdr_path)
            .unwrap_or_else(|e| panic!("Failed to open HDR '{hdr_path}': {e}"));
        let reader = std::io::BufReader::new(file);
        let decoder = image::codecs::hdr::HdrDecoder::new(reader)
            .unwrap_or_else(|e| panic!("Failed to decode HDR: {e}"));
        let meta = decoder.metadata();
        let width = meta.width;
        let height = meta.height;
        let num_pixels = (width * height) as usize;
        let mut raw_bytes = vec![0u8; num_pixels * 12]; // Rgb32F = 12 bytes/pixel
        use image::ImageDecoder;
        decoder.read_image(&mut raw_bytes)
            .unwrap_or_else(|e| panic!("Failed to read HDR: {e}"));

        // Rgb32F → Rgba32Float
        let mut rgba = Vec::with_capacity(num_pixels * 16);
        for i in 0..num_pixels {
            let off = i * 12;
            rgba.extend_from_slice(&raw_bytes[off..off + 12]);
            rgba.extend_from_slice(bytemuck::bytes_of(&1.0_f32));
        }

        let tex_size = wgpu::Extent3d { width, height, depth_or_array_layers: 1 };
        let texture = ctx.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("HDR Skybox"),
            size: tex_size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba32Float,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        ctx.queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &texture, mip_level: 0,
                origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All,
            },
            &rgba,
            wgpu::TexelCopyBufferLayout {
                offset: 0, bytes_per_row: Some(width * 16), rows_per_image: Some(height),
            },
            tex_size,
        );
        let tex_view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        // Filtering sampler (FLOAT32_FILTERABLE required on device)
        let sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("HDR Sampler"),
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        // Uniform buffer
        let uniforms = SkyUniforms {
            inv_view_proj: glam::Mat4::IDENTITY.to_cols_array_2d(),
            exposure: 0.4,
            horizon_dust: 0.5,
            _pad: [0.0; 2],
        };
        let uniform_buffer = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Sky Uniforms"),
            contents: bytemuck::bytes_of(&uniforms),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let shader = ctx.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Skybox Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/skybox.wgsl").into()),
        });

        let bgl = ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Skybox BGL"),
            entries: &[
                // Sky uniforms (inv_vp + exposure)
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // HDR texture (filterable float32)
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                // Sampler
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });

        let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Skybox BG"),
            layout: &bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: uniform_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(&tex_view) },
                wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::Sampler(&sampler) },
            ],
        });

        let pipeline_layout = ctx.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Skybox PL"),
            bind_group_layouts: &[&bgl],
            push_constant_ranges: &[],
        });

        let pipeline = ctx.device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Skybox Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader, entry_point: Some("vs_main"),
                buffers: &[], compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader, entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: ctx.format(), blend: None, write_mask: wgpu::ColorWrites::ALL,
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

        Self { pipeline, bind_group, uniform_buffer }
    }

    /// Upload inverse VP, exposure, and horizon dust to GPU. Call once per frame before render.
    pub fn update(&self, queue: &wgpu::Queue, inv_view_proj: glam::Mat4, exposure: f32, horizon_dust: f32) {
        let uniforms = SkyUniforms {
            inv_view_proj: inv_view_proj.to_cols_array_2d(),
            exposure,
            horizon_dust,
            _pad: [0.0; 2],
        };
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&uniforms));
    }

    pub fn render(&self, encoder: &mut wgpu::CommandEncoder, target: &wgpu::TextureView) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("Skybox Pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: None,
            ..Default::default()
        });
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind_group, &[]);
        pass.draw(0..3, 0..1);
    }
}
