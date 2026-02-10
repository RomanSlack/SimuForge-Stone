//! 2D overlay pipeline for text and UI elements.

use wgpu::util::DeviceExt;

/// Overlay vertex: 2D position, texcoord, color.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct OverlayVertex {
    pub position: [f32; 2],
    pub texcoord: [f32; 2],
    pub color: [f32; 4],
}

/// Screen size uniform.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ScreenUniform {
    pub screen_size: [f32; 2],
    pub _pad: [f32; 2],
}

/// Font atlas layout: 16 columns × 6 rows of 8×8 glyphs, covering ASCII 32..127.
const FONT_COLS: u32 = 16;
const FONT_ROWS: u32 = 6;
const GLYPH_W: u32 = 8;
const GLYPH_H: u32 = 8;
const ATLAS_W: u32 = FONT_COLS * GLYPH_W; // 128
const ATLAS_H: u32 = FONT_ROWS * GLYPH_H; // 48

/// 8×8 bitmap font data for ASCII 32..127 (96 characters, 8 bytes each).
/// Each byte is one row, MSB = leftmost pixel.
#[rustfmt::skip]
const FONT_DATA: [u8; 96 * 8] = [
    // 32 ' ' (space)
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    // 33 '!'
    0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x00,
    // 34 '"'
    0x6C,0x6C,0x24,0x00,0x00,0x00,0x00,0x00,
    // 35 '#'
    0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00,
    // 36 '$'
    0x18,0x7E,0xC0,0x7C,0x06,0xFC,0x18,0x00,
    // 37 '%'
    0x00,0xC6,0xCC,0x18,0x30,0x66,0xC6,0x00,
    // 38 '&'
    0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00,
    // 39 '\''
    0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00,
    // 40 '('
    0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00,
    // 41 ')'
    0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00,
    // 42 '*'
    0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,
    // 43 '+'
    0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,
    // 44 ','
    0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30,
    // 45 '-'
    0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,
    // 46 '.'
    0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,
    // 47 '/'
    0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00,
    // 48 '0'
    0x7C,0xC6,0xCE,0xD6,0xE6,0xC6,0x7C,0x00,
    // 49 '1'
    0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00,
    // 50 '2'
    0x7C,0xC6,0x06,0x1C,0x30,0x60,0xFE,0x00,
    // 51 '3'
    0x7C,0xC6,0x06,0x3C,0x06,0xC6,0x7C,0x00,
    // 52 '4'
    0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x0C,0x00,
    // 53 '5'
    0xFE,0xC0,0xFC,0x06,0x06,0xC6,0x7C,0x00,
    // 54 '6'
    0x38,0x60,0xC0,0xFC,0xC6,0xC6,0x7C,0x00,
    // 55 '7'
    0xFE,0xC6,0x0C,0x18,0x30,0x30,0x30,0x00,
    // 56 '8'
    0x7C,0xC6,0xC6,0x7C,0xC6,0xC6,0x7C,0x00,
    // 57 '9'
    0x7C,0xC6,0xC6,0x7E,0x06,0x0C,0x78,0x00,
    // 58 ':'
    0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00,
    // 59 ';'
    0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30,
    // 60 '<'
    0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00,
    // 61 '='
    0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00,
    // 62 '>'
    0x60,0x30,0x18,0x0C,0x18,0x30,0x60,0x00,
    // 63 '?'
    0x7C,0xC6,0x0C,0x18,0x18,0x00,0x18,0x00,
    // 64 '@'
    0x7C,0xC6,0xDE,0xDE,0xDE,0xC0,0x7C,0x00,
    // 65 'A'
    0x38,0x6C,0xC6,0xC6,0xFE,0xC6,0xC6,0x00,
    // 66 'B'
    0xFC,0xC6,0xC6,0xFC,0xC6,0xC6,0xFC,0x00,
    // 67 'C'
    0x3C,0x66,0xC0,0xC0,0xC0,0x66,0x3C,0x00,
    // 68 'D'
    0xF8,0xCC,0xC6,0xC6,0xC6,0xCC,0xF8,0x00,
    // 69 'E'
    0xFE,0xC0,0xC0,0xF8,0xC0,0xC0,0xFE,0x00,
    // 70 'F'
    0xFE,0xC0,0xC0,0xF8,0xC0,0xC0,0xC0,0x00,
    // 71 'G'
    0x3C,0x66,0xC0,0xCE,0xC6,0x66,0x3E,0x00,
    // 72 'H'
    0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0x00,
    // 73 'I'
    0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,
    // 74 'J'
    0x1E,0x06,0x06,0x06,0xC6,0xC6,0x7C,0x00,
    // 75 'K'
    0xC6,0xCC,0xD8,0xF0,0xD8,0xCC,0xC6,0x00,
    // 76 'L'
    0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xFE,0x00,
    // 77 'M'
    0xC6,0xEE,0xFE,0xD6,0xC6,0xC6,0xC6,0x00,
    // 78 'N'
    0xC6,0xE6,0xF6,0xDE,0xCE,0xC6,0xC6,0x00,
    // 79 'O'
    0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,
    // 80 'P'
    0xFC,0xC6,0xC6,0xFC,0xC0,0xC0,0xC0,0x00,
    // 81 'Q'
    0x7C,0xC6,0xC6,0xC6,0xD6,0xDE,0x7C,0x06,
    // 82 'R'
    0xFC,0xC6,0xC6,0xFC,0xD8,0xCC,0xC6,0x00,
    // 83 'S'
    0x7C,0xC6,0xC0,0x7C,0x06,0xC6,0x7C,0x00,
    // 84 'T'
    0xFF,0x18,0x18,0x18,0x18,0x18,0x18,0x00,
    // 85 'U'
    0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,
    // 86 'V'
    0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00,
    // 87 'W'
    0xC6,0xC6,0xC6,0xD6,0xFE,0xEE,0xC6,0x00,
    // 88 'X'
    0xC6,0xC6,0x6C,0x38,0x6C,0xC6,0xC6,0x00,
    // 89 'Y'
    0xC3,0xC3,0x66,0x3C,0x18,0x18,0x18,0x00,
    // 90 'Z'
    0xFE,0x06,0x0C,0x18,0x30,0x60,0xFE,0x00,
    // 91 '['
    0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,
    // 92 '\\'
    0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00,
    // 93 ']'
    0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,
    // 94 '^'
    0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,
    // 95 '_'
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
    // 96 '`'
    0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00,
    // 97 'a'
    0x00,0x00,0x7C,0x06,0x7E,0xC6,0x7E,0x00,
    // 98 'b'
    0xC0,0xC0,0xFC,0xC6,0xC6,0xC6,0xFC,0x00,
    // 99 'c'
    0x00,0x00,0x7C,0xC6,0xC0,0xC6,0x7C,0x00,
    // 100 'd'
    0x06,0x06,0x7E,0xC6,0xC6,0xC6,0x7E,0x00,
    // 101 'e'
    0x00,0x00,0x7C,0xC6,0xFE,0xC0,0x7C,0x00,
    // 102 'f'
    0x1C,0x36,0x30,0x78,0x30,0x30,0x30,0x00,
    // 103 'g'
    0x00,0x00,0x7E,0xC6,0xC6,0x7E,0x06,0x7C,
    // 104 'h'
    0xC0,0xC0,0xFC,0xC6,0xC6,0xC6,0xC6,0x00,
    // 105 'i'
    0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00,
    // 106 'j'
    0x0C,0x00,0x0C,0x0C,0x0C,0xCC,0xCC,0x78,
    // 107 'k'
    0xC0,0xC0,0xCC,0xD8,0xF0,0xD8,0xCC,0x00,
    // 108 'l'
    0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,
    // 109 'm'
    0x00,0x00,0xCC,0xFE,0xD6,0xC6,0xC6,0x00,
    // 110 'n'
    0x00,0x00,0xFC,0xC6,0xC6,0xC6,0xC6,0x00,
    // 111 'o'
    0x00,0x00,0x7C,0xC6,0xC6,0xC6,0x7C,0x00,
    // 112 'p'
    0x00,0x00,0xFC,0xC6,0xC6,0xFC,0xC0,0xC0,
    // 113 'q'
    0x00,0x00,0x7E,0xC6,0xC6,0x7E,0x06,0x06,
    // 114 'r'
    0x00,0x00,0xDC,0xE6,0xC0,0xC0,0xC0,0x00,
    // 115 's'
    0x00,0x00,0x7E,0xC0,0x7C,0x06,0xFC,0x00,
    // 116 't'
    0x30,0x30,0x7C,0x30,0x30,0x36,0x1C,0x00,
    // 117 'u'
    0x00,0x00,0xC6,0xC6,0xC6,0xC6,0x7E,0x00,
    // 118 'v'
    0x00,0x00,0xC6,0xC6,0xC6,0x6C,0x38,0x00,
    // 119 'w'
    0x00,0x00,0xC6,0xC6,0xD6,0xFE,0x6C,0x00,
    // 120 'x'
    0x00,0x00,0xC6,0x6C,0x38,0x6C,0xC6,0x00,
    // 121 'y'
    0x00,0x00,0xC6,0xC6,0xC6,0x7E,0x06,0x7C,
    // 122 'z'
    0x00,0x00,0xFE,0x0C,0x38,0x60,0xFE,0x00,
    // 123 '{'
    0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00,
    // 124 '|'
    0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,
    // 125 '}'
    0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00,
    // 126 '~'
    0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,
    // 127 DEL (filled block - used for solid quads)
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
];

/// Build an RGBA font atlas texture from the bitmap data.
fn build_font_atlas() -> Vec<u8> {
    let mut pixels = vec![0u8; (ATLAS_W * ATLAS_H) as usize];
    for ch in 0..96u32 {
        let col = ch % FONT_COLS;
        let row = ch / FONT_COLS;
        for y in 0..GLYPH_H {
            let byte = FONT_DATA[(ch * 8 + y) as usize];
            for x in 0..GLYPH_W {
                let px = col * GLYPH_W + x;
                let py = row * GLYPH_H + y;
                let bit = (byte >> (7 - x)) & 1;
                pixels[(py * ATLAS_W + px) as usize] = bit * 255;
            }
        }
    }
    pixels
}

/// 2D overlay pipeline for text and colored rectangles.
pub struct OverlayPipeline {
    pub pipeline: wgpu::RenderPipeline,
    pub screen_buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    max_quads: u32,
}

impl OverlayPipeline {
    /// Max overlay quads per frame.
    const MAX_QUADS: u32 = 2048;

    pub fn new(ctx: &crate::context::RenderContext) -> Self {
        let shader = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("Overlay Shader"),
                source: wgpu::ShaderSource::Wgsl(
                    include_str!("../shaders/overlay.wgsl").into(),
                ),
            });

        let screen_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Screen Uniform"),
            size: std::mem::size_of::<ScreenUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Font texture
        let atlas_data = build_font_atlas();
        let font_texture = ctx.device.create_texture_with_data(
            &ctx.queue,
            &wgpu::TextureDescriptor {
                label: Some("Font Atlas"),
                size: wgpu::Extent3d {
                    width: ATLAS_W,
                    height: ATLAS_H,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::R8Unorm,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            },
            wgpu::util::TextureDataOrder::LayerMajor,
            &atlas_data,
        );
        let font_view = font_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let font_sampler = ctx.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Font Sampler"),
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let bind_group_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("Overlay BGL"),
                    entries: &[
                        wgpu::BindGroupLayoutEntry {
                            binding: 0,
                            visibility: wgpu::ShaderStages::VERTEX,
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
                            ty: wgpu::BindingType::Texture {
                                sample_type: wgpu::TextureSampleType::Float { filterable: true },
                                view_dimension: wgpu::TextureViewDimension::D2,
                                multisampled: false,
                            },
                            count: None,
                        },
                        wgpu::BindGroupLayoutEntry {
                            binding: 2,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                            count: None,
                        },
                    ],
                });

        let bind_group = ctx
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Overlay BG"),
                layout: &bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: screen_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(&font_view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Sampler(&font_sampler),
                    },
                ],
            });

        let pipeline_layout =
            ctx.device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("Overlay Pipeline Layout"),
                    bind_group_layouts: &[&bind_group_layout],
                    push_constant_ranges: &[],
                });

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<OverlayVertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute {
                    offset: 8,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute {
                    offset: 16,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        };

        let pipeline =
            ctx.device
                .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("Overlay Pipeline"),
                    layout: Some(&pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &shader,
                        entry_point: Some("vs_main"),
                        buffers: &[vertex_layout],
                        compilation_options: Default::default(),
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &shader,
                        entry_point: Some("fs_main"),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: ctx.format(),
                            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
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

        let max_verts = Self::MAX_QUADS * 4;
        let max_indices = Self::MAX_QUADS * 6;
        let vertex_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Overlay VB"),
            size: (max_verts as usize * std::mem::size_of::<OverlayVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let index_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Overlay IB"),
            size: (max_indices as usize * std::mem::size_of::<u32>()) as u64,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            pipeline,
            screen_buffer,
            bind_group,
            vertex_buffer,
            index_buffer,
            max_quads: Self::MAX_QUADS,
        }
    }

    /// Update screen size uniform.
    pub fn update_screen_size(&self, queue: &wgpu::Queue, width: f32, height: f32) {
        let uniform = ScreenUniform {
            screen_size: [width, height],
            _pad: [0.0, 0.0],
        };
        queue.write_buffer(&self.screen_buffer, 0, bytemuck::bytes_of(&uniform));
    }
}

/// Builder for overlay quads (text + rectangles).
pub struct OverlayBuilder {
    pub vertices: Vec<OverlayVertex>,
    pub indices: Vec<u32>,
}

impl OverlayBuilder {
    pub fn new() -> Self {
        Self {
            vertices: Vec::with_capacity(4096),
            indices: Vec::with_capacity(6144),
        }
    }

    /// Add a solid colored rectangle (pixel coordinates).
    pub fn rect(&mut self, x: f32, y: f32, w: f32, h: f32, color: [f32; 4]) {
        let base = self.vertices.len() as u32;
        let uv = [-1.0, -1.0]; // Negative U signals solid color in shader
        self.vertices.push(OverlayVertex { position: [x, y], texcoord: uv, color });
        self.vertices.push(OverlayVertex { position: [x + w, y], texcoord: uv, color });
        self.vertices.push(OverlayVertex { position: [x + w, y + h], texcoord: uv, color });
        self.vertices.push(OverlayVertex { position: [x, y + h], texcoord: uv, color });
        self.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// Add a text string at pixel position (x, y) with given scale and color.
    /// Scale 1.0 = 8px glyphs, scale 2.0 = 16px, etc.
    pub fn text(&mut self, x: f32, y: f32, scale: f32, color: [f32; 4], text: &str) {
        let gw = GLYPH_W as f32 * scale;
        let gh = GLYPH_H as f32 * scale;
        let atlas_w = ATLAS_W as f32;
        let atlas_h = ATLAS_H as f32;

        let mut cx = x;
        for ch in text.chars() {
            let code = ch as u32;
            if code < 32 || code > 127 {
                cx += gw;
                continue;
            }
            let idx = code - 32;
            let col = idx % FONT_COLS;
            let row = idx / FONT_COLS;

            let u0 = (col * GLYPH_W) as f32 / atlas_w;
            let v0 = (row * GLYPH_H) as f32 / atlas_h;
            let u1 = ((col + 1) * GLYPH_W) as f32 / atlas_w;
            let v1 = ((row + 1) * GLYPH_H) as f32 / atlas_h;

            let base = self.vertices.len() as u32;
            self.vertices.push(OverlayVertex {
                position: [cx, y],
                texcoord: [u0, v0],
                color,
            });
            self.vertices.push(OverlayVertex {
                position: [cx + gw, y],
                texcoord: [u1, v0],
                color,
            });
            self.vertices.push(OverlayVertex {
                position: [cx + gw, y + gh],
                texcoord: [u1, v1],
                color,
            });
            self.vertices.push(OverlayVertex {
                position: [cx, y + gh],
                texcoord: [u0, v1],
                color,
            });
            self.indices
                .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);

            cx += gw;
        }
    }

    /// Upload and return the number of indices to draw.
    pub fn upload(&self, queue: &wgpu::Queue, pipeline: &OverlayPipeline) -> u32 {
        if self.vertices.is_empty() {
            return 0;
        }
        let count = self.indices.len().min(pipeline.max_quads as usize * 6) as u32;
        queue.write_buffer(
            &pipeline.vertex_buffer,
            0,
            bytemuck::cast_slice(&self.vertices),
        );
        queue.write_buffer(
            &pipeline.index_buffer,
            0,
            bytemuck::cast_slice(&self.indices),
        );
        count
    }
}
