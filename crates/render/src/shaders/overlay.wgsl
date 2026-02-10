// 2D overlay shader for text and UI rectangles.

struct ScreenUniform {
    screen_size: vec2<f32>,
    _pad: vec2<f32>,
};

@group(0) @binding(0) var<uniform> screen: ScreenUniform;
@group(0) @binding(1) var font_tex: texture_2d<f32>;
@group(0) @binding(2) var font_samp: sampler;

struct VertexInput {
    @location(0) position: vec2<f32>,
    @location(1) texcoord: vec2<f32>,
    @location(2) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    // Pixel coords → NDC: (0,0)=top-left, (w,h)=bottom-right
    let ndc = vec2<f32>(
        in.position.x / screen.screen_size.x * 2.0 - 1.0,
        1.0 - in.position.y / screen.screen_size.y * 2.0,
    );
    out.clip_position = vec4<f32>(ndc, 0.0, 1.0);
    out.uv = in.texcoord;
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Negative U = solid colored rect (no texture lookup)
    if (in.uv.x < 0.0) {
        return in.color;
    }
    let alpha = textureSample(font_tex, font_samp, in.uv).r;
    return vec4<f32>(in.color.rgb, in.color.a * alpha);
}
