// Subsurface scattering diffusion blur for marble

struct SssParams {
    scatter_radius: f32,
    strength: f32,
    scatter_color: vec3<f32>,
    _pad: f32,
    screen_size: vec2<f32>,
    _pad2: vec2<f32>,
};

@group(0) @binding(0) var color_tex: texture_2d<f32>;
@group(0) @binding(1) var depth_tex: texture_depth_2d;
@group(0) @binding(2) var tex_sampler: sampler;
@group(0) @binding(3) var<uniform> params: SssParams;

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    var out: VertexOutput;
    let x = f32(i32(vertex_index & 1u) * 4 - 1);
    let y = f32(i32(vertex_index & 2u) * 2 - 1);
    out.position = vec4<f32>(x, y, 0.0, 1.0);
    out.uv = vec2<f32>((x + 1.0) * 0.5, (1.0 - y) * 0.5);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let center_color = textureSample(color_tex, tex_sampler, in.uv);
    let center_depth = textureSample(depth_tex, tex_sampler, in.uv);

    if (center_depth >= 1.0 || params.strength <= 0.0) {
        return center_color;
    }

    let texel_size = 1.0 / params.screen_size;
    var blur_color = vec3<f32>(0.0, 0.0, 0.0);
    var total_weight = 0.0;

    // Gaussian-weighted blur samples
    let num_samples = 9;
    let offsets = array<vec2<f32>, 9>(
        vec2<f32>(-1.0, -1.0), vec2<f32>(0.0, -1.0), vec2<f32>(1.0, -1.0),
        vec2<f32>(-1.0,  0.0), vec2<f32>(0.0,  0.0), vec2<f32>(1.0,  0.0),
        vec2<f32>(-1.0,  1.0), vec2<f32>(0.0,  1.0), vec2<f32>(1.0,  1.0),
    );
    let weights = array<f32, 9>(
        0.0625, 0.125, 0.0625,
        0.125,  0.25,  0.125,
        0.0625, 0.125, 0.0625,
    );

    for (var i = 0; i < num_samples; i = i + 1) {
        let sample_uv = in.uv + offsets[i] * texel_size * params.scatter_radius;
        let sample_color = textureSample(color_tex, tex_sampler, sample_uv);
        let sample_depth = textureSample(depth_tex, tex_sampler, sample_uv);

        // Depth-aware weighting: don't blur across depth discontinuities
        let depth_diff = abs(center_depth - sample_depth);
        let depth_weight = exp(-depth_diff * 1000.0);

        let w = weights[i] * depth_weight;
        blur_color = blur_color + sample_color.rgb * w;
        total_weight = total_weight + w;
    }

    if (total_weight > 0.0) {
        blur_color = blur_color / total_weight;
    }

    // Tint the scattered light with subsurface color
    let sss_contribution = blur_color * params.scatter_color * params.strength;
    let final_color = mix(center_color.rgb, center_color.rgb + sss_contribution, params.strength);

    return vec4<f32>(final_color, center_color.a);
}
