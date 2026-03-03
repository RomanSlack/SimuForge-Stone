// Final compositing with tone mapping and gamma correction

struct CompositeParams {
    exposure: f32,
    gamma: f32,
    ssao_strength: f32,
    sss_strength: f32,
    thermal_mode: f32,
    _pad0: f32,
    _pad1: f32,
    _pad2: f32,
};

@group(0) @binding(0) var scene_tex: texture_2d<f32>;
@group(0) @binding(1) var tex_sampler: sampler;
@group(0) @binding(2) var<uniform> params: CompositeParams;
@group(0) @binding(3) var ssao_tex: texture_2d<f32>;

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

// Thermal IR color ramp: black → blue → purple → red → yellow → white
fn thermal_ramp(t: f32) -> vec3<f32> {
    let tc = clamp(t, 0.0, 1.0);
    if (tc < 0.2) {
        let f = tc / 0.2;
        return mix(vec3<f32>(0.0, 0.0, 0.0), vec3<f32>(0.0, 0.0, 0.8), f);
    } else if (tc < 0.4) {
        let f = (tc - 0.2) / 0.2;
        return mix(vec3<f32>(0.0, 0.0, 0.8), vec3<f32>(0.6, 0.0, 0.8), f);
    } else if (tc < 0.6) {
        let f = (tc - 0.4) / 0.2;
        return mix(vec3<f32>(0.6, 0.0, 0.8), vec3<f32>(1.0, 0.0, 0.0), f);
    } else if (tc < 0.8) {
        let f = (tc - 0.6) / 0.2;
        return mix(vec3<f32>(1.0, 0.0, 0.0), vec3<f32>(1.0, 1.0, 0.0), f);
    } else {
        let f = (tc - 0.8) / 0.2;
        return mix(vec3<f32>(1.0, 1.0, 0.0), vec3<f32>(1.0, 1.0, 1.0), f);
    }
}

// ACES filmic tone mapping
fn aces_tonemap(color: vec3<f32>) -> vec3<f32> {
    let a = 2.51;
    let b = 0.03;
    let c = 2.43;
    let d = 0.59;
    let e = 0.14;
    return clamp((color * (a * color + b)) / (color * (c * color + d) + e), vec3<f32>(0.0), vec3<f32>(1.0));
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let scene = textureSample(scene_tex, tex_sampler, in.uv);

    // Thermal IR mode
    if (params.thermal_mode > 0.5) {
        // Sky pixels: render as near-black (cold sky)
        if (scene.a < 0.01) {
            return vec4<f32>(0.02, 0.01, 0.05, 1.0);
        }

        var color = scene.rgb;
        let ao = textureSample(ssao_tex, tex_sampler, in.uv).r;
        color = color * mix(1.0, ao, params.ssao_strength);
        color = color * params.exposure;
        color = aces_tonemap(color);
        color = pow(color, vec3<f32>(1.0 / params.gamma));

        // Luminance from tone-mapped color
        let lum = dot(color, vec3<f32>(0.2126, 0.7152, 0.0722));
        // Thermal emission encoded in alpha (>1.0 = hot)
        let emission = max(scene.a - 1.0, 0.0);
        let thermal_value = lum * 0.3 + emission * 0.5;
        return vec4<f32>(thermal_ramp(thermal_value), 1.0);
    }

    // Discard sky pixels (HDR pass outputs alpha=0 where no geometry was drawn)
    if (scene.a < 0.01) {
        discard;
    }

    var color = scene.rgb;

    // Apply SSAO
    let ao = textureSample(ssao_tex, tex_sampler, in.uv).r;
    color = color * mix(1.0, ao, params.ssao_strength);

    // Apply exposure
    color = color * params.exposure;

    // ACES tone mapping
    color = aces_tonemap(color);

    // Gamma correction
    color = pow(color, vec3<f32>(1.0 / params.gamma));

    return vec4<f32>(color, 1.0);
}
