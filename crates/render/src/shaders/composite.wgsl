// Final compositing with tone mapping, thermal IR, and FLIR night vision

struct CompositeParams {
    exposure: f32,
    gamma: f32,
    ssao_strength: f32,
    sss_strength: f32,
    thermal_mode: f32,
    flir_mode: f32,
    _pad0: f32,
    _pad1: f32,
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

// Pseudo-random noise for FLIR sensor grain
fn flir_noise(uv: vec2<f32>) -> f32 {
    return fract(sin(dot(uv, vec2<f32>(12.9898, 78.233))) * 43758.5453);
}

// FLIR white-hot: grayscale with hot objects glowing white
fn flir_white_hot(uv: vec2<f32>, color: vec3<f32>, emission: f32) -> vec3<f32> {
    let tex_size = vec2<f32>(textureDimensions(scene_tex));

    // Base luminance from scene
    let lum = dot(color, vec3<f32>(0.2126, 0.7152, 0.0722));

    // Thermal contribution: hot objects (emission > 0) glow bright
    // Map scene brightness + thermal emission to FLIR intensity
    let thermal_intensity = lum * 0.5 + emission * 2.0;
    let flir_value = clamp(thermal_intensity, 0.0, 1.0);

    // White-hot palette: dark grey base, bright white for hot objects
    let cold = vec3<f32>(0.08, 0.08, 0.09);
    let warm = vec3<f32>(0.35, 0.35, 0.37);
    let hot = vec3<f32>(1.0, 1.0, 0.98);
    var flir_col: vec3<f32>;
    if (flir_value < 0.4) {
        flir_col = mix(cold, warm, flir_value / 0.4);
    } else {
        flir_col = mix(warm, hot, (flir_value - 0.4) / 0.6);
    }

    // Sensor noise grain — fine pixel-level noise
    let noise_uv = uv * tex_size * 0.5; // noise at half-pixel density
    let grain = (flir_noise(noise_uv) - 0.5) * 0.06;
    flir_col += vec3<f32>(grain);

    // Horizontal scanlines — subtle CRT-like effect
    let scanline_freq = tex_size.y * 0.5;
    let scanline = sin(uv.y * scanline_freq * 3.14159) * 0.5 + 0.5;
    let scanline_darken = 1.0 - (1.0 - scanline) * 0.08;
    flir_col *= scanline_darken;

    // Vignette — darken edges like a real FLIR optic
    let center = uv - vec2<f32>(0.5);
    let vignette_dist = length(center) * 1.4;
    let vignette = 1.0 - smoothstep(0.5, 1.1, vignette_dist);
    flir_col *= vignette;

    // Slight bloom on hot objects
    if (emission > 0.3) {
        let bloom = emission * 0.15;
        flir_col += vec3<f32>(bloom, bloom, bloom * 0.95);
    }

    return clamp(flir_col, vec3<f32>(0.0), vec3<f32>(1.0));
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let scene = textureSample(scene_tex, tex_sampler, in.uv);

    // ── FLIR night vision mode ──────────────────────────────────────
    if (params.flir_mode > 0.5) {
        // Sky pixels: render as dark (cold sky)
        if (scene.a < 0.01) {
            // Dark grey sky with slight noise
            let sky_noise = flir_noise(in.uv * 500.0) * 0.015;
            return vec4<f32>(0.04 + sky_noise, 0.04 + sky_noise, 0.045 + sky_noise, 1.0);
        }

        var color = scene.rgb;
        let ao = textureSample(ssao_tex, tex_sampler, in.uv).r;
        color = color * mix(1.0, ao, params.ssao_strength);
        color = color * params.exposure;
        color = aces_tonemap(color);
        color = pow(color, vec3<f32>(1.0 / params.gamma));

        let emission = max(scene.a - 1.0, 0.0);
        let flir = flir_white_hot(in.uv, color, emission);
        return vec4<f32>(flir, 1.0);
    }

    // ── Thermal IR mode ─────────────────────────────────────────────
    if (params.thermal_mode > 0.5) {
        if (scene.a < 0.01) {
            return vec4<f32>(0.02, 0.01, 0.05, 1.0);
        }

        var color = scene.rgb;
        let ao = textureSample(ssao_tex, tex_sampler, in.uv).r;
        color = color * mix(1.0, ao, params.ssao_strength);
        color = color * params.exposure;
        color = aces_tonemap(color);
        color = pow(color, vec3<f32>(1.0 / params.gamma));

        let lum = dot(color, vec3<f32>(0.2126, 0.7152, 0.0722));
        let emission = max(scene.a - 1.0, 0.0);
        let thermal_value = lum * 0.3 + emission * 0.5;
        return vec4<f32>(thermal_ramp(thermal_value), 1.0);
    }

    // ── Normal rendering ────────────────────────────────────────────
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
