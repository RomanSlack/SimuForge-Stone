// Screen-space ambient occlusion

struct SsaoParams {
    proj: mat4x4<f32>,
    radius: f32,
    bias: f32,
    intensity: f32,
    _pad: f32,
};

@group(0) @binding(0) var depth_tex: texture_depth_2d;
@group(0) @binding(1) var depth_sampler: sampler;
@group(0) @binding(2) var<uniform> params: SsaoParams;

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

// Full-screen triangle
@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    var out: VertexOutput;
    // Generate full-screen triangle from vertex index
    let x = f32(i32(vertex_index & 1u) * 4 - 1);
    let y = f32(i32(vertex_index & 2u) * 2 - 1);
    out.position = vec4<f32>(x, y, 0.0, 1.0);
    out.uv = vec2<f32>((x + 1.0) * 0.5, (1.0 - y) * 0.5);
    return out;
}

// Simple hash for pseudo-random sampling
fn hash(p: vec2<f32>) -> f32 {
    let h = dot(p, vec2<f32>(127.1, 311.7));
    return fract(sin(h) * 43758.5453123);
}

fn reconstruct_position(uv: vec2<f32>, depth: f32) -> vec3<f32> {
    let ndc = vec4<f32>(uv * 2.0 - 1.0, depth, 1.0);
    // Approximate view-space position from depth
    let z = params.proj[3][2] / (depth - params.proj[2][2]);
    let x = (uv.x * 2.0 - 1.0) * z / params.proj[0][0];
    let y = (uv.y * 2.0 - 1.0) * z / params.proj[1][1];
    return vec3<f32>(x, y, z);
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) f32 {
    let depth = textureSample(depth_tex, depth_sampler, in.uv);

    if (depth >= 1.0) {
        return 1.0; // Sky — no occlusion
    }

    let pos = reconstruct_position(in.uv, depth);
    var occlusion = 0.0;
    let kernel_size = 16u;

    for (var i = 0u; i < kernel_size; i = i + 1u) {
        let angle = f32(i) * 2.39996323 + hash(in.uv + vec2<f32>(f32(i), 0.0)); // golden angle
        let r = (f32(i) + 0.5) / f32(kernel_size);
        let offset = vec2<f32>(cos(angle), sin(angle)) * r * params.radius * 0.01;

        let sample_uv = in.uv + offset;
        let sample_depth = textureSample(depth_tex, depth_sampler, sample_uv);
        let sample_pos = reconstruct_position(sample_uv, sample_depth);

        let diff = pos.z - sample_pos.z;
        if (diff > params.bias && diff < params.radius) {
            occlusion = occlusion + 1.0;
        }
    }

    occlusion = occlusion / f32(kernel_size);
    return 1.0 - occlusion * params.intensity;
}
