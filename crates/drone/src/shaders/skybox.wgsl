// Equirectangular HDR skybox — proven approach.
// Inverse VP passed from CPU, textureSample with filtering.

struct SkyUniforms {
    inv_view_proj: mat4x4<f32>,
    exposure: f32,
    horizon_dust: f32,
    _pad0: f32,
    _pad1: f32,
};

@group(0) @binding(0) var<uniform> sky: SkyUniforms;
@group(0) @binding(1) var env_texture: texture_2d<f32>;
@group(0) @binding(2) var env_sampler: sampler;

struct VertexOutput {
    @builtin(position) pos: vec4<f32>,
    @location(0) clip_coord: vec2<f32>,
};

// Standard fullscreen oversized triangle
@vertex
fn vs_main(@builtin(vertex_index) vi: u32) -> VertexOutput {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, 3.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, -1.0),
    );
    let p = positions[vi];
    var out: VertexOutput;
    out.pos = vec4<f32>(p, 1.0, 1.0);
    out.clip_coord = p;
    return out;
}

fn direction_to_uv(dir: vec3<f32>) -> vec2<f32> {
    let d = normalize(dir);
    let PI = 3.14159265359;
    let u = atan2(d.z, d.x) / (2.0 * PI) + 0.5;
    let v = 1.0 - (asin(clamp(d.y, -1.0, 1.0)) / PI + 0.5);
    return vec2<f32>(u, v);
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Reconstruct world-space direction from clip coordinates
    let clip = vec4<f32>(in.clip_coord.x, in.clip_coord.y, 1.0, 1.0);
    let world = sky.inv_view_proj * clip;
    let direction = normalize(world.xyz / world.w);

    // Equirectangular UV
    let uv = direction_to_uv(direction);

    // Sample HDR with bilinear filtering
    let color = textureSample(env_texture, env_sampler, uv);

    // Apply exposure
    var exposed = color.rgb * sky.exposure;

    // Horizon dust: blend toward dusty haze color near horizon
    let dust_color = vec3<f32>(0.72, 0.68, 0.58);
    let elev = direction.y; // Y-up: positive = above horizon
    // smoothstep band: full dust below -0.02, fades to zero at +0.12
    let dust_band = 1.0 - smoothstep(-0.02, 0.12, elev);
    let dust_blend = dust_band * sky.horizon_dust;
    exposed = mix(exposed, dust_color * sky.exposure, dust_blend);

    // Reinhard tonemapping
    let tonemapped = exposed / (exposed + vec3<f32>(1.0));

    // Gamma correction
    let result = pow(tonemapped, vec3<f32>(1.0 / 2.2));

    return vec4<f32>(result, 1.0);
}
