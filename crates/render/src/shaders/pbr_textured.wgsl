// PBR Metallic-Roughness shader with satellite texture for terrain tiles

struct CameraUniform {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    proj: mat4x4<f32>,
    eye_pos: vec4<f32>,
};

struct LightUniform {
    direction: vec4<f32>,
    color: vec4<f32>,
    ambient: vec4<f32>,
    eye_pos: vec4<f32>,
};

struct MaterialUniform {
    base_color: vec4<f32>,
    params: vec4<f32>, // roughness, metallic, subsurface, pad
    model: mat4x4<f32>,
    bounds_min: vec4<f32>,
    bounds_max: vec4<f32>,
};

// Group 0: per-material uniforms
@group(0) @binding(0) var<uniform> camera: CameraUniform;
@group(0) @binding(1) var<uniform> light: LightUniform;
@group(0) @binding(2) var<uniform> material: MaterialUniform;

// Group 1: shadow map resources
@group(1) @binding(0) var shadow_map: texture_depth_2d;
@group(1) @binding(1) var shadow_sampler: sampler_comparison;
@group(1) @binding(2) var<uniform> shadow_light_vp: mat4x4<f32>;

// Group 2: satellite texture
@group(2) @binding(0) var satellite_texture: texture_2d<f32>;
@group(2) @binding(1) var satellite_sampler: sampler;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

// Cofactor matrix for correct normal transform under non-uniform scaling.
fn cofactor_mat3(m: mat3x3<f32>) -> mat3x3<f32> {
    return mat3x3<f32>(
        cross(m[1], m[2]),
        cross(m[2], m[0]),
        cross(m[0], m[1])
    );
}

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    let world_pos = material.model * vec4<f32>(in.position, 1.0);
    out.clip_position = camera.view_proj * world_pos;
    out.world_pos = world_pos.xyz;
    out.uv = in.uv;
    let model3 = mat3x3<f32>(
        material.model[0].xyz,
        material.model[1].xyz,
        material.model[2].xyz,
    );
    out.world_normal = normalize(cofactor_mat3(model3) * in.normal);
    return out;
}

// GGX/Trowbridge-Reitz normal distribution function
fn distribution_ggx(n_dot_h: f32, roughness: f32) -> f32 {
    let a = roughness * roughness;
    let a2 = a * a;
    let denom = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
    return a2 / (3.14159265 * denom * denom);
}

// Schlick-GGX geometry function
fn geometry_schlick_ggx(n_dot_v: f32, roughness: f32) -> f32 {
    let r = roughness + 1.0;
    let k = (r * r) / 8.0;
    return n_dot_v / (n_dot_v * (1.0 - k) + k);
}

fn geometry_smith(n_dot_v: f32, n_dot_l: f32, roughness: f32) -> f32 {
    return geometry_schlick_ggx(n_dot_v, roughness) * geometry_schlick_ggx(n_dot_l, roughness);
}

// Fresnel-Schlick approximation
fn fresnel_schlick(cos_theta: f32, f0: vec3<f32>) -> vec3<f32> {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - cos_theta, 0.0, 1.0), 5.0);
}

// 4-tap PCF shadow sampling
fn compute_shadow(world_pos: vec3<f32>) -> f32 {
    let light_space = shadow_light_vp * vec4<f32>(world_pos, 1.0);
    let ndc = light_space.xyz / light_space.w;
    let uv = vec2<f32>(ndc.x * 0.5 + 0.5, -ndc.y * 0.5 + 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || ndc.z < 0.0 || ndc.z > 1.0) {
        return 1.0;
    }
    let current_depth = ndc.z;
    let texel_size = 1.0 / 2048.0;
    var shadow = 0.0;
    shadow += textureSampleCompare(shadow_map, shadow_sampler, uv + vec2(-0.5, -0.5) * texel_size, current_depth);
    shadow += textureSampleCompare(shadow_map, shadow_sampler, uv + vec2( 0.5, -0.5) * texel_size, current_depth);
    shadow += textureSampleCompare(shadow_map, shadow_sampler, uv + vec2(-0.5,  0.5) * texel_size, current_depth);
    shadow += textureSampleCompare(shadow_map, shadow_sampler, uv + vec2( 0.5,  0.5) * texel_size, current_depth);
    shadow = shadow * 0.25;
    return shadow;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Sample satellite texture — Rgba8UnormSrgb format auto-linearizes on sample
    let albedo = textureSample(satellite_texture, satellite_sampler, in.uv).rgb;

    // Terrain is rough, non-metallic
    let roughness = 0.85;
    let metallic = 0.0;

    let n = normalize(in.world_normal);
    let v = normalize(camera.eye_pos.xyz - in.world_pos);
    let l = normalize(-light.direction.xyz);
    let h = normalize(v + l);

    let n_dot_l = max(dot(n, l), 0.0);
    let n_dot_v = max(dot(n, v), 0.001);
    let n_dot_h = max(dot(n, h), 0.0);
    let h_dot_v = max(dot(h, v), 0.0);

    let f0 = vec3<f32>(0.04, 0.04, 0.04);

    // Cook-Torrance BRDF
    let d = distribution_ggx(n_dot_h, roughness);
    let g = geometry_smith(n_dot_v, n_dot_l, roughness);
    let f = fresnel_schlick(h_dot_v, f0);

    let numerator = d * g * f;
    let denominator = 4.0 * n_dot_v * n_dot_l + 0.0001;
    let specular = numerator / denominator;

    let ks = f;
    let kd = (vec3<f32>(1.0, 1.0, 1.0) - ks) * (1.0 - metallic);
    let diffuse = kd * albedo / 3.14159265;
    let radiance = light.color.rgb * light.color.w;

    let shadow = compute_shadow(in.world_pos);
    let lo = (diffuse + specular) * radiance * n_dot_l * shadow;

    // Ambient with subtle hemisphere
    let hemisphere = dot(n, vec3(0.0, 1.0, 0.0)) * 0.5 + 0.5;
    let ambient_factor = mix(0.7, 1.0, hemisphere);
    let ambient = light.ambient.rgb * light.ambient.w * albedo * ambient_factor;

    var color = ambient + lo;

    // Distance haze: blend to horizon at far distances
    let dist = length(in.world_pos.xyz - camera.eye_pos.xyz);
    let haze_factor = smoothstep(8000.0, 15000.0, dist);
    let night_blend = light.eye_pos.w;
    let day_haze = vec3<f32>(0.72, 0.68, 0.58);
    let night_haze = vec3<f32>(0.03, 0.04, 0.08);
    let haze_color = mix(day_haze, night_haze, night_blend);
    color = mix(color, haze_color, haze_factor);

    return vec4<f32>(color, 1.0);
}
