// PBR Metallic-Roughness shader for SimuForge-Stone

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
};

@group(0) @binding(0) var<uniform> camera: CameraUniform;
@group(0) @binding(1) var<uniform> light: LightUniform;
@group(0) @binding(2) var<uniform> material: MaterialUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    let world_pos = vec4<f32>(in.position, 1.0);
    out.clip_position = camera.view_proj * world_pos;
    out.world_pos = in.position;
    out.world_normal = normalize(in.normal);
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

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let albedo = material.base_color.rgb;
    let roughness = material.params.x;
    let metallic = material.params.y;

    let n = normalize(in.world_normal);
    let v = normalize(camera.eye_pos.xyz - in.world_pos);
    let l = normalize(-light.direction.xyz);
    let h = normalize(v + l);

    let n_dot_l = max(dot(n, l), 0.0);
    let n_dot_v = max(dot(n, v), 0.001);
    let n_dot_h = max(dot(n, h), 0.0);
    let h_dot_v = max(dot(h, v), 0.0);

    // Fresnel reflectance at normal incidence
    let f0 = mix(vec3<f32>(0.04, 0.04, 0.04), albedo, metallic);

    // Cook-Torrance BRDF
    let d = distribution_ggx(n_dot_h, roughness);
    let g = geometry_smith(n_dot_v, n_dot_l, roughness);
    let f = fresnel_schlick(h_dot_v, f0);

    let numerator = d * g * f;
    let denominator = 4.0 * n_dot_v * n_dot_l + 0.0001;
    let specular = numerator / denominator;

    // Energy conservation
    let ks = f;
    let kd = (vec3<f32>(1.0, 1.0, 1.0) - ks) * (1.0 - metallic);

    let diffuse = kd * albedo / 3.14159265;

    let radiance = light.color.rgb * light.color.w;
    let lo = (diffuse + specular) * radiance * n_dot_l;

    // Ambient
    let ambient = light.ambient.rgb * light.ambient.w * albedo;

    var color = ambient + lo;

    // Simple procedural marble veining for subsurface materials
    let subsurface = material.params.z;
    if (subsurface > 0.0) {
        // Add subtle warm glow on thin areas (backlit approximation)
        let backlight = max(dot(n, -l), 0.0);
        let sss_color = vec3<f32>(1.0, 0.9, 0.8) * backlight * subsurface * 0.15;
        color = color + sss_color;

        // Procedural veining
        let vein = sin(in.world_pos.x * 40.0 + in.world_pos.y * 20.0 +
                       sin(in.world_pos.z * 30.0) * 2.0) * 0.5 + 0.5;
        let vein_color = mix(albedo, vec3<f32>(0.75, 0.73, 0.7), vein * 0.15);
        color = mix(color, vein_color * (ambient + radiance * n_dot_l), subsurface * 0.3);
    }

    return vec4<f32>(color, 1.0);
}
