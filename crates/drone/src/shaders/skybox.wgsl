// Equirectangular HDR skybox with procedural night sky.
// Day: bicubic-sampled HDR environment map.
// Night: fully procedural — deep dark sky, layered star field,
//        milky way band, atmospheric horizon glow. No HDR swap needed.

struct SkyUniforms {
    inv_view_proj: mat4x4<f32>,
    exposure: f32,
    horizon_dust: f32,
    night_blend: f32,
    _pad1: f32,
};

@group(0) @binding(0) var<uniform> sky: SkyUniforms;
@group(0) @binding(1) var env_texture: texture_2d<f32>;
@group(0) @binding(2) var env_sampler: sampler;

struct VertexOutput {
    @builtin(position) pos: vec4<f32>,
    @location(0) clip_coord: vec2<f32>,
};

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

// ── Bicubic HDR sampling (day sky) ──────────────────────────────────

fn cubic_weight(x: f32) -> vec4<f32> {
    let x2 = x * x;
    let x3 = x2 * x;
    return vec4<f32>(
        -0.5 * x3 + x2 - 0.5 * x,
         1.5 * x3 - 2.5 * x2 + 1.0,
        -1.5 * x3 + 2.0 * x2 + 0.5 * x,
         0.5 * x3 - 0.5 * x2
    );
}

fn sample_bicubic(uv: vec2<f32>) -> vec4<f32> {
    let tex_size = vec2<f32>(textureDimensions(env_texture));
    let inv_size = 1.0 / tex_size;
    let pixel = uv * tex_size - 0.5;
    let frac_p = fract(pixel);
    let base = (floor(pixel) + 0.5) * inv_size;
    let wx = cubic_weight(frac_p.x);
    let wy = cubic_weight(frac_p.y);
    let s0x = wx.x + wx.y;
    let s1x = wx.z + wx.w;
    let s0y = wy.x + wy.y;
    let s1y = wy.z + wy.w;
    let offset0x = (wx.x / s0x - 1.0) * inv_size.x;
    let offset1x = (wx.w / s1x + 1.0) * inv_size.x;
    let offset0y = (wy.x / s0y - 1.0) * inv_size.y;
    let offset1y = (wy.w / s1y + 1.0) * inv_size.y;
    let s00 = textureSample(env_texture, env_sampler, base + vec2(offset0x, offset0y));
    let s10 = textureSample(env_texture, env_sampler, base + vec2(offset1x, offset0y));
    let s01 = textureSample(env_texture, env_sampler, base + vec2(offset0x, offset1y));
    let s11 = textureSample(env_texture, env_sampler, base + vec2(offset1x, offset1y));
    let row0 = s00 * s0x + s10 * s1x;
    let row1 = s01 * s0x + s11 * s1x;
    return row0 * s0y + row1 * s1y;
}

// ── Procedural night sky ────────────────────────────────────────────

// Hash functions for star placement
fn hash2(p: vec2<f32>) -> f32 {
    return fract(sin(dot(p, vec2<f32>(127.1, 311.7))) * 43758.5453);
}

fn hash2v(p: vec2<f32>) -> vec2<f32> {
    return vec2<f32>(
        fract(sin(dot(p, vec2<f32>(127.1, 311.7))) * 43758.5453),
        fract(sin(dot(p, vec2<f32>(269.5, 183.3))) * 28947.6371)
    );
}

// Smooth noise for milky way
fn value_noise(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let a = hash2(i);
    let b = hash2(i + vec2(1.0, 0.0));
    let c = hash2(i + vec2(0.0, 1.0));
    let d = hash2(i + vec2(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

fn fbm(p: vec2<f32>) -> f32 {
    var val = 0.0;
    var amp = 0.5;
    var pos = p;
    for (var i = 0; i < 5; i++) {
        val += amp * value_noise(pos);
        pos = pos * 2.1 + vec2(1.7, 3.2);
        amp *= 0.5;
    }
    return val;
}

// Multi-layer star field: varying sizes, magnitudes, and colors
fn star_field(dir: vec3<f32>) -> vec3<f32> {
    let uv = direction_to_uv(dir);
    var stars = vec3<f32>(0.0);

    // Layer 1: Dense dim background stars (magnitude 6-8)
    let scale1 = 400.0;
    let suv1 = uv * scale1;
    let cell1 = floor(suv1);
    let h1 = hash2(cell1);
    if (h1 > 0.92) {
        let pos1 = hash2v(cell1 + vec2(5.0, 3.0));
        let d1 = length(fract(suv1) - pos1);
        let bright1 = (1.0 - smoothstep(0.0, 0.08, d1)) * (0.15 + h1 * 0.2);
        stars += vec3<f32>(0.7, 0.75, 0.9) * bright1;
    }

    // Layer 2: Medium stars (magnitude 3-5)
    let scale2 = 180.0;
    let suv2 = uv * scale2;
    let cell2 = floor(suv2);
    let h2 = hash2(cell2 + vec2(42.0, 17.0));
    if (h2 > 0.95) {
        let pos2 = hash2v(cell2 + vec2(11.0, 7.0));
        let d2 = length(fract(suv2) - pos2);
        let bright2 = (1.0 - smoothstep(0.0, 0.06, d2)) * (0.4 + h2 * 0.4);
        // Color temperature variation: blue-white to warm yellow
        let temp2 = hash2(cell2 + vec2(99.0, 13.0));
        let col2 = mix(vec3<f32>(0.7, 0.8, 1.0), vec3<f32>(1.0, 0.92, 0.7), temp2);
        stars += col2 * bright2;
    }

    // Layer 3: Bright prominent stars (magnitude 0-2)
    let scale3 = 60.0;
    let suv3 = uv * scale3;
    let cell3 = floor(suv3);
    let h3 = hash2(cell3 + vec2(73.0, 51.0));
    if (h3 > 0.97) {
        let pos3 = hash2v(cell3 + vec2(19.0, 31.0));
        let d3 = length(fract(suv3) - pos3);
        // Bright core + soft glow halo
        let core = (1.0 - smoothstep(0.0, 0.04, d3)) * 1.2;
        let glow = (1.0 - smoothstep(0.0, 0.18, d3)) * 0.15;
        let bright3 = core + glow;
        let temp3 = hash2(cell3 + vec2(37.0, 89.0));
        let col3 = mix(vec3<f32>(0.6, 0.7, 1.0), vec3<f32>(1.0, 0.85, 0.6), temp3);
        stars += col3 * bright3;
    }

    return stars;
}

// Milky Way band — a diffuse glow along a great circle
fn milky_way(dir: vec3<f32>) -> vec3<f32> {
    // Milky Way runs roughly along a tilted band
    // Galactic coordinates: tilt ~63° from celestial equator
    let tilt = 63.0 * 3.14159 / 180.0;
    let ct = cos(tilt);
    let st = sin(tilt);
    // Rotate direction into galactic frame
    let gal_y = dir.y * ct - dir.z * st;
    let gal_z = dir.y * st + dir.z * ct;
    let gal_dir = vec3<f32>(dir.x, gal_y, gal_z);

    // Distance from galactic plane
    let plane_dist = abs(gal_dir.y);
    // Band intensity: narrow gaussian-ish band
    let band = exp(-plane_dist * plane_dist * 25.0);

    // Longitudinal variation using noise
    let lon = atan2(gal_dir.z, gal_dir.x);
    let lat = asin(clamp(gal_dir.y, -1.0, 1.0));
    let ncoord = vec2<f32>(lon * 2.0, lat * 8.0);
    let cloud = fbm(ncoord * 3.0) * 0.7 + fbm(ncoord * 7.0 + vec2(5.0, 3.0)) * 0.3;

    // Dense core near galactic center
    let center_dist = length(vec2<f32>(gal_dir.x + 0.3, gal_dir.y));
    let core_boost = exp(-center_dist * center_dist * 8.0) * 0.5;

    let intensity = band * (cloud * 0.6 + 0.1) + core_boost * band;

    // Cool blue-white color with slight warmth in the core
    let mw_color = mix(
        vec3<f32>(0.12, 0.14, 0.22),
        vec3<f32>(0.18, 0.16, 0.14),
        core_boost * 2.0
    );

    return mw_color * intensity;
}

// Full procedural night sky
fn procedural_night(dir: vec3<f32>) -> vec3<f32> {
    let elev = dir.y;

    // Base sky: very dark blue-black, slightly lighter near horizon
    let zenith = vec3<f32>(0.005, 0.007, 0.02);
    let horizon_night = vec3<f32>(0.015, 0.018, 0.035);
    let base = mix(horizon_night, zenith, smoothstep(0.0, 0.5, elev));

    // Below horizon: very dark ground-glow
    if (elev < 0.0) {
        let ground_dark = vec3<f32>(0.004, 0.005, 0.008);
        let below_factor = smoothstep(0.0, -0.15, elev);
        return mix(base, ground_dark, below_factor);
    }

    var sky = base;

    // Atmospheric horizon glow (light pollution / airglow)
    let airglow = exp(-elev * 15.0) * 0.025;
    sky += vec3<f32>(0.02, 0.025, 0.04) * airglow;

    // Milky way
    sky += milky_way(dir);

    // Stars (only above horizon)
    sky += star_field(dir);

    // Moon — a bright disc at a fixed position
    let moon_dir = normalize(vec3<f32>(0.3, 0.55, -0.4));
    let moon_ang = acos(clamp(dot(dir, moon_dir), -1.0, 1.0));
    let moon_disc = 1.0 - smoothstep(0.008, 0.012, moon_ang);
    let moon_glow = exp(-moon_ang * moon_ang * 400.0) * 0.08;
    let moon_halo = exp(-moon_ang * 3.0) * 0.015;
    let moon_color = vec3<f32>(0.85, 0.88, 0.95);
    sky += moon_color * (moon_disc * 0.9 + moon_glow + moon_halo);

    return sky;
}

// ── Fragment shader ─────────────────────────────────────────────────

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let clip = vec4<f32>(in.clip_coord.x, in.clip_coord.y, 1.0, 1.0);
    let world = sky.inv_view_proj * clip;
    let direction = normalize(world.xyz / world.w);

    let uv = direction_to_uv(direction);

    // Day sky: HDR environment map
    let hdr = sample_bicubic(uv);
    let day_sky = hdr.rgb * sky.exposure;

    // Horizon dust for day
    let elev = direction.y;
    let dust_band = 1.0 - smoothstep(-0.02, 0.12, elev);
    let dust_blend = dust_band * sky.horizon_dust;
    let day_dust = vec3<f32>(0.72, 0.68, 0.58);
    let day_with_dust = mix(day_sky, day_dust * sky.exposure, dust_blend);

    // Night sky: fully procedural
    let night_sky = procedural_night(direction);

    // Blend day → night
    let blended = mix(day_with_dust, night_sky, sky.night_blend);

    // Tone mapping
    let tonemapped = blended / (blended + vec3<f32>(1.0));
    let result = pow(tonemapped, vec3<f32>(1.0 / 2.2));

    return vec4<f32>(result, 1.0);
}
