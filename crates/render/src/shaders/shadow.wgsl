// Shadow map depth-only pass

struct LightMatrix {
    light_vp: mat4x4<f32>,
};

@group(0) @binding(0) var<uniform> light: LightMatrix;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> @builtin(position) vec4<f32> {
    return light.light_vp * vec4<f32>(in.position, 1.0);
}
