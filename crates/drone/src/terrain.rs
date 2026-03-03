//! Procedural desert terrain, target building, and launch rail generation.

use simuforge_core::Vertex;
use simuforge_render::arm_visual::{generate_box, generate_cylinder};

/// Sand tan color for desert ground.
pub const SAND_COLOR: [f32; 4] = [0.76, 0.70, 0.50, 1.0];
/// Concrete grey for target building.
pub const BUILDING_COLOR: [f32; 4] = [0.6, 0.6, 0.6, 1.0];
/// Dark grey for launch rail.
pub const RAIL_COLOR: [f32; 4] = [0.3, 0.3, 0.3, 1.0];
/// Reddish brown for reference buildings.
pub const REF_BUILDING_COLOR: [f32; 4] = [0.55, 0.40, 0.30, 1.0];

/// Generate the desert ground plane (50km × 50km, centered).
/// Returns (vertices, indices).
pub fn generate_ground() -> (Vec<Vertex>, Vec<u32>) {
    // 25km half-extent, thin slab
    generate_box(25_000.0, 0.05, 25_000.0)
}

/// Generate the target building at (50000, 0, 0) in DH space.
/// The building is 5m wide, 3m tall, 4m deep (half extents).
/// Returns (vertices, indices).
pub fn generate_target_building() -> (Vec<Vertex>, Vec<u32>) {
    generate_box(2.5, 1.5, 2.0)
}

/// Generate the launch rail (tilted cylinder).
/// Returns (vertices, indices).
pub fn generate_launch_rail() -> (Vec<Vertex>, Vec<u32>) {
    generate_cylinder(0.05, 5.0, 8)
}

/// Reference buildings along the route for scale.
/// Returns list of (x_position_dh, half_width, half_height, half_depth).
pub fn reference_buildings() -> Vec<(f64, f32, f32, f32)> {
    vec![
        (10_000.0, 3.0, 2.0, 2.5),   // small compound at 10km
        (25_000.0, 4.0, 3.0, 3.0),   // larger building at 25km
        (40_000.0, 2.0, 1.5, 2.0),   // small structure at 40km
    ]
}

/// Generate mesh for a reference building.
pub fn generate_ref_building(hx: f32, hy: f32, hz: f32) -> (Vec<Vertex>, Vec<u32>) {
    generate_box(hx, hy, hz)
}
