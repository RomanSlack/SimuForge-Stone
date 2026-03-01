//! Ping pong table geometry constants and mesh generation.

use simuforge_core::Vertex;
use simuforge_render::arm_visual::generate_box;

/// ITTF regulation table dimensions (meters).
pub const TABLE_LENGTH: f32 = 2.74;
pub const TABLE_WIDTH: f32 = 1.525;
pub const TABLE_THICKNESS: f32 = 0.03;
pub const TABLE_HEIGHT: f32 = 0.76;

/// Net dimensions.
pub const NET_HEIGHT: f32 = 0.1525; // 15.25 cm
pub const NET_LENGTH: f32 = TABLE_WIDTH + 0.15; // extends 15cm beyond each side
pub const NET_THICKNESS: f32 = 0.002;
/// Net post dimensions.
pub const NET_POST_RADIUS: f32 = 0.01;
pub const NET_POST_HEIGHT: f32 = NET_HEIGHT;

/// Table leg dimensions.
pub const LEG_WIDTH: f32 = 0.05;
pub const LEG_DEPTH: f32 = 0.05;
pub const LEG_HEIGHT: f32 = TABLE_HEIGHT - TABLE_THICKNESS;

/// Floor size.
pub const FLOOR_SIZE: f32 = 8.0;
pub const FLOOR_THICKNESS: f32 = 0.01;

/// Table surface color (dark green, matte).
pub const TABLE_COLOR: [f32; 4] = [0.05, 0.25, 0.12, 1.0];
/// Table side color (slightly lighter).
pub const TABLE_SIDE_COLOR: [f32; 4] = [0.08, 0.30, 0.15, 1.0];
/// Net color (white mesh).
pub const NET_COLOR: [f32; 4] = [0.90, 0.90, 0.90, 1.0];
/// Net post color (dark metal).
pub const NET_POST_COLOR: [f32; 4] = [0.25, 0.25, 0.28, 1.0];
/// Leg color (dark steel).
pub const LEG_COLOR: [f32; 4] = [0.20, 0.20, 0.22, 1.0];
/// Floor color.
pub const FLOOR_COLOR: [f32; 4] = [0.30, 0.28, 0.25, 1.0];

/// Generate the table top mesh (centered at origin, y = table height).
/// Returns (vertices, indices).
pub fn generate_table_top() -> (Vec<Vertex>, Vec<u32>) {
    generate_box(TABLE_LENGTH / 2.0, TABLE_THICKNESS / 2.0, TABLE_WIDTH / 2.0)
}

/// Generate the net mesh (thin box at center of table).
pub fn generate_net() -> (Vec<Vertex>, Vec<u32>) {
    generate_box(NET_THICKNESS / 2.0, NET_HEIGHT / 2.0, NET_LENGTH / 2.0)
}

/// Generate a net post (cylinder approximated as thin box).
pub fn generate_net_post() -> (Vec<Vertex>, Vec<u32>) {
    generate_box(NET_POST_RADIUS, NET_POST_HEIGHT / 2.0, NET_POST_RADIUS)
}

/// Generate a table leg.
pub fn generate_leg() -> (Vec<Vertex>, Vec<u32>) {
    generate_box(LEG_WIDTH / 2.0, LEG_HEIGHT / 2.0, LEG_DEPTH / 2.0)
}

/// Generate the floor.
pub fn generate_floor() -> (Vec<Vertex>, Vec<u32>) {
    generate_box(FLOOR_SIZE / 2.0, FLOOR_THICKNESS / 2.0, FLOOR_SIZE / 2.0)
}

/// Leg positions relative to table center (x, z offsets).
/// Table legs are inset slightly from corners.
pub fn leg_positions() -> [(f32, f32); 4] {
    let lx = TABLE_LENGTH / 2.0 - 0.15; // 15cm inset from ends
    let lz = TABLE_WIDTH / 2.0 - 0.10;  // 10cm inset from sides
    [
        (-lx, -lz),
        (-lx, lz),
        (lx, -lz),
        (lx, lz),
    ]
}

/// Net post positions (at each side of the table).
pub fn net_post_positions() -> [(f32, f32); 2] {
    let z = TABLE_WIDTH / 2.0 + 0.075; // extends slightly beyond table
    [
        (0.0, -z),
        (0.0, z),
    ]
}

/// Generate table marking line segments in DH space (Z-up).
/// Returns pairs of (start, end) points in DH coordinates for the white lines.
/// Markings: perimeter border + center lengthwise line (doubles).
pub fn table_marking_lines() -> Vec<([f32; 3], [f32; 3])> {
    let hx = TABLE_LENGTH / 2.0;
    let hy = TABLE_WIDTH / 2.0;
    let z = TABLE_HEIGHT + TABLE_THICKNESS / 2.0 + 0.001; // just above surface

    let mut lines = Vec::new();

    // Perimeter (4 edges)
    lines.push(([-hx, -hy, z], [hx, -hy, z]));  // front edge
    lines.push(([hx, -hy, z], [hx, hy, z]));     // right edge
    lines.push(([hx, hy, z], [-hx, hy, z]));      // back edge
    lines.push(([-hx, hy, z], [-hx, -hy, z]));    // left edge

    // Center line (lengthwise, for doubles serve)
    lines.push(([-hx, 0.0, z], [hx, 0.0, z]));

    // End lines (cross-wise at each end)
    // These are the "serving lines" — already part of perimeter, but let's add
    // half-table lines at each end for visibility
    lines.push(([0.0, -hy, z], [0.0, hy, z])); // net line on table surface

    lines
}
