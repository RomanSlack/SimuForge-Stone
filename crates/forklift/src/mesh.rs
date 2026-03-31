use simuforge_core::Vertex;
use simuforge_render::arm_visual::{generate_box, generate_cylinder};

/// Merge multiple (verts, indices) meshes into one.
fn merge_meshes(meshes: Vec<(Vec<Vertex>, Vec<u32>)>) -> (Vec<Vertex>, Vec<u32>) {
    let mut all_verts = Vec::new();
    let mut all_idxs = Vec::new();
    for (verts, idxs) in meshes {
        let base = all_verts.len() as u32;
        all_verts.extend_from_slice(&verts);
        all_idxs.extend(idxs.into_iter().map(|i| i + base));
    }
    (all_verts, all_idxs)
}

/// Translate vertices by an offset.
fn translate(verts: &mut [Vertex], dx: f32, dy: f32, dz: f32) {
    for v in verts.iter_mut() {
        v.position[0] += dx;
        v.position[1] += dy;
        v.position[2] += dz;
    }
}

/// Scale vertices.
fn scale(verts: &mut [Vertex], sx: f32, sy: f32, sz: f32) {
    for v in verts.iter_mut() {
        v.position[0] *= sx;
        v.position[1] *= sy;
        v.position[2] *= sz;
        // Adjust normals for non-uniform scale
        v.normal[0] /= sx;
        v.normal[1] /= sy;
        v.normal[2] /= sz;
        let len = (v.normal[0] * v.normal[0]
            + v.normal[1] * v.normal[1]
            + v.normal[2] * v.normal[2])
            .sqrt();
        if len > 0.0 {
            v.normal[0] /= len;
            v.normal[1] /= len;
            v.normal[2] /= len;
        }
    }
}

/// Chassis body (render Y-up: X=right, Y=up, Z=forward).
/// Origin at rear axle center, ground level (Y=0).
/// Forklift faces -Z (forks at front = -Z direction in render space).
pub fn generate_chassis() -> (Vec<Vertex>, Vec<u32>) {
    // Main body: 2.0m long, 1.1m wide, 0.5m tall
    // Centered between axles, raised off ground
    let (mut body_v, body_i) = generate_box(0.55, 0.25, 1.0);
    // Center at rear axle: shift forward by half wheelbase, up by wheel center + half height
    translate(&mut body_v, 0.0, 0.55, -0.75);

    // Counterweight bulge at rear
    let (mut cw_v, cw_i) = generate_box(0.50, 0.30, 0.25);
    translate(&mut cw_v, 0.0, 0.50, 0.10);

    // Engine hood / battery cover (top surface above battery)
    let (mut hood_v, hood_i) = generate_box(0.45, 0.03, 0.40);
    translate(&mut hood_v, 0.0, 0.82, -0.10);

    merge_meshes(vec![
        (body_v, body_i),
        (cw_v, cw_i),
        (hood_v, hood_i),
    ])
}

/// Overhead guard (ROPS cage) - 4 posts + roof frame.
pub fn generate_overhead_guard() -> (Vec<Vertex>, Vec<u32>) {
    let post_r = 0.025;
    let post_h = 1.3;

    let positions = [
        (-0.45, -0.85),
        (0.45, -0.85),
        (-0.45, 0.05),
        (0.45, 0.05),
    ];

    let mut meshes = Vec::new();
    for &(x, z) in &positions {
        let (mut v, i) = generate_cylinder(post_r, post_h, 8);
        translate(&mut v, x, 0.80 + post_h / 2.0, z);
        meshes.push((v, i));
    }

    // Roof cross beams
    let (mut roof1_v, roof1_i) = generate_box(0.45, 0.02, 0.02);
    translate(&mut roof1_v, 0.0, 0.80 + post_h, -0.85);
    meshes.push((roof1_v, roof1_i));

    let (mut roof2_v, roof2_i) = generate_box(0.45, 0.02, 0.02);
    translate(&mut roof2_v, 0.0, 0.80 + post_h, 0.05);
    meshes.push((roof2_v, roof2_i));

    // Side beams
    let (mut side1_v, side1_i) = generate_box(0.02, 0.02, 0.45);
    translate(&mut side1_v, -0.45, 0.80 + post_h, -0.40);
    meshes.push((side1_v, side1_i));

    let (mut side2_v, side2_i) = generate_box(0.02, 0.02, 0.45);
    translate(&mut side2_v, 0.45, 0.80 + post_h, -0.40);
    meshes.push((side2_v, side2_i));

    merge_meshes(meshes)
}

/// Outer mast — two vertical C-channel rails + cross members.
/// Origin at mast pivot point.
pub fn generate_outer_mast() -> (Vec<Vertex>, Vec<u32>) {
    let rail_w = 0.04;
    let rail_d = 0.06;
    let rail_h = 1.8;

    // Left rail
    let (mut lv, li) = generate_box(rail_w / 2.0, rail_h / 2.0, rail_d / 2.0);
    translate(&mut lv, -0.25, rail_h / 2.0, 0.0);

    // Right rail
    let (mut rv, ri) = generate_box(rail_w / 2.0, rail_h / 2.0, rail_d / 2.0);
    translate(&mut rv, 0.25, rail_h / 2.0, 0.0);

    // Bottom cross member
    let (mut bv, bi) = generate_box(0.25, 0.02, rail_d / 2.0);
    translate(&mut bv, 0.0, 0.03, 0.0);

    // Top cross member
    let (mut tv, ti) = generate_box(0.25, 0.02, rail_d / 2.0);
    translate(&mut tv, 0.0, rail_h - 0.03, 0.0);

    merge_meshes(vec![(lv, li), (rv, ri), (bv, bi), (tv, ti)])
}

/// Inner mast — two thinner rails that slide inside outer mast.
/// Origin at bottom of inner mast (relative to outer mast base).
pub fn generate_inner_mast() -> (Vec<Vertex>, Vec<u32>) {
    let rail_w = 0.03;
    let rail_d = 0.04;
    let rail_h = 1.6;

    let (mut lv, li) = generate_box(rail_w / 2.0, rail_h / 2.0, rail_d / 2.0);
    translate(&mut lv, -0.20, rail_h / 2.0, 0.0);

    let (mut rv, ri) = generate_box(rail_w / 2.0, rail_h / 2.0, rail_d / 2.0);
    translate(&mut rv, 0.20, rail_h / 2.0, 0.0);

    // Cross member
    let (mut cv, ci) = generate_box(0.20, 0.015, rail_d / 2.0);
    translate(&mut cv, 0.0, rail_h - 0.02, 0.0);

    merge_meshes(vec![(lv, li), (rv, ri), (cv, ci)])
}

/// Fork carriage plate.
pub fn generate_carriage() -> (Vec<Vertex>, Vec<u32>) {
    let (v, i) = generate_box(0.25, 0.15, 0.03);
    (v, i)
}

/// Two fork tines (L-shaped).
/// Origin at carriage attachment point. Forks extend forward (-Z).
/// Fork tine bottoms sit at Y=0 (relative to carriage base).
pub fn generate_forks() -> (Vec<Vertex>, Vec<u32>) {
    let fork_len = 1.07;
    let fork_w = 0.10;
    let fork_thick = 0.035;
    let heel_h = 0.30; // taller heel to connect to carriage

    let mut meshes = Vec::new();
    let fork_spacing = 0.27; // distance from center to each fork (~0.54m apart, fits in EUR pallet)

    for side in [-1.0_f32, 1.0] {
        let fx = side * fork_spacing;

        // Horizontal blade: bottom at Y=0, extends forward (-Z)
        let (mut bv, bi) = generate_box(fork_w / 2.0, fork_thick / 2.0, fork_len / 2.0);
        translate(&mut bv, fx, fork_thick / 2.0, -fork_len / 2.0);
        meshes.push((bv, bi));

        // Vertical heel: from fork bottom up to carriage
        let (mut hv, hi) = generate_box(fork_w / 2.0, heel_h / 2.0, fork_thick / 2.0);
        translate(&mut hv, fx, heel_h / 2.0, fork_thick / 2.0);
        meshes.push((hv, hi));
    }

    merge_meshes(meshes)
}

/// Generate a cylinder along the X-axis (for wheels). Centered at origin.
/// Width = extent along X, radius in YZ plane.
fn generate_cylinder_x(radius: f32, width: f32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let half_w = width / 2.0;

    // Side vertices: ring around YZ, extruded along X
    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
        let (sin_a, cos_a) = angle.sin_cos();
        let ny = cos_a;
        let nz = sin_a;

        // Left face (-X)
        vertices.push(Vertex {
            position: [-half_w, radius * cos_a, radius * sin_a],
            normal: [0.0, ny, nz],
        });
        // Right face (+X)
        vertices.push(Vertex {
            position: [half_w, radius * cos_a, radius * sin_a],
            normal: [0.0, ny, nz],
        });
    }

    // Side indices (same pattern as Y-axis cylinder, just different axes)
    for i in 0..segments {
        let base = i * 2;
        // Triangle 1: left_i, right_i, left_{i+1}
        indices.push(base);
        indices.push(base + 1);
        indices.push(base + 2);
        // Triangle 2: right_i, right_{i+1}, left_{i+1}
        indices.push(base + 1);
        indices.push(base + 3);
        indices.push(base + 2);
    }

    // Right cap (+X) — center, then ring CCW from +X direction
    let rc = vertices.len() as u32;
    vertices.push(Vertex {
        position: [half_w, 0.0, 0.0],
        normal: [1.0, 0.0, 0.0],
    });
    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
        let (sin_a, cos_a) = angle.sin_cos();
        vertices.push(Vertex {
            position: [half_w, radius * cos_a, radius * sin_a],
            normal: [1.0, 0.0, 0.0],
        });
    }
    for i in 0..segments {
        indices.push(rc);
        indices.push(rc + 1 + i);
        indices.push(rc + 2 + i);
    }

    // Left cap (-X) — center, then ring CCW from -X direction (reversed winding)
    let lc = vertices.len() as u32;
    vertices.push(Vertex {
        position: [-half_w, 0.0, 0.0],
        normal: [-1.0, 0.0, 0.0],
    });
    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
        let (sin_a, cos_a) = angle.sin_cos();
        vertices.push(Vertex {
            position: [-half_w, radius * cos_a, radius * sin_a],
            normal: [-1.0, 0.0, 0.0],
        });
    }
    for i in 0..segments {
        indices.push(lc);
        indices.push(lc + 2 + i); // reversed for correct winding
        indices.push(lc + 1 + i);
    }

    (vertices, indices)
}

/// Front wheel — cylinder along X-axis (axle direction). Centered at origin.
pub fn generate_front_wheel() -> (Vec<Vertex>, Vec<u32>) {
    generate_cylinder_x(0.265, 0.20, 16)
}

/// Rear wheel — cylinder along X-axis. Centered at origin.
pub fn generate_rear_wheel() -> (Vec<Vertex>, Vec<u32>) {
    generate_cylinder_x(0.20, 0.15, 16)
}

/// Operator seat.
pub fn generate_seat() -> (Vec<Vertex>, Vec<u32>) {
    // Seat cushion
    let (mut cv, ci) = generate_box(0.22, 0.05, 0.22);
    translate(&mut cv, 0.0, 0.0, 0.0);

    // Backrest
    let (mut bv, bi) = generate_box(0.22, 0.20, 0.04);
    translate(&mut bv, 0.0, 0.20, 0.22);

    merge_meshes(vec![(cv, ci), (bv, bi)])
}

/// Ground floor quad (large flat surface).
pub fn generate_floor(half_size: f32) -> (Vec<Vertex>, Vec<u32>) {
    let (v, i) = generate_box(half_size, 0.01, half_size);
    (v, i)
}

/// A pallet (EUR standard: 1.2m x 0.8m x 0.144m).
/// X = 1.2m (wide side), Z = 0.8m (fork entry side).
/// Origin at ground center. Fork entry from ±Z sides.
/// Gap between ground and deck bottom = ~0.10m for fork insertion.
pub fn generate_pallet() -> (Vec<Vertex>, Vec<u32>) {
    let hw = 0.60_f32; // half-width (X)
    let hd = 0.40_f32; // half-depth (Z)
    let deck_thick = 0.022;
    let deck_top = 0.144_f32; // total pallet height
    let deck_bot = deck_top - deck_thick;
    let block_h = deck_bot - deck_thick; // block height fills gap between bottom boards and deck

    let mut meshes = Vec::new();

    // Top deck boards (5 boards across X)
    for i in 0..5 {
        let bw = if i == 0 || i == 4 { 0.10 } else { 0.12 };
        let bx = (i as f32 - 2.0) * 0.26;
        let (mut v, idx) = generate_box(bw / 2.0, deck_thick / 2.0, hd);
        translate(&mut v, bx, deck_top - deck_thick / 2.0, 0.0);
        meshes.push((v, idx));
    }

    // Bottom boards (3 runners along Z)
    for z in [-0.30_f32, 0.0, 0.30] {
        let (mut v, idx) = generate_box(hw, deck_thick / 2.0, 0.07);
        translate(&mut v, 0.0, deck_thick / 2.0, z);
        meshes.push((v, idx));
    }

    // Support blocks (3x3 grid between bottom boards and deck)
    for x in [-0.42_f32, 0.0, 0.42] {
        for z in [-0.30_f32, 0.0, 0.30] {
            let (mut v, idx) = generate_box(0.05, block_h / 2.0, 0.05);
            translate(&mut v, x, deck_thick + block_h / 2.0, z);
            meshes.push((v, idx));
        }
    }

    merge_meshes(meshes)
}

/// A simple cargo box to stack on pallets.
pub fn generate_cargo_box(hx: f32, hy: f32, hz: f32) -> (Vec<Vertex>, Vec<u32>) {
    generate_box(hx, hy, hz)
}
