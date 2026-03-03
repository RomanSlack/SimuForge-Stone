//! Drone state and procedural mesh generation.
//!
//! Generates a delta-wing loitering munition mesh (closely specced to Shahed-136)
//! using raw vertex/index data. Model space: nose = +X, wingspan = ±Y, up = +Z.

use simuforge_core::Vertex;

/// Olive drab color for the drone body.
pub const DRONE_COLOR: [f32; 4] = [0.33, 0.37, 0.31, 1.0];
/// Dark prop color.
pub const PROP_COLOR: [f32; 4] = [0.15, 0.12, 0.10, 1.0];

/// Generate the complete drone fuselage + wing + V-tail mesh.
/// Returns (vertices, indices).
pub fn generate_drone_mesh() -> (Vec<Vertex>, Vec<u32>) {
    let mut verts = Vec::new();
    let mut idxs = Vec::new();

    // ── Fuselage: tapered cylinder along X axis ──────────────────────────
    // Nose at x=1.75, tail at x=-1.75, max radius 0.16 at x=0
    let fuse_segments = 16_u32;
    let fuse_rings = 12_u32;
    let fuse_length = 3.5_f32;
    let fuse_half = fuse_length / 2.0;
    let max_radius = 0.16_f32;

    for ring in 0..=fuse_rings {
        let t = ring as f32 / fuse_rings as f32;
        let x = fuse_half - t * fuse_length; // from nose (+1.75) to tail (-1.75)

        // Radius profile: elliptical taper — wider at center, pointed at nose/tail
        let normalized = (t - 0.4).abs() / 0.6; // peak at 40% from nose
        let r = max_radius * (1.0 - normalized * normalized).max(0.02);

        for seg in 0..=fuse_segments {
            let theta = seg as f32 / fuse_segments as f32 * std::f32::consts::TAU;
            let (sin_t, cos_t) = theta.sin_cos();
            let y = r * cos_t;
            let z = r * sin_t;
            let nx = if ring == 0 || ring == fuse_rings {
                if ring == 0 { 1.0 } else { -1.0 }
            } else {
                0.0
            };
            let ny = cos_t;
            let nz = sin_t;
            let n_len = (nx * nx + ny * ny + nz * nz).sqrt();
            verts.push(Vertex {
                position: [x, y, z],
                normal: [nx / n_len, ny / n_len, nz / n_len],
            });
        }
    }

    // Fuselage indices
    let ring_size = fuse_segments + 1;
    for ring in 0..fuse_rings {
        for seg in 0..fuse_segments {
            let i0 = ring * ring_size + seg;
            let i1 = i0 + 1;
            let i2 = i0 + ring_size;
            let i3 = i2 + 1;
            idxs.extend_from_slice(&[i0, i2, i1, i1, i2, i3]);
        }
    }

    // ── Delta Wing ───────────────────────────────────────────────────────
    // Root chord: ~2.1m, tip chord: ~0.35m, half-span: 1.25m
    // Wing sits roughly from x = +0.9 (leading edge root) to x = -1.2 (trailing edge root)
    // Top surface + bottom surface
    let wing_thickness = 0.025_f32; // half thickness

    // Wing planform vertices (top view, right wing):
    // LE root: (0.9, 0.05)  LE tip: (-0.2, 1.25)
    // TE root: (-1.2, 0.05) TE tip: (-0.55, 1.25)
    let wing_pts = [
        // Right wing - top
        ([0.9_f32, 0.05, wing_thickness], [0.0, 0.0, 1.0]),   // LE root
        ([-0.2, 1.25, wing_thickness], [0.0, 0.0, 1.0]),       // LE tip
        ([-1.2, 0.05, wing_thickness], [0.0, 0.0, 1.0]),       // TE root
        ([-0.55, 1.25, wing_thickness], [0.0, 0.0, 1.0]),      // TE tip
        // Right wing - bottom
        ([0.9, 0.05, -wing_thickness], [0.0, 0.0, -1.0]),
        ([-0.2, 1.25, -wing_thickness], [0.0, 0.0, -1.0]),
        ([-1.2, 0.05, -wing_thickness], [0.0, 0.0, -1.0]),
        ([-0.55, 1.25, -wing_thickness], [0.0, 0.0, -1.0]),
    ];

    let base_idx = verts.len() as u32;
    for (pos, nor) in &wing_pts {
        verts.push(Vertex { position: *pos, normal: *nor });
    }

    // Right wing top: two triangles (LE_root, LE_tip, TE_root) and (LE_tip, TE_tip, TE_root)
    let b = base_idx;
    idxs.extend_from_slice(&[b, b + 1, b + 2, b + 1, b + 3, b + 2]); // top
    idxs.extend_from_slice(&[b + 4, b + 6, b + 5, b + 5, b + 6, b + 7]); // bottom (reversed winding)

    // Leading edge strip (connects top LE to bottom LE)
    let le_base = verts.len() as u32;
    verts.push(Vertex { position: [0.9, 0.05, wing_thickness], normal: [0.5, 0.0, 0.5] });
    verts.push(Vertex { position: [-0.2, 1.25, wing_thickness], normal: [0.5, 0.0, 0.5] });
    verts.push(Vertex { position: [0.9, 0.05, -wing_thickness], normal: [0.5, 0.0, -0.5] });
    verts.push(Vertex { position: [-0.2, 1.25, -wing_thickness], normal: [0.5, 0.0, -0.5] });
    idxs.extend_from_slice(&[le_base, le_base + 1, le_base + 2, le_base + 1, le_base + 3, le_base + 2]);

    // Left wing (mirror Y)
    let mirror_base = verts.len() as u32;
    for (pos, nor) in &wing_pts {
        verts.push(Vertex {
            position: [pos[0], -pos[1], pos[2]],
            normal: [nor[0], -nor[1], nor[2]],
        });
    }
    let b = mirror_base;
    idxs.extend_from_slice(&[b, b + 2, b + 1, b + 1, b + 2, b + 3]); // top (reversed for mirror)
    idxs.extend_from_slice(&[b + 4, b + 5, b + 6, b + 5, b + 7, b + 6]); // bottom

    // Left LE strip
    let le_base = verts.len() as u32;
    verts.push(Vertex { position: [0.9, -0.05, wing_thickness], normal: [0.5, 0.0, 0.5] });
    verts.push(Vertex { position: [-0.2, -1.25, wing_thickness], normal: [0.5, 0.0, 0.5] });
    verts.push(Vertex { position: [0.9, -0.05, -wing_thickness], normal: [0.5, 0.0, -0.5] });
    verts.push(Vertex { position: [-0.2, -1.25, -wing_thickness], normal: [0.5, 0.0, -0.5] });
    idxs.extend_from_slice(&[le_base, le_base + 2, le_base + 1, le_base + 1, le_base + 2, le_base + 3]);

    // ── V-Tail ───────────────────────────────────────────────────────────
    // Two small surfaces at the tail with 20° dihedral
    let tail_dihedral = 20.0_f32.to_radians();
    let (sd, cd) = tail_dihedral.sin_cos();

    for sign in &[1.0_f32, -1.0] {
        let base = verts.len() as u32;
        let y_off = sign * 0.05;
        let span = 0.4_f32;
        let chord_root = 0.3_f32;
        let chord_tip = 0.15_f32;

        // Root LE, Root TE, Tip LE, Tip TE
        let tip_y = y_off + sign * span * cd;
        let tip_z = span * sd;

        let nor = [0.0, -sign * sd, cd]; // surface normal

        verts.push(Vertex { position: [-1.2, y_off, 0.0], normal: nor });        // root LE
        verts.push(Vertex { position: [-1.2 - chord_root, y_off, 0.0], normal: nor }); // root TE
        verts.push(Vertex { position: [-1.2, tip_y, tip_z], normal: nor });       // tip LE
        verts.push(Vertex { position: [-1.2 - chord_tip, tip_y, tip_z], normal: nor }); // tip TE

        if *sign > 0.0 {
            idxs.extend_from_slice(&[base, base + 2, base + 1, base + 1, base + 2, base + 3]);
        } else {
            idxs.extend_from_slice(&[base, base + 1, base + 2, base + 1, base + 3, base + 2]);
        }
    }

    (verts, idxs)
}

/// Generate the propeller disc mesh (thin cylinder at rear of fuselage).
/// Returns (vertices, indices).
pub fn generate_prop_disc() -> (Vec<Vertex>, Vec<u32>) {
    // Two-blade prop: simplified as thin rectangles
    let mut verts = Vec::new();
    let mut idxs = Vec::new();

    let blade_length = 0.75_f32; // half-span of prop
    let blade_width = 0.06_f32;
    let blade_thick = 0.01_f32;

    // Blade 1 (along Y)
    let b = verts.len() as u32;
    for &z_sign in &[1.0_f32, -1.0] {
        for &y in &[-blade_length, blade_length] {
            verts.push(Vertex {
                position: [0.0, y, z_sign * blade_thick],
                normal: [0.0, 0.0, z_sign],
            });
        }
        // Width faces
        verts.push(Vertex {
            position: [blade_width, 0.0, z_sign * blade_thick],
            normal: [1.0, 0.0, 0.0],
        });
        verts.push(Vertex {
            position: [-blade_width, 0.0, z_sign * blade_thick],
            normal: [-1.0, 0.0, 0.0],
        });
    }
    // Simple quad for top face
    idxs.extend_from_slice(&[b, b + 1, b + 2, b, b + 2, b + 3]);
    // Bottom face
    idxs.extend_from_slice(&[b + 4, b + 6, b + 5, b + 4, b + 7, b + 6]);

    // Blade 2 (along Z) — perpendicular to blade 1
    let b = verts.len() as u32;
    for &y_sign in &[1.0_f32, -1.0] {
        for &z in &[-blade_length, blade_length] {
            verts.push(Vertex {
                position: [0.0, y_sign * blade_thick, z],
                normal: [0.0, y_sign, 0.0],
            });
        }
    }
    idxs.extend_from_slice(&[b, b + 1, b + 2, b, b + 2, b + 3]);

    (verts, idxs)
}
