//! Drone state and procedural mesh generation.
//!
//! Generates a delta-wing loitering munition mesh (closely specced to Shahed-136)
//! using raw vertex/index data. Model space: nose = +X, wingspan = ±Y, up = +Z.

use simuforge_core::Vertex;

/// Deep red color for the drone body.
pub const DRONE_COLOR: [f32; 4] = [0.55, 0.05, 0.05, 1.0];
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

    // Nose apex: single vertex at tip (index 0)
    verts.push(Vertex {
        position: [fuse_half, 0.0, 0.0],
        normal: [1.0, 0.0, 0.0],
    });

    // Generate rings 1..=fuse_rings (skip ring 0 — replaced by apex)
    for ring in 1..=fuse_rings {
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
            let (nx, ny, nz) = if ring <= 2 {
                // Near-nose rings: blend from axial to radial
                let blend = ring as f32 / 3.0;
                (1.0 - blend, cos_t * blend, sin_t * blend)
            } else if ring == fuse_rings {
                // Tail ring: blend toward tail axial
                let blend = 0.33;
                (-(1.0 - blend), cos_t * blend, sin_t * blend)
            } else if ring >= fuse_rings - 2 {
                // Near-tail rings: blend toward tail axial
                let blend = (fuse_rings - ring) as f32 / 3.0;
                (-(1.0 - blend), cos_t * blend, sin_t * blend)
            } else {
                // Mid-body: pure radial
                (0.0, cos_t, sin_t)
            };
            let n_len = (nx * nx + ny * ny + nz * nz).sqrt().max(0.001);
            verts.push(Vertex {
                position: [x, y, z],
                normal: [nx / n_len, ny / n_len, nz / n_len],
            });
        }
    }

    // Nose triangle fan: apex (0) → ring 1 vertices (indices 1..=fuse_segments+1)
    let ring1_base = 1_u32; // first vertex of ring 1
    for seg in 0..fuse_segments {
        idxs.extend_from_slice(&[0, ring1_base + seg, ring1_base + seg + 1]);
    }

    // Fuselage quad strips between rings 1..fuse_rings
    let ring_size = fuse_segments + 1;
    for ring in 0..(fuse_rings - 1) {
        // ring index in our vertex array: ring 1 starts at offset 1
        let r0_base = 1 + ring * ring_size;
        let r1_base = r0_base + ring_size;
        for seg in 0..fuse_segments {
            let i0 = r0_base + seg;
            let i1 = i0 + 1;
            let i2 = r1_base + seg;
            let i3 = i2 + 1;
            idxs.extend_from_slice(&[i0, i2, i1, i1, i2, i3]);
        }
    }

    // Tail apex: single vertex at tail tip
    let tail_apex = verts.len() as u32;
    verts.push(Vertex {
        position: [-fuse_half, 0.0, 0.0],
        normal: [-1.0, 0.0, 0.0],
    });

    // Tail triangle fan: last ring → tail apex (reversed winding)
    let last_ring_base = 1 + (fuse_rings - 1) * ring_size;
    for seg in 0..fuse_segments {
        idxs.extend_from_slice(&[last_ring_base + seg + 1, last_ring_base + seg, tail_apex]);
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

/// Generate a tapered 2-blade pusher propeller with central hub.
/// Returns (vertices, indices).
pub fn generate_prop_disc() -> (Vec<Vertex>, Vec<u32>) {
    let mut verts = Vec::new();
    let mut idxs = Vec::new();

    let blade_half_span = 0.30_f32; // 0.6m total diameter
    let chord_root = 0.08_f32;
    let chord_tip = 0.03_f32;
    let thick_root = 0.012_f32;
    let thick_tip = 0.004_f32;
    let span_segs = 6_u32;

    // Generate one blade along +Y, then mirror for -Y blade
    for blade_sign in &[1.0_f32, -1.0] {
        for i in 0..span_segs {
            let t0 = i as f32 / span_segs as f32;
            let t1 = (i + 1) as f32 / span_segs as f32;

            let y0 = blade_sign * t0 * blade_half_span;
            let y1 = blade_sign * t1 * blade_half_span;

            let c0 = chord_root + (chord_tip - chord_root) * t0;
            let c1 = chord_root + (chord_tip - chord_root) * t1;
            let h0 = thick_root + (thick_tip - thick_root) * t0;
            let h1 = thick_root + (thick_tip - thick_root) * t1;

            let b = verts.len() as u32;

            // Top face (4 verts: root-LE, root-TE, tip-LE, tip-TE)
            let nz_top = 1.0_f32;
            verts.push(Vertex { position: [c0 * 0.5, y0, h0], normal: [0.0, 0.0, nz_top] });
            verts.push(Vertex { position: [-c0 * 0.5, y0, h0], normal: [0.0, 0.0, nz_top] });
            verts.push(Vertex { position: [c1 * 0.5, y1, h1], normal: [0.0, 0.0, nz_top] });
            verts.push(Vertex { position: [-c1 * 0.5, y1, h1], normal: [0.0, 0.0, nz_top] });

            // Bottom face
            let nz_bot = -1.0_f32;
            verts.push(Vertex { position: [c0 * 0.5, y0, -h0], normal: [0.0, 0.0, nz_bot] });
            verts.push(Vertex { position: [-c0 * 0.5, y0, -h0], normal: [0.0, 0.0, nz_bot] });
            verts.push(Vertex { position: [c1 * 0.5, y1, -h1], normal: [0.0, 0.0, nz_bot] });
            verts.push(Vertex { position: [-c1 * 0.5, y1, -h1], normal: [0.0, 0.0, nz_bot] });

            // Top face triangles
            idxs.extend_from_slice(&[b, b + 2, b + 1, b + 1, b + 2, b + 3]);
            // Bottom face triangles (reversed winding)
            idxs.extend_from_slice(&[b + 4, b + 5, b + 6, b + 5, b + 7, b + 6]);
        }
    }

    // ── Hub: cylinder along X axis ──
    let hub_r = 0.035_f32;
    let hub_len = 0.04_f32;
    let hub_segs = 12_u32;

    // Front cap
    let cap_f_center = verts.len() as u32;
    verts.push(Vertex { position: [hub_len, 0.0, 0.0], normal: [1.0, 0.0, 0.0] });
    for seg in 0..=hub_segs {
        let theta = seg as f32 / hub_segs as f32 * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        verts.push(Vertex {
            position: [hub_len, hub_r * c, hub_r * s],
            normal: [1.0, 0.0, 0.0],
        });
    }
    for seg in 0..hub_segs {
        idxs.extend_from_slice(&[cap_f_center, cap_f_center + 1 + seg, cap_f_center + 2 + seg]);
    }

    // Back cap
    let cap_b_center = verts.len() as u32;
    verts.push(Vertex { position: [-hub_len, 0.0, 0.0], normal: [-1.0, 0.0, 0.0] });
    for seg in 0..=hub_segs {
        let theta = seg as f32 / hub_segs as f32 * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        verts.push(Vertex {
            position: [-hub_len, hub_r * c, hub_r * s],
            normal: [-1.0, 0.0, 0.0],
        });
    }
    for seg in 0..hub_segs {
        idxs.extend_from_slice(&[cap_b_center, cap_b_center + 2 + seg, cap_b_center + 1 + seg]);
    }

    // Cylinder wall
    let wall_base = verts.len() as u32;
    for seg in 0..=hub_segs {
        let theta = seg as f32 / hub_segs as f32 * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        // Front ring
        verts.push(Vertex {
            position: [hub_len, hub_r * c, hub_r * s],
            normal: [0.0, c, s],
        });
        // Back ring
        verts.push(Vertex {
            position: [-hub_len, hub_r * c, hub_r * s],
            normal: [0.0, c, s],
        });
    }
    for seg in 0..hub_segs {
        let i0 = wall_base + seg * 2;
        let i1 = i0 + 1;
        let i2 = i0 + 2;
        let i3 = i0 + 3;
        idxs.extend_from_slice(&[i0, i1, i2, i2, i1, i3]);
    }

    (verts, idxs)
}

/// Generate a random low-poly debris chunk (jagged shard).
/// `seed` produces a unique shape per chunk.
/// Returns geometry roughly 0.5-1m across, centered at origin.
pub fn generate_debris_chunk(seed: u32) -> (Vec<Vertex>, Vec<u32>) {
    let mut verts = Vec::new();
    let mut idxs = Vec::new();

    let mut h = seed;
    let mut rnd = || -> f32 {
        h = h.wrapping_mul(1103515245).wrapping_add(12345);
        ((h >> 16) as f32 / 32768.0) - 1.0
    };
    // 5-7 random vertices around origin
    let n_pts = 5 + ((rnd() + 1.0) * 0.5 * 3.0) as usize;
    let mut pts = Vec::with_capacity(n_pts);
    for _ in 0..n_pts {
        pts.push([rnd() * 0.5, rnd() * 0.3, rnd() * 0.4]);
    }

    // Centroid
    let mut cx = 0.0_f32; let mut cy = 0.0_f32; let mut cz = 0.0_f32;
    for p in &pts { cx += p[0]; cy += p[1]; cz += p[2]; }
    let nf = pts.len() as f32;
    cx /= nf; cy /= nf; cz /= nf;

    // Top layer: centroid + outer points, fan triangles
    verts.push(Vertex { position: [cx, cy, cz], normal: [0.0, 1.0, 0.0] });
    for p in &pts {
        let dx = p[0] - cx; let dy = p[1] - cy; let dz = p[2] - cz;
        let len = (dx * dx + dy * dy + dz * dz).sqrt().max(0.001);
        verts.push(Vertex { position: *p, normal: [dx / len, dy / len, dz / len] });
    }
    let np = pts.len() as u32;
    for i in 0..np {
        idxs.extend_from_slice(&[0, 1 + i, 1 + (i + 1) % np]);
    }

    // Bottom layer for thickness
    let base2 = verts.len() as u32;
    let thickness = 0.08 + (rnd() + 1.0) * 0.5 * 0.12;
    verts.push(Vertex { position: [cx, cy - thickness, cz], normal: [0.0, -1.0, 0.0] });
    for p in &pts {
        let dx = p[0] - cx; let dz = p[2] - cz;
        let len = (dx * dx + dz * dz).sqrt().max(0.001);
        verts.push(Vertex { position: [p[0], p[1] - thickness, p[2]], normal: [dx / len, -0.3, dz / len] });
    }
    for i in 0..np {
        idxs.extend_from_slice(&[base2, base2 + 1 + (i + 1) % np, base2 + 1 + i]);
    }

    // Side walls
    for i in 0..np {
        let t0 = 1 + i; let t1 = 1 + (i + 1) % np;
        let b0 = base2 + 1 + i; let b1 = base2 + 1 + (i + 1) % np;
        idxs.extend_from_slice(&[t0, b0, t1, t1, b0, b1]);
    }

    (verts, idxs)
}
