//! Procedural desert terrain with rolling dunes/hills.
//!
//! Ground tiles follow the camera. Each tile's vertices are generated
//! with noise-based height displacement seeded by world position, then
//! uploaded to a single large GPU buffer each frame.

use simuforge_core::Vertex;
use simuforge_render::arm_visual::{generate_box, generate_cylinder};

/// Sand tan color for desert ground (darkened to avoid washout under bright light).
pub const SAND_COLOR: [f32; 4] = [0.62, 0.55, 0.38, 1.0];
/// Concrete grey for target building.
pub const BUILDING_COLOR: [f32; 4] = [0.6, 0.6, 0.6, 1.0];
/// Target bullseye outer ring.
pub const TARGET_RED: [f32; 4] = [0.8, 0.1, 0.1, 1.0];
/// Target bullseye inner disc.
pub const TARGET_INNER: [f32; 4] = [1.0, 0.15, 0.15, 1.0];
/// Dark grey for launch rail.
pub const RAIL_COLOR: [f32; 4] = [0.3, 0.3, 0.3, 1.0];
/// Reddish brown for reference buildings.
pub const REF_BUILDING_COLOR: [f32; 4] = [0.55, 0.40, 0.30, 1.0];

/// Tile size (meters).
pub const TILE_SIZE: f32 = 500.0;
/// Vertices per tile edge.
const TILE_RES: u32 = 16;
/// Half-extent of tile grid. 5 → 11x11 = 121 tiles (good balance).
pub const TILE_RADIUS: i32 = 5;
/// Total ground tiles.
pub const NUM_TILES: usize = ((TILE_RADIUS * 2 + 1) * (TILE_RADIUS * 2 + 1)) as usize;
/// Vertices per tile.
pub const VERTS_PER_TILE: usize = ((TILE_RES + 1) * (TILE_RES + 1)) as usize;
/// Indices per tile.
pub const IDXS_PER_TILE: usize = (TILE_RES * TILE_RES * 6) as usize;

// ── Noise ────────────────────────────────────────────────────────────────────

fn hash2(x: i32, y: i32) -> f32 {
    let mut h = x.wrapping_add(1234567).wrapping_mul(374761393)
        ^ y.wrapping_add(7654321).wrapping_mul(668265263);
    h = (h ^ (h >> 13)).wrapping_mul(1274126177);
    h = h ^ (h >> 16);
    (h as u32 as f32) / (u32::MAX as f32)
}

fn value_noise(x: f32, y: f32) -> f32 {
    let ix = x.floor() as i32;
    let iy = y.floor() as i32;
    let fx = x - x.floor();
    let fy = y - y.floor();
    let sx = fx * fx * (3.0 - 2.0 * fx);
    let sy = fy * fy * (3.0 - 2.0 * fy);
    let n00 = hash2(ix, iy);
    let n10 = hash2(ix + 1, iy);
    let n01 = hash2(ix, iy + 1);
    let n11 = hash2(ix + 1, iy + 1);
    let nx0 = n00 + (n10 - n00) * sx;
    let nx1 = n01 + (n11 - n01) * sx;
    nx0 + (nx1 - nx0) * sy
}

/// Terrain height at a world XZ position (render space).
pub fn terrain_height(wx: f32, wz: f32) -> f32 {
    let mut height = 0.0_f32;
    let mut amplitude = 1.0_f32;
    let mut frequency = 1.0_f32;
    let mut amp_sum = 0.0_f32;

    let base_scale = 0.0006; // ~1600m primary wavelength
    for _ in 0..4 {
        let nx = wx * base_scale * frequency;
        let nz = wz * base_scale * frequency;
        height += (value_noise(nx, nz) - 0.5) * 2.0 * amplitude;
        amp_sum += amplitude;
        amplitude *= 0.45;
        frequency *= 2.3;
    }

    let normalized = height / amp_sum;
    // Desert profile: mostly gentle rolling dunes
    let shaped = normalized.abs().powf(0.7) * normalized.signum();
    let raw = shaped * 20.0; // ±20m — visible rolling dunes from altitude

    // Flatten near launch site (within ~1km of origin) so the rail sits on flat ground
    let dist_from_origin = (wx * wx + wz * wz).sqrt();
    let flatten = (dist_from_origin / 1000.0).clamp(0.0, 1.0); // 0 at origin, 1 at 1km+
    raw * flatten
}

/// Generate all visible tile geometry for a single upload.
/// Returns (all_vertices, all_indices) for ALL tiles combined.
/// Each tile is a TILE_RES×TILE_RES grid displaced by terrain noise.
pub fn generate_all_tiles(cam_x: f32, cam_z: f32) -> (Vec<Vertex>, Vec<u32>) {
    let snap_x = (cam_x / TILE_SIZE).round() * TILE_SIZE;
    let snap_z = (cam_z / TILE_SIZE).round() * TILE_SIZE;

    let total_verts = NUM_TILES * VERTS_PER_TILE;
    let total_idxs = NUM_TILES * IDXS_PER_TILE;
    let mut verts = Vec::with_capacity(total_verts);
    let mut idxs = Vec::with_capacity(total_idxs);

    let half = TILE_SIZE / 2.0;
    let step = TILE_SIZE / TILE_RES as f32;
    let n = TILE_RES + 1;

    for tdx in -TILE_RADIUS..=TILE_RADIUS {
        for tdz in -TILE_RADIUS..=TILE_RADIUS {
            let tile_cx = snap_x + tdx as f32 * TILE_SIZE;
            let tile_cz = snap_z + tdz as f32 * TILE_SIZE;
            let base_vertex = verts.len() as u32;

            // Vertices
            for iz in 0..n {
                for ix in 0..n {
                    let local_x = -half + ix as f32 * step;
                    let local_z = -half + iz as f32 * step;
                    let world_x = tile_cx + local_x;
                    let world_z = tile_cz + local_z;
                    let y = terrain_height(world_x, world_z);

                    verts.push(Vertex {
                        position: [world_x, y, world_z],
                        normal: [0.0, 1.0, 0.0], // computed below
                    });
                }
            }

            // Normals (finite difference)
            let eps = step * 0.5;
            let tile_base = base_vertex as usize;
            for iz in 0..n {
                for ix in 0..n {
                    let idx = tile_base + (iz * n + ix) as usize;
                    let wx = verts[idx].position[0];
                    let wz = verts[idx].position[2];
                    let dx = terrain_height(wx + eps, wz) - terrain_height(wx - eps, wz);
                    let dz = terrain_height(wx, wz + eps) - terrain_height(wx, wz - eps);
                    let nx = -dx;
                    let ny = 2.0 * eps;
                    let nz = -dz;
                    let len = (nx * nx + ny * ny + nz * nz).sqrt();
                    verts[idx].normal = [nx / len, ny / len, nz / len];
                }
            }

            // Indices
            for iz in 0..TILE_RES {
                for ix in 0..TILE_RES {
                    let i00 = base_vertex + iz * n + ix;
                    let i10 = i00 + 1;
                    let i01 = i00 + n;
                    let i11 = i01 + 1;
                    idxs.extend_from_slice(&[i00, i01, i10, i10, i01, i11]);
                }
            }
        }
    }

    (verts, idxs)
}

/// Generate the target building (small shed beside the bullseye).
pub fn generate_target_building() -> (Vec<Vertex>, Vec<u32>) {
    generate_box(1.0, 1.0, 1.5)
}

/// Generate the outer ring of the target bullseye (20m–100m radius annulus at y=0.01).
pub fn generate_target_outer_ring() -> (Vec<Vertex>, Vec<u32>) {
    let segments = 64_u32;
    let inner_r = 20.0_f32;
    let outer_r = 100.0_f32;
    let y = 0.01_f32;
    let mut verts = Vec::with_capacity((segments as usize + 1) * 2);
    let mut idxs = Vec::new();

    for i in 0..=segments {
        let theta = i as f32 / segments as f32 * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        verts.push(Vertex {
            position: [c * inner_r, y, s * inner_r],
            normal: [0.0, 1.0, 0.0],
        });
        verts.push(Vertex {
            position: [c * outer_r, y, s * outer_r],
            normal: [0.0, 1.0, 0.0],
        });
    }

    for i in 0..segments {
        let i0 = i * 2;
        let i1 = i0 + 1;
        let i2 = i0 + 2;
        let i3 = i0 + 3;
        idxs.extend_from_slice(&[i0, i2, i1, i1, i2, i3]);
    }

    (verts, idxs)
}

/// Generate the inner disc of the target bullseye (0–20m radius fan at y=0.02).
pub fn generate_target_inner_disc() -> (Vec<Vertex>, Vec<u32>) {
    let segments = 64_u32;
    let radius = 20.0_f32;
    let y = 0.02_f32;
    let mut verts = Vec::with_capacity(segments as usize + 2);
    let mut idxs = Vec::new();

    // Center vertex
    verts.push(Vertex {
        position: [0.0, y, 0.0],
        normal: [0.0, 1.0, 0.0],
    });

    for i in 0..=segments {
        let theta = i as f32 / segments as f32 * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        verts.push(Vertex {
            position: [c * radius, y, s * radius],
            normal: [0.0, 1.0, 0.0],
        });
    }

    for i in 0..segments {
        idxs.extend_from_slice(&[0, i + 1, i + 2]);
    }

    (verts, idxs)
}

/// Generate the launch rail.
pub fn generate_launch_rail() -> (Vec<Vertex>, Vec<u32>) {
    generate_cylinder(0.05, 5.0, 8)
}

/// Reference buildings along the route.
pub fn reference_buildings() -> Vec<(f64, f32, f32, f32)> {
    vec![
        (10_000.0, 3.0, 2.0, 2.5),
        (25_000.0, 4.0, 3.0, 3.0),
        (40_000.0, 2.0, 1.5, 2.0),
    ]
}

/// Generate mesh for a reference building.
pub fn generate_ref_building(hx: f32, hy: f32, hz: f32) -> (Vec<Vertex>, Vec<u32>) {
    generate_box(hx, hy, hz)
}
