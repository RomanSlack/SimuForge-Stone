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
const TILE_RES: u32 = 24;
/// Half-extent of tile grid. 10 → 21x21 = 441 tiles (2x render distance).
pub const TILE_RADIUS: i32 = 10;
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

    let base_scale = 0.0012; // ~830m primary wavelength
    for _ in 0..5 {
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
    let raw = shaped * 80.0; // ±80m — dramatic dunes visible from cruise altitude

    // Flatten near launch site (within ~1km of origin) so the rail sits on flat ground
    let dist_from_origin = (wx * wx + wz * wz).sqrt();
    let flatten = (dist_from_origin / 200.0).clamp(0.0, 1.0); // 0 at origin, 1 at 200m+
    raw * flatten
}

/// Side length of the tile grid.
pub const TILE_SIDE: usize = (TILE_RADIUS * 2 + 1) as usize;

/// Generate a single tile's geometry into pre-allocated buffers.
/// `slot` is the linear tile index [0, NUM_TILES).
/// `tile_cx`, `tile_cz` are the tile's world-space center coords.
/// Writes vertices and indices at the correct offsets for that slot.
pub fn generate_single_tile(
    tile_cx: f32,
    tile_cz: f32,
    slot: usize,
    verts: &mut [Vertex],
    idxs: &mut [u32],
) {
    let half = TILE_SIZE / 2.0;
    let step = TILE_SIZE / TILE_RES as f32;
    let n = TILE_RES + 1;

    let v_off = slot * VERTS_PER_TILE;
    let i_off = slot * IDXS_PER_TILE;
    let base_vertex = v_off as u32;

    // Vertices
    for iz in 0..n {
        for ix in 0..n {
            let local_x = -half + ix as f32 * step;
            let local_z = -half + iz as f32 * step;
            let world_x = tile_cx + local_x;
            let world_z = tile_cz + local_z;
            let y = terrain_height(world_x, world_z);
            verts[v_off + (iz * n + ix) as usize] = Vertex {
                position: [world_x, y, world_z],
                normal: [0.0, 1.0, 0.0],
            };
        }
    }

    // Normals (finite difference)
    let eps = step * 0.5;
    for iz in 0..n {
        for ix in 0..n {
            let idx = v_off + (iz * n + ix) as usize;
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
    let mut ii = i_off;
    for iz in 0..TILE_RES {
        for ix in 0..TILE_RES {
            let i00 = base_vertex + iz * n + ix;
            let i01 = i00 + n;
            idxs[ii] = i00;
            idxs[ii + 1] = i01;
            idxs[ii + 2] = i00 + 1;
            idxs[ii + 3] = i00 + 1;
            idxs[ii + 4] = i01;
            idxs[ii + 5] = i01 + 1;
            ii += 6;
        }
    }
}

/// Compute the tile slot index from local grid offsets.
#[inline]
pub fn tile_slot(tdx: i32, tdz: i32) -> usize {
    ((tdx + TILE_RADIUS) as usize) * TILE_SIDE + (tdz + TILE_RADIUS) as usize
}

/// Compute the world-space tile center for a given snap position and local offset.
#[inline]
pub fn tile_world_center(snap_x: f32, snap_z: f32, tdx: i32, tdz: i32) -> (f32, f32) {
    (snap_x + tdx as f32 * TILE_SIZE, snap_z + tdz as f32 * TILE_SIZE)
}

/// Generate all visible tile geometry for initial upload.
/// Returns (all_vertices, all_indices) for ALL tiles combined.
pub fn generate_all_tiles(cam_x: f32, cam_z: f32) -> (Vec<Vertex>, Vec<u32>) {
    let snap_x = (cam_x / TILE_SIZE).round() * TILE_SIZE;
    let snap_z = (cam_z / TILE_SIZE).round() * TILE_SIZE;

    let total_verts = NUM_TILES * VERTS_PER_TILE;
    let total_idxs = NUM_TILES * IDXS_PER_TILE;
    let mut verts = vec![Vertex { position: [0.0; 3], normal: [0.0, 1.0, 0.0] }; total_verts];
    let mut idxs = vec![0u32; total_idxs];

    for tdx in -TILE_RADIUS..=TILE_RADIUS {
        for tdz in -TILE_RADIUS..=TILE_RADIUS {
            let slot = tile_slot(tdx, tdz);
            let (cx, cz) = tile_world_center(snap_x, snap_z, tdx, tdz);
            generate_single_tile(cx, cz, slot, &mut verts, &mut idxs);
        }
    }

    (verts, idxs)
}

/// Generate the target building (small shed beside the bullseye).
pub fn generate_target_building() -> (Vec<Vertex>, Vec<u32>) {
    generate_box(1.0, 1.0, 1.5)
}

/// Generate a terrain-conforming ring around the target center (world-space vertices).
/// `cx`, `cz` are the ring center in render world space (Y-up).
fn generate_target_ring(cx: f32, cz: f32, inner_r: f32, outer_r: f32) -> (Vec<Vertex>, Vec<u32>) {
    let segments = 96_u32;
    let lift = 0.05_f32; // slight lift above terrain to prevent z-fighting
    let mut verts = Vec::with_capacity((segments as usize + 1) * 2);
    let mut idxs = Vec::new();

    for i in 0..=segments {
        let theta = i as f32 / segments as f32 * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();

        let ix = cx + c * inner_r;
        let iz = cz + s * inner_r;
        let iy = terrain_height(ix, iz) + lift;
        verts.push(Vertex {
            position: [ix, iy, iz],
            normal: [0.0, 1.0, 0.0],
        });

        let ox = cx + c * outer_r;
        let oz = cz + s * outer_r;
        let oy = terrain_height(ox, oz) + lift;
        verts.push(Vertex {
            position: [ox, oy, oz],
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

/// Generate the outer ring of the target bullseye: thin ring at 40–44m radius, terrain-conforming.
/// Vertices are in world space (model matrix = IDENTITY).
pub fn generate_target_outer_ring() -> (Vec<Vertex>, Vec<u32>) {
    // Target at DH (50000, 0, 0) → render (50000, 0, 0)
    generate_target_ring(50_000.0, 0.0, 40.0, 44.0)
}

/// Generate the inner ring + center disc of the target bullseye, terrain-conforming.
/// Inner ring: 15–18m. Center disc: 0–5m. Combined in one mesh, world-space vertices.
pub fn generate_target_inner_disc() -> (Vec<Vertex>, Vec<u32>) {
    let cx = 50_000.0_f32;
    let cz = 0.0_f32;

    // Inner ring (15–18m)
    let (mut verts, mut idxs) = generate_target_ring(cx, cz, 15.0, 18.0);

    // Center disc (0–5m) as a triangle fan
    let segments = 64_u32;
    let radius = 5.0_f32;
    let lift = 0.06_f32; // slightly above outer ring

    let center_base = verts.len() as u32;
    let cy = terrain_height(cx, cz) + lift;
    verts.push(Vertex {
        position: [cx, cy, cz],
        normal: [0.0, 1.0, 0.0],
    });

    for i in 0..=segments {
        let theta = i as f32 / segments as f32 * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        let px = cx + c * radius;
        let pz = cz + s * radius;
        let py = terrain_height(px, pz) + lift;
        verts.push(Vertex {
            position: [px, py, pz],
            normal: [0.0, 1.0, 0.0],
        });
    }

    for i in 0..segments {
        idxs.extend_from_slice(&[center_base, center_base + i + 1, center_base + i + 2]);
    }

    (verts, idxs)
}

/// Generate the launch rail.
pub fn generate_launch_rail() -> (Vec<Vertex>, Vec<u32>) {
    generate_cylinder(0.05, 5.0, 8)
}

/// White flag color.
pub const FLAG_COLOR: [f32; 4] = [0.95, 0.95, 0.92, 1.0];

/// Rock color (sandy brown).
pub const ROCK_COLOR: [f32; 4] = [0.50, 0.45, 0.38, 1.0];

/// Generate a flagpole: thin vertical cylinder (6m tall, 0.03m radius) with a flag quad at the top.
/// Geometry is in render Y-up space: pole along +Y, flag hangs from top.
pub fn generate_flagpole() -> (Vec<Vertex>, Vec<u32>) {
    // Pole: cylinder along Y axis
    let (mut verts, mut idxs) = generate_cylinder(0.03, 6.0, 8);

    // Shift pole up so base is at y=0 (generate_cylinder centers at origin)
    for v in &mut verts {
        v.position[1] += 3.0;
    }

    // Flag quad at top of pole (y ≈ 6.0), extending in +Z, hanging in -Y
    let flag_w = 0.8_f32;
    let flag_h = 0.5_f32;
    let top_y = 6.0_f32;

    // Front face (normal +X)
    let b = verts.len() as u32;
    verts.push(Vertex { position: [0.0, top_y, 0.0], normal: [1.0, 0.0, 0.0] });
    verts.push(Vertex { position: [0.0, top_y, flag_w], normal: [1.0, 0.0, 0.0] });
    verts.push(Vertex { position: [0.0, top_y - flag_h, flag_w], normal: [1.0, 0.0, 0.0] });
    verts.push(Vertex { position: [0.0, top_y - flag_h, 0.0], normal: [1.0, 0.0, 0.0] });
    idxs.extend_from_slice(&[b, b + 1, b + 2, b, b + 2, b + 3]);

    // Back face (normal -X)
    let b = verts.len() as u32;
    verts.push(Vertex { position: [0.0, top_y, 0.0], normal: [-1.0, 0.0, 0.0] });
    verts.push(Vertex { position: [0.0, top_y, flag_w], normal: [-1.0, 0.0, 0.0] });
    verts.push(Vertex { position: [0.0, top_y - flag_h, flag_w], normal: [-1.0, 0.0, 0.0] });
    verts.push(Vertex { position: [0.0, top_y - flag_h, 0.0], normal: [-1.0, 0.0, 0.0] });
    idxs.extend_from_slice(&[b, b + 2, b + 1, b, b + 3, b + 2]);

    (verts, idxs)
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

/// Deterministic hash for rock placement (returns 0..1).
fn rock_hash(seed: u32, i: u32) -> f32 {
    let mut h = seed.wrapping_add(i).wrapping_mul(2654435761);
    h = (h ^ (h >> 16)).wrapping_mul(2246822519);
    h = h ^ (h >> 13);
    (h as f32) / (u32::MAX as f32)
}

/// Generate ~50 low-poly rocks as a single combined mesh (world-space vertices).
/// Rocks cluster near launch (0–2km) and target (48–50km), sparse mid-route.
pub fn generate_rocks() -> (Vec<Vertex>, Vec<u32>) {
    let mut verts = Vec::new();
    let mut idxs = Vec::new();

    let num_rocks = 50_u32;

    for i in 0..num_rocks {
        // Determine X position: cluster near launch and target
        let t = rock_hash(100, i);
        let wx = if t < 0.45 {
            // Near launch: 50–2000m
            50.0 + rock_hash(200, i) * 1950.0
        } else if t < 0.90 {
            // Near target: 48000–50000m
            48_000.0 + rock_hash(201, i) * 2000.0
        } else {
            // Sparse mid-route
            2000.0 + rock_hash(202, i) * 46_000.0
        };

        // Z (render) position: ±200m from flight path
        let wz = (rock_hash(300, i) - 0.5) * 400.0;

        let base_y = terrain_height(wx, wz);

        // Rock size: 0.5–3m
        let size = 0.5 + rock_hash(400, i) * 2.5;

        // Generate irregular pyramid: 5 base vertices + 1 apex
        let base = verts.len() as u32;
        let n_base = 5_u32;
        let apex_y = base_y + size * (0.5 + rock_hash(500, i) * 0.5);

        // Base ring (on ground)
        for j in 0..n_base {
            let angle = j as f32 / n_base as f32 * std::f32::consts::TAU;
            let jitter_r = size * 0.4 * (0.6 + rock_hash(600 + j, i) * 0.8);
            let jitter_y = rock_hash(700 + j, i) * size * 0.15;
            let (s, c) = angle.sin_cos();
            let px = wx + c * jitter_r;
            let pz = wz + s * jitter_r;
            let py = base_y + jitter_y;
            // Approximate outward normal
            let ny = 0.3_f32;
            let len = (c * c + ny * ny + s * s).sqrt();
            verts.push(Vertex {
                position: [px, py, pz],
                normal: [c / len, ny / len, s / len],
            });
        }

        // Apex
        let apex_idx = verts.len() as u32;
        let ax = wx + (rock_hash(800, i) - 0.5) * size * 0.3;
        let az = wz + (rock_hash(801, i) - 0.5) * size * 0.3;
        verts.push(Vertex {
            position: [ax, apex_y, az],
            normal: [0.0, 1.0, 0.0],
        });

        // Side faces: triangles from each base edge to apex
        for j in 0..n_base {
            let j_next = (j + 1) % n_base;
            idxs.extend_from_slice(&[base + j, base + j_next, apex_idx]);
        }

        // Base face (fan from vertex 0)
        for j in 1..n_base - 1 {
            idxs.extend_from_slice(&[base, base + j + 1, base + j]); // wound opposite
        }
    }

    (verts, idxs)
}
