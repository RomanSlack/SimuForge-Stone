//! SDF redistancing: connectivity cleanup + Fast Sweeping Method.
//!
//! After many CSG `max()` operations, the SDF values no longer represent true
//! distances from the surface. This creates two problems:
//!
//! 1. **Isolated negative pockets** — tiny disconnected regions of negative
//!    (inside) values that produce floating geometry when meshed.
//! 2. **Corrupted distance values** — the Eikonal property |∇d| = 1 is violated,
//!    causing noisy vertex positions and normals in the mesh.
//!
//! This module fixes both:
//! - **Connectivity cleanup**: BFS flood-fill finds the largest connected
//!   component of negative (interior) voxels. All smaller components are
//!   flipped to positive, eliminating isolated pockets.
//! - **Fast Sweeping**: Recomputes proper signed distances from zero-crossings
//!   by solving the Eikonal equation in 8 sweep directions.

use crate::octree::{dequantize_sdf, quantize_sdf, OctreeSdf, CHUNK_SIZE};
use std::collections::VecDeque;

/// Redistance the SDF: remove isolated pockets and recompute proper distances.
///
/// 1. Builds a flat grid from all chunks
/// 2. Flood-fills to find connected interior components; removes isolated pockets
/// 3. Identifies zero-crossings and computes subcell distances
/// 4. Fast-sweeps to propagate correct Eikonal distances
/// 5. Writes back to chunks (only marks dirty if values changed)
pub fn redistance(sdf: &mut OctreeSdf) {
    if sdf.chunks.is_empty() {
        return;
    }

    let cs = sdf.cell_size;

    // Global grid dimensions from chunk extents
    let nx = (sdf.grid_max[0] - sdf.grid_min[0]) as usize * CHUNK_SIZE;
    let ny = (sdf.grid_max[1] - sdf.grid_min[1]) as usize * CHUNK_SIZE;
    let nz = (sdf.grid_max[2] - sdf.grid_min[2]) as usize * CHUNK_SIZE;
    let total = nx * ny * nz;

    if total == 0 || total > 50_000_000 {
        return;
    }

    let idx = |gx: usize, gy: usize, gz: usize| -> usize {
        gx + gy * nx + gz * nx * ny
    };

    // --- Step 1: Build flat grid from chunks ---
    let default_outside = cs * 100.0;
    let mut grid = vec![default_outside; total];

    for (&coord, leaf) in &sdf.chunks {
        let bx = (coord.x - sdf.grid_min[0]) as usize * CHUNK_SIZE;
        let by = (coord.y - sdf.grid_min[1]) as usize * CHUNK_SIZE;
        let bz = (coord.z - sdf.grid_min[2]) as usize * CHUNK_SIZE;

        for lz in 0..CHUNK_SIZE {
            for ly in 0..CHUNK_SIZE {
                for lx in 0..CHUNK_SIZE {
                    let i = idx(bx + lx, by + ly, bz + lz);
                    grid[i] = dequantize_sdf(leaf.get(lx, ly, lz), cs);
                }
            }
        }
    }

    // --- Step 2: Connectivity cleanup — remove isolated negative pockets ---
    //
    // BFS flood-fill through all 6-connected negative voxels. The largest
    // connected component is the workpiece body; all smaller components
    // are isolated pockets from CSG corruption → flip to positive.
    remove_isolated_pockets(&mut grid, nx, ny, nz);

    // --- Step 3: Zero-crossing detection and initialization ---
    let signs: Vec<i8> = grid.iter().map(|&v| if v < 0.0 { -1 } else { 1 }).collect();
    let mut frozen = vec![false; total];
    let huge = cs * 1000.0;

    for gz in 0..nz {
        for gy in 0..ny {
            for gx in 0..nx {
                let i = idx(gx, gy, gz);
                let s = signs[i];

                let has_crossing =
                    (gx > 0      && signs[idx(gx - 1, gy, gz)] != s)
                    || (gx < nx - 1 && signs[idx(gx + 1, gy, gz)] != s)
                    || (gy > 0      && signs[idx(gx, gy - 1, gz)] != s)
                    || (gy < ny - 1 && signs[idx(gx, gy + 1, gz)] != s)
                    || (gz > 0      && signs[idx(gx, gy, gz - 1)] != s)
                    || (gz < nz - 1 && signs[idx(gx, gy, gz + 1)] != s);

                if has_crossing {
                    // Subcell distance to zero-crossing via linear interpolation
                    let phi = grid[i].abs();
                    let mut min_dist = phi;

                    let subcell = |ni: usize| -> f32 {
                        let phi_n = grid[ni].abs();
                        let denom = phi + phi_n;
                        if denom > 1e-12 {
                            cs * phi / denom
                        } else {
                            cs * 0.5
                        }
                    };

                    if gx > 0 && signs[idx(gx - 1, gy, gz)] != s {
                        min_dist = min_dist.min(subcell(idx(gx - 1, gy, gz)));
                    }
                    if gx < nx - 1 && signs[idx(gx + 1, gy, gz)] != s {
                        min_dist = min_dist.min(subcell(idx(gx + 1, gy, gz)));
                    }
                    if gy > 0 && signs[idx(gx, gy - 1, gz)] != s {
                        min_dist = min_dist.min(subcell(idx(gx, gy - 1, gz)));
                    }
                    if gy < ny - 1 && signs[idx(gx, gy + 1, gz)] != s {
                        min_dist = min_dist.min(subcell(idx(gx, gy + 1, gz)));
                    }
                    if gz > 0 && signs[idx(gx, gy, gz - 1)] != s {
                        min_dist = min_dist.min(subcell(idx(gx, gy, gz - 1)));
                    }
                    if gz < nz - 1 && signs[idx(gx, gy, gz + 1)] != s {
                        min_dist = min_dist.min(subcell(idx(gx, gy, gz + 1)));
                    }

                    grid[i] = min_dist;
                    frozen[i] = true;
                } else {
                    grid[i] = huge;
                }
            }
        }
    }

    // --- Step 4: Fast Sweeping — 8 directions × 2 passes ---
    let h = cs;

    for _pass in 0..2 {
        for sweep in 0..8u8 {
            let x_fwd = (sweep & 1) == 0;
            let y_fwd = (sweep & 2) == 0;
            let z_fwd = (sweep & 4) == 0;

            for gz_i in 1..nz.saturating_sub(1) {
                let gz = if z_fwd { gz_i } else { nz - 1 - gz_i };
                for gy_i in 1..ny.saturating_sub(1) {
                    let gy = if y_fwd { gy_i } else { ny - 1 - gy_i };
                    for gx_i in 1..nx.saturating_sub(1) {
                        let gx = if x_fwd { gx_i } else { nx - 1 - gx_i };

                        let i = idx(gx, gy, gz);
                        if frozen[i] {
                            continue;
                        }

                        let ax = grid[idx(gx - 1, gy, gz)].min(grid[idx(gx + 1, gy, gz)]);
                        let ay = grid[idx(gx, gy - 1, gz)].min(grid[idx(gx, gy + 1, gz)]);
                        let az = grid[idx(gx, gy, gz - 1)].min(grid[idx(gx, gy, gz + 1)]);

                        let d_new = eikonal_update(ax, ay, az, h);

                        if d_new < grid[i] {
                            grid[i] = d_new;
                        }
                    }
                }
            }
        }
    }

    // --- Step 5: Restore signs and write back to chunks ---
    for (&coord, leaf) in sdf.chunks.iter_mut() {
        let bx = (coord.x - sdf.grid_min[0]) as usize * CHUNK_SIZE;
        let by = (coord.y - sdf.grid_min[1]) as usize * CHUNK_SIZE;
        let bz = (coord.z - sdf.grid_min[2]) as usize * CHUNK_SIZE;

        let mut changed = false;
        for lz in 0..CHUNK_SIZE {
            for ly in 0..CHUNK_SIZE {
                for lx in 0..CHUNK_SIZE {
                    let i = idx(bx + lx, by + ly, bz + lz);
                    let signed_val = grid[i] * signs[i] as f32;
                    let new_q = quantize_sdf(signed_val, cs);
                    let old_q = leaf.get(lx, ly, lz);
                    if new_q != old_q {
                        leaf.set(lx, ly, lz, new_q);
                        changed = true;
                    }
                }
            }
        }

        if changed {
            sdf.dirty_chunks.insert(coord);
        }
    }
}

/// Remove isolated negative (interior) pockets via 6-connected BFS flood-fill.
///
/// Finds all connected components of negative voxels. The largest component
/// (the workpiece body) is kept; all smaller components are flipped to positive.
fn remove_isolated_pockets(grid: &mut [f32], nx: usize, ny: usize, nz: usize) {
    let total = nx * ny * nz;

    let idx = |gx: usize, gy: usize, gz: usize| -> usize {
        gx + gy * nx + gz * nx * ny
    };

    // Component label per voxel (0 = unassigned or positive)
    let mut comp_id = vec![0u32; total];
    let mut comp_sizes: Vec<usize> = vec![0]; // index 0 unused; comp_sizes[id] = size
    let mut current_id = 0u32;

    let mut queue = VecDeque::new();

    for start in 0..total {
        // Skip positive (outside) voxels and already-labeled voxels
        if grid[start] >= 0.0 || comp_id[start] != 0 {
            continue;
        }

        current_id += 1;
        comp_sizes.push(0);

        queue.push_back(start);
        comp_id[start] = current_id;

        while let Some(i) = queue.pop_front() {
            comp_sizes[current_id as usize] += 1;

            let gx = i % nx;
            let gy = (i / nx) % ny;
            let gz = i / (nx * ny);

            // 6-connected neighbors
            let neighbors: [(usize, usize, usize); 6] = [
                (gx.wrapping_sub(1), gy, gz),
                (gx + 1, gy, gz),
                (gx, gy.wrapping_sub(1), gz),
                (gx, gy + 1, gz),
                (gx, gy, gz.wrapping_sub(1)),
                (gx, gy, gz + 1),
            ];

            for (ngx, ngy, ngz) in neighbors {
                if ngx < nx && ngy < ny && ngz < nz {
                    let ni = idx(ngx, ngy, ngz);
                    if grid[ni] < 0.0 && comp_id[ni] == 0 {
                        comp_id[ni] = current_id;
                        queue.push_back(ni);
                    }
                }
            }
        }
    }

    if current_id == 0 {
        return; // No negative voxels at all
    }

    // Find the largest component
    let largest_id = comp_sizes
        .iter()
        .enumerate()
        .skip(1)
        .max_by_key(|(_, &s)| s)
        .map(|(i, _)| i as u32)
        .unwrap_or(0);

    // Flip all non-largest negative components to positive
    let mut flipped = 0usize;
    for i in 0..total {
        if comp_id[i] != 0 && comp_id[i] != largest_id {
            grid[i] = grid[i].abs();
            flipped += 1;
        }
    }

    if flipped > 0 {
        eprintln!(
            "Redistance: removed {} isolated pocket voxels ({} components, largest={})",
            flipped,
            current_id - 1,
            comp_sizes[largest_id as usize]
        );
    }
}

/// Solve the Eikonal equation at one grid point.
#[inline]
fn eikonal_update(ax: f32, ay: f32, az: f32, h: f32) -> f32 {
    let (a, b, c) = sort3(ax, ay, az);

    let mut d = a + h;

    if d > b {
        let disc = 2.0 * h * h - (a - b) * (a - b);
        if disc >= 0.0 {
            d = (a + b + disc.sqrt()) * 0.5;
        }
    }

    if d > c {
        let sum = a + b + c;
        let sum_sq = a * a + b * b + c * c;
        let disc = sum * sum - 3.0 * (sum_sq - h * h);
        if disc >= 0.0 {
            d = (sum + disc.sqrt()) / 3.0;
        }
    }

    d
}

/// Sort three f32 values: returns (min, mid, max).
#[inline]
fn sort3(a: f32, b: f32, c: f32) -> (f32, f32, f32) {
    let (mut x, mut y, mut z) = (a, b, c);
    if x > y {
        std::mem::swap(&mut x, &mut y);
    }
    if y > z {
        std::mem::swap(&mut y, &mut z);
    }
    if x > y {
        std::mem::swap(&mut x, &mut y);
    }
    (x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::octree::{ChunkCoord, OctreeSdf};

    #[test]
    fn test_redistance_preserves_block() {
        let mut sdf = OctreeSdf::new_block([0.02, 0.02, 0.02], 0.002);
        sdf.dirty_chunks.clear();

        let center_before = sdf.sample(0.0, 0.0, 0.0);
        let surface_before = sdf.sample(0.02, 0.0, 0.0);

        redistance(&mut sdf);

        let center_after = sdf.sample(0.0, 0.0, 0.0);
        let surface_after = sdf.sample(0.02, 0.0, 0.0);

        assert!(
            center_after < 0.0,
            "Center should remain inside: before={}, after={}",
            center_before,
            center_after
        );
        assert!(
            surface_after.abs() < 0.003,
            "Surface should remain near zero: before={}, after={}",
            surface_before,
            surface_after
        );
    }

    #[test]
    fn test_redistance_fixes_isolated_pocket() {
        let mut sdf = OctreeSdf::new_block([0.02, 0.02, 0.02], 0.002);

        // Inject a fake negative pocket in an air chunk far from the block
        let chunk_world = CHUNK_SIZE as f32 * 0.002;
        let cx = (0.05_f32 / chunk_world).floor() as i32;
        let cy = (0.05_f32 / chunk_world).floor() as i32;
        let cz = (0.05_f32 / chunk_world).floor() as i32;
        let coord = ChunkCoord::new(cx, cy, cz);

        let mut leaf = crate::octree::LeafData::uniform(quantize_sdf(0.01, 0.002));
        for z in 14..17 {
            for y in 14..17 {
                for x in 14..17 {
                    leaf.set(x, y, z, quantize_sdf(-0.001, 0.002));
                }
            }
        }
        sdf.chunks.insert(coord, leaf);

        sdf.grid_min[0] = sdf.grid_min[0].min(cx);
        sdf.grid_min[1] = sdf.grid_min[1].min(cy);
        sdf.grid_min[2] = sdf.grid_min[2].min(cz);
        sdf.grid_max[0] = sdf.grid_max[0].max(cx + 1);
        sdf.grid_max[1] = sdf.grid_max[1].max(cy + 1);
        sdf.grid_max[2] = sdf.grid_max[2].max(cz + 1);

        sdf.dirty_chunks.clear();

        let origin = coord.world_origin(0.002);
        let pocket_pos = [
            origin[0] + 15.0 * 0.002,
            origin[1] + 15.0 * 0.002,
            origin[2] + 15.0 * 0.002,
        ];
        let pocket_val = sdf.sample(pocket_pos[0], pocket_pos[1], pocket_pos[2]);
        assert!(pocket_val < 0.0, "Pocket should be negative before: {}", pocket_val);

        redistance(&mut sdf);

        let pocket_after = sdf.sample(pocket_pos[0], pocket_pos[1], pocket_pos[2]);
        assert!(
            pocket_after > 0.0,
            "Isolated pocket should be positive after redistancing: {}",
            pocket_after
        );
    }

    #[test]
    fn test_redistance_after_carving() {
        // Subtract a sphere, then verify redistancing doesn't break the result
        let mut sdf = OctreeSdf::new_block([0.02, 0.02, 0.02], 0.002);

        let r = 0.01_f32;
        sdf.subtract(
            [-r - 0.005, -r - 0.005, -r - 0.005],
            [r + 0.005, r + 0.005, r + 0.005],
            |x, y, z| (x * x + y * y + z * z).sqrt() - r,
        );

        // Center should be outside (sphere removed material)
        let center_before = sdf.sample(0.0, 0.0, 0.0);
        assert!(center_before > 0.0, "Center should be outside: {}", center_before);

        // Corner should still be inside (sphere didn't reach corners)
        let corner_before = sdf.sample(0.018, 0.018, 0.018);
        assert!(corner_before < 0.0, "Corner should be inside: {}", corner_before);

        sdf.dirty_chunks.clear();
        redistance(&mut sdf);

        let center_after = sdf.sample(0.0, 0.0, 0.0);
        assert!(center_after > 0.0, "Center still outside: {}", center_after);

        let corner_after = sdf.sample(0.018, 0.018, 0.018);
        assert!(corner_after < 0.0, "Corner still inside: {}", corner_after);
    }

    #[test]
    fn test_eikonal_update() {
        let h = 1.0;
        assert!((eikonal_update(0.0, 100.0, 100.0, h) - 1.0).abs() < 1e-5);

        let d = eikonal_update(0.0, 0.0, 100.0, h);
        let expected = (2.0_f32).sqrt() * 0.5;
        assert!(
            (d - expected).abs() < 0.01,
            "2D Eikonal: expected {}, got {}",
            expected,
            d
        );
    }

    #[test]
    fn test_sort3() {
        assert_eq!(sort3(3.0, 1.0, 2.0), (1.0, 2.0, 3.0));
        assert_eq!(sort3(1.0, 2.0, 3.0), (1.0, 2.0, 3.0));
        assert_eq!(sort3(3.0, 2.0, 1.0), (1.0, 2.0, 3.0));
    }
}
