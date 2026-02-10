//! Chunk-based isosurface meshing using fast-surface-nets.

use crate::octree::{ChunkCoord, LeafData, OctreeSdf, CHUNK_SIZE};
use fast_surface_nets::ndshape::{ConstShape, ConstShape3u32};
use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};

/// Padded chunk size for surface nets (need +2 for boundary sampling).
const PADDED: u32 = CHUNK_SIZE as u32 + 2;

type PaddedShape = ConstShape3u32<PADDED, PADDED, PADDED>;

/// Mesh output for a single chunk.
pub struct ChunkMesh {
    pub coord: ChunkCoord,
    /// Vertex positions in world space.
    pub positions: Vec<[f32; 3]>,
    /// Vertex normals.
    pub normals: Vec<[f32; 3]>,
    /// Triangle indices.
    pub indices: Vec<u32>,
}

impl ChunkMesh {
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }
}

/// Mesh a single chunk from the octree, producing positions, normals, and indices.
///
/// Returns None if the chunk has no surface (fully inside or outside).
pub fn mesh_chunk(sdf: &OctreeSdf, coord: ChunkCoord) -> Option<ChunkMesh> {
    let leaf = sdf.chunks.get(&coord)?;

    // Build padded SDF grid: PADDED^3
    // We need 1 extra voxel on each side from neighboring chunks.
    let mut grid = vec![1.0f32; (PADDED * PADDED * PADDED) as usize];

    fill_padded_grid(sdf, coord, leaf, &mut grid);

    // Run surface nets
    let mut buffer = SurfaceNetsBuffer::default();
    surface_nets(
        &grid,
        &PaddedShape {},
        [0; 3],
        [PADDED - 1; 3],
        &mut buffer,
    );

    if buffer.positions.is_empty() {
        return None;
    }

    // Transform positions from grid-local to world-space
    let origin = coord.world_origin(sdf.cell_size);
    let cs = sdf.cell_size;

    let positions: Vec<[f32; 3]> = buffer
        .positions
        .iter()
        .map(|p| {
            [
                origin[0] + (p[0] - 1.0) * cs, // -1 for padding offset
                origin[1] + (p[1] - 1.0) * cs,
                origin[2] + (p[2] - 1.0) * cs,
            ]
        })
        .collect();

    let normals: Vec<[f32; 3]> = buffer.normals.clone();
    let indices: Vec<u32> = buffer.indices.clone();

    Some(ChunkMesh {
        coord,
        positions,
        normals,
        indices,
    })
}

/// Fill the padded grid from the chunk and its neighbors.
fn fill_padded_grid(sdf: &OctreeSdf, coord: ChunkCoord, leaf: &LeafData, grid: &mut [f32]) {
    let cs = sdf.cell_size;

    for gz in 0..PADDED {
        for gy in 0..PADDED {
            for gx in 0..PADDED {
                // Map padded grid coord to local chunk coord (with -1 offset)
                let lx = gx as i32 - 1;
                let ly = gy as i32 - 1;
                let lz = gz as i32 - 1;

                let value = if lx >= 0
                    && lx < CHUNK_SIZE as i32
                    && ly >= 0
                    && ly < CHUNK_SIZE as i32
                    && lz >= 0
                    && lz < CHUNK_SIZE as i32
                {
                    // Inside this chunk
                    crate::octree::dequantize_sdf(
                        leaf.get(lx as usize, ly as usize, lz as usize),
                        cs,
                    )
                } else {
                    // Need to sample from neighbor
                    sample_neighbor(sdf, coord, lx, ly, lz)
                };

                let idx = PaddedShape::linearize([gx, gy, gz]) as usize;
                grid[idx] = value;
            }
        }
    }
}

/// Sample a voxel that's outside the current chunk bounds.
fn sample_neighbor(sdf: &OctreeSdf, coord: ChunkCoord, lx: i32, ly: i32, lz: i32) -> f32 {
    let cs = CHUNK_SIZE as i32;

    // Determine which neighbor chunk and local coords
    let cx = coord.x + lx.div_euclid(cs);
    let cy = coord.y + ly.div_euclid(cs);
    let cz = coord.z + lz.div_euclid(cs);
    let nlx = lx.rem_euclid(cs) as usize;
    let nly = ly.rem_euclid(cs) as usize;
    let nlz = lz.rem_euclid(cs) as usize;

    let neighbor_coord = ChunkCoord::new(cx, cy, cz);
    if let Some(neighbor) = sdf.chunks.get(&neighbor_coord) {
        crate::octree::dequantize_sdf(neighbor.get(nlx, nly, nlz), sdf.cell_size)
    } else {
        sdf.cell_size * 10.0 // outside
    }
}

/// Remesh dirty chunks and return the meshes.
/// Caps at `max_per_frame` to avoid render stalls; remaining stay dirty for next frame.
pub fn remesh_dirty(sdf: &mut OctreeSdf) -> Vec<ChunkMesh> {
    remesh_dirty_capped(sdf, 8)
}

/// Remesh up to `max` dirty chunks. Remaining chunks stay in the dirty set.
pub fn remesh_dirty_capped(sdf: &mut OctreeSdf, max: usize) -> Vec<ChunkMesh> {
    let mut meshes = Vec::with_capacity(max);

    // Take only up to `max` dirty coords; leave the rest for next frame.
    let mut count = 0;
    let mut processed = Vec::with_capacity(max);
    for &coord in sdf.dirty_chunks.iter() {
        if count >= max {
            break;
        }
        processed.push(coord);
        count += 1;
    }

    for coord in &processed {
        sdf.dirty_chunks.remove(coord);
        if let Some(mesh) = mesh_chunk(sdf, *coord) {
            meshes.push(mesh);
        }
    }

    meshes
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::octree::OctreeSdf;

    #[test]
    fn test_mesh_block() {
        let mut sdf = OctreeSdf::new_block([0.02, 0.02, 0.02], 0.002);
        let meshes = remesh_dirty(&mut sdf);
        // Should produce some meshes
        let total_tris: usize = meshes.iter().map(|m| m.indices.len() / 3).sum();
        assert!(
            total_tris > 0,
            "Block should produce triangles, got {}",
            total_tris
        );
    }
}
