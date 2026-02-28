//! Sparse octree with Sd8 (i8) leaf values for signed distance field storage.
//!
//! The octree represents a 3D volume where each leaf stores a quantized SDF value.
//! Interior nodes subdivide space into 8 children. Uniform regions are stored as
//! Solid (fully inside) or Air (fully outside) to save memory.

use std::collections::HashSet;
use std::io::{Read, Write};
use std::path::Path;

/// Quantized signed distance: i8 maps to [-1.0, 1.0] range scaled by cell size.
/// Negative = inside, Positive = outside, 0 ≈ surface.
pub type Sd8 = i8;

/// Convert f32 SDF value to quantized Sd8, given the cell size for scaling.
pub fn quantize_sdf(value: f32, cell_size: f32) -> Sd8 {
    let normalized = (value / cell_size).clamp(-1.0, 1.0);
    (normalized * 127.0) as i8
}

/// Convert quantized Sd8 back to f32 SDF value.
pub fn dequantize_sdf(value: Sd8, cell_size: f32) -> f32 {
    (value as f32 / 127.0) * cell_size
}

/// Octree node.
#[derive(Debug, Clone)]
pub enum OctreeNode {
    /// Interior node with 8 children.
    Interior(Box<[OctreeNode; 8]>),
    /// Leaf node: a 3D grid of Sd8 values (CHUNK_SIZE^3).
    Leaf(LeafData),
    /// Uniform solid (all inside, SDF < 0).
    Solid,
    /// Uniform air (all outside, SDF > 0).
    Air,
}

/// Size of leaf chunks (each dimension).
pub const CHUNK_SIZE: usize = 32;
/// Total voxels in a leaf.
pub const CHUNK_VOXELS: usize = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;

/// Leaf node data: a flat array of Sd8 values.
#[derive(Debug, Clone)]
pub struct LeafData {
    pub values: Box<[Sd8; CHUNK_VOXELS]>,
}

impl LeafData {
    /// Create a leaf filled with a uniform value.
    pub fn uniform(value: Sd8) -> Self {
        Self {
            values: Box::new([value; CHUNK_VOXELS]),
        }
    }

    /// Index into the flat array from (x, y, z) local coordinates.
    #[inline]
    pub fn index(x: usize, y: usize, z: usize) -> usize {
        z * CHUNK_SIZE * CHUNK_SIZE + y * CHUNK_SIZE + x
    }

    /// Get value at local coordinates.
    #[inline]
    pub fn get(&self, x: usize, y: usize, z: usize) -> Sd8 {
        self.values[Self::index(x, y, z)]
    }

    /// Set value at local coordinates.
    #[inline]
    pub fn set(&mut self, x: usize, y: usize, z: usize, value: Sd8) {
        self.values[Self::index(x, y, z)] = value;
    }
}

/// Chunk coordinate (identifies a chunk in the grid).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ChunkCoord {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl ChunkCoord {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    /// World-space origin of this chunk.
    pub fn world_origin(&self, cell_size: f32) -> [f32; 3] {
        let cs = CHUNK_SIZE as f32 * cell_size;
        [
            self.x as f32 * cs,
            self.y as f32 * cs,
            self.z as f32 * cs,
        ]
    }
}

/// The sparse octree SDF volume.
///
/// For v1, we use a flat HashMap of chunks rather than a recursive octree,
/// which is simpler and still efficient for our use case (~6859 chunks max).
#[derive(Clone)]
pub struct OctreeSdf {
    /// Cell size in meters (resolution).
    pub cell_size: f32,
    /// Chunk storage.
    pub chunks: std::collections::HashMap<ChunkCoord, LeafData>,
    /// Set of chunks that have been modified and need remeshing.
    pub dirty_chunks: HashSet<ChunkCoord>,
    /// Grid extents in chunks (min inclusive).
    pub grid_min: [i32; 3],
    /// Grid extents in chunks (max exclusive).
    pub grid_max: [i32; 3],
}

impl OctreeSdf {
    /// Create a new octree SDF representing a solid rectangular block.
    ///
    /// `half_extents` are in meters. The block is centered at origin.
    pub fn new_block(half_extents: [f32; 3], cell_size: f32) -> Self {
        let chunk_world = CHUNK_SIZE as f32 * cell_size;

        // Compute chunk grid bounds
        let grid_min = [
            (-half_extents[0] / chunk_world).floor() as i32 - 1,
            (-half_extents[1] / chunk_world).floor() as i32 - 1,
            (-half_extents[2] / chunk_world).floor() as i32 - 1,
        ];
        let grid_max = [
            (half_extents[0] / chunk_world).ceil() as i32 + 1,
            (half_extents[1] / chunk_world).ceil() as i32 + 1,
            (half_extents[2] / chunk_world).ceil() as i32 + 1,
        ];

        let mut chunks = std::collections::HashMap::new();
        let mut dirty = HashSet::new();

        for cz in grid_min[2]..grid_max[2] {
            for cy in grid_min[1]..grid_max[1] {
                for cx in grid_min[0]..grid_max[0] {
                    let coord = ChunkCoord::new(cx, cy, cz);
                    let origin = coord.world_origin(cell_size);

                    let mut leaf = LeafData::uniform(127); // default: outside

                    let mut has_surface = false;
                    let mut all_inside = true;
                    let mut all_outside = true;

                    for lz in 0..CHUNK_SIZE {
                        for ly in 0..CHUNK_SIZE {
                            for lx in 0..CHUNK_SIZE {
                                let wx = origin[0] + lx as f32 * cell_size;
                                let wy = origin[1] + ly as f32 * cell_size;
                                let wz = origin[2] + lz as f32 * cell_size;

                                // Box SDF
                                let dx = wx.abs() - half_extents[0];
                                let dy = wy.abs() - half_extents[1];
                                let dz = wz.abs() - half_extents[2];
                                let outside =
                                    (dx.max(0.0).powi(2) + dy.max(0.0).powi(2) + dz.max(0.0).powi(2)).sqrt();
                                let inside = dx.max(dy).max(dz).min(0.0);
                                let sdf = outside + inside;

                                let q = quantize_sdf(sdf, cell_size);
                                leaf.set(lx, ly, lz, q);

                                if q < 0 {
                                    all_outside = false;
                                } else if q > 0 {
                                    all_inside = false;
                                } else {
                                    has_surface = true;
                                }
                            }
                        }
                    }

                    // Only store chunks that contain surface or interior
                    if !all_outside {
                        chunks.insert(coord, leaf);
                        if has_surface || (!all_inside && !all_outside) {
                            dirty.insert(coord);
                        }
                    }
                }
            }
        }

        Self {
            cell_size,
            chunks,
            dirty_chunks: dirty,
            grid_min,
            grid_max,
        }
    }

    /// Sample the SDF at a world-space point. Returns f32 distance.
    pub fn sample(&self, wx: f32, wy: f32, wz: f32) -> f32 {
        let chunk_world = CHUNK_SIZE as f32 * self.cell_size;
        let cx = (wx / chunk_world).floor() as i32;
        let cy = (wy / chunk_world).floor() as i32;
        let cz = (wz / chunk_world).floor() as i32;

        let coord = ChunkCoord::new(cx, cy, cz);

        if let Some(leaf) = self.chunks.get(&coord) {
            let origin = coord.world_origin(self.cell_size);
            let lx = ((wx - origin[0]) / self.cell_size) as usize;
            let ly = ((wy - origin[1]) / self.cell_size) as usize;
            let lz = ((wz - origin[2]) / self.cell_size) as usize;
            let lx = lx.min(CHUNK_SIZE - 1);
            let ly = ly.min(CHUNK_SIZE - 1);
            let lz = lz.min(CHUNK_SIZE - 1);
            dequantize_sdf(leaf.get(lx, ly, lz), self.cell_size)
        } else {
            // Not stored → outside
            self.cell_size * 10.0
        }
    }

    /// Apply CSG subtraction of a tool SDF over a region.
    /// `tool_sdf` is a closure: (wx, wy, wz) → signed distance to tool surface.
    /// Negative = inside tool volume (material to remove).
    ///
    /// Updates affected voxels: `new = max(existing, -tool)` (CSG subtract).
    pub fn subtract<F>(&mut self, bounds_min: [f32; 3], bounds_max: [f32; 3], tool_sdf: F)
    where
        F: Fn(f32, f32, f32) -> f32,
    {
        let chunk_world = CHUNK_SIZE as f32 * self.cell_size;

        let c_min = [
            (bounds_min[0] / chunk_world).floor() as i32,
            (bounds_min[1] / chunk_world).floor() as i32,
            (bounds_min[2] / chunk_world).floor() as i32,
        ];
        let c_max = [
            (bounds_max[0] / chunk_world).ceil() as i32,
            (bounds_max[1] / chunk_world).ceil() as i32,
            (bounds_max[2] / chunk_world).ceil() as i32,
        ];

        for cz in c_min[2]..=c_max[2] {
            for cy in c_min[1]..=c_max[1] {
                for cx in c_min[0]..=c_max[0] {
                    let coord = ChunkCoord::new(cx, cy, cz);

                    let leaf = match self.chunks.get_mut(&coord) {
                        Some(l) => l,
                        None => continue, // no material here
                    };

                    let origin = coord.world_origin(self.cell_size);
                    let mut modified = false;
                    // Track which chunk boundary faces are actually touched by modifications.
                    // Only dirty the neighbor if its shared face had voxels changed.
                    let mut touched_faces = [false; 6]; // -x, +x, -y, +y, -z, +z

                    // Compute local voxel range that overlaps the tool bounds
                    // to avoid iterating the full 32^3 chunk.
                    let lx_min = ((bounds_min[0] - origin[0]) / self.cell_size)
                        .floor().max(0.0) as usize;
                    let ly_min = ((bounds_min[1] - origin[1]) / self.cell_size)
                        .floor().max(0.0) as usize;
                    let lz_min = ((bounds_min[2] - origin[2]) / self.cell_size)
                        .floor().max(0.0) as usize;
                    let lx_max = ((bounds_max[0] - origin[0]) / self.cell_size)
                        .ceil().min(CHUNK_SIZE as f32) as usize;
                    let ly_max = ((bounds_max[1] - origin[1]) / self.cell_size)
                        .ceil().min(CHUNK_SIZE as f32) as usize;
                    let lz_max = ((bounds_max[2] - origin[2]) / self.cell_size)
                        .ceil().min(CHUNK_SIZE as f32) as usize;

                    let last = CHUNK_SIZE - 1;
                    for lz in lz_min..lz_max {
                        for ly in ly_min..ly_max {
                            for lx in lx_min..lx_max {
                                let old_q = leaf.get(lx, ly, lz);
                                // Skip voxels already fully outside (air)
                                if old_q >= 126 {
                                    continue;
                                }

                                let wx = origin[0] + lx as f32 * self.cell_size;
                                let wy = origin[1] + ly as f32 * self.cell_size;
                                let wz = origin[2] + lz as f32 * self.cell_size;

                                let tool_d = tool_sdf(wx, wy, wz);

                                // Only process voxels inside or near the tool surface
                                if tool_d < self.cell_size * 2.0 {
                                    let existing = dequantize_sdf(old_q, self.cell_size);
                                    let new_val = existing.max(-tool_d);
                                    let new_q = quantize_sdf(new_val, self.cell_size);
                                    if new_q != old_q {
                                        leaf.set(lx, ly, lz, new_q);
                                        modified = true;
                                        if lx == 0    { touched_faces[0] = true; }
                                        if lx == last  { touched_faces[1] = true; }
                                        if ly == 0    { touched_faces[2] = true; }
                                        if ly == last  { touched_faces[3] = true; }
                                        if lz == 0    { touched_faces[4] = true; }
                                        if lz == last  { touched_faces[5] = true; }
                                    }
                                }
                            }
                        }
                    }

                    if modified {
                        self.dirty_chunks.insert(coord);
                        // Only dirty face-adjacent neighbors whose shared boundary
                        // actually had voxels modified.
                        const FACE_OFFSETS: [(i32, i32, i32); 6] = [
                            (-1, 0, 0), (1, 0, 0),
                            (0, -1, 0), (0, 1, 0),
                            (0, 0, -1), (0, 0, 1),
                        ];
                        for (i, &(dx, dy, dz)) in FACE_OFFSETS.iter().enumerate() {
                            if touched_faces[i] {
                                let neighbor = ChunkCoord::new(cx + dx, cy + dy, cz + dz);
                                if self.chunks.contains_key(&neighbor) {
                                    self.dirty_chunks.insert(neighbor);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Take up to `max` dirty chunks nearest to `tool_pos` (SDF-local coords),
    /// removing them from the dirty set. Used for immediate sync meshing of
    /// the most visible chunks to eliminate 1-frame latency.
    pub fn take_nearest_dirty(&mut self, tool_pos: [f32; 3], max: usize) -> Vec<ChunkCoord> {
        if self.dirty_chunks.is_empty() || max == 0 {
            return Vec::new();
        }

        let chunk_half = CHUNK_SIZE as f32 * self.cell_size * 0.5;
        let mut dirty_with_dist: Vec<(ChunkCoord, f32)> = self
            .dirty_chunks
            .iter()
            .map(|&coord| {
                let origin = coord.world_origin(self.cell_size);
                let cx = origin[0] + chunk_half;
                let cy = origin[1] + chunk_half;
                let cz = origin[2] + chunk_half;
                let dist = (cx - tool_pos[0]).powi(2)
                    + (cy - tool_pos[1]).powi(2)
                    + (cz - tool_pos[2]).powi(2);
                (coord, dist)
            })
            .collect();

        dirty_with_dist.sort_by(|a, b| {
            a.1.partial_cmp(&b.1)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let taken: Vec<ChunkCoord> = dirty_with_dist
            .iter()
            .take(max)
            .map(|(coord, _)| *coord)
            .collect();

        for coord in &taken {
            self.dirty_chunks.remove(coord);
        }

        taken
    }

    /// Find all stored chunks that overlap the given world-space bounds.
    /// Used for predictive lookahead along upcoming toolpaths.
    pub fn chunks_in_bounds(&self, bounds_min: [f32; 3], bounds_max: [f32; 3]) -> Vec<ChunkCoord> {
        let chunk_world = CHUNK_SIZE as f32 * self.cell_size;
        let c_min = [
            (bounds_min[0] / chunk_world).floor() as i32,
            (bounds_min[1] / chunk_world).floor() as i32,
            (bounds_min[2] / chunk_world).floor() as i32,
        ];
        let c_max = [
            (bounds_max[0] / chunk_world).ceil() as i32,
            (bounds_max[1] / chunk_world).ceil() as i32,
            (bounds_max[2] / chunk_world).ceil() as i32,
        ];
        let mut coords = Vec::new();
        for cz in c_min[2]..=c_max[2] {
            for cy in c_min[1]..=c_max[1] {
                for cx in c_min[0]..=c_max[0] {
                    let coord = ChunkCoord::new(cx, cy, cz);
                    if self.chunks.contains_key(&coord) {
                        coords.push(coord);
                    }
                }
            }
        }
        coords
    }

    /// Clamp this SDF so no voxel is more "inside" than the ground truth.
    ///
    /// For each dirty chunk, sets `voxel = max(voxel, ground_truth_voxel)`.
    /// This eliminates floating material artifacts: any voxel that should be air
    /// in the final carved result cannot persist as material in intermediate frames.
    pub fn clamp_to_ground_truth(&mut self, ground_truth: &OctreeSdf) {
        let dirty: Vec<ChunkCoord> = self.dirty_chunks.iter().copied().collect();
        for coord in dirty {
            let current = match self.chunks.get_mut(&coord) {
                Some(c) => c,
                None => continue,
            };
            if let Some(gt) = ground_truth.chunks.get(&coord) {
                for i in 0..CHUNK_VOXELS {
                    let cv = current.values[i];
                    let gv = gt.values[i];
                    // max: push toward "outside" (positive) — can't be more inside than ground truth
                    if gv > cv {
                        current.values[i] = gv;
                    }
                }
            }
            // If ground truth has no chunk here (all air), the entire chunk is outside
            // in the final result — clear it to air.
            else {
                for i in 0..CHUNK_VOXELS {
                    current.values[i] = 127;
                }
            }
        }
    }

    /// Number of stored chunks.
    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    /// Number of dirty chunks pending remesh.
    pub fn dirty_count(&self) -> usize {
        self.dirty_chunks.len()
    }

    /// Take dirty chunks (clears the set, returns the coords).
    pub fn take_dirty(&mut self) -> Vec<ChunkCoord> {
        self.dirty_chunks.drain().collect()
    }

    /// Save the SDF to a binary file.
    ///
    /// Format: "SDF1" magic (4B) + cell_size f32 LE (4B) + num_chunks u32 LE (4B)
    /// + per chunk: x,y,z as i32 LE (12B) + values [i8; 32768].
    pub fn save_binary(&self, path: &Path) {
        let mut file = std::fs::File::create(path).expect("Failed to create SDF file");

        // Sort chunk coords for deterministic output
        let mut coords: Vec<ChunkCoord> = self.chunks.keys().copied().collect();
        coords.sort_by(|a, b| {
            a.z.cmp(&b.z).then(a.y.cmp(&b.y)).then(a.x.cmp(&b.x))
        });

        file.write_all(b"SDF1").unwrap();
        file.write_all(&self.cell_size.to_le_bytes()).unwrap();
        file.write_all(&(coords.len() as u32).to_le_bytes()).unwrap();

        for coord in &coords {
            let leaf = &self.chunks[coord];
            file.write_all(&coord.x.to_le_bytes()).unwrap();
            file.write_all(&coord.y.to_le_bytes()).unwrap();
            file.write_all(&coord.z.to_le_bytes()).unwrap();
            // Safety: Sd8 is i8, same layout as u8 for byte writing
            let bytes: &[u8] = unsafe {
                std::slice::from_raw_parts(leaf.values.as_ptr() as *const u8, CHUNK_VOXELS)
            };
            file.write_all(bytes).unwrap();
        }
    }

    /// Load an SDF from a binary file previously saved with `save_binary`.
    pub fn load_binary(path: &Path) -> Self {
        let mut file = std::fs::File::open(path).expect("Failed to open SDF file");

        let mut magic = [0u8; 4];
        file.read_exact(&mut magic).unwrap();
        assert_eq!(&magic, b"SDF1", "Invalid SDF file magic");

        let mut buf4 = [0u8; 4];
        file.read_exact(&mut buf4).unwrap();
        let cell_size = f32::from_le_bytes(buf4);

        file.read_exact(&mut buf4).unwrap();
        let num_chunks = u32::from_le_bytes(buf4) as usize;

        let mut chunks = std::collections::HashMap::with_capacity(num_chunks);
        let mut grid_min = [i32::MAX; 3];
        let mut grid_max = [i32::MIN; 3];

        for _ in 0..num_chunks {
            let mut buf12 = [0u8; 12];
            file.read_exact(&mut buf12).unwrap();
            let x = i32::from_le_bytes([buf12[0], buf12[1], buf12[2], buf12[3]]);
            let y = i32::from_le_bytes([buf12[4], buf12[5], buf12[6], buf12[7]]);
            let z = i32::from_le_bytes([buf12[8], buf12[9], buf12[10], buf12[11]]);

            let mut values = Box::new([0i8; CHUNK_VOXELS]);
            let bytes: &mut [u8] = unsafe {
                std::slice::from_raw_parts_mut(values.as_mut_ptr() as *mut u8, CHUNK_VOXELS)
            };
            file.read_exact(bytes).unwrap();

            let coord = ChunkCoord::new(x, y, z);
            chunks.insert(coord, LeafData { values });

            grid_min[0] = grid_min[0].min(x);
            grid_min[1] = grid_min[1].min(y);
            grid_min[2] = grid_min[2].min(z);
            grid_max[0] = grid_max[0].max(x + 1);
            grid_max[1] = grid_max[1].max(y + 1);
            grid_max[2] = grid_max[2].max(z + 1);
        }

        Self {
            cell_size,
            chunks,
            dirty_chunks: HashSet::new(),
            grid_min,
            grid_max,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quantize_roundtrip() {
        let cell_size = 0.0005; // 0.5mm
        let original = 0.0003_f32;
        let q = quantize_sdf(original, cell_size);
        let back = dequantize_sdf(q, cell_size);
        assert!(
            (back - original).abs() < cell_size * 0.02,
            "Roundtrip error: {} vs {}",
            back,
            original
        );
    }

    #[test]
    fn test_new_block() {
        let sdf = OctreeSdf::new_block([0.05, 0.05, 0.05], 0.005);
        assert!(sdf.chunk_count() > 0, "Block should have chunks");
        assert!(sdf.dirty_count() > 0, "New block should have dirty chunks");

        // Center should be inside
        let center_val = sdf.sample(0.0, 0.0, 0.0);
        assert!(center_val < 0.0, "Center should be inside: {}", center_val);

        // Far outside should be positive
        let outside_val = sdf.sample(1.0, 1.0, 1.0);
        assert!(outside_val > 0.0, "Far point should be outside: {}", outside_val);
    }

    #[test]
    fn test_subtract_sphere() {
        let mut sdf = OctreeSdf::new_block([0.05, 0.05, 0.05], 0.005);

        // Subtract a sphere at center
        let r = 0.02_f32;
        sdf.subtract(
            [-r - 0.01, -r - 0.01, -r - 0.01],
            [r + 0.01, r + 0.01, r + 0.01],
            |x, y, z| (x * x + y * y + z * z).sqrt() - r,
        );

        // Center should now be outside (material removed)
        let center_val = sdf.sample(0.0, 0.0, 0.0);
        assert!(
            center_val > 0.0,
            "Center should be outside after sphere subtract: {}",
            center_val
        );
    }
}
