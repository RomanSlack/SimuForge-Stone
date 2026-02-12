//! Background chunk meshing using a dedicated rayon thread pool.
//!
//! The main thread snapshots dirty chunk data and submits mesh requests.
//! A rayon pool meshes them in parallel, and completed meshes are polled
//! back on the main thread for GPU upload.

use crate::mesher::ChunkMesh;
use crate::octree::{ChunkCoord, LeafData, OctreeSdf, CHUNK_SIZE};
use std::sync::mpsc;

/// Snapshot of one chunk plus its 6 face-neighbor data, sufficient for
/// padded-grid meshing without access to the OctreeSdf.
pub struct MeshRequest {
    pub coord: ChunkCoord,
    pub center: LeafData,
    /// Face neighbors in order: -x, +x, -y, +y, -z, +z.
    pub neighbors: [Option<LeafData>; 6],
    pub cell_size: f32,
}

const FACE_OFFSETS: [(i32, i32, i32); 6] = [
    (-1, 0, 0),
    (1, 0, 0),
    (0, -1, 0),
    (0, 1, 0),
    (0, 0, -1),
    (0, 0, 1),
];

impl MeshRequest {
    /// Create a snapshot from the octree for a given chunk coordinate.
    pub fn snapshot(sdf: &OctreeSdf, coord: ChunkCoord) -> Option<Self> {
        let center = sdf.chunks.get(&coord)?.clone();
        let neighbors = FACE_OFFSETS.map(|(dx, dy, dz)| {
            let nc = ChunkCoord::new(coord.x + dx, coord.y + dy, coord.z + dz);
            sdf.chunks.get(&nc).cloned()
        });
        Some(Self {
            coord,
            center,
            neighbors,
            cell_size: sdf.cell_size,
        })
    }
}

/// Mesh a chunk from a snapshot (no OctreeSdf reference needed).
/// Suitable for calling on worker threads.
fn mesh_from_snapshot(req: &MeshRequest) -> Option<ChunkMesh> {
    use crate::octree::dequantize_sdf;
    use fast_surface_nets::ndshape::{ConstShape, ConstShape3u32};
    use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};

    const PADDED: u32 = CHUNK_SIZE as u32 + 2;
    type PaddedShape = ConstShape3u32<PADDED, PADDED, PADDED>;

    // Use thread-local buffers for the worker thread
    thread_local! {
        static GRID: std::cell::RefCell<Vec<f32>> = std::cell::RefCell::new(
            vec![1.0f32; (PADDED * PADDED * PADDED) as usize]
        );
        static SNB: std::cell::RefCell<SurfaceNetsBuffer> = std::cell::RefCell::new(
            SurfaceNetsBuffer::default()
        );
    }

    GRID.with(|grid_cell| {
        SNB.with(|snb_cell| {
            let mut grid = grid_cell.borrow_mut();
            let mut sn_buffer = snb_cell.borrow_mut();

            grid.fill(1.0f32);

            let cs = req.cell_size;
            let cs_i = CHUNK_SIZE as i32;

            // Fill padded grid from snapshot
            for gz in 0..PADDED {
                for gy in 0..PADDED {
                    for gx in 0..PADDED {
                        let lx = gx as i32 - 1;
                        let ly = gy as i32 - 1;
                        let lz = gz as i32 - 1;

                        let value = if lx >= 0
                            && lx < cs_i
                            && ly >= 0
                            && ly < cs_i
                            && lz >= 0
                            && lz < cs_i
                        {
                            dequantize_sdf(
                                req.center.get(lx as usize, ly as usize, lz as usize),
                                cs,
                            )
                        } else {
                            sample_from_neighbors(req, lx, ly, lz)
                        };

                        let idx = PaddedShape::linearize([gx, gy, gz]) as usize;
                        grid[idx] = value;
                    }
                }
            }

            surface_nets(
                &grid,
                &PaddedShape {},
                [0; 3],
                [PADDED - 1; 3],
                &mut sn_buffer,
            );

            if sn_buffer.positions.is_empty() {
                return None;
            }

            let origin = req.coord.world_origin(cs);

            let positions: Vec<[f32; 3]> = sn_buffer
                .positions
                .iter()
                .map(|p| {
                    [
                        origin[0] + (p[0] - 1.0) * cs,
                        origin[1] + (p[1] - 1.0) * cs,
                        origin[2] + (p[2] - 1.0) * cs,
                    ]
                })
                .collect();

            let mut normals = Vec::new();
            std::mem::swap(&mut normals, &mut sn_buffer.normals);
            let mut indices = Vec::new();
            std::mem::swap(&mut indices, &mut sn_buffer.indices);

            Some(ChunkMesh {
                coord: req.coord,
                positions,
                normals,
                indices,
            })
        })
    })
}

/// Sample a boundary voxel from the snapshot's neighbor data.
/// For edge/corner voxels (multiple axes out of range), we clamp the
/// extra out-of-range coords to the nearest valid value in the face neighbor.
/// This preserves SDF sign continuity and avoids micro-hole artifacts.
fn sample_from_neighbors(req: &MeshRequest, lx: i32, ly: i32, lz: i32) -> f32 {
    let cs = CHUNK_SIZE as i32;
    let clamp = |v: i32| -> usize { v.clamp(0, cs - 1) as usize };

    // Pick the first out-of-range axis as the face neighbor to sample from.
    // Any remaining out-of-range coords get clamped to the nearest valid index.
    let (neighbor_idx, nlx, nly, nlz) = if lx < 0 {
        (0, (lx + cs) as usize, clamp(ly), clamp(lz))
    } else if lx >= cs {
        (1, (lx - cs) as usize, clamp(ly), clamp(lz))
    } else if ly < 0 {
        (2, clamp(lx), (ly + cs) as usize, clamp(lz))
    } else if ly >= cs {
        (3, clamp(lx), (ly - cs) as usize, clamp(lz))
    } else if lz < 0 {
        (4, clamp(lx), clamp(ly), (lz + cs) as usize)
    } else {
        (5, clamp(lx), clamp(ly), (lz - cs) as usize)
    };

    if let Some(neighbor) = &req.neighbors[neighbor_idx] {
        crate::octree::dequantize_sdf(neighbor.get(nlx, nly, nlz), req.cell_size)
    } else {
        req.cell_size * 10.0
    }
}

/// Background mesher that processes chunks on a rayon thread pool.
pub struct AsyncMesher {
    req_tx: mpsc::Sender<Vec<MeshRequest>>,
    result_rx: mpsc::Receiver<ChunkMesh>,
    num_threads: usize,
}

impl AsyncMesher {
    /// Create a new background mesher.
    /// `num_threads`: number of worker threads (0 = auto-detect).
    pub fn new(num_threads: usize) -> Self {
        let (req_tx, req_rx) = mpsc::channel::<Vec<MeshRequest>>();
        let (result_tx, result_rx) = mpsc::channel::<ChunkMesh>();

        let threads = if num_threads == 0 {
            num_cpus()
        } else {
            num_threads
        };

        // The driver thread owns the rayon pool and dispatches work into it.
        std::thread::Builder::new()
            .name("mesh-driver".into())
            .spawn(move || {
                let pool = rayon::ThreadPoolBuilder::new()
                    .num_threads(threads)
                    .thread_name(|i| format!("mesher-{}", i))
                    .build()
                    .expect("Failed to create rayon thread pool");

                while let Ok(batch) = req_rx.recv() {
                    let tx = &result_tx;
                    pool.scope(|s| {
                        for req in batch {
                            let tx = tx.clone();
                            s.spawn(move |_| {
                                if let Some(mesh) = mesh_from_snapshot(&req) {
                                    let _ = tx.send(mesh);
                                }
                            });
                        }
                    });
                }
            })
            .expect("Failed to spawn mesh driver thread");

        Self {
            req_tx,
            result_rx,
            num_threads: threads,
        }
    }

    /// Snapshot dirty chunks from the octree and submit them for background meshing.
    /// Clears the dirty set.
    pub fn submit_dirty(&self, sdf: &mut OctreeSdf) {
        let dirty: Vec<ChunkCoord> = sdf.dirty_chunks.drain().collect();
        if dirty.is_empty() {
            return;
        }

        let batch: Vec<MeshRequest> = dirty
            .into_iter()
            .filter_map(|coord| MeshRequest::snapshot(sdf, coord))
            .collect();

        if !batch.is_empty() {
            let _ = self.req_tx.send(batch);
        }
    }

    /// Poll for completed meshes. Non-blocking — returns whatever is ready.
    pub fn poll_completed(&self) -> Vec<ChunkMesh> {
        let mut results = Vec::new();
        while let Ok(mesh) = self.result_rx.try_recv() {
            results.push(mesh);
        }
        results
    }

    /// Number of worker threads in the pool.
    pub fn num_threads(&self) -> usize {
        self.num_threads
    }
}

fn num_cpus() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get().max(1))
        .unwrap_or(4)
}
