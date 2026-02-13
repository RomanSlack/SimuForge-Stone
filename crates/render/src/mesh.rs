//! Dynamic mesh buffer management for chunk-based rendering.
//!
//! Stores per-chunk CPU data and builds a single merged GPU buffer
//! for the entire workpiece, reducing draw calls from ~2000 to 1.
//!
//! The merged buffer is persistent: it grows when needed (2x capacity)
//! but is never reallocated when only data changes. This avoids
//! GPU alloc/dealloc stalls during cutting (~10MB buffer per frame).

use std::collections::HashMap;
use simuforge_core::Vertex;

/// CPU-side mesh data for one chunk.
struct ChunkCpuMesh {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
}

/// Manages per-chunk meshes and a single merged GPU buffer for efficient rendering.
///
/// Individual chunks are stored CPU-side. When any chunk changes, the merged
/// GPU vertex+index buffers are overwritten via `write_buffer` so the entire
/// workpiece can be drawn in a single `draw_indexed()` call.
pub struct ChunkMeshManager {
    chunks: HashMap<(i32, i32, i32), ChunkCpuMesh>,
    /// Persistent merged GPU vertex buffer.
    merged_vb: Option<wgpu::Buffer>,
    /// Persistent merged GPU index buffer.
    merged_ib: Option<wgpu::Buffer>,
    merged_num_indices: u32,
    /// Allocated capacity in bytes (grows with 2x factor, never shrinks).
    vb_capacity: u64,
    ib_capacity: u64,
    /// Persistent CPU-side merged arrays (reused across rebuilds to avoid allocation).
    merged_verts: Vec<Vertex>,
    merged_indices: Vec<u32>,
    dirty: bool,
}

impl ChunkMeshManager {
    pub fn new() -> Self {
        Self {
            chunks: HashMap::new(),
            merged_vb: None,
            merged_ib: None,
            merged_num_indices: 0,
            vb_capacity: 0,
            ib_capacity: 0,
            merged_verts: Vec::new(),
            merged_indices: Vec::new(),
            dirty: false,
        }
    }

    /// Store or update a chunk's mesh data. Call `rebuild_if_dirty` before rendering.
    pub fn upload_chunk(
        &mut self,
        _device: &wgpu::Device,
        _queue: &wgpu::Queue,
        coord: (i32, i32, i32),
        positions: &[[f32; 3]],
        normals: &[[f32; 3]],
        indices: &[u32],
    ) {
        if indices.is_empty() {
            if self.chunks.remove(&coord).is_some() {
                self.dirty = true;
            }
            return;
        }

        let vertices: Vec<Vertex> = positions
            .iter()
            .zip(normals.iter())
            .map(|(p, n)| Vertex {
                position: *p,
                normal: *n,
            })
            .collect();

        self.chunks.insert(coord, ChunkCpuMesh {
            vertices,
            indices: indices.to_vec(),
        });
        self.dirty = true;
    }

    /// Rebuild the merged GPU buffer if any chunks have changed.
    ///
    /// Fast path (common during cutting): reuses existing GPU buffer via `write_buffer`.
    /// Slow path (startup / chunk count grows): reallocates with 2x capacity.
    pub fn rebuild_if_dirty(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        if !self.dirty {
            return;
        }
        self.dirty = false;

        if self.chunks.is_empty() {
            self.merged_num_indices = 0;
            return;
        }

        // Concatenate all chunks into persistent CPU-side arrays.
        self.merged_verts.clear();
        self.merged_indices.clear();

        for chunk in self.chunks.values() {
            let base = self.merged_verts.len() as u32;
            self.merged_verts.extend_from_slice(&chunk.vertices);
            self.merged_indices.extend(chunk.indices.iter().map(|&i| i + base));
        }

        let vb_data = bytemuck::cast_slice::<Vertex, u8>(&self.merged_verts);
        let ib_data = bytemuck::cast_slice::<u32, u8>(&self.merged_indices);
        let vb_needed = vb_data.len() as u64;
        let ib_needed = ib_data.len() as u64;

        // Fast path: existing buffers have enough capacity → just overwrite.
        let vb_fits = self.merged_vb.is_some() && self.vb_capacity >= vb_needed;
        let ib_fits = self.merged_ib.is_some() && self.ib_capacity >= ib_needed;

        if vb_fits && ib_fits {
            queue.write_buffer(self.merged_vb.as_ref().unwrap(), 0, vb_data);
            queue.write_buffer(self.merged_ib.as_ref().unwrap(), 0, ib_data);
        } else {
            // Slow path: reallocate with 2x growth factor.
            if !vb_fits {
                let new_cap = (vb_needed * 2).max(4096);
                self.merged_vb = Some(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("Merged Workpiece VB"),
                    size: new_cap,
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }));
                self.vb_capacity = new_cap;
            }
            if !ib_fits {
                let new_cap = (ib_needed * 2).max(4096);
                self.merged_ib = Some(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("Merged Workpiece IB"),
                    size: new_cap,
                    usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }));
                self.ib_capacity = new_cap;
            }
            queue.write_buffer(self.merged_vb.as_ref().unwrap(), 0, vb_data);
            queue.write_buffer(self.merged_ib.as_ref().unwrap(), 0, ib_data);
        }

        self.merged_num_indices = self.merged_indices.len() as u32;
    }

    /// Access the merged vertex buffer for drawing.
    pub fn merged_vertex_buffer(&self) -> Option<&wgpu::Buffer> {
        self.merged_vb.as_ref()
    }

    /// Access the merged index buffer for drawing.
    pub fn merged_index_buffer(&self) -> Option<&wgpu::Buffer> {
        self.merged_ib.as_ref()
    }

    /// Number of indices in the merged buffer.
    pub fn merged_num_indices(&self) -> u32 {
        self.merged_num_indices
    }

    /// Remove a chunk mesh.
    pub fn remove_chunk(&mut self, coord: (i32, i32, i32)) {
        if self.chunks.remove(&coord).is_some() {
            self.dirty = true;
        }
    }

    /// Number of chunks with mesh data.
    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    /// Total number of triangles across all chunks.
    pub fn total_triangles(&self) -> u32 {
        self.chunks.values().map(|c| c.indices.len() as u32 / 3).sum()
    }
}

impl Default for ChunkMeshManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Vertex buffer layout for Vertex struct.
pub fn vertex_buffer_layout() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            // position
            wgpu::VertexAttribute {
                offset: 0,
                shader_location: 0,
                format: wgpu::VertexFormat::Float32x3,
            },
            // normal
            wgpu::VertexAttribute {
                offset: 12,
                shader_location: 1,
                format: wgpu::VertexFormat::Float32x3,
            },
        ],
    }
}
