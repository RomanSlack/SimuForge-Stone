//! Dynamic mesh buffer management for chunk-based rendering.

use std::collections::HashMap;
use simuforge_core::Vertex;

/// A GPU mesh for one chunk.
pub struct ChunkGpuMesh {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub num_indices: u32,
}

/// Manages per-chunk GPU meshes, uploading only dirty chunks.
pub struct ChunkMeshManager {
    pub meshes: HashMap<(i32, i32, i32), ChunkGpuMesh>,
}

impl ChunkMeshManager {
    pub fn new() -> Self {
        Self {
            meshes: HashMap::new(),
        }
    }

    /// Upload or update a chunk mesh on the GPU.
    pub fn upload_chunk(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        coord: (i32, i32, i32),
        positions: &[[f32; 3]],
        normals: &[[f32; 3]],
        indices: &[u32],
    ) {
        if indices.is_empty() {
            self.meshes.remove(&coord);
            return;
        }

        // Build vertex data
        let vertices: Vec<Vertex> = positions
            .iter()
            .zip(normals.iter())
            .map(|(p, n)| Vertex {
                position: *p,
                normal: *n,
            })
            .collect();

        let vertex_data = bytemuck::cast_slice(&vertices);
        let index_data = bytemuck::cast_slice(indices);

        // Check if we can reuse existing buffers
        if let Some(existing) = self.meshes.get(&coord) {
            if existing.vertex_buffer.size() >= vertex_data.len() as u64
                && existing.index_buffer.size() >= index_data.len() as u64
            {
                queue.write_buffer(&existing.vertex_buffer, 0, vertex_data);
                queue.write_buffer(&existing.index_buffer, 0, index_data);
                // Update index count
                let mesh = self.meshes.get_mut(&coord).unwrap();
                mesh.num_indices = indices.len() as u32;
                return;
            }
        }

        // Create new buffers
        use wgpu::util::DeviceExt;
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(&format!("Chunk {:?} Vertices", coord)),
            contents: vertex_data,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(&format!("Chunk {:?} Indices", coord)),
            contents: index_data,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
        });

        self.meshes.insert(
            coord,
            ChunkGpuMesh {
                vertex_buffer,
                index_buffer,
                num_indices: indices.len() as u32,
            },
        );
    }

    /// Remove a chunk mesh.
    pub fn remove_chunk(&mut self, coord: (i32, i32, i32)) {
        self.meshes.remove(&coord);
    }

    /// Number of chunks with GPU meshes.
    pub fn chunk_count(&self) -> usize {
        self.meshes.len()
    }

    /// Total number of triangles across all chunks.
    pub fn total_triangles(&self) -> u32 {
        self.meshes.values().map(|m| m.num_indices / 3).sum()
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
