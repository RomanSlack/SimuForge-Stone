//! Minimal binary STL parser. No external dependencies.

use nalgebra::Vector3;

/// A triangle mesh loaded from an STL file.
pub struct StlMesh {
    /// Triangle vertices (3 per triangle).
    pub vertices: Vec<Vector3<f32>>,
    /// Triangle normals (1 per triangle, repeated 3x for flat shading).
    pub normals: Vec<Vector3<f32>>,
    /// Bounding box min.
    pub bb_min: Vector3<f32>,
    /// Bounding box max.
    pub bb_max: Vector3<f32>,
}

impl StlMesh {
    /// Load a binary STL file.
    pub fn load(path: &str) -> Result<Self, String> {
        let data = std::fs::read(path)
            .map_err(|e| format!("Failed to read STL file '{}': {}", path, e))?;

        if data.len() < 84 {
            return Err("STL file too small for header + triangle count".to_string());
        }

        // Skip 80-byte header
        let num_triangles = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;

        let expected_size = 84 + num_triangles * 50; // 50 bytes per triangle
        if data.len() < expected_size {
            return Err(format!(
                "STL file truncated: expected {} bytes for {} triangles, got {}",
                expected_size, num_triangles, data.len()
            ));
        }

        let mut vertices = Vec::with_capacity(num_triangles * 3);
        let mut normals = Vec::with_capacity(num_triangles * 3);
        let mut bb_min = Vector3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut bb_max = Vector3::new(f32::MIN, f32::MIN, f32::MIN);

        let mut offset = 84;
        for _ in 0..num_triangles {
            let n = read_vec3(&data, offset);
            offset += 12;

            for _ in 0..3 {
                let v = read_vec3(&data, offset);
                offset += 12;

                bb_min.x = bb_min.x.min(v.x);
                bb_min.y = bb_min.y.min(v.y);
                bb_min.z = bb_min.z.min(v.z);
                bb_max.x = bb_max.x.max(v.x);
                bb_max.y = bb_max.y.max(v.y);
                bb_max.z = bb_max.z.max(v.z);

                vertices.push(v);
                normals.push(n);
            }

            offset += 2; // attribute byte count
        }

        Ok(Self { vertices, normals, bb_min, bb_max })
    }

    /// Center the mesh at origin and scale to fit within `target_extent` (half-size).
    pub fn center_and_scale(&mut self, target_extent: f32) {
        let center = (self.bb_min + self.bb_max) * 0.5;
        let size = self.bb_max - self.bb_min;
        let max_dim = size.x.max(size.y).max(size.z);
        let scale = if max_dim > 1e-9 { 2.0 * target_extent / max_dim } else { 1.0 };

        for v in &mut self.vertices {
            *v = (*v - center) * scale;
        }
        self.bb_min = (self.bb_min - center) * scale;
        self.bb_max = (self.bb_max - center) * scale;
    }

    /// Get flat vertex positions as [f32; 3] arrays (for GPU upload).
    pub fn positions(&self) -> Vec<[f32; 3]> {
        self.vertices.iter().map(|v| [v.x, v.y, v.z]).collect()
    }

    /// Get flat normals as [f32; 3] arrays.
    pub fn normals_flat(&self) -> Vec<[f32; 3]> {
        self.normals.iter().map(|n| [n.x, n.y, n.z]).collect()
    }

    /// Get triangle indices (trivial: 0, 1, 2, 3, 4, 5, ...).
    pub fn indices(&self) -> Vec<u32> {
        (0..self.vertices.len() as u32).collect()
    }

    /// Number of triangles.
    pub fn num_triangles(&self) -> usize {
        self.vertices.len() / 3
    }
}

fn read_f32(data: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes([data[offset], data[offset + 1], data[offset + 2], data[offset + 3]])
}

fn read_vec3(data: &[u8], offset: usize) -> Vector3<f32> {
    Vector3::new(
        read_f32(data, offset),
        read_f32(data, offset + 4),
        read_f32(data, offset + 8),
    )
}
