//! Robot arm visual rendering using geometric primitives.

use simuforge_core::Vertex;

/// Generate a cylinder mesh (vertices + indices) along the Y axis.
/// Center at origin, height along Y from -h/2 to +h/2.
pub fn generate_cylinder(radius: f32, height: f32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let half_h = height / 2.0;

    // Side vertices
    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
        let (sin_a, cos_a) = angle.sin_cos();
        let nx = cos_a;
        let nz = sin_a;

        // Bottom vertex
        vertices.push(Vertex {
            position: [radius * cos_a, -half_h, radius * sin_a],
            normal: [nx, 0.0, nz],
        });
        // Top vertex
        vertices.push(Vertex {
            position: [radius * cos_a, half_h, radius * sin_a],
            normal: [nx, 0.0, nz],
        });
    }

    // Side indices
    for i in 0..segments {
        let base = i * 2;
        indices.push(base);
        indices.push(base + 1);
        indices.push(base + 2);
        indices.push(base + 1);
        indices.push(base + 3);
        indices.push(base + 2);
    }

    // Top cap center
    let top_center = vertices.len() as u32;
    vertices.push(Vertex {
        position: [0.0, half_h, 0.0],
        normal: [0.0, 1.0, 0.0],
    });

    // Top cap ring
    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
        let (sin_a, cos_a) = angle.sin_cos();
        vertices.push(Vertex {
            position: [radius * cos_a, half_h, radius * sin_a],
            normal: [0.0, 1.0, 0.0],
        });
    }
    for i in 0..segments {
        indices.push(top_center);
        indices.push(top_center + 1 + i);
        indices.push(top_center + 2 + i);
    }

    // Bottom cap center
    let bot_center = vertices.len() as u32;
    vertices.push(Vertex {
        position: [0.0, -half_h, 0.0],
        normal: [0.0, -1.0, 0.0],
    });

    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
        let (sin_a, cos_a) = angle.sin_cos();
        vertices.push(Vertex {
            position: [radius * cos_a, -half_h, radius * sin_a],
            normal: [0.0, -1.0, 0.0],
        });
    }
    for i in 0..segments {
        indices.push(bot_center);
        indices.push(bot_center + 2 + i);
        indices.push(bot_center + 1 + i);
    }

    (vertices, indices)
}

/// Generate a box mesh with given half-extents, centered at origin.
pub fn generate_box(hx: f32, hy: f32, hz: f32) -> (Vec<Vertex>, Vec<u32>) {
    let positions = [
        // Front face (+Z)
        ([-hx, -hy, hz], [0.0, 0.0, 1.0]),
        ([hx, -hy, hz], [0.0, 0.0, 1.0]),
        ([hx, hy, hz], [0.0, 0.0, 1.0]),
        ([-hx, hy, hz], [0.0, 0.0, 1.0]),
        // Back face (-Z)
        ([hx, -hy, -hz], [0.0, 0.0, -1.0]),
        ([-hx, -hy, -hz], [0.0, 0.0, -1.0]),
        ([-hx, hy, -hz], [0.0, 0.0, -1.0]),
        ([hx, hy, -hz], [0.0, 0.0, -1.0]),
        // Top face (+Y)
        ([-hx, hy, hz], [0.0, 1.0, 0.0]),
        ([hx, hy, hz], [0.0, 1.0, 0.0]),
        ([hx, hy, -hz], [0.0, 1.0, 0.0]),
        ([-hx, hy, -hz], [0.0, 1.0, 0.0]),
        // Bottom face (-Y)
        ([-hx, -hy, -hz], [0.0, -1.0, 0.0]),
        ([hx, -hy, -hz], [0.0, -1.0, 0.0]),
        ([hx, -hy, hz], [0.0, -1.0, 0.0]),
        ([-hx, -hy, hz], [0.0, -1.0, 0.0]),
        // Right face (+X)
        ([hx, -hy, hz], [1.0, 0.0, 0.0]),
        ([hx, -hy, -hz], [1.0, 0.0, 0.0]),
        ([hx, hy, -hz], [1.0, 0.0, 0.0]),
        ([hx, hy, hz], [1.0, 0.0, 0.0]),
        // Left face (-X)
        ([-hx, -hy, -hz], [-1.0, 0.0, 0.0]),
        ([-hx, -hy, hz], [-1.0, 0.0, 0.0]),
        ([-hx, hy, hz], [-1.0, 0.0, 0.0]),
        ([-hx, hy, -hz], [-1.0, 0.0, 0.0]),
    ];

    let vertices: Vec<Vertex> = positions
        .iter()
        .map(|(p, n)| Vertex {
            position: *p,
            normal: *n,
        })
        .collect();

    let mut indices = Vec::new();
    for face in 0..6 {
        let base = face * 4;
        indices.push(base);
        indices.push(base + 1);
        indices.push(base + 2);
        indices.push(base);
        indices.push(base + 2);
        indices.push(base + 3);
    }

    (vertices, indices)
}

/// Link visual description for the robot arm.
pub struct ArmLinkVisual {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    /// Color tint [R, G, B, A].
    pub color: [f32; 4],
}

/// Generate visuals for the default 6-DOF arm.
/// Returns one visual per link (6 total).
pub fn generate_arm_visuals() -> Vec<ArmLinkVisual> {
    let colors = [
        [0.3, 0.3, 0.35, 1.0], // J1: dark gray (base)
        [0.8, 0.4, 0.1, 1.0],  // J2: orange (upper arm)
        [0.8, 0.4, 0.1, 1.0],  // J3: orange (forearm)
        [0.2, 0.2, 0.25, 1.0], // J4: dark (wrist 1)
        [0.2, 0.2, 0.25, 1.0], // J5: dark (wrist 2)
        [0.6, 0.6, 0.65, 1.0], // J6: light gray (flange)
    ];

    let link_params: Vec<(f32, f32, u32)> = vec![
        (0.06, 0.30, 16), // J1: base column
        (0.05, 0.50, 16), // J2: upper arm
        (0.045, 0.40, 16), // J3: forearm
        (0.035, 0.10, 12), // J4: wrist 1
        (0.035, 0.30, 12), // J5: wrist 2
        (0.03, 0.08, 12),  // J6: flange
    ];

    link_params
        .iter()
        .zip(colors.iter())
        .map(|(&(radius, height, segs), &color)| {
            let (vertices, indices) = generate_cylinder(radius, height, segs);
            ArmLinkVisual {
                vertices,
                indices,
                color,
            }
        })
        .collect()
}
