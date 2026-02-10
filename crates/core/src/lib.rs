//! SimuForge-Stone core types shared across crates.
//!
//! Provides coordinate conversion between nalgebra (physics) and glam (render),
//! common transform types, and shared constants.

use nalgebra as na;

// Re-export key types so downstream crates don't repeat use-declarations
pub use na::{Isometry3, Matrix4, Point3, Rotation3, UnitQuaternion, Vector3};

/// Gravitational acceleration (m/s²), pointing -Z in DH base frame (Z-up convention).
/// The renderer uses Y-up, so a coordinate swap is needed for display.
pub const GRAVITY: Vector3<f64> = Vector3::new(0.0, 0.0, -9.80665);

/// Convert nalgebra Isometry3<f64> → glam Mat4 (for GPU upload).
pub fn isometry_to_glam(iso: &Isometry3<f64>) -> glam::Mat4 {
    let m = iso.to_homogeneous();
    glam::Mat4::from_cols_array(&[
        m[(0, 0)] as f32,
        m[(1, 0)] as f32,
        m[(2, 0)] as f32,
        m[(3, 0)] as f32,
        m[(0, 1)] as f32,
        m[(1, 1)] as f32,
        m[(2, 1)] as f32,
        m[(3, 1)] as f32,
        m[(0, 2)] as f32,
        m[(1, 2)] as f32,
        m[(2, 2)] as f32,
        m[(3, 2)] as f32,
        m[(0, 3)] as f32,
        m[(1, 3)] as f32,
        m[(2, 3)] as f32,
        m[(3, 3)] as f32,
    ])
}

/// Convert nalgebra Matrix4<f64> → glam Mat4.
pub fn mat4_to_glam(m: &Matrix4<f64>) -> glam::Mat4 {
    glam::Mat4::from_cols_array(&[
        m[(0, 0)] as f32,
        m[(1, 0)] as f32,
        m[(2, 0)] as f32,
        m[(3, 0)] as f32,
        m[(0, 1)] as f32,
        m[(1, 1)] as f32,
        m[(2, 1)] as f32,
        m[(3, 1)] as f32,
        m[(0, 2)] as f32,
        m[(1, 2)] as f32,
        m[(2, 2)] as f32,
        m[(3, 2)] as f32,
        m[(0, 3)] as f32,
        m[(1, 3)] as f32,
        m[(2, 3)] as f32,
        m[(3, 3)] as f32,
    ])
}

/// Convert nalgebra Point3<f64> → glam Vec3.
pub fn point_to_glam(p: &Point3<f64>) -> glam::Vec3 {
    glam::Vec3::new(p.x as f32, p.y as f32, p.z as f32)
}

/// Convert nalgebra Vector3<f64> → glam Vec3.
pub fn vec3_to_glam(v: &Vector3<f64>) -> glam::Vec3 {
    glam::Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Convert glam Vec3 → nalgebra Vector3<f64>.
pub fn glam_to_vec3(v: glam::Vec3) -> Vector3<f64> {
    Vector3::new(v.x as f64, v.y as f64, v.z as f64)
}

/// Convert glam Vec3 → nalgebra Point3<f64>.
pub fn glam_to_point(v: glam::Vec3) -> Point3<f64> {
    Point3::new(v.x as f64, v.y as f64, v.z as f64)
}

/// DH parameter set for a single link.
#[derive(Debug, Clone, Copy)]
pub struct DhParams {
    /// Link length (m) — distance along x_{i} from z_{i-1} to z_{i}.
    pub a: f64,
    /// Link offset (m) — distance along z_{i-1} from x_{i-1} to x_{i}.
    pub d: f64,
    /// Link twist (rad) — angle about x_{i} from z_{i-1} to z_{i}.
    pub alpha: f64,
    /// Joint angle offset (rad) — added to variable θ for revolute joints.
    pub theta_offset: f64,
}

impl DhParams {
    pub fn new(a: f64, d: f64, alpha: f64) -> Self {
        Self {
            a,
            d,
            alpha,
            theta_offset: 0.0,
        }
    }

    /// Compute the homogeneous transform for this link at joint angle θ.
    pub fn transform(&self, theta: f64) -> Isometry3<f64> {
        let t = theta + self.theta_offset;
        let (st, ct) = t.sin_cos();
        let (sa, ca) = self.alpha.sin_cos();

        // Standard DH transformation matrix
        let rot = na::Matrix3::new(
            ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0.0, sa, ca,
        );
        let translation = na::Translation3::new(self.a * ct, self.a * st, self.d);
        let rotation = Rotation3::from_matrix_unchecked(rot);

        Isometry3::from_parts(translation, UnitQuaternion::from_rotation_matrix(&rotation))
    }
}

/// GPU-uploadable vertex for mesh rendering.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

/// GPU-uploadable per-instance transform (model matrix).
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct InstanceRaw {
    pub model: [[f32; 4]; 4],
}

impl InstanceRaw {
    pub fn from_glam(mat: glam::Mat4) -> Self {
        Self {
            model: mat.to_cols_array_2d(),
        }
    }
}

/// Axis-aligned bounding box.
#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min: Vector3<f64>,
    pub max: Vector3<f64>,
}

impl Aabb {
    pub fn new(min: Vector3<f64>, max: Vector3<f64>) -> Self {
        Self { min, max }
    }

    pub fn contains(&self, p: &Vector3<f64>) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    pub fn intersects(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    pub fn center(&self) -> Vector3<f64> {
        (self.min + self.max) * 0.5
    }

    pub fn half_extents(&self) -> Vector3<f64> {
        (self.max - self.min) * 0.5
    }

    /// Expand to include a point.
    pub fn expand(&mut self, p: &Vector3<f64>) {
        self.min = self.min.inf(p);
        self.max = self.max.sup(p);
    }

    /// Expand by a margin on all sides.
    pub fn padded(&self, margin: f64) -> Self {
        Self {
            min: self.min - Vector3::repeat(margin),
            max: self.max + Vector3::repeat(margin),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_isometry_roundtrip() {
        let iso = Isometry3::translation(1.0, 2.0, 3.0);
        let g = isometry_to_glam(&iso);
        let col3 = g.col(3);
        assert!((col3.x - 1.0).abs() < 1e-6);
        assert!((col3.y - 2.0).abs() < 1e-6);
        assert!((col3.z - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_dh_identity() {
        let dh = DhParams::new(0.0, 0.0, 0.0);
        let t = dh.transform(0.0);
        let m = t.to_homogeneous();
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (m[(i, j)] - expected).abs() < 1e-10,
                    "m[{},{}] = {} != {}",
                    i,
                    j,
                    m[(i, j)],
                    expected
                );
            }
        }
    }

    #[test]
    fn test_aabb_intersect() {
        let a = Aabb::new(Vector3::zeros(), Vector3::new(1.0, 1.0, 1.0));
        let b = Aabb::new(Vector3::new(0.5, 0.5, 0.5), Vector3::new(1.5, 1.5, 1.5));
        assert!(a.intersects(&b));

        let c = Aabb::new(Vector3::new(2.0, 2.0, 2.0), Vector3::new(3.0, 3.0, 3.0));
        assert!(!a.intersects(&c));
    }
}
