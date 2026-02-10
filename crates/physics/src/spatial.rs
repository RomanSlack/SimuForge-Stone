//! Spatial vector algebra for Featherstone's Articulated Body Algorithm.
//!
//! Spatial vectors combine angular and linear components into 6D vectors.
//! Convention: [angular (3); linear (3)] (Featherstone "Motion" convention).

use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};
use simuforge_core::DhParams;

/// Spatial velocity / acceleration / force vector.
pub type SpatialVector = Vector6<f64>;

/// 6×6 spatial inertia or spatial transform matrix.
pub type SpatialMatrix = Matrix6<f64>;

/// Extract angular part (top 3) of a spatial vector.
pub fn angular(v: &SpatialVector) -> Vector3<f64> {
    Vector3::new(v[0], v[1], v[2])
}

/// Extract linear part (bottom 3) of a spatial vector.
pub fn linear(v: &SpatialVector) -> Vector3<f64> {
    Vector3::new(v[3], v[4], v[5])
}

/// Construct a spatial vector from angular and linear parts.
pub fn spatial_vec(ang: &Vector3<f64>, lin: &Vector3<f64>) -> SpatialVector {
    SpatialVector::new(ang.x, ang.y, ang.z, lin.x, lin.y, lin.z)
}

/// Skew-symmetric (cross-product) matrix for vector v.
pub fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Spatial cross product for motion vectors: v ×_m w
/// [ω×  0 ] [ω_w]
/// [v×  ω×] [v_w]
pub fn spatial_cross_motion(v: &SpatialVector, w: &SpatialVector) -> SpatialVector {
    let omega = angular(v);
    let vel = linear(v);
    let omega_w = angular(w);
    let vel_w = linear(w);

    let ang = omega.cross(&omega_w);
    let lin = omega.cross(&vel_w) + vel.cross(&omega_w);
    spatial_vec(&ang, &lin)
}

/// Spatial cross product for force vectors: v ×_f f = -(v ×_m)^T f
pub fn spatial_cross_force(v: &SpatialVector, f: &SpatialVector) -> SpatialVector {
    let omega = angular(v);
    let vel = linear(v);
    let tau = angular(f);
    let force = linear(f);

    let ang = omega.cross(&tau) + vel.cross(&force);
    let lin = omega.cross(&force);
    spatial_vec(&ang, &lin)
}

/// Build spatial inertia matrix from mass, center of mass (in body frame), and
/// rotational inertia about the center of mass.
///
/// I_sp = [ I_rot + m*cx*cx^T   m*cx ]
///        [ m*cx^T               m*I  ]
///
/// where cx = skew(com)
pub fn spatial_inertia(mass: f64, com: &Vector3<f64>, inertia: &Matrix3<f64>) -> SpatialMatrix {
    let cx = skew(com);
    let m_i3 = Matrix3::identity() * mass;
    let m_cx = cx * mass;

    let top_left = inertia + m_cx * cx.transpose();
    let top_right = m_cx;
    let bot_left = m_cx.transpose();
    let bot_right = m_i3;

    let mut result = SpatialMatrix::zeros();
    result
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&top_left);
    result
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&top_right);
    result
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&bot_left);
    result
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&bot_right);
    result
}

/// Spatial transform from DH parameters at a given joint angle.
/// Returns (X_J, S) where X_J is the 6x6 spatial transform and S is the
/// joint motion subspace vector (for revolute: [0,0,1,0,0,0]).
///
/// The spatial transform maps motion vectors from child frame to parent frame.
pub fn spatial_transform_from_dh(dh: &DhParams, theta: f64) -> SpatialMatrix {
    let t = theta + dh.theta_offset;
    let (st, ct) = t.sin_cos();
    let (sa, ca) = dh.alpha.sin_cos();

    // Rotation matrix from DH convention (child→parent)
    let r = Matrix3::new(ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0.0, sa, ca);

    // Translation in parent frame
    let p = Vector3::new(dh.a * ct, dh.a * st, dh.d);

    // Spatial transform: Xj = [R, 0; -R*px, R] where px = skew(p)
    // This transforms spatial motion vectors from child to parent.
    let r_t = r.transpose();
    let px = skew(&p);

    let mut x = SpatialMatrix::zeros();
    x.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_t);
    let bot_left = -r_t * px;
    x.fixed_view_mut::<3, 3>(3, 0).copy_from(&bot_left);
    x.fixed_view_mut::<3, 3>(3, 3).copy_from(&r_t);
    x
}

/// Joint motion subspace for revolute joint about local Z axis.
pub fn revolute_motion_subspace() -> SpatialVector {
    SpatialVector::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_skew_cross_product() {
        let a = Vector3::new(1.0, 0.0, 0.0);
        let b = Vector3::new(0.0, 1.0, 0.0);
        let result = skew(&a) * b;
        assert!((result - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-10);
    }

    #[test]
    fn test_spatial_inertia_symmetric() {
        let mass = 5.0;
        let com = Vector3::new(0.1, 0.0, 0.0);
        let inertia = Matrix3::identity() * 0.01;
        let si = spatial_inertia(mass, &com, &inertia);
        // Spatial inertia must be symmetric
        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    (si[(i, j)] - si[(j, i)]).abs() < 1e-10,
                    "Asymmetric at ({},{}): {} vs {}",
                    i,
                    j,
                    si[(i, j)],
                    si[(j, i)]
                );
            }
        }
    }
}
