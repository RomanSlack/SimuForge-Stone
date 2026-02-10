//! SDF operations for material removal: swept volumes, CSG.

/// Signed distance to a sphere at origin.
#[inline]
pub fn sdf_sphere(px: f32, py: f32, pz: f32, radius: f32) -> f32 {
    (px * px + py * py + pz * pz).sqrt() - radius
}

/// Signed distance to a capsule between two points.
/// The capsule is a swept sphere of given radius along the line segment.
pub fn sdf_capsule(
    px: f32,
    py: f32,
    pz: f32,
    ax: f32,
    ay: f32,
    az: f32,
    bx: f32,
    by: f32,
    bz: f32,
    radius: f32,
) -> f32 {
    let pax = px - ax;
    let pay = py - ay;
    let paz = pz - az;
    let bax = bx - ax;
    let bay = by - ay;
    let baz = bz - az;

    let ba_dot = bax * bax + bay * bay + baz * baz;
    let t = if ba_dot > 1e-12 {
        ((pax * bax + pay * bay + paz * baz) / ba_dot).clamp(0.0, 1.0)
    } else {
        0.0
    };

    let cx = ax + t * bax - px;
    let cy = ay + t * bay - py;
    let cz = az + t * baz - pz;

    (cx * cx + cy * cy + cz * cz).sqrt() - radius
}

/// Signed distance to an infinite cylinder along the Z axis.
#[inline]
pub fn sdf_cylinder_z(px: f32, py: f32, radius: f32) -> f32 {
    (px * px + py * py).sqrt() - radius
}

/// Signed distance to a box centered at origin with given half-extents.
pub fn sdf_box(px: f32, py: f32, pz: f32, hx: f32, hy: f32, hz: f32) -> f32 {
    let dx = px.abs() - hx;
    let dy = py.abs() - hy;
    let dz = pz.abs() - hz;
    let outside = (dx.max(0.0).powi(2) + dy.max(0.0).powi(2) + dz.max(0.0).powi(2)).sqrt();
    let inside = dx.max(dy).max(dz).min(0.0);
    outside + inside
}

/// CSG union: min(a, b)
#[inline]
pub fn sdf_union(a: f32, b: f32) -> f32 {
    a.min(b)
}

/// CSG intersection: max(a, b)
#[inline]
pub fn sdf_intersection(a: f32, b: f32) -> f32 {
    a.max(b)
}

/// CSG subtraction: max(a, -b) — removes b from a.
#[inline]
pub fn sdf_subtract(a: f32, b: f32) -> f32 {
    a.max(-b)
}

/// Smooth union with blending radius k.
#[inline]
pub fn sdf_smooth_union(a: f32, b: f32, k: f32) -> f32 {
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    b + (a - b) * h - k * h * (1.0 - h)
}

/// Compute the bounding box of a swept capsule (for octree intersection).
pub fn swept_capsule_bounds(
    ax: f32,
    ay: f32,
    az: f32,
    bx: f32,
    by: f32,
    bz: f32,
    radius: f32,
) -> ([f32; 3], [f32; 3]) {
    let min_x = ax.min(bx) - radius;
    let min_y = ay.min(by) - radius;
    let min_z = az.min(bz) - radius;
    let max_x = ax.max(bx) + radius;
    let max_y = ay.max(by) + radius;
    let max_z = az.max(bz) + radius;
    ([min_x, min_y, min_z], [max_x, max_y, max_z])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sphere_sdf() {
        assert!((sdf_sphere(0.0, 0.0, 0.0, 1.0) - (-1.0)).abs() < 1e-6);
        assert!((sdf_sphere(1.0, 0.0, 0.0, 1.0) - 0.0).abs() < 1e-6);
        assert!((sdf_sphere(2.0, 0.0, 0.0, 1.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_capsule_sdf() {
        // Point on the capsule surface at midpoint
        let d = sdf_capsule(1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0);
        assert!(d.abs() < 1e-5, "On surface: {}", d);

        // Point inside
        let d = sdf_capsule(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0);
        assert!(d < 0.0, "Should be inside: {}", d);
    }

    #[test]
    fn test_csg_subtract() {
        // A big sphere with a small sphere removed
        let a = sdf_sphere(0.0, 0.0, 0.0, 2.0); // inside big sphere
        let b = sdf_sphere(0.0, 0.0, 0.0, 1.0); // inside small sphere
        let result = sdf_subtract(a, b);
        // After removing small sphere from big sphere, center should be outside
        assert!(result > 0.0, "Center should be outside: {}", result);
    }
}
