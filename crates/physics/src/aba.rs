//! Featherstone's Articulated Body Algorithm (ABA).
//!
//! Three-pass O(n) algorithm for forward dynamics of an articulated rigid body system.
//! Given joint torques → computes joint accelerations.

use nalgebra::DVector;

use crate::arm::RobotArm;
use crate::spatial::{
    revolute_motion_subspace, spatial_cross_force, spatial_cross_motion, spatial_transform_from_dh,
    spatial_vec, SpatialMatrix, SpatialVector,
};

/// Compute forward dynamics using the Articulated Body Algorithm.
///
/// Given the current arm state and applied joint torques, returns joint accelerations.
///
/// # Arguments
/// * `arm` - The robot arm (provides DH params, link inertias, joint state)
/// * `torques` - Applied torques at each joint (including friction/limits)
/// * `gravity` - Gravity vector in base frame [gx, gy, gz]
///
/// # Returns
/// Joint accelerations as a DVector<f64>
pub fn articulated_body_algorithm(
    arm: &RobotArm,
    torques: &[f64],
    gravity: &nalgebra::Vector3<f64>,
) -> DVector<f64> {
    let n = arm.num_joints();
    assert_eq!(torques.len(), n);

    let s = revolute_motion_subspace(); // Same for all revolute joints

    // ----- Storage -----
    let mut x_j: Vec<SpatialMatrix> = Vec::with_capacity(n); // joint transforms
    let mut v: Vec<SpatialVector> = vec![SpatialVector::zeros(); n]; // link velocities
    let mut c: Vec<SpatialVector> = vec![SpatialVector::zeros(); n]; // bias accelerations
    let mut i_a: Vec<SpatialMatrix> = Vec::with_capacity(n); // articulated inertias
    let mut p_a: Vec<SpatialVector> = vec![SpatialVector::zeros(); n]; // bias forces

    // ----- Pass 1: Outward — compute velocities and bias terms -----
    // Base "acceleration" to account for gravity (virtual acceleration of base)
    // a_0 = -gravity in spatial form: [0; -g]
    let a_grav = spatial_vec(
        &nalgebra::Vector3::zeros(),
        &(-gravity),
    );

    for i in 0..n {
        let dh = &arm.dh_params[i];
        let qi = arm.joints[i].angle;
        let qdi = arm.joints[i].velocity;

        // Joint spatial transform
        let xj = spatial_transform_from_dh(dh, qi);
        x_j.push(xj);

        // Velocity propagation
        let v_parent = if i == 0 {
            SpatialVector::zeros()
        } else {
            v[i - 1]
        };

        let v_j = s * qdi; // joint velocity contribution
        v[i] = xj * v_parent + v_j;

        // Coriolis/centrifugal bias acceleration
        c[i] = spatial_cross_motion(&v[i], &v_j);

        // Initialize articulated inertia with rigid body inertia
        i_a.push(arm.link_spatial_inertias[i]);

        // Bias force: v × I*v (Coriolis in body frame)
        p_a[i] = spatial_cross_force(&v[i], &(arm.link_spatial_inertias[i] * v[i]));
    }

    // ----- Pass 2: Inward — articulated inertias and bias forces -----
    for i in (0..n).rev() {
        let u_i = i_a[i] * s;
        let d_i = s.dot(&u_i);

        if d_i.abs() < 1e-12 {
            continue; // Degenerate — skip
        }

        let u_over_d = u_i / d_i;

        // Articulated bias force contribution from this joint
        let residual = torques[i] - s.dot(&p_a[i]);

        if i > 0 {
            let xj = x_j[i];
            let xj_t = xj.transpose();

            // Project articulated inertia to parent
            let ia_proj = i_a[i] - u_over_d * u_i.transpose();
            i_a[i - 1] += xj_t * ia_proj * xj;

            // Project bias force to parent
            let pa_proj = p_a[i] + ia_proj * c[i] + u_over_d * residual;
            p_a[i - 1] += xj_t * pa_proj;
        }
    }

    // ----- Pass 3: Outward — accelerations -----
    let mut qdd = DVector::zeros(n);
    let mut a: Vec<SpatialVector> = vec![SpatialVector::zeros(); n];

    for i in 0..n {
        let a_parent = if i == 0 { a_grav } else { a[i - 1] };

        let a_prime = x_j[i] * a_parent + c[i];

        let u_i = i_a[i] * s;
        let d_i = s.dot(&u_i);

        if d_i.abs() < 1e-12 {
            qdd[i] = 0.0;
            a[i] = a_prime;
            continue;
        }

        qdd[i] = (torques[i] - s.dot(&(i_a[i] * a_prime + p_a[i]))) / d_i;
        a[i] = a_prime + s * qdd[i];
    }

    qdd
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::RobotArm;

    #[test]
    fn test_aba_applied_torque() {
        let arm = RobotArm::default_6dof();
        let mut torques = vec![0.0; arm.num_joints()];
        torques[0] = 10.0; // Apply 10 Nm to J1
        let gravity = nalgebra::Vector3::zeros();

        let qdd = articulated_body_algorithm(&arm, &torques, &gravity);

        // With a positive torque on J1 and no gravity, J1 should accelerate positively
        assert!(
            qdd[0] > 0.0,
            "J1 should accelerate with applied torque: qdd[0]={}",
            qdd[0]
        );
    }

    #[test]
    fn test_aba_gravity_extended_arm() {
        let arm = RobotArm::default_6dof();
        // DH convention: Z is up. Gravity along -Z.
        let torques = vec![0.0; arm.num_joints()];
        let gravity = nalgebra::Vector3::new(0.0, 0.0, -9.81);

        let qdd = articulated_body_algorithm(&arm, &torques, &gravity);

        // At home position with Z-up gravity, the arm links extend horizontally
        // (after J1's alpha=-pi/2 tilts the Z axis), so gravity creates torques
        // on joints that support the arm weight.
        let max_accel = qdd.iter().map(|a| a.abs()).fold(0.0f64, f64::max);
        assert!(
            max_accel > 0.01,
            "Expected non-zero accelerations under gravity, got max={}. qdd={:?}",
            max_accel,
            qdd.as_slice()
        );
    }

    #[test]
    fn test_aba_zero_gravity() {
        let arm = RobotArm::default_6dof();
        let torques = vec![0.0; arm.num_joints()];
        let gravity = nalgebra::Vector3::zeros();

        let qdd = articulated_body_algorithm(&arm, &torques, &gravity);

        // No gravity, no torques, at rest → no acceleration
        for (i, acc) in qdd.iter().enumerate() {
            assert!(
                acc.abs() < 1e-6,
                "Joint {} has acceleration {} with no forces",
                i,
                acc
            );
        }
    }
}
