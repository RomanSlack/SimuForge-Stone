//! Featherstone's Articulated Body Algorithm (ABA).
//!
//! Three-pass O(n) algorithm for forward dynamics of an articulated rigid body system.
//! Given joint torques → computes joint accelerations.
//!
//! All storage uses fixed-size stack arrays (N=6) to avoid heap allocation
//! in the physics hot loop (~80+ calls per frame during cutting).

use crate::arm::RobotArm;
use crate::spatial::{
    revolute_motion_subspace, spatial_cross_force, spatial_cross_motion, spatial_transform_from_dh,
    spatial_vec, SpatialMatrix, SpatialVector,
};

/// Max joints supported by stack-allocated arrays.
const MAX_JOINTS: usize = 6;

/// Compute exact gravity compensation torques using Recursive Newton-Euler.
///
/// Returns the joint torques needed to hold the arm statically against gravity.
/// These should be applied directly as feedforward torques — the ABA will then
/// produce zero accelerations (no drift).
///
/// This is RNEA with qd=0, qdd=0: only gravity propagation, no Coriolis.
pub fn gravity_compensation_rnea(
    arm: &RobotArm,
    gravity: &nalgebra::Vector3<f64>,
) -> [f64; MAX_JOINTS] {
    let n = arm.num_joints();
    debug_assert!(n <= MAX_JOINTS);
    let s = revolute_motion_subspace();
    let a_grav = spatial_vec(&nalgebra::Vector3::zeros(), &(-gravity));

    // Forward pass: propagate gravity acceleration through the chain
    let zero_sv = SpatialVector::zeros();
    let mut x_j = [SpatialMatrix::zeros(); MAX_JOINTS];
    let mut a = [zero_sv; MAX_JOINTS];

    for i in 0..n {
        let xj = spatial_transform_from_dh(&arm.dh_params[i], arm.joints[i].angle);
        x_j[i] = xj;
        let a_parent = if i == 0 { a_grav } else { a[i - 1] };
        a[i] = xj * a_parent; // static: no joint velocity or acceleration terms
    }

    // Backward pass: compute link forces and project onto joint axes
    let mut f = [zero_sv; MAX_JOINTS];
    let mut torques = [0.0; MAX_JOINTS];

    for i in (0..n).rev() {
        // Force = I * a (no Coriolis since v=0)
        f[i] += arm.link_spatial_inertias[i] * a[i];
        // Joint torque = projection onto joint axis
        torques[i] = s.dot(&f[i]);
        // Propagate to parent
        if i > 0 {
            let propagated = x_j[i].transpose() * f[i];
            f[i - 1] += propagated;
        }
    }

    torques
}

/// Compute forward dynamics using the Articulated Body Algorithm.
///
/// Given the current arm state and applied joint torques, returns joint accelerations.
/// Includes reflected motor inertia from `arm.motor_reflected_inertias` in `d_i`,
/// which is critical for geared systems where motor inertia dominates link inertia.
///
/// # Arguments
/// * `arm` - The robot arm (provides DH params, link inertias, joint state, motor inertias)
/// * `torques` - Applied torques at each joint (including friction/limits)
/// * `gravity` - Gravity vector in base frame [gx, gy, gz]
///
/// # Returns
/// Joint accelerations as `[f64; MAX_JOINTS]`
pub fn articulated_body_algorithm(
    arm: &RobotArm,
    torques: &[f64],
    gravity: &nalgebra::Vector3<f64>,
) -> [f64; MAX_JOINTS] {
    let n = arm.num_joints();
    debug_assert!(n <= MAX_JOINTS);
    assert_eq!(torques.len(), n);

    let s = revolute_motion_subspace(); // Same for all revolute joints

    // ----- Storage (stack-allocated) -----
    let zero_sv = SpatialVector::zeros();
    let zero_sm = SpatialMatrix::zeros();
    let mut x_j = [zero_sm; MAX_JOINTS]; // joint transforms
    let mut v = [zero_sv; MAX_JOINTS]; // link velocities
    let mut c = [zero_sv; MAX_JOINTS]; // bias accelerations
    let mut i_a = [zero_sm; MAX_JOINTS]; // articulated inertias
    let mut p_a = [zero_sv; MAX_JOINTS]; // bias forces

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
        x_j[i] = xj;

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
        i_a[i] = arm.link_spatial_inertias[i];

        // Bias force: v × I*v (Coriolis in body frame)
        p_a[i] = spatial_cross_force(&v[i], &(arm.link_spatial_inertias[i] * v[i]));
    }

    // ----- Pass 2: Inward — articulated inertias and bias forces -----
    for i in (0..n).rev() {
        let u_i = i_a[i] * s;
        // d_i = s^T * I_A * s + I_motor_reflected
        // Reflected motor inertia dominates for geared joints (20 kg·m² vs 0.1 kg·m² link).
        let motor_inertia = if i < arm.motor_reflected_inertias.len() {
            arm.motor_reflected_inertias[i]
        } else {
            0.0
        };
        let d_i = s.dot(&u_i) + motor_inertia;

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
    let mut qdd = [0.0; MAX_JOINTS];
    let mut a = [zero_sv; MAX_JOINTS];

    for i in 0..n {
        let a_parent = if i == 0 { a_grav } else { a[i - 1] };

        let a_prime = x_j[i] * a_parent + c[i];

        let u_i = i_a[i] * s;
        let motor_inertia = if i < arm.motor_reflected_inertias.len() {
            arm.motor_reflected_inertias[i]
        } else {
            0.0
        };
        let d_i = s.dot(&u_i) + motor_inertia;

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
            qdd
        );
    }

    #[test]
    fn test_aba_reflected_inertia_reduces_acceleration() {
        // With reflected motor inertia (20 kg·m² for J1-J3), accelerations should be
        // much smaller than without. This verifies the fix for arm instability.
        let arm = RobotArm::default_6dof();
        let mut torques = vec![0.0; arm.num_joints()];
        torques[0] = 100.0; // 100 Nm on J1
        let gravity = nalgebra::Vector3::zeros();
        let qdd = articulated_body_algorithm(&arm, &torques, &gravity);

        // With reflected inertia of 20 kg·m², acceleration should be ~100/20 = 5 rad/s²
        // Without it, link inertia ~0.12 kg·m² → 100/0.12 = 833 rad/s² (way too high)
        assert!(
            qdd[0] < 10.0,
            "J1 acceleration should be reasonable with reflected motor inertia: qdd[0]={:.1} rad/s² (expected ~5)",
            qdd[0]
        );
        assert!(
            qdd[0] > 1.0,
            "J1 should still accelerate: qdd[0]={}",
            qdd[0]
        );
    }

    #[test]
    fn test_aba_gravity_compensation_stability() {
        // Simulate the arm for 1 second with RNEA gravity compensation + PID.
        // Verifies the arm holds position with <2° max error (no flailing).
        use crate::arm::RobotArm;
        let mut arm = RobotArm::default_6dof();
        let targets = vec![0.0, 0.4, -0.4, 0.0, 0.0, 0.0];
        arm.set_joint_angles(&targets);

        let gravity = nalgebra::Vector3::new(0.0, 0.0, -9.81);
        let dt = 1.0 / 1000.0;
        let n = arm.num_joints();

        // PD controller gains (motor-side, same as pid.rs)
        let kp = [5.0, 5.0, 5.0, 2.0, 2.0, 2.0];
        let kd = [0.8, 0.8, 0.8, 0.4, 0.4, 0.4];
        let gear_ratio = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0];
        let gear_eff = 0.9;
        let hold_torque = [12.0, 12.0, 8.5, 3.0, 3.0, 2.0];

        let mut max_err = 0.0_f64;
        for _step in 0..1000 {
            // RNEA-based gravity compensation (exact, includes coupling)
            let grav_comp = gravity_compensation_rnea(&arm, &gravity);
            let mut taus = [0.0; MAX_JOINTS];
            for i in 0..n {
                let err = targets[i] - arm.joints[i].angle;
                let pd = kp[i] * err - kd[i] * arm.joints[i].velocity;
                // grav_comp[i] is the exact torque needed at the joint to hold against gravity.
                // Convert to motor-side then back through gearbox.
                let ff = grav_comp[i] / (gear_ratio[i] * gear_eff);
                let motor_cmd = (pd + ff).clamp(-hold_torque[i], hold_torque[i]);
                let joint_torque = motor_cmd * gear_ratio[i] * gear_eff;
                arm.joints[i].torque = joint_torque;
                taus[i] = arm.joints[i].net_torque();
            }
            let qdd = articulated_body_algorithm(&arm, &taus[..n], &gravity);

            for i in 0..n {
                arm.joints[i].integrate(qdd[i], dt);
                arm.joints[i].clamp_to_limits();
                max_err = max_err.max((arm.joints[i].angle - targets[i]).abs());
            }
        }

        let max_err_deg = max_err.to_degrees();
        assert!(
            max_err_deg < 2.0,
            "Arm should hold position with <2° max error, got {:.2}°",
            max_err_deg
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
