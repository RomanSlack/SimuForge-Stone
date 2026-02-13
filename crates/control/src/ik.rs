//! Damped least-squares inverse kinematics solver.

use nalgebra::{DMatrix, DVector, Isometry3, Vector3, Vector6};
use simuforge_physics::arm::RobotArm;

/// IK solver configuration.
#[derive(Debug, Clone)]
pub struct IkSolver {
    /// Damping factor for singularity robustness.
    pub damping: f64,
    /// Maximum iterations per solve.
    pub max_iterations: u32,
    /// Position error tolerance (m).
    pub position_tolerance: f64,
    /// Orientation error tolerance (rad).
    pub orientation_tolerance: f64,
    /// Maximum joint step per iteration (rad).
    pub max_step: f64,
}

impl Default for IkSolver {
    fn default() -> Self {
        Self {
            damping: 0.05,
            max_iterations: 50,
            position_tolerance: 1e-4,   // 0.1mm
            orientation_tolerance: 1e-3, // ~0.06°
            max_step: 0.1,              // ~5.7°
        }
    }
}

impl IkSolver {
    /// Solve IK for a target pose. Modifies joint angles in-place.
    ///
    /// Returns (converged, iterations, final_error).
    pub fn solve(
        &self,
        arm: &mut RobotArm,
        target: &Isometry3<f64>,
    ) -> (bool, u32, f64) {
        for iter in 0..self.max_iterations {
            let current = arm.forward_kinematics();
            let error = pose_error(&current, target);
            let err_norm = error.norm();

            let pos_err = Vector3::new(error[3], error[4], error[5]).norm();
            let rot_err = Vector3::new(error[0], error[1], error[2]).norm();

            if pos_err < self.position_tolerance && rot_err < self.orientation_tolerance {
                return (true, iter, err_norm);
            }

            let jac = arm.jacobian();
            let dq = self.damped_least_squares(&jac, &error);

            // Apply joint angle update with step limiting
            for (i, joint) in arm.joints.iter_mut().enumerate() {
                let step = dq[i].clamp(-self.max_step, self.max_step);
                joint.angle += step;
                // Respect joint limits
                joint.angle = joint.angle.clamp(joint.angle_min, joint.angle_max);
            }
        }

        let final_error = pose_error(&arm.forward_kinematics(), target).norm();
        (false, self.max_iterations, final_error)
    }

    /// Solve IK for position only (3-DOF). Ignores orientation.
    pub fn solve_position(
        &self,
        arm: &mut RobotArm,
        target_pos: &Vector3<f64>,
    ) -> (bool, u32, f64) {
        for iter in 0..self.max_iterations {
            let current_pos = arm.tool_position();
            let pos_error = target_pos - current_pos;
            let err_norm = pos_error.norm();

            if err_norm < self.position_tolerance {
                return (true, iter, err_norm);
            }

            let jac_full = arm.jacobian();
            // Use only the linear (position) rows of the Jacobian
            let jac_pos = jac_full.rows(3, 3).clone_owned();
            let error_vec = DVector::from_column_slice(pos_error.as_slice());

            let dq = self.damped_least_squares_rect(&jac_pos, &error_vec);

            for (i, joint) in arm.joints.iter_mut().enumerate() {
                let step = dq[i].clamp(-self.max_step, self.max_step);
                joint.angle += step;
                joint.angle = joint.angle.clamp(joint.angle_min, joint.angle_max);
            }
        }

        let final_error = (target_pos - arm.tool_position()).norm();
        (false, self.max_iterations, final_error)
    }

    /// Solve IK with weighted orientation bias.
    /// `orientation_weight` of 0.0 = position-only, 1.0 = full 6DOF.
    /// Returns (converged, iterations, position_error).
    pub fn solve_weighted(
        &self,
        arm: &mut RobotArm,
        target: &Isometry3<f64>,
        orientation_weight: f64,
    ) -> (bool, u32, f64) {
        let w = orientation_weight.clamp(0.0, 1.0);

        for iter in 0..self.max_iterations {
            let current = arm.forward_kinematics();
            let error = pose_error(&current, target);

            let pos_err = Vector3::new(error[3], error[4], error[5]).norm();
            let rot_err = Vector3::new(error[0], error[1], error[2]).norm();

            // Convergence: position uses standard tolerance; orientation relaxed
            let ori_tol = if w > 1e-9 {
                self.orientation_tolerance / w
            } else {
                f64::MAX // no orientation constraint
            };
            if pos_err < self.position_tolerance && rot_err < ori_tol {
                return (true, iter, pos_err);
            }

            let jac = arm.jacobian();

            // Weight the orientation rows (0..3) of both Jacobian and error
            let mut weighted_jac = jac.clone();
            let mut weighted_error = DVector::from_column_slice(error.as_slice());
            for row in 0..3 {
                for col in 0..weighted_jac.ncols() {
                    weighted_jac[(row, col)] *= w;
                }
                weighted_error[row] *= w;
            }

            let dq = self.damped_least_squares_rect(&weighted_jac, &weighted_error);

            for (i, joint) in arm.joints.iter_mut().enumerate() {
                let step = dq[i].clamp(-self.max_step, self.max_step);
                joint.angle += step;
                joint.angle = joint.angle.clamp(joint.angle_min, joint.angle_max);
            }
        }

        let final_pos_err = (target.translation.vector - arm.tool_position()).norm();
        (false, self.max_iterations, final_pos_err)
    }

    /// Damped least-squares pseudoinverse: dq = J^T (J J^T + λ²I)^{-1} e
    fn damped_least_squares(
        &self,
        jac: &DMatrix<f64>,
        error: &Vector6<f64>,
    ) -> DVector<f64> {
        let e = DVector::from_column_slice(error.as_slice());
        self.damped_least_squares_rect(jac, &e)
    }

    /// Generic rectangular DLS.
    fn damped_least_squares_rect(
        &self,
        jac: &DMatrix<f64>,
        error: &DVector<f64>,
    ) -> DVector<f64> {
        let jjt = jac * jac.transpose();
        let m = jjt.nrows();
        let damping_matrix = DMatrix::identity(m, m) * (self.damping * self.damping);
        let to_invert = jjt + damping_matrix;

        match to_invert.try_inverse() {
            Some(inv) => jac.transpose() * inv * error,
            None => DVector::zeros(jac.ncols()), // Singular — return zero
        }
    }
}

/// Compute pose error as a 6-vector [rotation_error; position_error].
fn pose_error(current: &Isometry3<f64>, target: &Isometry3<f64>) -> Vector6<f64> {
    // Position error
    let pos_err = target.translation.vector - current.translation.vector;

    // Orientation error (axis-angle representation)
    let rot_err_quat = target.rotation * current.rotation.inverse();
    let (axis, angle) = match rot_err_quat.axis_angle() {
        Some((axis, angle)) => (axis.into_inner(), angle),
        None => (Vector3::z(), 0.0),
    };
    let rot_err = axis * angle;

    Vector6::new(rot_err.x, rot_err.y, rot_err.z, pos_err.x, pos_err.y, pos_err.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ik_reaches_target() {
        let mut arm = RobotArm::default_6dof();
        let solver = IkSolver::default();

        // Set a reachable target (near home position)
        let target_pos = Vector3::new(0.5, 0.3, 0.3);
        let (converged, iters, error) = solver.solve_position(&mut arm, &target_pos);

        assert!(
            error < 0.01,
            "IK error too large: {} (converged={}, iters={})",
            error,
            converged,
            iters
        );
    }

    #[test]
    fn test_ik_weighted_orientation_bias() {
        use nalgebra::{Isometry3, Translation3, UnitQuaternion};

        let solver = IkSolver {
            max_iterations: 200,
            position_tolerance: 1e-3,
            max_step: 0.2,
            ..IkSolver::default()
        };

        // Tool-down target: Rx(π) means tool Z points down (-Z in DH)
        let target_pos = Vector3::new(0.5, 0.0, 0.3);
        let tool_down = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI);
        let target = Isometry3::from_parts(Translation3::from(target_pos), tool_down);

        // First solve position-only to get close, then refine with orientation
        let mut arm = RobotArm::default_6dof();
        let seed = [0.0, 0.8, 0.3, 0.0, std::f64::consts::FRAC_PI_2, 0.0];
        arm.set_joint_angles(&seed.to_vec());

        // Position-only first to get into workspace
        let pos_solver = IkSolver { max_iterations: 50, ..solver.clone() };
        pos_solver.solve_position(&mut arm, &target_pos);

        // Then refine with orientation weight
        let (_, _, pos_err) = solver.solve_weighted(&mut arm, &target, 0.2);

        assert!(
            pos_err < 0.01,
            "Weighted IK position error too large: {:.4}m",
            pos_err
        );

        // Check tool Z has a significant downward component (negative Z in DH)
        let fk = arm.forward_kinematics();
        let tool_z = fk.rotation * Vector3::z();
        assert!(
            tool_z.z < 0.0,
            "Tool Z should have downward component, got ({:.2},{:.2},{:.2})",
            tool_z.x, tool_z.y, tool_z.z
        );
    }
}
