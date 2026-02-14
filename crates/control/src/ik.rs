//! Damped least-squares inverse kinematics solver with:
//! - Adaptive Nakamura damping (Phase 2)
//! - Null-space joint centering (Phase 3)
//! - Per-joint velocity limiting (Phase 1)

use nalgebra::{DMatrix, DVector, Isometry3, Vector3, Vector6};
use simuforge_physics::arm::RobotArm;

/// Nakamura adaptive damping constants.
const MANIPULABILITY_THRESHOLD: f64 = 0.01; // w0: below this, increase damping
const LAMBDA_MAX: f64 = 0.15;              // Maximum damping near singularities
const LAMBDA_MIN: f64 = 0.001;             // Minimum damping far from singularities

/// IK solver configuration.
#[derive(Debug, Clone)]
pub struct IkSolver {
    /// Damping factor for singularity robustness (used as fallback).
    pub damping: f64,
    /// Maximum iterations per solve.
    pub max_iterations: u32,
    /// Position error tolerance (m).
    pub position_tolerance: f64,
    /// Orientation error tolerance (rad).
    pub orientation_tolerance: f64,
    /// Maximum joint step per iteration (rad).
    pub max_step: f64,
    /// Null-space centering gain (0 = disabled).
    pub nullspace_gain: f64,
}

impl Default for IkSolver {
    fn default() -> Self {
        Self {
            damping: 0.05,
            max_iterations: 50,
            position_tolerance: 1e-4,   // 0.1mm
            orientation_tolerance: 1e-3, // ~0.06°
            max_step: 0.1,              // ~5.7°
            nullspace_gain: 0.5,
        }
    }
}

impl IkSolver {
    /// Solve IK for a target pose (full 6DOF). Modifies joint angles in-place.
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
            let lambda = adaptive_damping(&jac);
            let e = DVector::from_column_slice(error.as_slice());
            let dq = damped_least_squares_rect(&jac, &e, lambda);

            // No null-space for 6DOF with 6 joints (zero redundancy)
            apply_step(arm, &dq, self.max_step);
        }

        let final_error = pose_error(&arm.forward_kinematics(), target).norm();
        (false, self.max_iterations, final_error)
    }

    /// Solve IK for position only (3-DOF). Ignores orientation.
    /// Uses null-space centering to keep wrist joints stable.
    pub fn solve_position(
        &self,
        arm: &mut RobotArm,
        target_pos: &Vector3<f64>,
    ) -> (bool, u32, f64) {
        let joint_centers = arm.joint_centers();

        for iter in 0..self.max_iterations {
            let current_pos = arm.tool_position();
            let pos_error = target_pos - current_pos;
            let err_norm = pos_error.norm();

            if err_norm < self.position_tolerance {
                return (true, iter, err_norm);
            }

            let jac_full = arm.jacobian();
            let jac_pos = jac_full.rows(3, 3).clone_owned();
            let error_vec = DVector::from_column_slice(pos_error.as_slice());

            let lambda = adaptive_damping(&jac_pos);
            let mut dq = damped_least_squares_rect(&jac_pos, &error_vec, lambda);

            // Null-space centering (3×6 Jacobian → 3 redundant DOFs)
            if self.nullspace_gain > 0.0 {
                let dq_null = nullspace_centering(
                    &jac_pos, &dq, arm, &joint_centers, self.nullspace_gain, lambda,
                );
                dq += dq_null;
            }

            apply_step(arm, &dq, self.max_step);
        }

        let final_error = (target_pos - arm.tool_position()).norm();
        (false, self.max_iterations, final_error)
    }

    /// Solve IK with weighted orientation bias.
    /// `orientation_weight` of 0.0 = position-only, 1.0 = full 6DOF.
    /// Uses null-space centering when orientation weight < 1.0 (some redundancy).
    /// Returns (converged, iterations, position_error).
    pub fn solve_weighted(
        &self,
        arm: &mut RobotArm,
        target: &Isometry3<f64>,
        orientation_weight: f64,
    ) -> (bool, u32, f64) {
        let w = orientation_weight.clamp(0.0, 1.0);
        let joint_centers = arm.joint_centers();

        for iter in 0..self.max_iterations {
            let current = arm.forward_kinematics();
            let error = pose_error(&current, target);

            let pos_err = Vector3::new(error[3], error[4], error[5]).norm();
            let rot_err = Vector3::new(error[0], error[1], error[2]).norm();

            // Convergence: position uses standard tolerance; orientation relaxed by weight
            let ori_tol = if w > 1e-9 {
                self.orientation_tolerance / w
            } else {
                f64::MAX
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

            // Use unweighted Jacobian for damping — weighted rows would falsely signal singularity
            let lambda = adaptive_damping(&jac);
            let mut dq = damped_least_squares_rect(&weighted_jac, &weighted_error, lambda);

            // Null-space centering when orientation is weak (effective < 6DOF constraint)
            if self.nullspace_gain > 0.0 && w < 0.9 {
                let dq_null = nullspace_centering(
                    &weighted_jac, &dq, arm, &joint_centers, self.nullspace_gain, lambda,
                );
                dq += dq_null;
            }

            apply_step(arm, &dq, self.max_step);
        }

        let final_pos_err = (target.translation.vector - arm.tool_position()).norm();
        (false, self.max_iterations, final_pos_err)
    }

    /// Scale the entire dq vector proportionally so no joint exceeds its velocity limit.
    /// `dt` is the time interval this step represents (e.g. frame_dt for rate limiting).
    pub fn apply_velocity_limits(dq: &mut [f64], arm: &RobotArm, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        // Find the worst violator: max(|dq_i| / (vel_limit_i * dt))
        let mut max_ratio = 1.0_f64;
        for (i, joint) in arm.joints.iter().enumerate() {
            let limit = joint.velocity_limit * dt;
            if limit > 0.0 {
                let ratio = dq[i].abs() / limit;
                if ratio > max_ratio {
                    max_ratio = ratio;
                }
            }
        }
        // Scale all joints proportionally to preserve direction
        if max_ratio > 1.0 {
            let scale = 1.0 / max_ratio;
            for d in dq.iter_mut() {
                *d *= scale;
            }
        }
    }
}

/// Compute manipulability: w = sqrt(det(J * J^T)).
fn manipulability(jac: &DMatrix<f64>) -> f64 {
    let jjt = jac * jac.transpose();
    let det = jjt.determinant();
    if det > 0.0 { det.sqrt() } else { 0.0 }
}

/// Nakamura's variable damping: high near singularities, low elsewhere.
fn adaptive_damping(jac: &DMatrix<f64>) -> f64 {
    let w = manipulability(jac);
    if w < MANIPULABILITY_THRESHOLD {
        let ratio = w / MANIPULABILITY_THRESHOLD;
        LAMBDA_MAX * (1.0 - ratio * ratio).sqrt()
    } else {
        LAMBDA_MIN
    }
}

/// Damped least-squares: dq = J^T (J J^T + λ²I)^{-1} e
fn damped_least_squares_rect(
    jac: &DMatrix<f64>,
    error: &DVector<f64>,
    lambda: f64,
) -> DVector<f64> {
    let jjt = jac * jac.transpose();
    let m = jjt.nrows();
    let damping_matrix = DMatrix::identity(m, m) * (lambda * lambda);
    let to_invert = jjt + damping_matrix;

    match to_invert.try_inverse() {
        Some(inv) => jac.transpose() * inv * error,
        None => DVector::zeros(jac.ncols()),
    }
}

/// Null-space joint centering: projects a centering gradient into the null space of J.
fn nullspace_centering(
    jac: &DMatrix<f64>,
    dq_task: &DVector<f64>,
    arm: &RobotArm,
    joint_centers: &[f64],
    gain: f64,
    lambda: f64,
) -> DVector<f64> {
    let n = jac.ncols();
    let _ = dq_task; // Not needed — we compute N = I - J^+ J directly

    // Compute pseudoinverse: J^+ = J^T (J J^T + λ²I)^{-1}
    let jjt = jac * jac.transpose();
    let m = jjt.nrows();
    let damping_matrix = DMatrix::identity(m, m) * (lambda * lambda);
    let to_invert = jjt + damping_matrix;

    let j_pinv = match to_invert.try_inverse() {
        Some(inv) => jac.transpose() * inv,
        None => return DVector::zeros(n),
    };

    // Null-space projector: N = I - J^+ J
    let identity = DMatrix::identity(n, n);
    let null_proj = &identity - &j_pinv * jac;

    // Centering gradient: pull each joint toward its center
    let mut q0 = DVector::zeros(n);
    for i in 0..n {
        q0[i] = -gain * (arm.joints[i].angle - joint_centers[i]);
    }

    null_proj * q0
}

/// Apply a joint step with per-step clamping and joint limits.
fn apply_step(arm: &mut RobotArm, dq: &DVector<f64>, max_step: f64) {
    for (i, joint) in arm.joints.iter_mut().enumerate() {
        let step = dq[i].clamp(-max_step, max_step);
        joint.angle += step;
        joint.angle = joint.angle.clamp(joint.angle_min, joint.angle_max);
    }
}

/// Compute pose error as a 6-vector [rotation_error; position_error].
fn pose_error(current: &Isometry3<f64>, target: &Isometry3<f64>) -> Vector6<f64> {
    let pos_err = target.translation.vector - current.translation.vector;
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

        let target_pos = Vector3::new(0.5, 0.3, 0.3);
        let (converged, iters, error) = solver.solve_position(&mut arm, &target_pos);

        assert!(
            error < 0.01,
            "IK error too large: {} (converged={}, iters={})",
            error, converged, iters
        );
    }

    #[test]
    fn test_ik_weighted_orientation_bias() {
        use nalgebra::{Isometry3, Translation3, UnitQuaternion};

        let solver = IkSolver {
            max_iterations: 300,
            position_tolerance: 1e-3,
            max_step: 0.2,
            ..IkSolver::default()
        };

        let target_pos = Vector3::new(0.5, 0.0, 0.3);
        let tool_down = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI);
        let target = Isometry3::from_parts(Translation3::from(target_pos), tool_down);

        let mut arm = RobotArm::default_6dof();
        let seed = [0.0, 0.8, 0.3, 0.0, std::f64::consts::FRAC_PI_2, 0.0];
        arm.set_joint_angles(&seed.to_vec());

        let pos_solver = IkSolver { max_iterations: 100, ..solver.clone() };
        pos_solver.solve_position(&mut arm, &target_pos);

        let (_, _, pos_err) = solver.solve_weighted(&mut arm, &target, 0.2);

        assert!(
            pos_err < 0.01,
            "Weighted IK position error too large: {:.4}m",
            pos_err
        );

        let fk = arm.forward_kinematics();
        let tool_z = fk.rotation * Vector3::z();
        assert!(
            tool_z.z < 0.0,
            "Tool Z should have downward component, got ({:.2},{:.2},{:.2})",
            tool_z.x, tool_z.y, tool_z.z
        );
    }

    #[test]
    fn test_adaptive_damping_near_singularity() {
        // Near singularity: manipulability ≈ 0 → damping should be high
        let jac = DMatrix::from_row_slice(3, 3, &[
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1e-6, // nearly singular
        ]);
        let lambda = adaptive_damping(&jac);
        assert!(lambda > 0.1, "Damping should be high near singularity, got {}", lambda);
    }

    #[test]
    fn test_adaptive_damping_far_from_singularity() {
        let jac = DMatrix::from_row_slice(3, 3, &[
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]);
        let lambda = adaptive_damping(&jac);
        assert!(lambda < 0.01, "Damping should be low far from singularity, got {}", lambda);
    }

    #[test]
    fn test_velocity_limit_scaling() {
        let arm = RobotArm::default_6dof();
        // J1 limit = 0.8 rad/s, dt = 0.016 → max delta = 0.0128
        let mut dq = vec![0.1, 0.0, 0.0, 0.0, 0.0, 0.0]; // way over limit for J1
        IkSolver::apply_velocity_limits(&mut dq, &arm, 0.016);
        // Should be scaled down proportionally
        assert!(dq[0].abs() <= 0.8 * 0.016 + 1e-10,
            "J1 delta should be within velocity limit, got {}", dq[0]);
    }

    #[test]
    fn test_ik_tracks_carving_trajectory() {
        use nalgebra::{Isometry3, Translation3, UnitQuaternion};

        // Simulate the actual carving loop:
        // 1. Pre-position arm near workpiece
        // 2. Feed a sequence of cartesian targets through IK
        // 3. Verify arm tracks them with velocity limiting

        let mut arm = RobotArm::default_6dof();
        let solver = IkSolver {
            max_iterations: 50,
            position_tolerance: 5e-4,
            orientation_tolerance: 0.005,
            max_step: 0.2,
            nullspace_gain: 0.5,
            ..IkSolver::default()
        };

        // Pre-position with multiple seeds (matching load_gcode IK seeds)
        let pi = std::f64::consts::PI;
        let pi2 = std::f64::consts::FRAC_PI_2;

        // Tool-down orientation
        let tool_down = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pi);
        let orientation_weight = 0.6;

        // Workpiece params: center=(0.65, 0, -0.05), half=0.1525
        // NOTE: config.toml had center_z=0.05 which is WRONG — workpiece too high for arm
        let workpiece_top = Vector3::new(0.65, 0.0, -0.05 + 0.1525);
        let start_pos = workpiece_top + Vector3::new(0.0, 0.0, 0.015);

        // Try multiple IK seeds (same as load_gcode)
        let start_target = Isometry3::from_parts(Translation3::from(start_pos), tool_down);
        let seeds: &[[f64; 6]] = &[
            [0.0, 1.0, 0.57, 0.0, pi2, 0.0],
            [0.0, 0.9, 0.67, 0.0, pi2, 0.0],
            [0.0, 0.8, 0.77, 0.0, pi2, 0.0],
            [0.0, 1.1, 0.47, 0.0, pi2, 0.0],
            [0.0, 1.0, 0.57, pi, -pi2, 0.0],
        ];

        let mut best_err = f64::MAX;
        let mut best_ok = false;
        for (si, seed) in seeds.iter().enumerate() {
            let mut scratch = arm.clone();
            scratch.set_joint_angles(&seed.to_vec());

            // First check where this seed's FK puts the tool
            let seed_pos = scratch.tool_position();
            eprintln!("  Seed {}: FK=({:.3},{:.3},{:.3})", si,
                seed_pos.x, seed_pos.y, seed_pos.z);

            let (ok, iters, err) = solver.solve_weighted(&mut scratch, &start_target, 0.5);
            eprintln!("  Seed {}: converged={}, iters={}, error={:.1}mm, pos=({:.3},{:.3},{:.3})",
                si, ok, iters, err * 1000.0,
                scratch.tool_position().x, scratch.tool_position().y, scratch.tool_position().z);
            if ok && err < best_err {
                best_err = err;
                best_ok = true;
                arm.set_joint_angles(&scratch.joint_angles());
            } else if err < best_err {
                best_err = err;
                arm.set_joint_angles(&scratch.joint_angles());
            }
        }

        eprintln!("Pre-position: best_converged={}, best_error={:.1}mm", best_ok, best_err * 1000.0);
        eprintln!("Target pos: ({:.3},{:.3},{:.3})", start_pos.x, start_pos.y, start_pos.z);
        eprintln!("Actual pos: ({:.3},{:.3},{:.3})", arm.tool_position().x, arm.tool_position().y, arm.tool_position().z);

        // Even if not perfectly converged, proceed if error is reasonable
        assert!(best_err < 0.05, "Pre-positioning IK error too large: {:.1}mm", best_err * 1000.0);

        let mut joint_targets = arm.joint_angles();

        // Simulate carving: interpolate between waypoints (like trajectory planner)
        // Keep moves within the comfortable workspace (center ± 50mm in XY)
        let waypoints = [
            workpiece_top + Vector3::new(0.0, 0.0, 0.01),     // Z+10mm (safe)
            workpiece_top + Vector3::new(0.0, 0.0, -0.003),   // Z-3mm (shallow cut)
            workpiece_top + Vector3::new(0.05, 0.0, -0.003),  // Move +X 50mm
            workpiece_top + Vector3::new(0.05, 0.05, -0.003), // Move +Y 50mm
            workpiece_top + Vector3::new(0.0, 0.05, -0.003),  // Back to center X
            workpiece_top + Vector3::new(0.0, -0.05, -0.003), // Move -Y 100mm (through center)
            workpiece_top + Vector3::new(0.0, 0.0, -0.02),    // Center, deeper Z-20mm
            workpiece_top + Vector3::new(0.0, 0.0, 0.01),     // Retract
        ];

        let frame_dt = 1.0 / 60.0;
        let feed_rate = 0.01; // 10mm/s (600mm/min) — typical cutting speed
        let mut stuck_count = 0;
        let mut total_moves = 0;
        let mut current_target = arm.tool_position();

        for (seg_idx, &waypoint_pos) in waypoints.iter().enumerate() {
            // Interpolate smoothly from current_target to waypoint_pos
            let seg_dist = (waypoint_pos - current_target).norm();
            let seg_time = if seg_dist > 1e-6 { seg_dist / feed_rate } else { 0.0 };
            let num_frames = (seg_time / frame_dt).ceil() as usize + 1;
            let direction = if seg_dist > 1e-6 {
                (waypoint_pos - current_target) / seg_dist
            } else {
                Vector3::zeros()
            };

            let mut seg_rejected = 0;
            for f in 0..num_frames {
                let t = (f as f64 * frame_dt * feed_rate).min(seg_dist);
                let interp_pos = current_target + direction * t;
                let cart_target = Isometry3::from_parts(
                    Translation3::from(interp_pos), tool_down,
                );

                let mut scratch = arm.clone();
                scratch.set_joint_angles(&joint_targets);

                let (_converged, _iters, ik_err) = solver.solve_weighted(
                    &mut scratch, &cart_target, orientation_weight,
                );

                let new_angles = scratch.joint_angles();
                let prev_config = arm.configuration_flags();
                let new_config = scratch.configuration_flags();
                let config_dist = scratch.config_distance(&joint_targets);

                // Config consistency check (same as update_ik)
                if new_config != prev_config && config_dist > 0.3 {
                    stuck_count += 1;
                    seg_rejected += 1;
                    if seg_rejected <= 3 || seg_rejected % 100 == 0 {
                        eprintln!("  REJECT seg{} f{}: cfg {:?}→{:?} dist={:.2} ik_err={:.1}mm",
                            seg_idx, f, prev_config, new_config, config_dist, ik_err * 1000.0);
                    }
                    continue;
                }

                // Velocity limiting
                let mut deltas: Vec<f64> = new_angles.iter().zip(joint_targets.iter())
                    .map(|(new, old)| new - old)
                    .collect();
                IkSolver::apply_velocity_limits(&mut deltas, &arm, frame_dt);

                for i in 0..joint_targets.len() {
                    joint_targets[i] += deltas[i];
                }
                arm.set_joint_angles(&joint_targets);
                total_moves += 1;

                // Log first and last frames for debugging
                if f == 0 || f == num_frames - 1 {
                    let pos_err = (arm.tool_position() - interp_pos).norm();
                    eprintln!("  seg{} f{}/{}: err={:.1}mm ik_err={:.1}mm",
                        seg_idx, f, num_frames, pos_err*1000.0, ik_err*1000.0);
                }
            }
            if seg_rejected > 0 {
                eprintln!("  Segment {} had {} config rejections out of {} frames",
                    seg_idx, seg_rejected, num_frames);
            }
            current_target = waypoint_pos;

            let pos_err = (arm.tool_position() - waypoint_pos).norm();
            eprintln!(
                "Waypoint {}: ({:.0},{:.0},{:.0})mm err={:.1}mm config=({},{},{})",
                seg_idx,
                waypoint_pos.x * 1000.0, waypoint_pos.y * 1000.0, waypoint_pos.z * 1000.0,
                pos_err * 1000.0,
                arm.configuration_flags().0, arm.configuration_flags().1, arm.configuration_flags().2,
            );
            assert!(
                pos_err < 0.01, // 10mm — smooth interpolation should track well
                "Waypoint {} tracking error too large: {:.1}mm",
                seg_idx, pos_err * 1000.0
            );
        }

        eprintln!("Total frames: {}, config rejected: {}", total_moves, stuck_count);
        assert!(stuck_count < total_moves / 4,
            "Too many config rejections: {}/{}", stuck_count, total_moves);
    }

    #[test]
    fn test_nullspace_centering_reduces_drift() {
        let mut arm = RobotArm::default_6dof();
        // Set wrist joints far from center
        arm.joints[3].angle = 1.5;
        arm.joints[4].angle = -1.0;
        arm.joints[5].angle = 2.0;

        // Target slightly offset from current to force solver iterations
        let target_pos = arm.tool_position() + Vector3::new(0.001, 0.0, 0.0);
        let solver = IkSolver {
            max_iterations: 200,
            nullspace_gain: 1.0,
            position_tolerance: 1e-4,
            max_step: 0.15,
            ..IkSolver::default()
        };

        // Record initial wrist angles
        let j4_before = arm.joints[3].angle;
        let j5_before = arm.joints[4].angle;
        let j6_before = arm.joints[5].angle;

        // Solve position-only (should use null space to center wrist)
        solver.solve_position(&mut arm, &target_pos);

        // Wrist joints should have moved closer to center (0.0)
        let j4_moved = arm.joints[3].angle.abs() < j4_before.abs() - 0.01;
        let j5_moved = arm.joints[4].angle.abs() < j5_before.abs() - 0.01;
        let j6_moved = arm.joints[5].angle.abs() < j6_before.abs() - 0.01;
        assert!(
            j4_moved || j5_moved || j6_moved,
            "At least one wrist joint should move toward center. Before: J4={:.2} J5={:.2} J6={:.2}, After: J4={:.2} J5={:.2} J6={:.2}",
            j4_before, j5_before, j6_before,
            arm.joints[3].angle, arm.joints[4].angle, arm.joints[5].angle
        );
    }
}
