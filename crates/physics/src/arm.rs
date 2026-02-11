//! Robot arm model: DH parameters, link properties, forward kinematics, Jacobian.

use nalgebra::{DMatrix, Isometry3, Matrix3, Vector3};

use crate::joint::RevoluteJoint;
use crate::spatial::{spatial_inertia, SpatialMatrix};
use simuforge_core::DhParams;

/// A serial robot arm defined by DH parameters and link properties.
#[derive(Debug, Clone)]
pub struct RobotArm {
    /// DH parameters for each joint.
    pub dh_params: Vec<DhParams>,
    /// Joint state (angle, velocity, torque).
    pub joints: Vec<RevoluteJoint>,
    /// Spatial inertia of each link in its body frame.
    pub link_spatial_inertias: Vec<SpatialMatrix>,
    /// Mass of each link (kg).
    pub link_masses: Vec<f64>,
    /// Reflected motor inertia at each joint (kg·m²).
    /// I_reflected = I_rotor * gear_ratio².  Dominates link inertia for geared joints.
    pub motor_reflected_inertias: Vec<f64>,
    /// Link names (for debug/display).
    pub link_names: Vec<String>,
    /// Tool offset along the flange Z-axis (meters).
    /// FK, Jacobian, and tool_position all include this offset.
    pub tool_offset: f64,
}

impl RobotArm {
    pub fn num_joints(&self) -> usize {
        self.joints.len()
    }

    /// Compute forward kinematics: returns the pose of the tool tip (end-effector)
    /// in the base frame. Includes tool_offset along the flange Z-axis.
    pub fn forward_kinematics(&self) -> Isometry3<f64> {
        let mut t = Isometry3::identity();
        for (i, dh) in self.dh_params.iter().enumerate() {
            t *= dh.transform(self.joints[i].angle);
        }
        if self.tool_offset != 0.0 {
            t *= Isometry3::from_parts(
                Vector3::new(0.0, 0.0, self.tool_offset).into(),
                nalgebra::UnitQuaternion::identity(),
            );
        }
        t
    }

    /// Compute forward kinematics for the flange only (no tool offset).
    /// Used for rendering the arm links separately from the tool.
    pub fn flange_pose(&self) -> Isometry3<f64> {
        let mut t = Isometry3::identity();
        for (i, dh) in self.dh_params.iter().enumerate() {
            t *= dh.transform(self.joints[i].angle);
        }
        t
    }

    /// Compute all link frames (base to each joint).
    /// Returns n+1 frames: base frame + each link frame.
    pub fn link_frames(&self) -> Vec<Isometry3<f64>> {
        let mut frames = Vec::with_capacity(self.joints.len() + 1);
        let mut t = Isometry3::identity();
        frames.push(t);
        for (i, dh) in self.dh_params.iter().enumerate() {
            t *= dh.transform(self.joints[i].angle);
            frames.push(t);
        }
        frames
    }

    /// Compute the geometric Jacobian (6×n) in the base frame.
    /// Top 3 rows: angular velocity. Bottom 3 rows: linear velocity.
    /// End-effector includes tool_offset (Jacobian is for the tool tip).
    pub fn jacobian(&self) -> DMatrix<f64> {
        let n = self.num_joints();
        let mut jac = DMatrix::zeros(6, n);
        let frames = self.link_frames();
        // Use tool tip position (includes tool_offset)
        let p_ee = self.forward_kinematics().translation.vector;

        for i in 0..n {
            let frame_i = &frames[i];
            // Z-axis of joint i in base frame
            let z_i = frame_i.rotation * Vector3::z();
            // Position of joint i in base frame
            let p_i = frame_i.translation.vector;

            // Angular part (revolute joint → z_i)
            jac[(0, i)] = z_i.x;
            jac[(1, i)] = z_i.y;
            jac[(2, i)] = z_i.z;

            // Linear part: z_i × (p_ee - p_i)
            let lin = z_i.cross(&(p_ee - p_i));
            jac[(3, i)] = lin.x;
            jac[(4, i)] = lin.y;
            jac[(5, i)] = lin.z;
        }

        jac
    }

    /// Create the default 6-DOF arm matching the BOM specification.
    /// All dimensions converted to meters.
    pub fn default_6dof() -> Self {
        use std::f64::consts::FRAC_PI_2;

        let dh_params = vec![
            DhParams::new(0.0, 0.300, -FRAC_PI_2),   // J1
            DhParams::new(0.500, 0.0, 0.0),           // J2
            DhParams::new(0.400, 0.0, 0.0),           // J3
            DhParams::new(0.0, 0.0, -FRAC_PI_2),      // J4
            DhParams::new(0.0, 0.300, FRAC_PI_2),     // J5
            DhParams::new(0.0, 0.080, 0.0),           // J6
        ];

        let joints = vec![
            // Large joints (100:1 planetary): high friction from gearbox
            RevoluteJoint::new(-360.0, 360.0).with_friction(10.0, 3.0),  // J1: base, continuous rotation
            RevoluteJoint::new(-45.0, 135.0).with_friction(10.0, 3.0),   // J2
            RevoluteJoint::new(-135.0, 135.0).with_friction(8.0, 2.5),   // J3
            // Small joints (50:1 planetary): less friction
            RevoluteJoint::new(-180.0, 180.0).with_friction(3.0, 1.0),   // J4
            RevoluteJoint::new(-120.0, 120.0).with_friction(3.0, 1.0),   // J5
            RevoluteJoint::new(-360.0, 360.0).with_friction(2.0, 0.5),   // J6
        ];

        // Link masses (approximate for steel/aluminum arm)
        let link_masses = vec![15.0, 12.0, 10.0, 5.0, 4.0, 2.0];

        // Build spatial inertias for each link
        let link_spatial_inertias: Vec<SpatialMatrix> = dh_params
            .iter()
            .zip(link_masses.iter())
            .map(|(dh, &mass)| {
                // Approximate each link as a cylinder along its primary axis
                let length = (dh.a.abs() + dh.d.abs()).max(0.05); // min 5cm
                let radius = 0.04; // 4cm radius
                let ixx = mass * (3.0 * radius * radius + length * length) / 12.0;
                let izz = mass * radius * radius / 2.0;
                let inertia = Matrix3::new(ixx, 0.0, 0.0, 0.0, ixx, 0.0, 0.0, 0.0, izz);
                let com = Vector3::new(dh.a / 2.0, 0.0, dh.d / 2.0);
                spatial_inertia(mass, &com, &inertia)
            })
            .collect();

        // Reflected motor inertia: I_rotor * gear_ratio²
        // This DOMINATES link inertia (40-670x) and must be in the dynamics model.
        let motor_reflected_inertias = vec![
            0.002 * 100.0_f64.powi(2),  // J1: NEMA34, 100:1 → 20.0 kg·m²
            0.002 * 100.0_f64.powi(2),  // J2: NEMA34, 100:1 → 20.0 kg·m²
            0.002 * 100.0_f64.powi(2),  // J3: NEMA34, 100:1 → 20.0 kg·m²
            0.0005 * 50.0_f64.powi(2),  // J4: NEMA23, 50:1 → 1.25 kg·m²
            0.0005 * 50.0_f64.powi(2),  // J5: NEMA23, 50:1 → 1.25 kg·m²
            0.0005 * 50.0_f64.powi(2),  // J6: NEMA23, 50:1 → 1.25 kg·m²
        ];

        let link_names = vec![
            "Base/Shoulder".into(),
            "Upper Arm".into(),
            "Forearm".into(),
            "Wrist 1".into(),
            "Wrist 2".into(),
            "Wrist 3/Flange".into(),
        ];

        Self {
            dh_params,
            joints,
            link_spatial_inertias,
            link_masses,
            motor_reflected_inertias,
            link_names,
            tool_offset: 0.0, // No tool by default; sim sets this
        }
    }

    /// Get joint angles as a vector.
    pub fn joint_angles(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.angle).collect()
    }

    /// Get joint velocities as a vector.
    pub fn joint_velocities(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.velocity).collect()
    }

    /// Set joint angles from a slice.
    pub fn set_joint_angles(&mut self, angles: &[f64]) {
        for (j, &a) in self.joints.iter_mut().zip(angles.iter()) {
            j.angle = a;
        }
    }

    /// Integrate all joints forward by dt given accelerations.
    pub fn integrate(&mut self, accelerations: &[f64], dt: f64) {
        for (j, &acc) in self.joints.iter_mut().zip(accelerations.iter()) {
            j.integrate(acc, dt);
        }
    }

    /// Get net torques (motor + friction + limits) for each joint.
    pub fn net_torques(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.net_torque()).collect()
    }

    /// Tool tip position in base frame (convenience).
    pub fn tool_position(&self) -> Vector3<f64> {
        self.forward_kinematics().translation.vector
    }

    /// Compute gravity compensation torques (motor-side).
    ///
    /// Returns the joint torques needed to hold the arm static against gravity.
    /// For each joint j, sums gravity contributions from all downstream links.
    pub fn gravity_compensation(&self, gravity: &Vector3<f64>) -> Vec<f64> {
        let n = self.num_joints();
        let frames = self.link_frames();
        let mut torques = vec![0.0; n];

        for i in 0..n {
            let mass = self.link_masses[i];
            // Approximate COM: midpoint between consecutive frames
            let com = (frames[i].translation.vector + frames[i + 1].translation.vector) * 0.5;
            let f_grav = mass * gravity;

            // Project gravity force onto each upstream joint axis
            for j in 0..=i {
                let z_j = frames[j].rotation * Vector3::z();
                let r = com - frames[j].translation.vector;
                torques[j] += z_j.cross(&r).dot(&f_grav);
            }
        }

        torques
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fk_home_position() {
        let arm = RobotArm::default_6dof();
        let fk = arm.forward_kinematics();
        let pos = fk.translation.vector;

        // At home (all zeros), compute expected reach:
        // J1: translate Z by 0.3
        // J2: translate X by 0.5
        // J3: translate X by 0.4
        // J4: no translation
        // J5: translate Z by 0.3
        // J6: translate Z by 0.08
        // Total reach depends on DH convention — just check it's reasonable
        let reach = pos.norm();
        assert!(
            reach > 0.5 && reach < 2.0,
            "Unexpected reach at home: {} (pos: {:?})",
            reach,
            pos
        );
    }

    #[test]
    fn test_jacobian_dimensions() {
        let arm = RobotArm::default_6dof();
        let jac = arm.jacobian();
        assert_eq!(jac.nrows(), 6);
        assert_eq!(jac.ncols(), 6);
    }

    #[test]
    fn test_link_frames_count() {
        let arm = RobotArm::default_6dof();
        let frames = arm.link_frames();
        assert_eq!(frames.len(), 7); // base + 6 links
    }
}
