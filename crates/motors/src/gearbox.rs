//! Gearbox model: ratio, efficiency, and backlash.

/// Gearbox connecting a motor to a joint.
#[derive(Debug, Clone)]
pub struct Gearbox {
    /// Gear ratio (motor turns per output turn). e.g. 100:1 → ratio = 100.0
    pub ratio: f64,
    /// Forward efficiency (0..1). Torque at output = motor_torque * ratio * efficiency.
    pub efficiency: f64,
    /// Backlash half-width (rad) at the output shaft.
    pub backlash: f64,
    /// Whether currently in the backlash deadband.
    contact_state: ContactState,
    /// Last known output position for backlash tracking.
    last_output_angle: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(dead_code)]
enum ContactState {
    /// Gear teeth in contact, torque transmits normally.
    InContact,
    /// In backlash deadband, no torque transmission.
    InBacklash,
}

impl Gearbox {
    /// Create a planetary gearbox.
    pub fn planetary(ratio: f64) -> Self {
        Self {
            ratio,
            efficiency: 0.90, // typical planetary
            backlash: 0.0005, // ~0.03° at output
            contact_state: ContactState::InContact,
            last_output_angle: 0.0,
        }
    }

    /// Create a worm gearbox (higher ratio, lower efficiency, lower backlash).
    pub fn worm(ratio: f64) -> Self {
        Self {
            ratio,
            efficiency: 0.70, // worm drives are less efficient
            backlash: 0.0002, // very low backlash
            contact_state: ContactState::InContact,
            last_output_angle: 0.0,
        }
    }

    /// Transform motor torque to joint torque, accounting for ratio and efficiency.
    pub fn motor_to_joint_torque(&self, motor_torque: f64) -> f64 {
        match self.contact_state {
            ContactState::InContact => motor_torque * self.ratio * self.efficiency,
            ContactState::InBacklash => 0.0, // no torque transmitted in deadband
        }
    }

    /// Transform joint velocity to motor velocity.
    pub fn joint_to_motor_velocity(&self, joint_velocity: f64) -> f64 {
        joint_velocity * self.ratio
    }

    /// Update backlash state based on current output angle.
    pub fn update_backlash(&mut self, output_angle: f64) {
        let delta = output_angle - self.last_output_angle;
        match self.contact_state {
            ContactState::InContact => {
                // Check if direction reversal puts us in backlash
                // (simplified: we'd need to track direction properly)
                self.last_output_angle = output_angle;
            }
            ContactState::InBacklash => {
                if delta.abs() > self.backlash {
                    self.contact_state = ContactState::InContact;
                    self.last_output_angle = output_angle;
                }
            }
        }
    }

    /// Reflected motor inertia at the joint (J_motor * ratio²).
    pub fn reflected_inertia(&self, motor_inertia: f64) -> f64 {
        motor_inertia * self.ratio * self.ratio
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_planetary_torque_amplification() {
        let gb = Gearbox::planetary(100.0);
        let motor_torque = 1.0; // 1 Nm
        let joint_torque = gb.motor_to_joint_torque(motor_torque);
        // 1.0 * 100 * 0.9 = 90 Nm
        assert!((joint_torque - 90.0).abs() < 0.01);
    }

    #[test]
    fn test_worm_gear() {
        let gb = Gearbox::worm(80.0);
        let joint_torque = gb.motor_to_joint_torque(1.0);
        // 1.0 * 80 * 0.7 = 56 Nm
        assert!((joint_torque - 56.0).abs() < 0.01);
    }

    #[test]
    fn test_velocity_transform() {
        let gb = Gearbox::planetary(100.0);
        let motor_vel = gb.joint_to_motor_velocity(1.0);
        assert!((motor_vel - 100.0).abs() < 0.01);
    }
}
