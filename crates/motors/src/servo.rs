//! Servo motor model with flat torque to rated speed, linear dropoff above.

/// Servo motor model: constant torque up to rated speed, linear dropoff to peak speed.
#[derive(Debug, Clone)]
pub struct ServoMotor {
    /// Rated torque (Nm) — available from 0 to rated_speed.
    pub rated_torque: f64,
    /// Rated speed (rad/s) — full torque available below this.
    pub peak_speed: f64,
    /// Current motor shaft velocity (rad/s), before gearbox.
    pub shaft_velocity: f64,
    /// Rotor inertia (kg*m^2).
    pub rotor_inertia: f64,
}

impl ServoMotor {
    /// Create a servo motor with given torque and speed ratings.
    pub fn new(rated_torque: f64, peak_speed: f64, rotor_inertia: f64) -> Self {
        Self {
            rated_torque,
            peak_speed,
            shaft_velocity: 0.0,
            rotor_inertia,
        }
    }

    /// Maximum available torque at the current shaft speed.
    /// Flat from 0 to rated_speed * 0.8, then linear dropoff to zero at peak_speed.
    pub fn available_torque(&self) -> f64 {
        let speed = self.shaft_velocity.abs();
        let knee = self.peak_speed * 0.8;
        if speed <= knee {
            self.rated_torque
        } else if speed >= self.peak_speed {
            0.0
        } else {
            let t = (speed - knee) / (self.peak_speed - knee);
            self.rated_torque * (1.0 - t)
        }
    }

    /// Clamp a torque command to the available torque envelope.
    pub fn clamp_torque(&self, commanded: f64) -> f64 {
        let available = self.available_torque();
        commanded.clamp(-available, available)
    }

    /// Update shaft velocity (called after gearbox transforms the joint velocity).
    pub fn update_velocity(&mut self, joint_velocity: f64, gear_ratio: f64) {
        self.shaft_velocity = joint_velocity * gear_ratio;
    }
}

/// Per-joint servo state.
#[derive(Debug, Clone)]
pub struct ServoState {
    pub motor: ServoMotor,
    pub output_torque: f64,
}

impl ServoState {
    pub fn new(motor: ServoMotor) -> Self {
        Self {
            motor,
            output_torque: 0.0,
        }
    }

    /// Apply a torque command, clamping to motor limits. Returns actual torque.
    pub fn apply_torque(&mut self, commanded: f64, _dt: f64) -> f64 {
        let actual = self.motor.clamp_torque(commanded);
        self.output_torque = actual;
        actual
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_servo_flat_torque_region() {
        let motor = ServoMotor::new(8.4, 600.0, 0.0003);
        // At zero speed, full torque
        assert!((motor.available_torque() - 8.4).abs() < 1e-6);
    }

    #[test]
    fn test_servo_dropoff() {
        let mut motor = ServoMotor::new(8.4, 600.0, 0.0003);
        // At peak speed, zero torque
        motor.shaft_velocity = 600.0;
        assert!(motor.available_torque() < 0.01);
        // At 90% of peak (above knee at 80%), partial torque
        motor.shaft_velocity = 540.0;
        let t = motor.available_torque();
        assert!(t > 0.0 && t < 8.4);
    }
}
