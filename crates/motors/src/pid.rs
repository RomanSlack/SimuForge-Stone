//! PID controller with anti-windup and output clamping.

/// PID controller for joint position control.
#[derive(Debug, Clone)]
pub struct PidController {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,

    /// Maximum output magnitude (torque limit).
    pub output_max: f64,

    /// Accumulated integral error.
    integral: f64,
    /// Previous error (for derivative).
    prev_error: f64,
    /// Anti-windup: max integral accumulation.
    integral_max: f64,
}

impl PidController {
    /// Create a PID controller with given gains and output limit.
    pub fn new(kp: f64, ki: f64, kd: f64, output_max: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            output_max,
            integral: 0.0,
            prev_error: 0.0,
            integral_max: output_max / ki.max(1e-6), // anti-windup limit
        }
    }

    /// Compute control output given target and current position.
    ///
    /// Returns clamped torque command.
    pub fn update(&mut self, target: f64, current: f64, dt: f64) -> f64 {
        let error = target - current;

        // Integral with anti-windup clamping
        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.integral_max, self.integral_max);

        // Derivative (on error, with filtering)
        let derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };
        self.prev_error = error;

        // PID output
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;

        // Clamp output
        output.clamp(-self.output_max, self.output_max)
    }

    /// Reset integral and derivative state.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    /// Current error (last computed).
    pub fn error(&self) -> f64 {
        self.prev_error
    }
}

/// Create a PID controller tuned for a large joint (J1-J3).
///
/// Gains are motor-side: output feeds into motor clamp (12 Nm NEMA34)
/// then gearbox (100:1 * 0.9 = 90× amplification).
/// Effective joint stiffness: kp * 90 ≈ 450 Nm/rad — adequate for ~300 Nm gravity loads.
pub fn large_joint_pid() -> PidController {
    PidController::new(
        5.0,  // kp — 1 rad error → 5 Nm motor → 450 Nm at joint
        0.3,  // ki — slow integral for steady-state
        0.8,  // kd — 1 rad/s → 0.8 Nm motor → 72 Nm damping
        12.0, // max output: motor holding torque limit
    )
}

/// Create a PID controller tuned for a small wrist joint (J4-J6).
///
/// Gearbox: 50:1 * 0.9 = 45× amplification.
/// Effective joint stiffness: kp * 45 ≈ 90 Nm/rad.
pub fn small_joint_pid() -> PidController {
    PidController::new(
        2.0,  // kp — 1 rad error → 2 Nm motor → 90 Nm at joint
        0.15, // ki
        0.4,  // kd — 1 rad/s → 0.4 Nm motor → 18 Nm damping
        3.0,  // max output: motor holding torque limit
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_converges() {
        let mut pid = PidController::new(100.0, 10.0, 20.0, 1000.0);
        let target = 1.0;
        let mut position = 0.0;
        let mut velocity = 0.0;
        let dt = 0.001;
        let mass = 1.0; // simplified single-mass system

        for _ in 0..5000 {
            let torque = pid.update(target, position, dt);
            let accel = torque / mass;
            velocity += accel * dt;
            velocity *= 0.99; // damping
            position += velocity * dt;
        }

        assert!(
            (position - target).abs() < 0.05,
            "PID didn't converge: position={}, target={}",
            position,
            target
        );
    }

    #[test]
    fn test_pid_output_clamped() {
        let mut pid = PidController::new(100.0, 10.0, 20.0, 50.0);
        let output = pid.update(1000.0, 0.0, 0.001);
        assert!(output.abs() <= 50.0, "Output not clamped: {}", output);
    }
}
