//! PID controller with anti-windup and output clamping.

/// PID controller for joint position control.
/// Uses derivative-on-measurement (not error) to prevent kick on target changes.
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
    /// Previous measurement (for derivative-on-measurement).
    prev_measurement: f64,
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
            prev_measurement: 0.0,
            integral_max: output_max / ki.max(1e-6), // anti-windup limit
        }
    }

    /// Compute control output given target and current position.
    ///
    /// Uses derivative-on-measurement: D term is -kd * d(current)/dt.
    /// This avoids derivative kick when the target changes suddenly,
    /// producing smooth, non-bouncy motion.
    ///
    /// Returns clamped torque command.
    pub fn update(&mut self, target: f64, current: f64, dt: f64) -> f64 {
        let error = target - current;

        // Integral with anti-windup clamping
        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.integral_max, self.integral_max);

        // Derivative on measurement (not error) — prevents kick on target change
        let derivative = if dt > 0.0 {
            -(current - self.prev_measurement) / dt
        } else {
            0.0
        };
        self.prev_measurement = current;

        // PID output
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;

        // Clamp output
        output.clamp(-self.output_max, self.output_max)
    }

    /// Reset integral and derivative state.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = 0.0;
    }
}

/// Create a PID controller tuned for a large joint (J1-J3).
///
/// Gains are motor-side: output feeds into motor clamp (12 Nm NEMA34)
/// then gearbox (100:1 * 0.9 = 90× amplification).
/// With reflected motor inertia (20 kg·m²), natural freq = sqrt(kp*90 / 20).
/// kp=50: ω_n = sqrt(4500/20) = 15 rad/s ≈ 2.4 Hz.
/// Critical damping: kd_eff = 2*ω_n*I = 2*15*20 = 600 → kd = 600/90 ≈ 6.7.
/// Using kd=8 for slight overdamping (ζ ≈ 1.2) — no bounce, precise hold.
pub fn large_joint_pid() -> PidController {
    PidController::new(
        50.0, // kp — 0.1 rad error → 5 Nm motor → 450 Nm at joint → 22 rad/s²
        2.0,  // ki — integral for steady-state (wind-up limited)
        8.0,  // kd — overdamped: ζ≈1.2, no oscillation
        12.0, // max output: motor holding torque limit
    )
}

/// Create a PID controller tuned for a small wrist joint (J4-J6).
///
/// Gearbox: 50:1 * 0.9 = 45× amplification.
/// Reflected inertia: 1.25 kg·m². Natural freq = sqrt(kp*45 / 1.25).
/// kp=20: ω_n = sqrt(900/1.25) = 26.8 rad/s ≈ 4.3 Hz.
/// Critical damping: kd_eff = 2*26.8*1.25 = 67 → kd = 67/45 ≈ 1.5.
/// Using kd=2.5 for overdamping (ζ ≈ 1.7) — rock solid wrist.
pub fn small_joint_pid() -> PidController {
    PidController::new(
        20.0, // kp — 0.1 rad error → 2 Nm motor → 90 Nm at joint → 72 rad/s²
        1.0,  // ki
        2.5,  // kd — overdamped: ζ≈1.7
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
