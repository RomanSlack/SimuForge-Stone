//! Stepper motor model with speed-torque curve.

/// Stepper motor model with pull-out torque curve.
#[derive(Debug, Clone)]
pub struct StepperMotor {
    /// Holding torque at zero speed (Nm).
    pub holding_torque: f64,
    /// Speed at which torque drops to zero (rad/s).
    pub max_speed: f64,
    /// Current motor shaft velocity (rad/s), before gearbox.
    pub shaft_velocity: f64,
    /// Rotor inertia (kg·m²).
    pub rotor_inertia: f64,
    /// Winding resistance (Ω) — for thermal estimation.
    pub winding_resistance: f64,
    /// Current draw estimate (A).
    pub current_estimate: f64,
}

impl StepperMotor {
    /// Create a NEMA34-class motor.
    /// max_speed ~80 rad/s (~760 RPM) — realistic for 48V closed-loop drive.
    /// Through 100:1 gearbox this allows 0.8 rad/s (46°/s) joint motion with torque.
    pub fn nema34(holding_torque: f64) -> Self {
        Self {
            holding_torque,
            max_speed: 80.0,
            shaft_velocity: 0.0,
            rotor_inertia: 0.002, // ~2000 g·cm²
            winding_resistance: 0.8,
            current_estimate: 0.0,
        }
    }

    /// Create a NEMA23-class motor.
    /// max_speed ~120 rad/s (~1150 RPM) — lighter rotor, higher speed.
    /// Through 50:1 gearbox this allows 2.4 rad/s (137°/s) joint motion.
    pub fn nema23(holding_torque: f64) -> Self {
        Self {
            holding_torque,
            max_speed: 120.0,
            shaft_velocity: 0.0,
            rotor_inertia: 0.0005,
            winding_resistance: 1.2,
            current_estimate: 0.0,
        }
    }

    /// Maximum available torque at the current shaft speed.
    /// Linear pull-out approximation: T = T_hold * (1 - |ω|/ω_max)
    pub fn available_torque(&self) -> f64 {
        let speed_ratio = (self.shaft_velocity.abs() / self.max_speed).min(1.0);
        self.holding_torque * (1.0 - speed_ratio)
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

    /// Estimate current draw based on torque output (rough model).
    pub fn estimate_current(&mut self, torque_output: f64) {
        // Torque ≈ Kt * I, and Kt ≈ holding_torque / rated_current
        let rated_current = 6.0; // typical NEMA34
        let kt = self.holding_torque / rated_current;
        self.current_estimate = if kt > 0.0 {
            torque_output.abs() / kt
        } else {
            0.0
        };
    }
}

/// Per-joint motor state combining stepper and dynamic state.
#[derive(Debug, Clone)]
pub struct MotorState {
    pub motor: StepperMotor,
    /// Accumulated thermal energy estimate (J) — very simplified.
    pub thermal_energy: f64,
    /// Torque actually delivered to the gearbox input (Nm).
    pub output_torque: f64,
}

impl MotorState {
    pub fn new(motor: StepperMotor) -> Self {
        Self {
            motor,
            thermal_energy: 0.0,
            output_torque: 0.0,
        }
    }

    /// Apply a torque command, clamping to motor limits. Returns actual torque.
    pub fn apply_torque(&mut self, commanded: f64, dt: f64) -> f64 {
        let actual = self.motor.clamp_torque(commanded);
        self.output_torque = actual;
        // Rough thermal accumulation: P = I²R
        self.motor.estimate_current(actual);
        self.thermal_energy +=
            self.motor.current_estimate.powi(2) * self.motor.winding_resistance * dt;
        actual
    }

    /// Estimated motor temperature rise above ambient (°C), very rough.
    pub fn estimated_temp_rise(&self) -> f64 {
        // Thermal mass ~0.5 J/°C for NEMA34
        self.thermal_energy / 0.5
    }
}
