//! Revolute joint model with limits, friction, and backlash.

/// A single revolute joint with physical properties.
#[derive(Debug, Clone)]
pub struct RevoluteJoint {
    /// Current joint angle (rad).
    pub angle: f64,
    /// Current joint angular velocity (rad/s).
    pub velocity: f64,
    /// Applied torque from motor (Nm).
    pub torque: f64,

    /// Lower angle limit (rad).
    pub angle_min: f64,
    /// Upper angle limit (rad).
    pub angle_max: f64,

    /// Viscous friction coefficient (Nm·s/rad).
    pub viscous_friction: f64,
    /// Coulomb (dry) friction torque (Nm).
    pub coulomb_friction: f64,
    /// Backlash deadband half-width (rad).
    pub backlash: f64,

    /// Joint stiffness for limit enforcement (Nm/rad) — spring constant.
    pub limit_stiffness: f64,
    /// Joint damping for limit enforcement (Nm·s/rad).
    pub limit_damping: f64,
    /// Maximum joint velocity (rad/s). Per-joint limit based on motor/gearbox.
    pub velocity_limit: f64,
}

impl RevoluteJoint {
    /// Create a new joint with angle limits in degrees.
    pub fn new(min_deg: f64, max_deg: f64) -> Self {
        Self {
            angle: 0.0,
            velocity: 0.0,
            torque: 0.0,
            angle_min: min_deg.to_radians(),
            angle_max: max_deg.to_radians(),
            // Damping tuned for geared robot joints:
            // Planetary gearboxes have significant viscous drag from grease + gear mesh.
            viscous_friction: 8.0,   // Nm·s/rad — dominant damping source
            coulomb_friction: 2.0,   // Nm — static/kinetic friction from seals+gears
            backlash: 0.0,
            limit_stiffness: 1000.0, // Nm/rad — soft spring at limits (hard clamp is backup)
            limit_damping: 200.0,    // Nm·s/rad — absorb energy at limits
            velocity_limit: 3.0,     // rad/s — default, overridden per-joint
        }
    }

    /// Compute total friction torque opposing motion.
    pub fn friction_torque(&self) -> f64 {
        let viscous = -self.viscous_friction * self.velocity;
        let coulomb = if self.velocity.abs() > 1e-6 {
            -self.coulomb_friction * self.velocity.signum()
        } else {
            0.0
        };
        viscous + coulomb
    }

    /// Compute joint-limit penalty torque (spring-damper at limits).
    pub fn limit_torque(&self) -> f64 {
        if self.angle < self.angle_min {
            let penetration = self.angle_min - self.angle;
            self.limit_stiffness * penetration - self.limit_damping * self.velocity
        } else if self.angle > self.angle_max {
            let penetration = self.angle_max - self.angle;
            self.limit_stiffness * penetration - self.limit_damping * self.velocity
        } else {
            0.0
        }
    }

    /// Net torque seen by the joint (motor + friction + limits).
    pub fn net_torque(&self) -> f64 {
        self.torque + self.friction_torque() + self.limit_torque()
    }

    /// Integrate joint state forward by dt given acceleration.
    pub fn integrate(&mut self, acceleration: f64, dt: f64) {
        self.velocity += acceleration * dt;
        // Safety clamp: prevent runaway velocities (per-joint limit)
        self.velocity = self.velocity.clamp(-self.velocity_limit, self.velocity_limit);
        self.angle += self.velocity * dt;
    }

    /// Clamp angle to limits (hard clamp, for safety).
    pub fn clamp_to_limits(&mut self) {
        if self.angle < self.angle_min {
            self.angle = self.angle_min;
            if self.velocity < 0.0 {
                self.velocity = 0.0;
            }
        } else if self.angle > self.angle_max {
            self.angle = self.angle_max;
            if self.velocity > 0.0 {
                self.velocity = 0.0;
            }
        }
    }
}

impl RevoluteJoint {
    /// Set viscous and coulomb friction (builder pattern).
    pub fn with_friction(mut self, viscous: f64, coulomb: f64) -> Self {
        self.viscous_friction = viscous;
        self.coulomb_friction = coulomb;
        self
    }

    /// Set per-joint velocity limit (builder pattern).
    pub fn with_velocity_limit(mut self, limit: f64) -> Self {
        self.velocity_limit = limit;
        self
    }
}

impl Default for RevoluteJoint {
    fn default() -> Self {
        Self::new(-180.0, 180.0)
    }
}

/// Rotary table (A-axis) — similar to revolute joint but continuous.
#[derive(Debug, Clone)]
pub struct RotaryTable {
    pub angle: f64,
    pub velocity: f64,
    /// Target angle for internal PD control (set by carving loop).
    pub target_angle: f64,
    pub inertia: f64,
}

impl RotaryTable {
    pub fn new(inertia: f64) -> Self {
        Self {
            angle: 0.0,
            velocity: 0.0,
            target_angle: 0.0,
            inertia,
        }
    }

    /// Step with internal overdamped PD control at physics rate.
    /// kp=200, I=5 → ω_n=6.3 → critical kd=2*ω_n*I=63.
    /// Use kd=80 + viscous=20 for heavily overdamped response (ζ≈1.6).
    pub fn step(&mut self, dt: f64) {
        let err = self.target_angle - self.angle;
        let pd_torque = 200.0 * err - 100.0 * self.velocity;
        let accel = pd_torque / self.inertia;
        self.velocity += accel * dt;
        self.velocity = self.velocity.clamp(-4.0, 4.0);
        self.angle += self.velocity * dt;
    }

    /// Angular error from target.
    pub fn error(&self) -> f64 {
        (self.target_angle - self.angle).abs()
    }
}

/// Linear track (U-axis) — translates the arm base along X.
/// Mechanically independent from the 6DOF arm (like RotaryTable).
#[derive(Debug, Clone)]
pub struct LinearTrack {
    pub position: f64,
    pub velocity: f64,
    /// Target position for internal PD control (set by carving loop).
    pub target_position: f64,
    /// Effective mass of the carriage (kg).
    pub inertia: f64,
    pub min_position: f64,
    pub max_position: f64,
}

impl LinearTrack {
    pub fn new(inertia: f64) -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            target_position: 0.0,
            inertia,
            min_position: -0.5,
            max_position: 0.5,
        }
    }

    /// Step with internal overdamped PD control at physics rate.
    /// kp=2000, kd=800 — overdamped (ζ≈1.3) for ~50kg carriage.
    /// Settles 75mm step in ~0.3s sim time.
    pub fn step(&mut self, dt: f64) {
        let err = self.target_position - self.position;
        let pd_force = 2000.0 * err - 800.0 * self.velocity;
        let accel = pd_force / self.inertia;
        self.velocity += accel * dt;
        self.velocity = self.velocity.clamp(-1.0, 1.0); // 1 m/s max
        self.position += self.velocity * dt;
        self.position = self.position.clamp(self.min_position, self.max_position);
    }

    /// Position error from target.
    pub fn error(&self) -> f64 {
        (self.target_position - self.position).abs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joint_limits() {
        let mut joint = RevoluteJoint::new(-90.0, 90.0);
        joint.angle = 100.0_f64.to_radians(); // past limit
        let torque = joint.limit_torque();
        // Should push back toward the limit
        assert!(torque < 0.0, "Limit torque should be negative: {}", torque);
    }

    #[test]
    fn test_friction_opposes_motion() {
        let mut joint = RevoluteJoint::default();
        joint.velocity = 1.0;
        let f = joint.friction_torque();
        assert!(f < 0.0, "Friction should oppose positive velocity");

        joint.velocity = -1.0;
        let f = joint.friction_torque();
        assert!(f > 0.0, "Friction should oppose negative velocity");
    }
}
