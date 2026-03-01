//! Ball rigid body: gravity + drag + Magnus effect, spin, integration.
//!
//! Phase 2 will implement full ball physics.

use nalgebra::Vector3;

/// Ball physical constants.
pub const BALL_MASS: f64 = 0.0027;     // 2.7g
pub const BALL_RADIUS: f64 = 0.020;    // 20mm
/// Moment of inertia for a hollow sphere: I = (2/3) * m * r^2
pub const BALL_INERTIA: f64 = 2.0 / 3.0 * BALL_MASS * BALL_RADIUS * BALL_RADIUS;

/// Drag coefficient for a smooth sphere.
pub const DRAG_CD: f64 = 0.40;
/// Magnus coefficient.
pub const MAGNUS_CL: f64 = 0.60;
/// Air density (kg/m^3).
pub const AIR_DENSITY: f64 = 1.225;
/// Cross-sectional area.
pub const BALL_AREA: f64 = std::f64::consts::PI * BALL_RADIUS * BALL_RADIUS;

/// Ball state.
#[derive(Debug, Clone)]
pub struct Ball {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub spin: Vector3<f64>,      // angular velocity (rad/s)
    pub active: bool,            // false = ball is dead/out-of-play
    pub floor_bounces: u32,      // ball dead after 3 floor bounces
}

impl Ball {
    pub fn new() -> Self {
        Self {
            position: Vector3::new(0.0, 0.0, 1.0), // above table
            velocity: Vector3::zeros(),
            spin: Vector3::zeros(),
            active: false,
            floor_bounces: 0,
        }
    }

    /// Serve the ball from a position with initial velocity and spin.
    pub fn serve(&mut self, pos: Vector3<f64>, vel: Vector3<f64>, spin: Vector3<f64>) {
        self.position = pos;
        self.velocity = vel;
        self.spin = spin;
        self.active = true;
        self.floor_bounces = 0;
    }

    /// Compute net force on ball (gravity + drag + Magnus).
    pub fn compute_forces(&self) -> Vector3<f64> {
        let gravity = Vector3::new(0.0, 0.0, -9.81) * BALL_MASS;

        let speed = self.velocity.norm();
        let drag = if speed > 1e-6 {
            -0.5 * DRAG_CD * AIR_DENSITY * BALL_AREA * speed * self.velocity
        } else {
            Vector3::zeros()
        };

        let magnus = if speed > 1e-6 && self.spin.norm() > 1e-6 {
            let vol = (4.0 / 3.0) * std::f64::consts::PI * BALL_RADIUS.powi(3);
            MAGNUS_CL * vol * AIR_DENSITY * self.spin.cross(&self.velocity)
        } else {
            Vector3::zeros()
        };

        gravity + drag + magnus
    }

    /// Integrate one timestep (semi-implicit Euler).
    pub fn step(&mut self, dt: f64) {
        if !self.active {
            return;
        }
        let force = self.compute_forces();
        let accel = force / BALL_MASS;
        self.velocity += accel * dt;
        self.position += self.velocity * dt;
    }
}
