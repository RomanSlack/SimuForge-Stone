//! Aerodynamic point-mass flight model for a delta-wing loitering munition.
//!
//! Physics runs at 200 Hz. The model computes lift, drag, thrust, and gravity,
//! then integrates position and velocity. Orientation (heading, pitch, bank)
//! is updated by the guidance system's commands.

use nalgebra::Vector3;

/// Physics timestep: 200 Hz.
pub const PHYSICS_DT: f64 = 1.0 / 200.0;

// ── Shahed-136 constants ──────────────────────────────────────────────────────

/// Total launch mass (kg).
pub const MASS_TOTAL: f64 = 200.0;
/// Empty airframe + engine (no fuel, no warhead) (kg).
pub const MASS_EMPTY: f64 = 70.0;
/// Warhead mass (kg).
pub const MASS_WARHEAD: f64 = 50.0;
/// Initial fuel load (kg).
pub const FUEL_INIT: f64 = 80.0;

/// Wing reference area (m²).
pub const S_WING: f64 = 2.75;
/// Zero-lift drag coefficient.
pub const CD0: f64 = 0.030;
/// Oswald efficiency factor.
pub const OSWALD_E: f64 = 0.65;
/// Wing aspect ratio.
pub const ASPECT_RATIO: f64 = 2.27;

/// Maximum lift coefficient (vortex-augmented delta stall).
pub const CL_MAX: f64 = 1.05;

/// Cruise prop thrust (N) — enough to sustain ~185 km/h level flight.
pub const THRUST_CRUISE: f64 = 400.0;
/// RATO booster thrust (N).
pub const THRUST_BOOSTER: f64 = 10_000.0;
/// Fuel consumption rate at cruise (kg/s). ~9.6 L/hr * 0.87 kg/L / 3600.
pub const FUEL_RATE_CRUISE: f64 = 0.0023;

/// Sea-level air density (kg/m³).
const RHO_0: f64 = 1.225;
/// Scale height for barometric formula (m).
const SCALE_HEIGHT: f64 = 8500.0;

/// Drone flight state.
pub struct FlightState {
    /// Position in DH Z-up world frame (meters).
    pub position: Vector3<f64>,
    /// Velocity in world frame (m/s).
    pub velocity: Vector3<f64>,
    /// Heading (radians, 0 = +X axis, positive = toward +Y).
    pub heading: f64,
    /// Pitch angle (radians, positive = nose up).
    pub pitch: f64,
    /// Bank angle (radians, positive = right wing down).
    pub bank: f64,
    /// Remaining fuel mass (kg).
    pub fuel_mass: f64,
    /// Propeller rotation angle for visual spinning (radians).
    pub prop_angle: f64,
}

impl FlightState {
    /// Create a new drone state sitting on the launch rail at the origin.
    pub fn new() -> Self {
        Self {
            position: Vector3::new(0.0, 0.0, 1.0), // on launch rail, slightly above ground
            velocity: Vector3::zeros(),
            heading: 0.0,  // facing +X
            pitch: 10.0_f64.to_radians(), // launch rail angle
            bank: 0.0,
            fuel_mass: FUEL_INIT,
            prop_angle: 0.0,
        }
    }

    /// Total mass (airframe + warhead + remaining fuel).
    pub fn total_mass(&self) -> f64 {
        MASS_EMPTY + MASS_WARHEAD + self.fuel_mass
    }

    /// Airspeed (magnitude of velocity, m/s).
    pub fn airspeed(&self) -> f64 {
        self.velocity.norm()
    }

    /// Airspeed in km/h.
    pub fn airspeed_kmh(&self) -> f64 {
        self.airspeed() * 3.6
    }

    /// Altitude above ground level (m). Ground is at z=0.
    pub fn altitude(&self) -> f64 {
        self.position.z.max(0.0)
    }

    /// Heading in degrees (0-360).
    pub fn heading_deg(&self) -> f64 {
        self.heading.to_degrees().rem_euclid(360.0)
    }

    /// Forward direction unit vector from heading + pitch.
    pub fn forward_dir(&self) -> Vector3<f64> {
        let (sh, ch) = self.heading.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        Vector3::new(ch * cp, sh * cp, sp)
    }

    /// Up direction in the drone's body frame (perpendicular to forward, in pitch plane).
    fn body_up(&self) -> Vector3<f64> {
        let (sh, ch) = self.heading.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        Vector3::new(-ch * sp, -sh * sp, cp)
    }

    /// Right-wing direction (perpendicular to forward and body up, accounting for bank).
    pub fn right_dir(&self) -> Vector3<f64> {
        let fwd = self.forward_dir();
        let up = self.body_up();
        let right = fwd.cross(&up);
        // Apply bank rotation around forward axis
        let (sb, cb) = self.bank.sin_cos();
        right * cb + up * sb
    }
}

/// Air density at given altitude using barometric approximation.
fn air_density(altitude: f64) -> f64 {
    RHO_0 * (-altitude / SCALE_HEIGHT).exp()
}

/// Compute angle of attack from velocity and drone pitch.
/// Returns alpha in radians.
fn angle_of_attack(state: &FlightState) -> f64 {
    let speed = state.airspeed();
    if speed < 1.0 {
        return 0.0;
    }
    let vel_hat = state.velocity / speed;
    let fwd = state.forward_dir();
    // Alpha = angle between velocity vector and forward direction in pitch plane
    let dot = vel_hat.dot(&fwd).clamp(-1.0, 1.0);
    let cross = fwd.cross(&vel_hat);
    let right = state.right_dir();
    let sign = cross.dot(&right).signum();
    sign * dot.acos()
}

/// Lift coefficient as a function of angle of attack.
/// Simple linear model with stall: Cl = 2π * α, clamped to ±CL_MAX.
fn lift_coefficient(alpha: f64) -> f64 {
    // For a delta wing, lift slope is lower than 2π due to low AR.
    // Effective lift slope ≈ π * AR / (1 + sqrt(1 + (AR/2)²))
    let lift_slope = std::f64::consts::PI * ASPECT_RATIO
        / (1.0 + (1.0 + (ASPECT_RATIO / 2.0).powi(2)).sqrt());
    (lift_slope * alpha).clamp(-CL_MAX, CL_MAX)
}

/// Drag coefficient from Cl using drag polar: Cd = Cd0 + Cl²/(π·AR·e).
fn drag_coefficient(cl: f64) -> f64 {
    CD0 + cl * cl / (std::f64::consts::PI * ASPECT_RATIO * OSWALD_E)
}

/// Advance the flight state by one physics timestep.
///
/// `thrust_n`: total thrust force (N) along forward direction.
/// `pitch_cmd`: commanded pitch angle (radians), smoothly tracked.
/// `bank_cmd`: commanded bank angle (radians), smoothly tracked.
/// `heading_rate`: heading rate from bank-to-turn (rad/s).
pub fn step(
    state: &mut FlightState,
    thrust_n: f64,
    pitch_cmd: f64,
    bank_cmd: f64,
) {
    let dt = PHYSICS_DT;
    let mass = state.total_mass();
    let speed = state.airspeed();
    let alt = state.altitude();
    let rho = air_density(alt);

    // --- Orientation update (smooth tracking of commanded angles) ---
    // Bank rate: ~60 deg/s max
    let bank_rate = 1.0_f64; // rad/s responsiveness
    let bank_error = bank_cmd - state.bank;
    state.bank += (bank_error * bank_rate * dt * 10.0).clamp(-bank_rate * dt, bank_rate * dt);

    // Pitch rate: ~30 deg/s max
    let pitch_rate = 0.5;
    let pitch_error = pitch_cmd - state.pitch;
    state.pitch += (pitch_error * pitch_rate * dt * 10.0).clamp(-pitch_rate * dt, pitch_rate * dt);

    // Bank-to-turn: heading rate proportional to bank angle and airspeed
    // Turn rate = g * tan(bank) / V for coordinated turn
    if speed > 5.0 {
        let g = 9.81;
        let turn_rate = g * state.bank.tan().clamp(-2.0, 2.0) / speed;
        state.heading += turn_rate * dt;
    }

    // --- Aerodynamic forces ---
    let alpha = angle_of_attack(state);
    let cl = lift_coefficient(alpha);
    let cd = drag_coefficient(cl);

    let q = 0.5 * rho * speed * speed; // dynamic pressure

    // Drag: opposes velocity
    let drag_mag = q * cd * S_WING;
    let drag = if speed > 0.1 {
        -state.velocity.normalize() * drag_mag
    } else {
        Vector3::zeros()
    };

    // Lift: perpendicular to velocity, in the pitch-up direction
    let lift_mag = q * cl * S_WING;
    let lift = if speed > 1.0 {
        let vel_hat = state.velocity / speed;
        let up = state.body_up();
        // Lift acts perpendicular to velocity in the plane of velocity and body up
        let lift_dir = vel_hat.cross(&state.right_dir());
        let lift_dir = if lift_dir.norm() > 0.01 {
            lift_dir.normalize()
        } else {
            up
        };
        lift_dir * lift_mag
    } else {
        Vector3::zeros()
    };

    // Thrust: along forward direction
    let thrust = state.forward_dir() * thrust_n;

    // Gravity
    let gravity = Vector3::new(0.0, 0.0, -9.81) * mass;

    // --- Integration ---
    let total_force = thrust + lift + drag + gravity;
    let accel = total_force / mass;
    state.velocity += accel * dt;
    state.position += state.velocity * dt;

    // Ground clamp — don't go below z=0
    if state.position.z < 0.0 {
        state.position.z = 0.0;
        if state.velocity.z < 0.0 {
            state.velocity.z = 0.0;
        }
    }

    // --- Fuel burn ---
    if thrust_n > 0.0 {
        // Scale fuel rate with thrust ratio
        let thrust_ratio = thrust_n / THRUST_CRUISE;
        let fuel_burn = FUEL_RATE_CRUISE * thrust_ratio.min(3.0) * dt;
        state.fuel_mass = (state.fuel_mass - fuel_burn).max(0.0);
    }

    // --- Prop animation ---
    // RPM proportional to thrust
    let rpm = if thrust_n > 0.0 { 5500.0 + (thrust_n / THRUST_CRUISE) * 2000.0 } else { 0.0 };
    state.prop_angle += rpm * 2.0 * std::f64::consts::PI / 60.0 * dt;
}
