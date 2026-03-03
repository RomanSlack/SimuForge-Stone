//! Aerodynamic point-mass flight model for a delta-wing loitering munition.
//!
//! Physics runs at 200 Hz. The model computes lift, drag, thrust, and gravity,
//! then integrates position and velocity. Orientation (heading, pitch, bank)
//! is updated by the guidance system's commands.
//!
//! All aerodynamic forces are computed from **air-relative velocity** (velocity − wind),
//! which is the physically correct formulation.

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
/// Wingspan (m) — for ground effect calculation.
pub const WINGSPAN: f64 = 2.5;

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

/// Ground friction coefficient (concrete/sand).
const MU_GROUND: f64 = 0.4;

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

    /// True airspeed relative to the wind (m/s).
    pub fn true_airspeed(&self, wind: &Vector3<f64>) -> f64 {
        (self.velocity - wind).norm()
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

/// Compute angle of attack from airspeed vector and drone pitch.
/// Returns alpha in radians.
fn angle_of_attack(state: &FlightState, airspeed_vec: &Vector3<f64>) -> f64 {
    let speed = airspeed_vec.norm();
    if speed < 1.0 {
        return 0.0;
    }
    let vel_hat = airspeed_vec / speed;
    let fwd = state.forward_dir();
    let dot = vel_hat.dot(&fwd).clamp(-1.0, 1.0);
    let cross = fwd.cross(&vel_hat);
    let right = state.right_dir();
    let sign = cross.dot(&right).signum();
    sign * dot.acos()
}

/// Lift coefficient as a function of angle of attack.
fn lift_coefficient(alpha: f64) -> f64 {
    let lift_slope = std::f64::consts::PI * ASPECT_RATIO
        / (1.0 + (1.0 + (ASPECT_RATIO / 2.0).powi(2)).sqrt());
    (lift_slope * alpha).clamp(-CL_MAX, CL_MAX)
}

/// Drag coefficient from Cl using drag polar: Cd = Cd0 + Cl²/(π·AR·e).
fn drag_coefficient(cl: f64) -> f64 {
    CD0 + cl * cl / (std::f64::consts::PI * ASPECT_RATIO * OSWALD_E)
}

/// McCormick ground effect factor: reduces induced drag near the ground.
/// Returns a multiplier < 1.0 when altitude < wingspan.
fn ground_effect_factor(altitude: f64) -> f64 {
    if altitude >= WINGSPAN {
        1.0
    } else {
        let h_ratio = (altitude / WINGSPAN).clamp(0.0, 1.0);
        // 50% drag reduction at ground level, linearly increasing to 0% at wingspan height
        1.0 - (1.0 - h_ratio) * 0.5
    }
}

/// Advance the flight state by one physics timestep.
///
/// `thrust_n`: total thrust force (N) along forward direction.
/// `pitch_cmd`: commanded pitch angle (radians), smoothly tracked.
/// `bank_cmd`: commanded bank angle (radians), smoothly tracked.
/// `wind`: wind velocity vector in world frame (m/s).
pub fn step(
    state: &mut FlightState,
    thrust_n: f64,
    pitch_cmd: f64,
    bank_cmd: f64,
    wind: &Vector3<f64>,
) {
    let dt = PHYSICS_DT;
    let mass = state.total_mass();
    let alt = state.altitude();
    let rho = air_density(alt);

    // --- Air-relative velocity (all aero forces use this) ---
    let airspeed_vec = state.velocity - wind;
    let airspeed = airspeed_vec.norm();

    // --- Orientation update (smooth tracking of commanded angles) ---
    let bank_rate = 1.0_f64;
    let bank_error = bank_cmd - state.bank;
    state.bank += (bank_error * bank_rate * dt * 10.0).clamp(-bank_rate * dt, bank_rate * dt);

    let pitch_rate = 0.5;
    let pitch_error = pitch_cmd - state.pitch;
    state.pitch += (pitch_error * pitch_rate * dt * 10.0).clamp(-pitch_rate * dt, pitch_rate * dt);

    // Bank-to-turn: heading rate proportional to bank angle and airspeed
    if airspeed > 5.0 {
        let g = 9.81;
        let turn_rate = g * state.bank.tan().clamp(-2.0, 2.0) / airspeed;
        state.heading += turn_rate * dt;
    }

    // --- Aerodynamic forces (computed from air-relative velocity) ---
    let alpha = angle_of_attack(state, &airspeed_vec);
    let cl = lift_coefficient(alpha);
    let cd_base = drag_coefficient(cl);

    // Apply ground effect to induced drag portion only
    let cd_induced = cd_base - CD0;
    let cd = CD0 + cd_induced * ground_effect_factor(alt);

    let q = 0.5 * rho * airspeed * airspeed; // dynamic pressure

    // Drag: opposes air-relative velocity
    let drag = if airspeed > 0.1 {
        -(airspeed_vec / airspeed) * q * cd * S_WING
    } else {
        Vector3::zeros()
    };

    // Lift: perpendicular to air-relative velocity
    let lift = if airspeed > 1.0 {
        let vel_hat = airspeed_vec / airspeed;
        let lift_dir = vel_hat.cross(&state.right_dir());
        let lift_dir = if lift_dir.norm() > 0.01 {
            lift_dir.normalize()
        } else {
            state.body_up()
        };
        lift_dir * q * cl * S_WING
    } else {
        Vector3::zeros()
    };

    // Thrust: along forward direction
    let thrust = state.forward_dir() * thrust_n;

    // Gravity
    let gravity = Vector3::new(0.0, 0.0, -9.81) * mass;

    // --- Ground friction (for sliding phase) ---
    let ground_speed = state.velocity.norm();
    let friction = if state.position.z < 0.5 && ground_speed > 0.1 {
        let horiz_vel = Vector3::new(state.velocity.x, state.velocity.y, 0.0);
        let horiz_speed = horiz_vel.norm();
        if horiz_speed > 0.1 {
            let friction_mag = MU_GROUND * mass * 9.81;
            // Clamp so friction doesn't reverse velocity in one timestep
            let max_decel = horiz_speed * mass / dt;
            let f = friction_mag.min(max_decel);
            -(horiz_vel / horiz_speed) * f
        } else {
            Vector3::zeros()
        }
    } else {
        Vector3::zeros()
    };

    // --- Integration ---
    let total_force = thrust + lift + drag + gravity + friction;
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
        let thrust_ratio = thrust_n / THRUST_CRUISE;
        let fuel_burn = FUEL_RATE_CRUISE * thrust_ratio.min(3.0) * dt;
        state.fuel_mass = (state.fuel_mass - fuel_burn).max(0.0);
    }

    // --- Prop animation ---
    let rpm = if thrust_n > 0.0 { 5500.0 + (thrust_n / THRUST_CRUISE) * 2000.0 } else { 0.0 };
    state.prop_angle += rpm * 2.0 * std::f64::consts::PI / 60.0 * dt;
}
