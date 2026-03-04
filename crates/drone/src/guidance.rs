//! GPS/INS waypoint guidance and flight phase state machine.
//!
//! The guidance system commands pitch, bank, and thrust based on the current
//! flight phase and waypoint navigation.

use nalgebra::Vector3;

use crate::flight::{self, FlightState, THRUST_BOOSTER, THRUST_CRUISE};

/// Flight phases of the mission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightPhase {
    /// Sitting on launch rail, awaiting Space key.
    PreLaunch,
    /// RATO booster firing (0-3s after launch).
    Launch,
    /// Climbing to cruise altitude on prop only.
    Climb,
    /// Cruising toward target via waypoints.
    Cruise,
    /// Terminal dive toward target (< 1500m).
    Terminal,
    /// Impact — simulation complete.
    Impact,
}

impl FlightPhase {
    /// Display name for HUD.
    pub fn label(&self) -> &'static str {
        match self {
            Self::PreLaunch => "PRE-LAUNCH",
            Self::Launch => "LAUNCH",
            Self::Climb => "CLIMB",
            Self::Terminal => "TERMINAL",
            Self::Cruise => "CRUISE",
            Self::Impact => "IMPACT",
        }
    }

    /// Color for HUD phase label [r, g, b, a].
    pub fn color(&self) -> [f32; 4] {
        match self {
            Self::PreLaunch => [0.6, 0.6, 0.6, 1.0],
            Self::Launch => [1.0, 0.2, 0.2, 1.0],
            Self::Climb => [1.0, 1.0, 0.2, 1.0],
            Self::Cruise => [0.2, 1.0, 1.0, 1.0],
            Self::Terminal => [1.0, 0.5, 0.0, 1.0],
            Self::Impact => [1.0, 0.0, 0.0, 1.0],
        }
    }

    /// Trail line color for this phase.
    pub fn trail_color(&self) -> [f32; 4] {
        match self {
            Self::PreLaunch => [0.5, 0.5, 0.5, 1.0],
            Self::Launch => [1.0, 0.2, 0.2, 1.0],
            Self::Climb => [1.0, 1.0, 0.2, 1.0],
            Self::Cruise => [0.2, 1.0, 1.0, 1.0],
            Self::Terminal => [1.0, 0.5, 0.0, 1.0],
            Self::Impact => [1.0, 0.0, 0.0, 1.0],
        }
    }
}

/// Target position in the world.
pub const TARGET_POS: Vector3<f64> = Vector3::new(50_000.0, 0.0, 0.0);

/// Cruise altitude (m AGL).
const CRUISE_ALT: f64 = 300.0;

/// Booster burn duration (seconds).
const BOOSTER_DURATION: f64 = 3.0;

/// Distance to target at which terminal phase begins (m).
const TERMINAL_RANGE: f64 = 800.0;

/// Terminal dive pitch angle (radians, negative = nose down). Steep nose-dive.
const TERMINAL_PITCH: f64 = -55.0_f64 * std::f64::consts::PI / 180.0;

/// CEP (circular error probable) radius in meters — 50% of shots land within this radius.
/// Real Shahed-136 with commercial GPS/GLONASS CEP is estimated at 10-15m.
const CEP_RADIUS: f64 = 12.0;

/// Guidance state machine.
pub struct Guidance {
    /// Current flight phase.
    pub phase: FlightPhase,
    /// Time since launch (seconds).
    pub mission_time: f64,
    /// Waypoints (world coordinates, Z-up). Last waypoint is the target.
    pub waypoints: Vec<Vector3<f64>>,
    /// Current waypoint index.
    pub waypoint_idx: usize,
    /// Per-drone aim point (TARGET_POS + random CEP scatter).
    pub aim_point: Vector3<f64>,
}

impl Guidance {
    pub fn new() -> Self {
        // S-curve route with lateral avoidance maneuvers
        let waypoints = vec![
            Vector3::new(3_000.0, 1_500.0, CRUISE_ALT),     // W1: veer left after launch
            Vector3::new(10_000.0, -1_000.0, CRUISE_ALT),   // W2: hard right avoidance
            Vector3::new(20_000.0, 2_000.0, CRUISE_ALT),    // W3: left avoidance
            Vector3::new(35_000.0, -800.0, CRUISE_ALT),     // W4: right correction
            Vector3::new(45_000.0, 200.0, CRUISE_ALT),      // W5: line up for terminal
            Vector3::new(TARGET_POS.x, TARGET_POS.y, TARGET_POS.z + 3.0),
        ];
        let (dx, dy) = random_cep_offset(0);
        let aim_point = Vector3::new(TARGET_POS.x + dx, TARGET_POS.y + dy, TARGET_POS.z);
        Self {
            phase: FlightPhase::PreLaunch,
            mission_time: 0.0,
            waypoints,
            waypoint_idx: 0,
            aim_point,
        }
    }

    /// Create guidance from a custom launch position, generating S-curve waypoints toward the target.
    pub fn new_from(launch_pos: Vector3<f64>) -> Self {
        let to_target = TARGET_POS - launch_pos;
        let dist = (to_target.x * to_target.x + to_target.y * to_target.y).sqrt();
        let dir = Vector3::new(to_target.x / dist, to_target.y / dist, 0.0);
        let perp = Vector3::new(-dir.y, dir.x, 0.0);

        let fractions = [0.06, 0.20, 0.40, 0.70, 0.90];
        let offsets = [1500.0, -1000.0, 2000.0, -800.0, 200.0];

        let mut waypoints = Vec::with_capacity(6);
        for i in 0..5 {
            let along = launch_pos + dir * dist * fractions[i];
            let lateral = perp * offsets[i];
            waypoints.push(Vector3::new(
                along.x + lateral.x,
                along.y + lateral.y,
                CRUISE_ALT,
            ));
        }
        waypoints.push(Vector3::new(TARGET_POS.x, TARGET_POS.y, TARGET_POS.z + 3.0));

        // Unique seed from launch position for reproducible scatter
        let seed = (launch_pos.x as u64).wrapping_mul(2654435761)
            ^ (launch_pos.y as u64).wrapping_mul(2246822519);
        let (dx, dy) = random_cep_offset(seed);
        let aim_point = Vector3::new(TARGET_POS.x + dx, TARGET_POS.y + dy, TARGET_POS.z);

        Self {
            phase: FlightPhase::PreLaunch,
            mission_time: 0.0,
            waypoints,
            waypoint_idx: 0,
            aim_point,
        }
    }

    /// Begin the launch sequence.
    pub fn launch(&mut self) {
        if self.phase == FlightPhase::PreLaunch {
            self.phase = FlightPhase::Launch;
            self.mission_time = 0.0;
        }
    }

    /// Distance to final target (m).
    pub fn distance_to_target(&self, pos: &Vector3<f64>) -> f64 {
        let dx = TARGET_POS.x - pos.x;
        let dy = TARGET_POS.y - pos.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Update guidance and return (thrust, pitch_cmd, bank_cmd).
    /// `ground_height`: terrain elevation (DH Z) directly below the drone. Used for
    /// terrain-relative altitude hold during climb and cruise.
    pub fn update(&mut self, state: &FlightState, ground_height: f64) -> (f64, f64, f64) {
        if self.phase == FlightPhase::PreLaunch || self.phase == FlightPhase::Impact {
            return (0.0, state.pitch, 0.0);
        }

        self.mission_time += flight::PHYSICS_DT;

        let dist_to_target = self.distance_to_target(&state.position);
        let agl = state.position.z - ground_height;

        match self.phase {
            FlightPhase::Launch => {
                // RATO booster + prop, fixed pitch, no steering
                let thrust = THRUST_BOOSTER + THRUST_CRUISE;
                let pitch_cmd = 15.0_f64.to_radians();
                let bank_cmd = 0.0;

                if self.mission_time > BOOSTER_DURATION {
                    self.phase = FlightPhase::Climb;
                }

                (thrust, pitch_cmd, bank_cmd)
            }

            FlightPhase::Climb => {
                let thrust = THRUST_CRUISE;
                let pitch_cmd = 10.0_f64.to_radians();
                let bank_cmd = 0.0;

                if agl >= CRUISE_ALT * 0.9 {
                    self.phase = FlightPhase::Cruise;
                }

                (thrust, pitch_cmd, bank_cmd)
            }

            FlightPhase::Cruise => {
                // Check for terminal phase
                if dist_to_target < TERMINAL_RANGE {
                    self.phase = FlightPhase::Terminal;
                    return self.update(state, ground_height);
                }

                let thrust = THRUST_CRUISE;

                // Navigate to current waypoint
                let wp = self.waypoints[self.waypoint_idx];
                let to_wp = wp - state.position;
                let horiz_dist = (to_wp.x * to_wp.x + to_wp.y * to_wp.y).sqrt();

                // Advance waypoint when close enough (but not the last one)
                if horiz_dist < 500.0 && self.waypoint_idx < self.waypoints.len() - 1 {
                    self.waypoint_idx += 1;
                }

                let wp = self.waypoints[self.waypoint_idx];
                let to_wp = wp - state.position;

                // Heading error
                let desired_heading = to_wp.y.atan2(to_wp.x);
                let heading_error = angle_diff(desired_heading, state.heading);

                // Bank-to-turn: proportional to heading error
                let kp_heading = 2.0;
                let bank_cmd = (kp_heading * heading_error).clamp(
                    -30.0_f64.to_radians(),
                    30.0_f64.to_radians(),
                );

                // Terrain-relative altitude hold: maintain CRUISE_ALT above ground
                let target_z = ground_height + CRUISE_ALT;
                let alt_error = target_z - state.position.z;
                let kp_alt = 0.02;
                let pitch_cmd = (kp_alt * alt_error).clamp(
                    -10.0_f64.to_radians(),
                    10.0_f64.to_radians(),
                );

                (thrust, pitch_cmd, bank_cmd)
            }

            FlightPhase::Terminal => {
                // Terrain-aware impact is handled in the main loop.
                // Fallback only at extreme depth (well below any terrain).
                if state.position.z <= -200.0 {
                    self.phase = FlightPhase::Impact;
                    return (0.0, state.pitch, 0.0);
                }

                let thrust = THRUST_CRUISE;

                // Heading to aim point — high gain + wider bank for terminal accuracy
                let to_target = self.aim_point - state.position;
                let desired_heading = to_target.y.atan2(to_target.x);
                let heading_error = angle_diff(desired_heading, state.heading);
                let bank_cmd = (4.0 * heading_error).clamp(
                    -25.0_f64.to_radians(),
                    25.0_f64.to_radians(),
                );

                // Linear dive onset reaching -55° at target
                let dive_fraction = (1.0 - dist_to_target / TERMINAL_RANGE).clamp(0.0, 1.0);
                let pitch_cmd = TERMINAL_PITCH * dive_fraction;

                (thrust, pitch_cmd, bank_cmd)
            }

            FlightPhase::PreLaunch | FlightPhase::Impact => unreachable!(),
        }
    }

    /// Whether the phase should auto-slow time (launch or terminal).
    pub fn should_auto_slow(&self) -> bool {
        matches!(self.phase, FlightPhase::Launch | FlightPhase::Terminal)
    }
}

/// Generate a random aim point offset using Box-Muller transform on a hash seed.
/// Returns (dx, dy) in meters following a Rayleigh distribution with the given CEP.
fn random_cep_offset(seed: u64) -> (f64, f64) {
    // Two independent hash values in [0, 1)
    let h1 = {
        let mut s = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        s = (s ^ (s >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        s = (s ^ (s >> 27)).wrapping_mul(0x94d049bb133111eb);
        s = s ^ (s >> 31);
        (s as f64) / (u64::MAX as f64)
    };
    let h2 = {
        let mut s = seed.wrapping_add(7).wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        s = (s ^ (s >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        s = (s ^ (s >> 27)).wrapping_mul(0x94d049bb133111eb);
        s = s ^ (s >> 31);
        (s as f64) / (u64::MAX as f64)
    };
    // Box-Muller: two uniform → two normal
    let u1 = h1.max(1e-10); // avoid log(0)
    let r = (-2.0 * u1.ln()).sqrt();
    let theta = 2.0 * std::f64::consts::PI * h2;
    // CEP → sigma: for 2D Gaussian, CEP = sigma * 1.1774
    let sigma = CEP_RADIUS / 1.1774;
    (r * theta.cos() * sigma, r * theta.sin() * sigma)
}

/// Normalize angle difference to [-π, π].
fn angle_diff(target: f64, current: f64) -> f64 {
    let d = target - current;
    let pi = std::f64::consts::PI;
    ((d + pi) % (2.0 * pi) + 2.0 * pi) % (2.0 * pi) - pi
}
