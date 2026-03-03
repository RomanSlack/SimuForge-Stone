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
const TERMINAL_RANGE: f64 = 1500.0;

/// Terminal dive pitch angle (radians, negative = nose down). Steep nose-dive.
const TERMINAL_PITCH: f64 = -55.0_f64 * std::f64::consts::PI / 180.0;

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
}

impl Guidance {
    pub fn new() -> Self {
        // Straight-line mission: a few intermediate waypoints for visual interest
        let waypoints = vec![
            Vector3::new(5_000.0, 0.0, CRUISE_ALT),
            Vector3::new(15_000.0, 500.0, CRUISE_ALT),
            Vector3::new(30_000.0, -300.0, CRUISE_ALT),
            Vector3::new(45_000.0, 100.0, CRUISE_ALT),
            Vector3::new(TARGET_POS.x, TARGET_POS.y, TARGET_POS.z + 3.0), // aim at building center
        ];
        Self {
            phase: FlightPhase::PreLaunch,
            mission_time: 0.0,
            waypoints,
            waypoint_idx: 0,
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
    pub fn update(&mut self, state: &FlightState) -> (f64, f64, f64) {
        if self.phase == FlightPhase::PreLaunch || self.phase == FlightPhase::Impact {
            return (0.0, state.pitch, 0.0);
        }

        self.mission_time += flight::PHYSICS_DT;

        let dist_to_target = self.distance_to_target(&state.position);

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

                if state.altitude() >= CRUISE_ALT * 0.9 {
                    self.phase = FlightPhase::Cruise;
                }

                (thrust, pitch_cmd, bank_cmd)
            }

            FlightPhase::Cruise => {
                // Check for terminal phase
                if dist_to_target < TERMINAL_RANGE {
                    self.phase = FlightPhase::Terminal;
                    return self.update(state);
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

                // Altitude hold via pitch
                let alt_error = CRUISE_ALT - state.altitude();
                let kp_alt = 0.02;
                let pitch_cmd = (kp_alt * alt_error).clamp(
                    -10.0_f64.to_radians(),
                    10.0_f64.to_radians(),
                );

                (thrust, pitch_cmd, bank_cmd)
            }

            FlightPhase::Terminal => {
                // Impact: nose-dive into ground
                if state.position.z <= 0.5 {
                    self.phase = FlightPhase::Impact;
                    return (0.0, state.pitch, 0.0);
                }

                let thrust = THRUST_CRUISE;

                // Heading to target
                let to_target = TARGET_POS - state.position;
                let desired_heading = to_target.y.atan2(to_target.x);
                let heading_error = angle_diff(desired_heading, state.heading);
                let bank_cmd = (2.0 * heading_error).clamp(
                    -15.0_f64.to_radians(),
                    15.0_f64.to_radians(),
                );

                // Progressive steep dive — faster onset with sqrt, reaching -55° at target
                let dive_fraction = (1.0 - dist_to_target / TERMINAL_RANGE).clamp(0.0, 1.0);
                let pitch_cmd = TERMINAL_PITCH * dive_fraction.powf(0.5);

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

/// Normalize angle difference to [-π, π].
fn angle_diff(target: f64, current: f64) -> f64 {
    let d = target - current;
    let pi = std::f64::consts::PI;
    ((d + pi) % (2.0 * pi) + 2.0 * pi) % (2.0 * pi) - pi
}
