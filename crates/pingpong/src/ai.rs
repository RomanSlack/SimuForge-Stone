//! AI controller: trajectory prediction, shot planning, swing execution.

use nalgebra::Vector3;

use crate::arm_config;
use crate::ball::{Ball, BALL_MASS, BALL_RADIUS, DRAG_CD, AIR_DENSITY, BALL_AREA, MAGNUS_CL};
use crate::table;

/// AI state machine phases.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AiPhase {
    /// Waiting at ready position.
    Ready,
    /// Tracking ball trajectory, positioning arm.
    Tracking,
    /// Executing swing to hit ball.
    Swinging,
    /// Returning to ready position after hit.
    Recovery,
}

/// AI difficulty parameters.
#[derive(Debug, Clone)]
pub struct AiDifficulty {
    /// Reaction delay (seconds).
    pub reaction_delay: f64,
    /// Aim noise (meters).
    pub aim_noise: f64,
    /// Swing speed multiplier (higher = faster swing = more ball speed on hit).
    pub swing_speed: f64,
}

impl AiDifficulty {
    pub fn expert() -> Self {
        Self {
            reaction_delay: 0.05,
            aim_noise: 0.01,
            swing_speed: 1.0,
        }
    }
}

/// Per-player AI controller.
#[derive(Debug, Clone)]
pub struct AiController {
    pub phase: AiPhase,
    pub difficulty: AiDifficulty,
    /// Which side of table this AI is on (negative X = P1, positive X = P2).
    pub side_sign: f64,
    /// Target position for the paddle in DH world space.
    pub target_position: Vector3<f64>,
    /// Time since ball was detected heading toward this player.
    pub tracking_time: f64,
    /// Predicted intercept point.
    pub intercept: Option<Vector3<f64>>,
    /// Ready position (DH world space).
    pub ready_position: Vector3<f64>,
    /// Swing progress (0 = wind-up, 0.5 = contact, 1.0 = follow-through).
    pub swing_progress: f64,
    /// Wind-up position (pulled back before swing).
    pub windup_position: Option<Vector3<f64>>,
    /// Follow-through position (pushed forward after contact).
    pub followthrough_position: Option<Vector3<f64>>,
    /// Simple noise seed.
    pub noise_seed: f64,
}

impl AiController {
    pub fn new(player_id: u8) -> Self {
        let side_sign = if player_id == 1 { -1.0 } else { 1.0 };
        let base_x = side_sign * arm_config::ARM_X_OFFSET;
        let ready = Vector3::new(
            base_x + side_sign * (-0.4),
            0.0,
            arm_config::TABLE_HEIGHT + 0.20,
        );

        Self {
            phase: AiPhase::Ready,
            difficulty: AiDifficulty::expert(),
            side_sign,
            target_position: ready,
            tracking_time: 0.0,
            intercept: None,
            ready_position: ready,
            swing_progress: 0.0,
            windup_position: None,
            followthrough_position: None,
            noise_seed: player_id as f64 * 17.3,
        }
    }

    /// Update AI state based on ball position and velocity.
    /// Returns the desired paddle position in DH world space.
    pub fn update(
        &mut self,
        ball: &Ball,
        dt: f64,
    ) -> Vector3<f64> {
        if !ball.active {
            self.phase = AiPhase::Ready;
            self.tracking_time = 0.0;
            self.intercept = None;
            self.windup_position = None;
            self.followthrough_position = None;
            self.target_position = self.ready_position;
            return self.target_position;
        }

        let ball_heading_toward_me = (ball.velocity.x * self.side_sign) > 0.0;
        let ball_on_my_side = (ball.position.x * self.side_sign) > 0.0
            || ball.position.x.abs() < 0.5;

        match self.phase {
            AiPhase::Ready => {
                if ball_heading_toward_me || ball_on_my_side {
                    self.tracking_time = 0.0;
                    self.phase = AiPhase::Tracking;
                }
                self.target_position = self.ready_position;
            }
            AiPhase::Tracking => {
                self.tracking_time += dt;

                if !ball_heading_toward_me && !ball_on_my_side {
                    self.phase = AiPhase::Recovery;
                    self.intercept = None;
                    return self.ready_position;
                }

                if self.tracking_time > self.difficulty.reaction_delay {
                    let intercept = predict_intercept(ball, self.side_sign);

                    if let Some(mut pt) = intercept {
                        // Add small aim noise
                        let t = self.tracking_time + self.noise_seed;
                        pt.y += self.difficulty.aim_noise * (t * 7.3).sin();
                        pt.z += self.difficulty.aim_noise * 0.5 * (t * 11.7).cos();

                        self.intercept = Some(pt);

                        // Move toward the WIND-UP position (pulled back from intercept)
                        let pullback = 0.18 * self.difficulty.swing_speed;
                        let windup = Vector3::new(
                            pt.x + self.side_sign * pullback, // pull back toward own base
                            pt.y,
                            pt.z + 0.04, // slightly above for downward swing
                        );
                        self.windup_position = Some(windup);

                        let followthrough = Vector3::new(
                            pt.x - self.side_sign * pullback * 1.5, // push through toward opponent
                            pt.y,
                            pt.z - 0.02, // follow through slightly down
                        );
                        self.followthrough_position = Some(followthrough);

                        // Smoothly move toward wind-up position
                        self.target_position = self.target_position * 0.85 + windup * 0.15;

                        // Start swing when ball gets close
                        let dist = (ball.position - pt).norm();
                        if dist < 0.5 {
                            self.phase = AiPhase::Swinging;
                            self.swing_progress = 0.0;
                        }
                    } else {
                        // Defensive position
                        let base_x = self.side_sign * arm_config::ARM_X_OFFSET;
                        let defensive = Vector3::new(
                            base_x + self.side_sign * (-0.3),
                            ball.position.y.clamp(-0.4, 0.4),
                            arm_config::TABLE_HEIGHT + 0.15,
                        );
                        self.target_position = self.target_position * 0.9 + defensive * 0.1;
                    }
                }
            }
            AiPhase::Swinging => {
                // The swing moves the paddle from wind-up THROUGH the intercept to follow-through.
                // This creates real paddle VELOCITY at the point of contact.
                let swing_duration = 0.20 / self.difficulty.swing_speed; // 200ms swing
                self.swing_progress += dt / swing_duration;

                if self.swing_progress > 1.0 || !ball.active {
                    self.phase = AiPhase::Recovery;
                    self.intercept = None;
                    self.windup_position = None;
                    self.followthrough_position = None;
                } else if let (Some(windup), Some(followthrough)) =
                    (self.windup_position, self.followthrough_position)
                {
                    // Smooth swing curve: ease-in-ease-out via smoothstep
                    let t = self.swing_progress;
                    let smooth_t = t * t * (3.0 - 2.0 * t); // smoothstep

                    // Interpolate from windup through to followthrough
                    self.target_position = windup * (1.0 - smooth_t) + followthrough * smooth_t;

                    // Also continuously update intercept for better tracking
                    if let Some(new_intercept) = predict_intercept(ball, self.side_sign) {
                        // Blend in updated intercept only slightly (don't jitter)
                        if let Some(old) = &mut self.intercept {
                            *old = *old * 0.8 + new_intercept * 0.2;
                        }
                    }
                } else {
                    // No wind-up/follow-through computed — fallback to intercept
                    if let Some(pt) = self.intercept {
                        self.target_position = pt;
                    }
                    self.phase = AiPhase::Recovery;
                }
            }
            AiPhase::Recovery => {
                self.tracking_time = 0.0;
                self.intercept = None;
                self.windup_position = None;
                self.followthrough_position = None;

                // If ball comes back, start tracking immediately
                if ball_heading_toward_me || ball_on_my_side {
                    self.phase = AiPhase::Tracking;
                    self.tracking_time = 0.0;
                    return self.target_position;
                }

                // Smoothly return to ready
                let blend = (3.0 * dt).min(1.0);
                self.target_position = self.target_position * (1.0 - blend) + self.ready_position * blend;

                if (self.target_position - self.ready_position).norm() < 0.01 {
                    self.phase = AiPhase::Ready;
                }
            }
        }

        self.target_position
    }
}

/// Forward-simulate ball trajectory to predict where it will cross the player's hitting zone.
fn predict_intercept(
    ball: &Ball,
    side_sign: f64,
) -> Option<Vector3<f64>> {
    let base_x = side_sign * arm_config::ARM_X_OFFSET;

    // Hitting zone: from 30% to 85% of the way from center to base
    let zone_start = base_x * 0.3;
    let zone_end = base_x * 0.85;

    let sim_dt = 0.005;
    let max_steps = 600; // 3 seconds

    let mut pos = ball.position;
    let mut vel = ball.velocity;
    let spin = ball.spin;

    for _ in 0..max_steps {
        let gravity = Vector3::new(0.0, 0.0, -9.81) * BALL_MASS;
        let speed = vel.norm();
        let drag = if speed > 1e-6 {
            -0.5 * DRAG_CD * AIR_DENSITY * BALL_AREA * speed * vel
        } else {
            Vector3::zeros()
        };
        let magnus = if speed > 1e-6 && spin.norm() > 1e-6 {
            let vol = (4.0 / 3.0) * std::f64::consts::PI * BALL_RADIUS.powi(3);
            MAGNUS_CL * vol * AIR_DENSITY * spin.cross(&vel)
        } else {
            Vector3::zeros()
        };

        let force = gravity + drag + magnus;
        vel += (force / BALL_MASS) * sim_dt;
        pos += vel * sim_dt;

        // Table bounce
        let table_z = table::TABLE_HEIGHT as f64;
        if pos.z - BALL_RADIUS <= table_z
            && pos.x.abs() <= table::TABLE_LENGTH as f64 / 2.0
            && pos.y.abs() <= table::TABLE_WIDTH as f64 / 2.0
            && vel.z < 0.0
        {
            pos.z = table_z + BALL_RADIUS;
            vel.z = -vel.z * 0.9;
            vel.x *= 0.95;
            vel.y *= 0.95;
        }

        // Floor
        if pos.z < 0.0 {
            return None;
        }

        // In zone check
        let in_zone = if side_sign > 0.0 {
            pos.x > zone_start && pos.x < zone_end
        } else {
            pos.x < zone_start && pos.x > zone_end
        };

        let good_height = pos.z > table_z - 0.05 && pos.z < table_z + 0.6;

        if in_zone && good_height {
            let max_reach = 0.55;
            let clamped = Vector3::new(
                pos.x.clamp(base_x - max_reach, base_x + max_reach),
                pos.y.clamp(-0.5, 0.5),
                pos.z.clamp(table_z, table_z + 0.5),
            );
            return Some(clamped);
        }
    }

    None
}
