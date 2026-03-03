//! AI controller: reach-through return.
//!
//! No swing arcs, no windup, no followthrough. The arm just reaches to where
//! the ball will be, and as the ball gets close, the target shifts forward
//! (toward the opponent) so the arm is still moving through when contact
//! happens. That forward motion IS the "swing." Dead simple for the IK.
//!
//! The paddle targets 2cm below the ball so the collision normal tilts
//! upward for net clearance.

use nalgebra::Vector3;

use crate::arm_config;
use crate::ball::{Ball, BALL_MASS, BALL_RADIUS, DRAG_CD, AIR_DENSITY, BALL_AREA, MAGNUS_CL};
use crate::table;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AiPhase {
    Ready,
    Tracking,
    Recovery,
}

#[derive(Debug, Clone)]
pub struct AiDifficulty {
    pub reaction_delay: f64,
    pub aim_noise: f64,
    pub swing_speed: f64,
}

impl AiDifficulty {
    pub fn expert() -> Self {
        Self {
            reaction_delay: 0.03,
            aim_noise: 0.005,
            swing_speed: 1.2,
        }
    }
}

/// Paddle targets this far below the ball for upward collision-normal tilt.
const PADDLE_Z_OFFSET: f64 = -0.015;
// No windup, no drive phases. The arm just goes to the intercept.
// Forward paddle velocity comes naturally from the arm reaching toward
// the intercept from its ready position.

#[derive(Debug, Clone)]
pub struct AiController {
    pub phase: AiPhase,
    pub difficulty: AiDifficulty,
    pub side_sign: f64,
    pub target_position: Vector3<f64>,
    pub tracking_time: f64,
    pub intercept: Option<Vector3<f64>>,
    pub time_to_intercept: f64,
    pub ready_position: Vector3<f64>,
    pub noise_seed: f64,
    // keep these fields so main.rs doesn't break (swing phase removed)
    pub swing_progress: f64,
    pub windup_position: Option<Vector3<f64>>,
    pub followthrough_position: Option<Vector3<f64>>,
}

impl AiController {
    pub fn new(player_id: u8) -> Self {
        let side_sign = if player_id == 1 { -1.0 } else { 1.0 };
        let base_x = side_sign * arm_config::ARM_X_OFFSET;
        let ready = Vector3::new(
            base_x + side_sign * (-0.4),
            0.0,
            arm_config::TABLE_HEIGHT + 0.25,
        );

        Self {
            phase: AiPhase::Ready,
            difficulty: AiDifficulty::expert(),
            side_sign,
            target_position: ready,
            tracking_time: 0.0,
            intercept: None,
            time_to_intercept: f64::MAX,
            ready_position: ready,
            noise_seed: player_id as f64 * 17.3,
            swing_progress: 0.0,
            windup_position: None,
            followthrough_position: None,
        }
    }

    pub fn update(&mut self, ball: &Ball, dt: f64) -> Vector3<f64> {
        if !ball.active {
            self.phase = AiPhase::Ready;
            self.tracking_time = 0.0;
            self.intercept = None;
            self.time_to_intercept = f64::MAX;
            self.target_position = self.ready_position;
            return self.target_position;
        }

        let ball_toward_me = (ball.velocity.x * self.side_sign) > 0.0;
        let ball_on_my_side = (ball.position.x * self.side_sign) > 0.0
            || ball.position.x.abs() < 0.3;

        match self.phase {
            AiPhase::Ready => {
                if ball_toward_me || ball_on_my_side {
                    self.tracking_time = 0.0;
                    self.phase = AiPhase::Tracking;
                }
                let mut ready = self.ready_position;
                if ball.active {
                    let blend = (2.0 * dt).min(1.0);
                    ready.y = self.target_position.y * (1.0 - blend)
                        + ball.position.y.clamp(-0.6, 0.6) * blend;
                }
                self.target_position = ready;
            }
            AiPhase::Tracking => {
                self.tracking_time += dt;

                if !ball_toward_me && !ball_on_my_side {
                    self.phase = AiPhase::Recovery;
                    self.intercept = None;
                    return self.ready_position;
                }

                if self.tracking_time > self.difficulty.reaction_delay {
                    if let Some((pt, arrival_time)) = predict_intercept(ball, self.side_sign) {
                        self.intercept = Some(pt);
                        self.time_to_intercept = arrival_time;

                        let target_z = pt.z + PADDLE_Z_OFFSET;

                        if arrival_time < 0.15 {
                            // DRIVE: rush to intercept. Arm still moving = paddle velocity.
                            self.target_position = Vector3::new(pt.x, pt.y, target_z);
                        } else {
                            // WINDUP: wait 12cm behind intercept (toward base).
                            // Clamp to stay within arm workspace.
                            let base_x = self.side_sign * arm_config::ARM_X_OFFSET;
                            let behind_x = pt.x + self.side_sign * 0.12;
                            let clamped_x = behind_x.clamp(
                                base_x - 0.50, base_x + 0.50,
                            );
                            self.target_position = Vector3::new(clamped_x, pt.y, target_z);
                        }
                    } else {
                        // No intercept — defensive position
                        let base_x = self.side_sign * arm_config::ARM_X_OFFSET;
                        self.target_position = Vector3::new(
                            base_x + self.side_sign * (-0.3),
                            ball.position.y.clamp(-0.8, 0.8),
                            arm_config::TABLE_HEIGHT + 0.20,
                        );
                    }
                }
            }
            AiPhase::Recovery => {
                self.tracking_time = 0.0;
                self.intercept = None;
                self.time_to_intercept = f64::MAX;

                if ball_toward_me || ball_on_my_side {
                    self.phase = AiPhase::Tracking;
                    self.tracking_time = 0.0;
                    return self.target_position;
                }

                let blend = (5.0 * dt).min(1.0);
                self.target_position =
                    self.target_position * (1.0 - blend) + self.ready_position * blend;
                if (self.target_position - self.ready_position).norm() < 0.02 {
                    self.phase = AiPhase::Ready;
                }
            }
        }

        self.target_position
    }
}

// ── Intercept prediction ─────────────────────────────────────────────────────

fn predict_intercept(ball: &Ball, side_sign: f64) -> Option<(Vector3<f64>, f64)> {
    let base_x = side_sign * arm_config::ARM_X_OFFSET;
    // Hit zone in the comfortable MIDDLE of the arm's workspace.
    // Leaves room behind the intercept for the windup drive.
    let hit_reach = 0.40; // 40cm forward, not the max 55cm
    let zone_lo = (base_x - side_sign * hit_reach).min(base_x - side_sign * 0.08);
    let zone_hi = (base_x - side_sign * hit_reach).max(base_x - side_sign * 0.08);

    let sim_dt = 0.004;
    let max_steps = 750;

    let mut pos = ball.position;
    let mut vel = ball.velocity;
    let spin = ball.spin;

    let table_z = table::TABLE_HEIGHT as f64;
    let half_len = table::TABLE_LENGTH as f64 / 2.0;
    let half_wid = table::TABLE_WIDTH as f64 / 2.0;

    let mut bounced = ball.last_bounce_x * side_sign > 0.0;

    for step in 0..max_steps {
        let t = step as f64 * sim_dt;

        let speed = vel.norm();
        let gravity = Vector3::new(0.0, 0.0, -9.81 * BALL_MASS);
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

        vel += ((gravity + drag + magnus) / BALL_MASS) * sim_dt;
        pos += vel * sim_dt;

        if pos.z - BALL_RADIUS <= table_z
            && pos.x.abs() <= half_len
            && pos.y.abs() <= half_wid
            && vel.z < 0.0
        {
            if pos.x * side_sign > 0.0 {
                bounced = true;
            }
            pos.z = table_z + BALL_RADIUS;
            vel.z = -vel.z * 0.9;
            vel.x *= 0.95;
            vel.y *= 0.95;
        }

        if pos.z < 0.0 { break; }
        if !bounced { continue; }

        let h = pos.z - table_z;
        if pos.x > zone_lo && pos.x < zone_hi && h > 0.02 && h < 0.55 {
            let clamped = Vector3::new(
                pos.x.clamp(base_x - hit_reach, base_x + hit_reach),
                pos.y.clamp(-0.8, 0.8),
                pos.z.clamp(table_z + 0.02, table_z + 0.50),
            );
            return Some((clamped, t));
        }
    }

    None
}
