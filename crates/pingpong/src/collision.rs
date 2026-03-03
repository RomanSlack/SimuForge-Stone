//! Collision detection and response: ball-table, ball-paddle, ball-net, ball-floor.
//!
//! Clean, deterministic physics. No random variation — predictable bounces
//! that the AI can reliably plan around.

use nalgebra::Vector3;

use crate::ball::{Ball, BALL_INERTIA, BALL_MASS, BALL_RADIUS};
use crate::table;

/// Coefficient of restitution for each surface.
pub const COR_TABLE: f64 = 0.95;
pub const COR_PADDLE: f64 = 0.95;
pub const COR_NET: f64 = 0.30;
pub const COR_FLOOR: f64 = 0.80;

/// Friction coefficients.
pub const FRICTION_TABLE: f64 = 0.10;
pub const FRICTION_PADDLE: f64 = 0.10;
pub const FRICTION_FLOOR: f64 = 0.35;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionEvent {
    Table,
    Paddle { player: u8 },
    Net,
    Floor,
}

#[derive(Debug, Clone)]
pub struct PaddleState {
    pub position: Vector3<f64>,
    pub normal: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub player: u8,
}

/// Check and resolve all collisions for the ball.
pub fn resolve_collisions(
    ball: &mut Ball,
    paddles: &[PaddleState],
) -> Vec<CollisionEvent> {
    let mut events = Vec::new();
    if !ball.active {
        return events;
    }

    let r = BALL_RADIUS;

    // ── Ball-table collision ─────────────────────────────────────────────
    let table_top_z = table::TABLE_HEIGHT as f64;
    let half_len = table::TABLE_LENGTH as f64 / 2.0;
    let half_wid = table::TABLE_WIDTH as f64 / 2.0;

    if ball.position.x.abs() <= half_len
        && ball.position.y.abs() <= half_wid
        && ball.position.z - r <= table_top_z
        && ball.velocity.z < 0.0
    {
        let impact_speed = ball.velocity.z.abs();
        ball.position.z = table_top_z + r;
        ball.last_bounce_x = ball.position.x;

        if impact_speed < 0.05 {
            ball.velocity.z = 0.0;
            let horiz = (ball.velocity.x.powi(2) + ball.velocity.y.powi(2)).sqrt();
            if horiz > 0.01 {
                let factor = (1.0 - 0.8 * 0.001 / horiz).max(0.0);
                ball.velocity.x *= factor;
                ball.velocity.y *= factor;
            } else {
                ball.velocity.x = 0.0;
                ball.velocity.y = 0.0;
            }
        } else {
            let normal = Vector3::new(0.0, 0.0, 1.0);
            apply_bounce(ball, &normal, COR_TABLE, FRICTION_TABLE);
            events.push(CollisionEvent::Table);
        }
    }

    // ── Ball-net collision ───────────────────────────────────────────────
    let net_top = table_top_z + table::NET_HEIGHT as f64;
    if ball.position.z <= net_top
        && ball.position.z >= table_top_z
        && ball.position.y.abs() <= table::NET_LENGTH as f64 / 2.0
        && ball.position.x.abs() <= r + table::NET_THICKNESS as f64 / 2.0
    {
        let sign = ball.position.x.signum();
        ball.velocity.x = -ball.velocity.x * COR_NET;
        ball.velocity.z *= 0.8;
        ball.spin *= 0.3;
        ball.position.x = sign * (r + table::NET_THICKNESS as f64 / 2.0 + 0.001);
        events.push(CollisionEvent::Net);
    }

    // ── Ball-paddle collisions ───────────────────────────────────────────
    for paddle in paddles {
        let to_ball = ball.position - paddle.position;
        let dist = to_ball.norm();
        let paddle_r = crate::arm_config::PADDLE_RADIUS;

        if dist < r + paddle_r {
            // Use a fixed collision normal: horizontal toward opponent + 15° upward tilt.
            // This gives the ball a consistent upward kick for net clearance,
            // regardless of the arm's actual configuration.
            // The 15° tilt gives ~26% of the impulse as vertical lift.
            // Fixed collision normal: toward opponent + 25° upward tilt.
            // NEVER flip based on ball position — that causes a normal-flip
            // race condition where the ball tunnels through the paddle.
            let side = if paddle.player == 1 { 1.0 } else { -1.0 }; // toward opponent
            let tilt = 0.37_f64; // sin(22°) ≈ 0.37, cos(22°) ≈ 0.93
            let normal = Vector3::new(side * (1.0 - tilt * tilt).sqrt(), 0.0, tilt);

            let v_rel = ball.velocity - paddle.velocity;
            let v_n = v_rel.dot(&normal);

            if v_n < 0.0 {
                // Normal impulse (COR reflection)
                let j_n = -(1.0 + COR_PADDLE) * v_n * BALL_MASS;
                ball.velocity += normal * (j_n / BALL_MASS);

                // Tangential friction
                let v_t = v_rel - normal * v_n;
                let v_t_mag = v_t.norm();
                if v_t_mag > 1e-6 {
                    let t_dir = v_t / v_t_mag;
                    let j_t = (-FRICTION_PADDLE * j_n.abs()).max(-v_t_mag * BALL_MASS);
                    ball.velocity += t_dir * (j_t / BALL_MASS);

                    // Spin from friction
                    let r_vec = -normal * r;
                    let delta_omega = r_vec.cross(&(t_dir * j_t)) / BALL_INERTIA;
                    ball.spin += delta_omega;
                }

                // Push ball out of paddle
                let overlap = r + paddle_r - dist;
                if overlap > 0.0 {
                    ball.position += normal * overlap;
                }

                events.push(CollisionEvent::Paddle {
                    player: paddle.player,
                });
            }
        }
    }

    // ── Ball-floor collision ─────────────────────────────────────────────
    if ball.position.z - r <= 0.0 && ball.velocity.z < 0.0 {
        let impact_speed = ball.velocity.z.abs();
        ball.position.z = r;

        if impact_speed < 0.05 {
            ball.velocity = Vector3::zeros();
            ball.active = false;
        } else {
            let normal = Vector3::new(0.0, 0.0, 1.0);
            apply_bounce(ball, &normal, COR_FLOOR, FRICTION_FLOOR);
            ball.floor_bounces += 1;
            if ball.floor_bounces >= 3 {
                ball.active = false;
            }
            events.push(CollisionEvent::Floor);
        }
    }

    // ── Spin decay (air resistance on rotation) ──────────────────────────
    // Prevents spin from accumulating to insane levels over a rally.
    let spin_mag = ball.spin.norm();
    if spin_mag > 1.0 {
        // Exponential decay: lose ~10% per second at low spin, more at high spin
        let decay = (-0.1 * 0.001 * spin_mag.sqrt()).exp(); // per physics step
        ball.spin *= decay;
    }

    events
}

/// Apply a bounce off a surface with given normal, COR, and friction.
fn apply_bounce(ball: &mut Ball, normal: &Vector3<f64>, cor: f64, friction: f64) {
    let v_n = ball.velocity.dot(normal);
    let v_t = ball.velocity - normal * v_n;

    // Reflect normal component with COR
    ball.velocity = v_t - normal * (v_n * cor);

    // Friction: reduce tangential velocity
    let v_t_mag = v_t.norm();
    if v_t_mag > 1e-6 {
        let t_dir = v_t / v_t_mag;
        let friction_impulse = friction * v_n.abs() * BALL_MASS;
        let max_friction = v_t_mag * BALL_MASS;
        let j_t = friction_impulse.min(max_friction);

        ball.velocity -= t_dir * (j_t / BALL_MASS);

        // Spin from surface friction
        let r_vec = -normal * BALL_RADIUS;
        let delta_omega = r_vec.cross(&(t_dir * j_t)) / BALL_INERTIA;
        ball.spin += delta_omega;
    }
}
