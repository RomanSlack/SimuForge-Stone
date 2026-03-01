//! Collision detection and response: ball-table, ball-paddle, ball-net, ball-floor.

use nalgebra::Vector3;

use crate::ball::{Ball, BALL_INERTIA, BALL_MASS, BALL_RADIUS};
use crate::table;

/// Base coefficient of restitution for each surface.
pub const COR_TABLE: f64 = 0.90;
pub const COR_PADDLE: f64 = 0.85;
pub const COR_NET: f64 = 0.30;
pub const COR_FLOOR: f64 = 0.85;

/// Friction coefficients.
pub const FRICTION_TABLE: f64 = 0.35;
pub const FRICTION_PADDLE: f64 = 0.60;
pub const FRICTION_FLOOR: f64 = 0.40;

/// Collision event type for audio/scoring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionEvent {
    Table,
    Paddle { player: u8 },
    Net,
    Floor,
}

/// Paddle state for collision detection.
#[derive(Debug, Clone)]
pub struct PaddleState {
    /// Paddle center position in DH world space.
    pub position: Vector3<f64>,
    /// Paddle face normal (direction the paddle is facing).
    pub normal: Vector3<f64>,
    /// Paddle velocity (for impulse transfer).
    pub velocity: Vector3<f64>,
    /// Paddle angular velocity (for spin transfer).
    pub angular_velocity: Vector3<f64>,
    /// Player ID (1 or 2).
    pub player: u8,
}

/// Simple LCG random for per-collision variation.
static mut RNG_STATE: u64 = 123456789;

fn rand_f64() -> f64 {
    unsafe {
        RNG_STATE = RNG_STATE.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((RNG_STATE >> 33) as f64) / (u32::MAX as f64)
    }
}

/// Random variation around a base value: base * (1.0 + noise * [-1, 1])
fn vary(base: f64, noise: f64) -> f64 {
    base * (1.0 + noise * (2.0 * rand_f64() - 1.0))
}

/// Check and resolve all collisions for the ball. Returns collision events.
pub fn resolve_collisions(
    ball: &mut Ball,
    paddles: &[PaddleState],
) -> Vec<CollisionEvent> {
    let mut events = Vec::new();
    if !ball.active {
        return events;
    }

    let r = BALL_RADIUS;

    // Ball-table collision
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

        if impact_speed < 0.05 {
            // Ball has settled on table — just rest it, no bounce, no sound
            ball.velocity.z = 0.0;
            // Apply rolling friction to slow horizontal movement
            let horiz_speed = (ball.velocity.x * ball.velocity.x + ball.velocity.y * ball.velocity.y).sqrt();
            if horiz_speed > 0.01 {
                let friction_decel = 0.8; // m/s^2 rolling friction
                let factor = (1.0 - friction_decel * 0.001 / horiz_speed).max(0.0);
                ball.velocity.x *= factor;
                ball.velocity.y *= factor;
            } else {
                ball.velocity.x = 0.0;
                ball.velocity.y = 0.0;
            }
        } else {
            let normal = Vector3::new(0.0, 0.0, 1.0);
            let cor = vary(COR_TABLE, 0.03);
            let friction = vary(FRICTION_TABLE, 0.05);
            apply_bounce(ball, &normal, cor, friction);
            // Only emit audio event for real bounces
            events.push(CollisionEvent::Table);
        }
    }

    // Ball-net collision (thin plane at x=0)
    let net_top = table_top_z + table::NET_HEIGHT as f64;
    if ball.position.z <= net_top
        && ball.position.z >= table_top_z
        && ball.position.y.abs() <= table::NET_LENGTH as f64 / 2.0
        && ball.position.x.abs() <= r + table::NET_THICKNESS as f64 / 2.0
    {
        let sign = ball.position.x.signum();
        let cor = vary(COR_NET, 0.10); // ±10% — nets are unpredictable
        ball.velocity.x = -ball.velocity.x * cor;
        ball.velocity.z *= 0.8;
        ball.spin *= 0.5;
        ball.position.x = sign * (r + table::NET_THICKNESS as f64 / 2.0 + 0.001);
        events.push(CollisionEvent::Net);
    }

    // Ball-paddle collisions
    for paddle in paddles {
        let to_ball = ball.position - paddle.position;
        let dist = to_ball.norm();
        let paddle_r = crate::arm_config::PADDLE_RADIUS;

        if dist < r + paddle_r {
            // Ball is touching paddle — compute impulse response
            let normal = if to_ball.norm() > 1e-6 {
                to_ball.normalize()
            } else {
                paddle.normal
            };

            // Relative velocity of ball w.r.t. paddle
            let v_rel = ball.velocity - paddle.velocity;
            let v_n = v_rel.dot(&normal);

            // Only resolve if approaching
            if v_n < 0.0 {
                let cor = vary(COR_PADDLE, 0.04);       // ±4% rubber variation
                let friction = vary(FRICTION_PADDLE, 0.06); // ±6% rubber variation

                // Normal impulse
                let j_n = -(1.0 + cor) * v_n * BALL_MASS;
                ball.velocity += normal * (j_n / BALL_MASS);

                // Tangential friction impulse for spin transfer
                let v_t = v_rel - normal * v_n;
                let v_t_mag = v_t.norm();
                if v_t_mag > 1e-6 {
                    let t_dir = v_t / v_t_mag;
                    let j_t = (-friction * j_n.abs()).max(-v_t_mag * BALL_MASS);
                    ball.velocity += t_dir * (j_t / BALL_MASS);

                    // Spin from friction: delta_omega = (r x j_t) / I
                    let r_vec = -normal * r;
                    let delta_omega = r_vec.cross(&(t_dir * j_t)) / BALL_INERTIA;
                    ball.spin += delta_omega;
                }

                // Transfer paddle angular velocity to ball spin (with variation)
                let spin_transfer = vary(0.3, 0.15); // 0.3 ± 15%
                ball.spin += paddle.angular_velocity * spin_transfer;

                // Add small random spin perturbation (surface imperfections)
                let spin_noise = 5.0; // rad/s
                ball.spin += Vector3::new(
                    spin_noise * (2.0 * rand_f64() - 1.0),
                    spin_noise * (2.0 * rand_f64() - 1.0),
                    spin_noise * (2.0 * rand_f64() - 1.0),
                );

                // Push ball out of paddle
                let overlap = r + paddle_r - dist;
                ball.position += normal * overlap;

                events.push(CollisionEvent::Paddle {
                    player: paddle.player,
                });
            }
        }
    }

    // Ball-floor collision
    if ball.position.z - r <= 0.0 && ball.velocity.z < 0.0 {
        let impact_speed = ball.velocity.z.abs();
        ball.position.z = r;

        if impact_speed < 0.05 {
            // Settled on floor — stop it
            ball.velocity = Vector3::zeros();
            ball.active = false;
        } else {
            let normal = Vector3::new(0.0, 0.0, 1.0);
            let cor = vary(COR_FLOOR, 0.05);
            let friction = vary(FRICTION_FLOOR, 0.05);
            apply_bounce(ball, &normal, cor, friction);
            ball.floor_bounces += 1;
            if ball.floor_bounces >= 3 {
                ball.active = false;
            }
            events.push(CollisionEvent::Floor);
        }
    }

    events
}

/// Apply a bounce off a surface with given normal, COR, and friction.
fn apply_bounce(ball: &mut Ball, normal: &Vector3<f64>, cor: f64, friction: f64) {
    let v_n = ball.velocity.dot(normal);
    let v_t = ball.velocity - normal * v_n;

    // Reflect normal component with COR
    ball.velocity = v_t - normal * (v_n * cor);

    // Friction: reduce tangential velocity and add spin
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

    // Small random spin perturbation on every bounce
    let noise = 2.0;
    ball.spin += Vector3::new(
        noise * (2.0 * rand_f64() - 1.0),
        noise * (2.0 * rand_f64() - 1.0),
        noise * (2.0 * rand_f64() - 1.0),
    );
}
