//! Configuration loading for the ping pong scenario.

/// Ping pong simulation configuration.
pub struct PingPongConfig {
    pub physics_hz: f64,
    pub ai_difficulty: f64,
}

impl PingPongConfig {
    pub fn default() -> Self {
        Self {
            physics_hz: 1000.0,
            ai_difficulty: 0.5,
        }
    }
}
