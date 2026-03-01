//! Game state machine: serve, rally, scoring, rules.
//!
//! Phase 3 will implement full game logic.

/// Game state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GamePhase {
    /// Waiting for serve (Space to serve).
    WaitingForServe,
    /// Ball is in play.
    Rally,
    /// Point just scored, brief pause.
    PointScored,
}

/// Which player is serving / scored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Player {
    Player1,
    Player2,
}

/// Full game state.
#[derive(Debug, Clone)]
pub struct GameState {
    pub phase: GamePhase,
    pub score: [u32; 2],
    pub serving: Player,
    pub serves_in_set: u32,
    pub last_table_bounce_side: Option<Player>, // which side the ball last bounced on
}

impl GameState {
    pub fn new() -> Self {
        Self {
            phase: GamePhase::WaitingForServe,
            score: [0, 0],
            serving: Player::Player1,
            serves_in_set: 0,
            last_table_bounce_side: None,
        }
    }

    /// Award a point to the given player.
    pub fn score_point(&mut self, player: Player) {
        match player {
            Player::Player1 => self.score[0] += 1,
            Player::Player2 => self.score[1] += 1,
        }
        self.phase = GamePhase::PointScored;
        self.serves_in_set += 1;
        // Switch server every 2 points
        if self.serves_in_set >= 2 {
            self.serves_in_set = 0;
            self.serving = match self.serving {
                Player::Player1 => Player::Player2,
                Player::Player2 => Player::Player1,
            };
        }
    }

    /// Transition to waiting for serve.
    pub fn reset_for_serve(&mut self) {
        self.phase = GamePhase::WaitingForServe;
        self.last_table_bounce_side = None;
    }
}
