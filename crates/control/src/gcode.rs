//! G-code interpreter for CNC toolpath execution.
//!
//! Supports: G0 (rapid), G1 (linear feed), G2/G3 (arc CW/CCW), A-axis rotation.

use nalgebra::Vector3;

/// A parsed G-code command.
#[derive(Debug, Clone)]
pub enum GCommand {
    /// G0: Rapid move to position at maximum speed.
    Rapid {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        a: Option<f64>,
    },
    /// G1: Linear feed move.
    LinearFeed {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        a: Option<f64>,
        f: Option<f64>,
    },
    /// G2: Clockwise arc (XY plane).
    ArcCW {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        i: f64,
        j: f64,
        f: Option<f64>,
    },
    /// G3: Counter-clockwise arc (XY plane).
    ArcCCW {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        i: f64,
        j: f64,
        f: Option<f64>,
    },
    /// M3: Spindle on CW.
    SpindleOn { rpm: f64 },
    /// M5: Spindle off.
    SpindleOff,
    /// Comment or unrecognized line.
    Comment(String),
}

/// Parser state for modal G-code interpretation.
#[derive(Debug, Clone)]
pub struct GCodeInterpreter {
    /// Parsed program as a sequence of commands.
    pub commands: Vec<GCommand>,
    /// Current command index during execution.
    pub current: usize,
    /// Current modal position (absolute coordinates, in meters).
    pub position: Vector3<f64>,
    /// Current A-axis angle (rad).
    pub a_axis: f64,
    /// Current feed rate (m/s). G-code uses mm/min, converted on parse.
    pub feed_rate: f64,
    /// Rapid feed rate (m/s).
    pub rapid_rate: f64,
}

impl GCodeInterpreter {
    pub fn new() -> Self {
        Self {
            commands: Vec::new(),
            current: 0,
            position: Vector3::zeros(),
            a_axis: 0.0,
            feed_rate: 0.01,  // 600 mm/min default
            rapid_rate: 0.1,  // 6000 mm/min
        }
    }

    /// Parse a G-code program from text.
    pub fn parse(&mut self, program: &str) {
        self.commands.clear();
        self.current = 0;

        for line in program.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('%') {
                continue;
            }

            // Strip inline comments
            let line = if let Some(idx) = line.find('(') {
                &line[..idx]
            } else {
                line
            };
            let line = line.trim();
            if line.is_empty() {
                continue;
            }

            if let Some(cmd) = self.parse_line(line) {
                self.commands.push(cmd);
            }
        }
    }

    fn parse_line(&self, line: &str) -> Option<GCommand> {
        let tokens = tokenize(line);
        let g_code = find_value(&tokens, 'G');
        let m_code = find_value(&tokens, 'M');

        if let Some(m) = m_code {
            let m = m as u32;
            return match m {
                3 => {
                    let rpm = find_value(&tokens, 'S').unwrap_or(10000.0);
                    Some(GCommand::SpindleOn { rpm })
                }
                5 => Some(GCommand::SpindleOff),
                _ => Some(GCommand::Comment(line.to_string())),
            };
        }

        if let Some(g) = g_code {
            let g = g as u32;
            let x = find_value(&tokens, 'X').map(mm_to_m);
            let y = find_value(&tokens, 'Y').map(mm_to_m);
            let z = find_value(&tokens, 'Z').map(mm_to_m);
            let a = find_value(&tokens, 'A').map(|v| v.to_radians());
            let f = find_value(&tokens, 'F').map(mmpm_to_mps);

            return match g {
                0 => Some(GCommand::Rapid { x, y, z, a }),
                1 => Some(GCommand::LinearFeed { x, y, z, a, f }),
                2 => {
                    let i = mm_to_m(find_value(&tokens, 'I').unwrap_or(0.0));
                    let j = mm_to_m(find_value(&tokens, 'J').unwrap_or(0.0));
                    Some(GCommand::ArcCW { x, y, z, i, j, f })
                }
                3 => {
                    let i = mm_to_m(find_value(&tokens, 'I').unwrap_or(0.0));
                    let j = mm_to_m(find_value(&tokens, 'J').unwrap_or(0.0));
                    Some(GCommand::ArcCCW { x, y, z, i, j, f })
                }
                _ => Some(GCommand::Comment(line.to_string())),
            };
        }

        Some(GCommand::Comment(line.to_string()))
    }

    /// Get the next target position and feed rate from the program.
    /// Returns None when program is complete.
    pub fn next_target(&mut self) -> Option<(Vector3<f64>, f64, f64)> {
        if self.current >= self.commands.len() {
            return None;
        }

        let cmd = self.commands[self.current].clone();
        self.current += 1;

        match cmd {
            GCommand::Rapid { x, y, z, a } => {
                if let Some(x) = x { self.position.x = x; }
                if let Some(y) = y { self.position.y = y; }
                if let Some(z) = z { self.position.z = z; }
                if let Some(a) = a { self.a_axis = a; }
                Some((self.position, self.rapid_rate, self.a_axis))
            }
            GCommand::LinearFeed { x, y, z, a, f } => {
                if let Some(f) = f { self.feed_rate = f; }
                if let Some(x) = x { self.position.x = x; }
                if let Some(y) = y { self.position.y = y; }
                if let Some(z) = z { self.position.z = z; }
                if let Some(a) = a { self.a_axis = a; }
                Some((self.position, self.feed_rate, self.a_axis))
            }
            GCommand::ArcCW { x, y, z, f, .. } | GCommand::ArcCCW { x, y, z, f, .. } => {
                // Simplified: treat arcs as linear moves for now
                if let Some(f) = f { self.feed_rate = f; }
                if let Some(x) = x { self.position.x = x; }
                if let Some(y) = y { self.position.y = y; }
                if let Some(z) = z { self.position.z = z; }
                Some((self.position, self.feed_rate, self.a_axis))
            }
            GCommand::SpindleOn { .. } | GCommand::SpindleOff | GCommand::Comment(_) => {
                // Skip non-motion commands, try next
                self.next_target()
            }
        }
    }

    /// Is the program complete?
    pub fn is_complete(&self) -> bool {
        self.current >= self.commands.len()
    }

    /// Peek at the current command without advancing.
    pub fn peek_command(&self) -> Option<&GCommand> {
        self.commands.get(self.current)
    }

    /// Load a G-code program from a file path.
    pub fn load_file(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read G-code file '{}': {}", path, e))?;
        let mut interp = Self::new();
        interp.parse(&text);
        Ok(interp)
    }

    /// Reset to beginning of program.
    pub fn reset(&mut self) {
        self.current = 0;
        self.position = Vector3::zeros();
        self.a_axis = 0.0;
    }
}

impl Default for GCodeInterpreter {
    fn default() -> Self {
        Self::new()
    }
}

/// Tokenize a G-code line into (letter, value) pairs.
fn tokenize(line: &str) -> Vec<(char, f64)> {
    let mut tokens = Vec::new();
    let mut chars = line.chars().peekable();

    while let Some(&c) = chars.peek() {
        if c.is_ascii_alphabetic() {
            let letter = c.to_ascii_uppercase();
            chars.next();

            // Collect the number
            let mut num_str = String::new();
            while let Some(&nc) = chars.peek() {
                if nc.is_ascii_digit() || nc == '.' || nc == '-' || nc == '+' {
                    num_str.push(nc);
                    chars.next();
                } else {
                    break;
                }
            }

            if let Ok(val) = num_str.parse::<f64>() {
                tokens.push((letter, val));
            }
        } else {
            chars.next();
        }
    }

    tokens
}

fn find_value(tokens: &[(char, f64)], letter: char) -> Option<f64> {
    tokens.iter().find(|(l, _)| *l == letter).map(|(_, v)| *v)
}

/// Convert mm to meters.
fn mm_to_m(mm: f64) -> f64 {
    mm * 0.001
}

/// Convert mm/min to m/s.
fn mmpm_to_mps(mmpm: f64) -> f64 {
    mmpm * 0.001 / 60.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_program() {
        let program = r#"
G0 X0 Y0 Z50
M3 S10000
G1 X100 Y0 Z-5 F600
G1 X100 Y100 Z-5
G0 Z50
M5
"#;

        let mut interp = GCodeInterpreter::new();
        interp.parse(program);

        // Should have parsed commands (excluding M codes and comments)
        assert!(
            interp.commands.len() >= 4,
            "Expected >=4 commands, got {}",
            interp.commands.len()
        );

        // Step through targets
        let mut positions = Vec::new();
        while let Some((pos, _rate, _a)) = interp.next_target() {
            positions.push(pos);
        }

        assert!(!positions.is_empty(), "Should have produced target positions");
    }

    #[test]
    fn test_tokenize() {
        let tokens = tokenize("G1 X100.5 Y-20 F600");
        assert_eq!(tokens.len(), 4);
        assert_eq!(tokens[0], ('G', 1.0));
        assert_eq!(tokens[1], ('X', 100.5));
        assert_eq!(tokens[2], ('Y', -20.0));
        assert_eq!(tokens[3], ('F', 600.0));
    }
}
