//! Carving execution engine — state machine bridging G-code → trajectory → Cartesian targets.

use nalgebra::{Isometry3, UnitQuaternion, Vector3};

use crate::gcode::{GCodeInterpreter, GCommand};
use crate::trajectory::{TrajectoryPlanner, Waypoint};

/// State of the carving session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarvingState {
    Idle,
    Running,
    Paused,
    Complete,
}

/// A waypoint extracted from G-code for visualization.
#[derive(Debug, Clone)]
pub struct CarvingWaypoint {
    pub position: Vector3<f64>,
    pub is_rapid: bool,
    pub feed_rate: f64,
}

/// Orchestrates G-code execution through trajectory planning.
pub struct CarvingSession {
    gcode: GCodeInterpreter,
    planner: TrajectoryPlanner,
    /// All waypoints pre-extracted for visualization.
    pub waypoints: Vec<CarvingWaypoint>,
    /// Index of the waypoint we're currently moving toward.
    current_waypoint: usize,
    /// Current interpolated position from the planner.
    current_position: Vector3<f64>,
    pub state: CarvingState,
    pub speed_multiplier: f64,
    /// Offset from G-code local coords to DH world coords.
    pub workpiece_offset: Vector3<f64>,
    /// Fixed tool orientation (tool-down for 3-axis milling).
    pub tool_orientation: UnitQuaternion<f64>,
    pub spindle_on: bool,
    /// Whether the current segment is a rapid move (G0) — no cutting during rapids.
    pub is_rapid: bool,
    pub elapsed_time: f64,
    pub estimated_total_time: f64,
    /// A-axis (rotary table) target angle in radians from G-code.
    pub a_axis_target: f64,
}

impl CarvingSession {
    /// Load a G-code file and prepare the carving session.
    pub fn load_file(path: &str, workpiece_offset: Vector3<f64>, tool_orientation: UnitQuaternion<f64>) -> Result<Self, String> {
        let gcode = GCodeInterpreter::load_file(path)?;
        Self::from_interpreter(gcode, workpiece_offset, tool_orientation)
    }

    /// Create from a pre-parsed interpreter.
    pub fn from_interpreter(
        gcode: GCodeInterpreter,
        workpiece_offset: Vector3<f64>,
        tool_orientation: UnitQuaternion<f64>,
    ) -> Result<Self, String> {
        // Pre-extract all waypoints for visualization and time estimation
        let mut preview = gcode.clone();
        let mut waypoints = Vec::new();

        // Start position (G-code origin mapped to world)
        waypoints.push(CarvingWaypoint {
            position: workpiece_offset,
            is_rapid: true,
            feed_rate: preview.rapid_rate,
        });

        // Walk through all commands to extract waypoints
        while let Some((pos, feed, _a)) = preview.next_target() {
            let world_pos = pos + workpiece_offset;
            // Determine if this was a rapid by checking feed rate against rapid_rate
            let is_rapid = (feed - preview.rapid_rate).abs() < 1e-6;
            waypoints.push(CarvingWaypoint {
                position: world_pos,
                is_rapid,
                feed_rate: feed,
            });
        }

        // Estimate total time from waypoint distances and feed rates
        let mut total_time = 0.0;
        for pair in waypoints.windows(2) {
            let dist = (pair[1].position - pair[0].position).norm();
            if pair[1].feed_rate > 1e-9 {
                total_time += dist / pair[1].feed_rate;
            }
        }

        Ok(Self {
            gcode,
            planner: TrajectoryPlanner::new(0.5), // 0.5 m/s² default accel
            waypoints,
            current_waypoint: 0,
            current_position: workpiece_offset,
            state: CarvingState::Idle,
            speed_multiplier: 1.0,
            workpiece_offset,
            tool_orientation,
            spindle_on: false,
            is_rapid: true,
            elapsed_time: 0.0,
            estimated_total_time: total_time,
            a_axis_target: 0.0,
        })
    }

    pub fn start(&mut self) {
        if self.state == CarvingState::Idle || self.state == CarvingState::Paused {
            // Reset the G-code interpreter for execution
            if self.state == CarvingState::Idle {
                self.gcode.reset();
                self.current_waypoint = 0;
                self.spindle_on = false;
                self.elapsed_time = 0.0;
                self.current_position = self.workpiece_offset;
                self.load_next_segment();
            }
            self.state = CarvingState::Running;
        }
    }

    /// Override the starting position (use the arm's actual position instead of G-code origin).
    pub fn set_start_position(&mut self, pos: Vector3<f64>) {
        self.current_position = pos;
        // Also update the first waypoint for visualization
        if !self.waypoints.is_empty() {
            self.waypoints[0].position = pos;
        }
    }

    pub fn pause(&mut self) {
        if self.state == CarvingState::Running {
            self.state = CarvingState::Paused;
        }
    }

    pub fn resume(&mut self) {
        if self.state == CarvingState::Paused {
            self.state = CarvingState::Running;
        }
    }

    pub fn toggle(&mut self) {
        match self.state {
            CarvingState::Idle => self.start(),
            CarvingState::Running => self.pause(),
            CarvingState::Paused => self.resume(),
            CarvingState::Complete => {
                self.reset();
                self.start();
            }
        }
    }

    pub fn reset(&mut self) {
        self.gcode.reset();
        self.planner = TrajectoryPlanner::new(0.5);
        self.current_waypoint = 0;
        self.current_position = self.workpiece_offset;
        self.state = CarvingState::Idle;
        self.spindle_on = false;
        self.is_rapid = true;
        self.elapsed_time = 0.0;
        self.a_axis_target = 0.0;
    }

    /// Advance the carving by dt seconds. Returns the next Cartesian target pose, or None when complete.
    pub fn step(&mut self, dt: f64) -> Option<Isometry3<f64>> {
        if self.state != CarvingState::Running {
            return Some(Isometry3::from_parts(
                nalgebra::Translation3::from(self.current_position),
                self.tool_orientation,
            ));
        }

        self.elapsed_time += dt;

        // Try stepping the current planner segment
        if let Some(pos) = self.planner.step(dt) {
            self.current_position = pos;
            return Some(Isometry3::from_parts(
                nalgebra::Translation3::from(pos),
                self.tool_orientation,
            ));
        }

        // Current segment complete — keep loading until we find one with nonzero duration
        // or the program ends. This handles high-speed or short segments that complete instantly.
        loop {
            if !self.load_next_segment() {
                self.state = CarvingState::Complete;
                return None;
            }

            // Step with 0 dt to get the start position of the new segment
            // (actual time advancement happens on the next call)
            if let Some(pos) = self.planner.step(0.0) {
                self.current_position = pos;
                return Some(Isometry3::from_parts(
                    nalgebra::Translation3::from(pos),
                    self.tool_orientation,
                ));
            }
            // Segment completed immediately (zero-length slipped through) — try next
        }
    }

    /// Load the next motion segment from G-code into the trajectory planner.
    /// Returns false when the program is complete.
    fn load_next_segment(&mut self) -> bool {
        loop {
            // Process spindle and non-motion commands first
            while let Some(cmd) = self.gcode.peek_command() {
                match cmd {
                    GCommand::SpindleOn { .. } => {
                        self.spindle_on = true;
                        self.gcode.current += 1;
                    }
                    GCommand::SpindleOff => {
                        self.spindle_on = false;
                        self.gcode.current += 1;
                    }
                    GCommand::Comment(_) => {
                        self.gcode.current += 1;
                    }
                    _ => break,
                }
            }

            // Get next motion target
            let (target_pos, feed_rate, a) = match self.gcode.next_target() {
                Some(t) => t,
                None => return false,
            };
            self.a_axis_target = a;

            // Detect rapid moves (feed rate matches rapid_rate)
            self.is_rapid = (feed_rate - self.gcode.rapid_rate).abs() < 1e-6;

            let world_target = target_pos + self.workpiece_offset;
            let dist = (world_target - self.current_position).norm();

            self.current_waypoint += 1;

            // Skip zero-length moves
            if dist < 1e-6 {
                self.current_position = world_target;
                continue;
            }

            // Plan a 2-waypoint segment: current → target
            let waypoints = [
                Waypoint {
                    position: self.current_position,
                    feed_rate,
                },
                Waypoint {
                    position: world_target,
                    feed_rate,
                },
            ];
            self.planner.plan(&waypoints);
            return true;
        }
    }

    /// Progress as 0.0 to 1.0.
    pub fn progress(&self) -> f64 {
        if self.waypoints.len() <= 1 {
            return 0.0;
        }
        (self.current_waypoint as f64 / (self.waypoints.len() - 1) as f64).clamp(0.0, 1.0)
    }

    /// Number of completed waypoints vs total.
    pub fn progress_counts(&self) -> (usize, usize) {
        (self.current_waypoint, self.waypoints.len().saturating_sub(1))
    }

    /// Current G-code command index.
    pub fn current_line(&self) -> usize {
        self.gcode.current
    }

    /// Total number of G-code commands.
    pub fn total_lines(&self) -> usize {
        self.gcode.commands.len()
    }

    /// Remaining time estimate at current speed.
    pub fn eta(&self) -> f64 {
        let prog = self.progress();
        if prog < 1e-6 || self.elapsed_time < 1e-6 {
            return self.estimated_total_time;
        }
        let rate = prog / self.elapsed_time;
        ((1.0 - prog) / rate).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_carving_session_basic() {
        let program = "G0 X0 Y0 Z10\nM3 S10000\nG1 X50 Y0 Z0 F600\nG1 X50 Y50 Z0\nM5\nG0 X0 Y0 Z10\n";
        let mut gcode = GCodeInterpreter::new();
        gcode.parse(program);

        let offset = Vector3::new(0.86, 0.25, -0.10);
        let orient = UnitQuaternion::identity();
        let mut session = CarvingSession::from_interpreter(gcode, offset, orient).unwrap();

        assert_eq!(session.state, CarvingState::Idle);
        assert!(session.waypoints.len() >= 3);

        session.start();
        assert_eq!(session.state, CarvingState::Running);

        // Step until complete or max iterations
        let mut steps = 0;
        while session.state == CarvingState::Running && steps < 1_000_000 {
            session.step(0.001);
            steps += 1;
        }
        assert_eq!(session.state, CarvingState::Complete);
        assert!(session.progress() > 0.9);
    }
}
