//! Carving execution engine — state machine bridging G-code → trajectory → Cartesian targets.

use nalgebra::{Isometry3, Rotation3, UnitQuaternion, Vector3};

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
    /// Workpiece center in DH world coords.
    pub workpiece_center: Vector3<f64>,
    /// Half-extent of the cube (for computing surface offset).
    pub half_extent: f64,
    /// Base tool orientation at A=0 (tool-down for top-face milling).
    pub base_tool_orientation: UnitQuaternion<f64>,
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
    pub fn load_file(path: &str, workpiece_center: Vector3<f64>, half_extent: f64, tool_orientation: UnitQuaternion<f64>) -> Result<Self, String> {
        let gcode = GCodeInterpreter::load_file(path)?;
        Self::from_interpreter(gcode, workpiece_center, half_extent, tool_orientation)
    }

    /// Compute world position from G-code local position and A-axis angle.
    fn gcode_to_world(&self, gcode_pos: &Vector3<f64>, a_angle: f64) -> Vector3<f64> {
        let local = *gcode_pos + Vector3::new(0.0, 0.0, self.half_extent);
        let rot = Rotation3::from_axis_angle(&Vector3::x_axis(), a_angle);
        self.workpiece_center + rot * local
    }

    /// Compute tool orientation for the given A-axis angle.
    fn tool_orientation_at(&self, a_angle: f64) -> UnitQuaternion<f64> {
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), a_angle);
        rot * self.base_tool_orientation
    }

    /// Create from a pre-parsed interpreter.
    pub fn from_interpreter(
        gcode: GCodeInterpreter,
        workpiece_center: Vector3<f64>,
        half_extent: f64,
        tool_orientation: UnitQuaternion<f64>,
    ) -> Result<Self, String> {
        let workpiece_top = workpiece_center + Vector3::new(0.0, 0.0, half_extent);

        // Pre-extract all waypoints for visualization and time estimation
        let mut preview = gcode.clone();
        let mut waypoints = Vec::new();

        // Start position (G-code origin mapped to world at A=0)
        waypoints.push(CarvingWaypoint {
            position: workpiece_top,
            is_rapid: true,
            feed_rate: preview.rapid_rate,
        });

        // Walk through all commands to extract waypoints, tracking A-axis
        let mut preview_a: f64;
        while let Some((pos, feed, a)) = preview.next_target() {
            preview_a = a;
            let local = pos + Vector3::new(0.0, 0.0, half_extent);
            let rot = Rotation3::from_axis_angle(&Vector3::x_axis(), preview_a);
            let world_pos = workpiece_center + rot * local;
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
            current_position: workpiece_top,
            state: CarvingState::Idle,
            speed_multiplier: 1.0,
            workpiece_center,
            half_extent,
            base_tool_orientation: tool_orientation,
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
                let workpiece_top = self.workpiece_center + Vector3::new(0.0, 0.0, self.half_extent);
                self.current_position = workpiece_top;
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
        self.current_position = self.workpiece_center + Vector3::new(0.0, 0.0, self.half_extent);
        self.state = CarvingState::Idle;
        self.spindle_on = false;
        self.is_rapid = true;
        self.elapsed_time = 0.0;
        self.a_axis_target = 0.0;
    }

    /// Advance the carving by dt seconds. Returns the next Cartesian target pose, or None when complete.
    pub fn step(&mut self, dt: f64) -> Option<Isometry3<f64>> {
        let orient = self.tool_orientation_at(self.a_axis_target);
        if self.state != CarvingState::Running {
            return Some(Isometry3::from_parts(
                nalgebra::Translation3::from(self.current_position),
                orient,
            ));
        }

        self.elapsed_time += dt;

        // Try stepping the current planner segment
        if let Some(pos) = self.planner.step(dt) {
            self.current_position = pos;
            return Some(Isometry3::from_parts(
                nalgebra::Translation3::from(pos),
                orient,
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
                    orient,
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

            let world_target = self.gcode_to_world(&target_pos, self.a_axis_target);
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

        let center = Vector3::new(0.86, 0.25, -0.20);
        let orient = UnitQuaternion::identity();
        let mut session = CarvingSession::from_interpreter(gcode, center, 0.1, orient).unwrap();

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

    #[test]
    fn test_calibration_gcode_loads_and_produces_targets() {
        // Load the actual calibration G-code file
        let gcode_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/calibration.gcode");
        let gcode = GCodeInterpreter::load_file(gcode_path)
            .expect("Failed to load calibration.gcode");

        // Workpiece params matching config.toml: center=(0.65, 0, -0.2), half=0.1525
        let workpiece_center = Vector3::new(0.65, 0.0, -0.2);
        let half_extent = 0.1525;
        let orient = UnitQuaternion::from_axis_angle(
            &Vector3::x_axis(), std::f64::consts::PI,
        );

        let cmd_count = gcode.commands.len();
        eprintln!("Calibration G-code: {} commands parsed", cmd_count);
        assert!(cmd_count > 50, "Expected >50 commands, got {}", cmd_count);

        let mut session = CarvingSession::from_interpreter(gcode, workpiece_center, half_extent, orient)
            .expect("Failed to create CarvingSession");

        let wp_count = session.waypoints.len();
        eprintln!("Calibration G-code: {} waypoints extracted", wp_count);
        assert!(wp_count > 30, "Expected >30 waypoints, got {}", wp_count);
        eprintln!("Estimated time: {:.1}s", session.estimated_total_time);

        // Start the session and step through, collecting all targets
        session.start();
        assert_eq!(session.state, CarvingState::Running);

        let mut targets: Vec<Vector3<f64>> = Vec::new();
        let mut steps = 0;
        let dt = 0.001; // 1ms steps
        while session.state == CarvingState::Running && steps < 5_000_000 {
            if let Some(target) = session.step(dt) {
                targets.push(target.translation.vector);
            }
            steps += 1;
        }

        assert_eq!(session.state, CarvingState::Complete,
            "Session did not complete after {} steps", steps);
        eprintln!("Session completed after {} steps, {} targets sampled", steps, targets.len());

        // Verify targets are within reasonable workspace bounds
        // The arm can reach ~1.4m, workpiece center is at ~0.65m
        let mut min_pos = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max_pos = Vector3::new(f64::MIN, f64::MIN, f64::MIN);
        for t in &targets {
            for i in 0..3 {
                min_pos[i] = min_pos[i].min(t[i]);
                max_pos[i] = max_pos[i].max(t[i]);
            }
        }
        eprintln!("Target bounds: min=({:.3},{:.3},{:.3}) max=({:.3},{:.3},{:.3})",
            min_pos.x, min_pos.y, min_pos.z, max_pos.x, max_pos.y, max_pos.z);

        // All targets should be within arm's reachable workspace (roughly)
        for (i, t) in targets.iter().enumerate() {
            let dist_from_base = (t.x * t.x + t.y * t.y).sqrt();
            assert!(dist_from_base < 1.5,
                "Target {} at ({:.3},{:.3},{:.3}) too far from base: {:.3}m",
                i, t.x, t.y, t.z, dist_from_base);
            // Z should be within reasonable range (base is at z=0.3, workspace extends down)
            assert!(t.z > -0.5 && t.z < 1.5,
                "Target {} Z out of range: {:.3}", i, t.z);
        }

        // Verify the workspace spans a reasonable range around the workpiece
        let x_range = max_pos.x - min_pos.x;
        let y_range = max_pos.y - min_pos.y;
        eprintln!("Workspace coverage: X={:.1}mm, Y={:.1}mm, Z range=[{:.1}mm, {:.1}mm]",
            x_range * 1000.0, y_range * 1000.0,
            (min_pos.z - (workpiece_center.z + half_extent)) * 1000.0,
            (max_pos.z - (workpiece_center.z + half_extent)) * 1000.0);
        assert!(x_range > 0.1, "X range too small: {:.3}m", x_range);
        assert!(y_range > 0.1, "Y range too small: {:.3}m", y_range);
    }

    #[test]
    fn test_carving_session_first_segment_loads() {
        // Minimal test: ensure that after start(), the first segment is loaded
        // and step() returns a target immediately
        let program = "G0 Z10\nM3 S10000\nG0 X0 Y0\nG1 Z-3 F300\nG0 Z5\nM5\n";
        let mut gcode = GCodeInterpreter::new();
        gcode.parse(program);

        let center = Vector3::new(0.65, 0.0, -0.2);
        let orient = UnitQuaternion::identity();
        let mut session = CarvingSession::from_interpreter(gcode, center, 0.1525, orient).unwrap();

        session.start();
        assert_eq!(session.state, CarvingState::Running);

        // The very first step should return a target
        let target = session.step(0.001);
        assert!(target.is_some(), "First step() should return a target, not None");

        let pos = target.unwrap().translation.vector;
        eprintln!("First target: ({:.4}, {:.4}, {:.4})", pos.x, pos.y, pos.z);

        // Should be near the workpiece offset (first G-code move is G0 Z10 → +0.01m)
        assert!((pos.x - 0.65).abs() < 0.5, "First target X too far: {:.3}", pos.x);
    }
}
