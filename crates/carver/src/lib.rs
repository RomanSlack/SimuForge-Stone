//! Reusable direct carver: CSG subtraction of G-code tool paths from an OctreeSdf workpiece.
//!
//! Used by both the headless CLI (`simuforge-carve`) and the in-engine preview mode.

use nalgebra::Vector3;
use simuforge_control::gcode::GCommand;
use simuforge_cutting::tool::Tool;
use simuforge_material::octree::OctreeSdf;

/// Direct carver: walks G-code commands and subtracts tool geometry from an SDF workpiece.
pub struct DirectCarver {
    tool: Tool,
    commands: Vec<GCommand>,
    position: Vector3<f64>,
    a_axis: f64,
    spindle_on: bool,
    current_cmd: usize,
    cut_count: u64,
    half_extent: f64,
    cell_size: f64,
    /// The carved workpiece SDF.
    pub workpiece: OctreeSdf,
}

impl DirectCarver {
    /// Create a new carver with a fresh block workpiece.
    pub fn new(
        commands: Vec<GCommand>,
        tool: Tool,
        half_extent: f64,
        cell_size: f64,
    ) -> Self {
        let he = half_extent as f32;
        let workpiece = OctreeSdf::new_block([he, he, he], cell_size as f32);
        Self {
            tool,
            commands,
            position: Vector3::zeros(),
            a_axis: 0.0,
            spindle_on: false,
            current_cmd: 0,
            cut_count: 0,
            half_extent,
            cell_size,
            workpiece,
        }
    }

    pub fn total_commands(&self) -> usize {
        self.commands.len()
    }

    pub fn current_command(&self) -> usize {
        self.current_cmd
    }

    pub fn cut_count(&self) -> u64 {
        self.cut_count
    }

    /// Process up to `count` commands. Returns the number actually processed.
    pub fn carve_batch(&mut self, count: usize) -> usize {
        let start = self.current_cmd;
        let end = (start + count).min(self.commands.len());
        for i in start..end {
            self.process_command(i);
        }
        self.current_cmd = end;
        end - start
    }

    /// Carve to a target command index. If target < current, resets and re-carves.
    pub fn carve_to(&mut self, target_cmd: usize) {
        let target = target_cmd.min(self.commands.len());
        if target < self.current_cmd {
            self.reset();
        }
        if target > self.current_cmd {
            self.carve_batch(target - self.current_cmd);
        }
    }

    /// Reset: fresh workpiece, rewind to command 0.
    pub fn reset(&mut self) {
        let he = self.half_extent as f32;
        self.workpiece = OctreeSdf::new_block([he, he, he], self.cell_size as f32);
        self.position = Vector3::zeros();
        self.a_axis = 0.0;
        self.spindle_on = false;
        self.current_cmd = 0;
        self.cut_count = 0;
    }

    /// Process a single command by index.
    fn process_command(&mut self, idx: usize) {
        let cmd = &self.commands[idx];
        match cmd {
            GCommand::SpindleOn { .. } => {
                self.spindle_on = true;
            }
            GCommand::SpindleOff => {
                self.spindle_on = false;
            }
            GCommand::Rapid { x, y, z, a, u: _ } => {
                if let Some(v) = x { self.position.x = *v; }
                if let Some(v) = y { self.position.y = *v; }
                if let Some(v) = z { self.position.z = *v; }
                if let Some(v) = a { self.a_axis = *v; }
            }
            GCommand::LinearFeed { x, y, z, a, u: _, f: _ } => {
                let prev_pos = self.position;
                let prev_a = self.a_axis;
                if let Some(v) = x { self.position.x = *v; }
                if let Some(v) = y { self.position.y = *v; }
                if let Some(v) = z { self.position.z = *v; }
                if let Some(v) = a { self.a_axis = *v; }

                if self.spindle_on && (self.a_axis - prev_a).abs() < 1e-6 {
                    cut_segment(
                        &self.tool,
                        &mut self.workpiece,
                        &prev_pos,
                        &self.position,
                        self.a_axis,
                        self.half_extent,
                    );
                    self.cut_count += 1;
                }
            }
            GCommand::ArcCW { x, y, z, f: _, .. }
            | GCommand::ArcCCW { x, y, z, f: _, .. } => {
                let prev_pos = self.position;
                if let Some(v) = x { self.position.x = *v; }
                if let Some(v) = y { self.position.y = *v; }
                if let Some(v) = z { self.position.z = *v; }

                if self.spindle_on {
                    cut_segment(
                        &self.tool,
                        &mut self.workpiece,
                        &prev_pos,
                        &self.position,
                        self.a_axis,
                        self.half_extent,
                    );
                    self.cut_count += 1;
                }
            }
            GCommand::Comment(_) => {}
        }
    }
}

/// Apply a single cut segment in SDF-local coordinates.
///
/// Coordinate transform: gcode_pos + (0,0,half_extent) → rotated by -a_angle around X.
pub fn cut_segment(
    tool: &Tool,
    workpiece: &mut OctreeSdf,
    start: &Vector3<f64>,
    end: &Vector3<f64>,
    a_angle: f64,
    half_extent: f64,
) {
    let he_vec = Vector3::new(0.0, 0.0, half_extent);
    let rot_inv = nalgebra::Rotation3::from_axis_angle(&Vector3::x_axis(), -a_angle);

    let start_local = rot_inv * (*start + he_vec);
    let end_local = rot_inv * (*end + he_vec);
    let tool_axis_local = rot_inv * Vector3::new(0.0, 0.0, 1.0);

    // Quick AABB reject — skip segments entirely outside workpiece bounds
    let bound = half_extent + 0.02;
    let s_inside = start_local.x.abs() <= bound
        && start_local.y.abs() <= bound
        && start_local.z.abs() <= bound;
    let e_inside = end_local.x.abs() <= bound
        && end_local.y.abs() <= bound
        && end_local.z.abs() <= bound;
    if !s_inside && !e_inside {
        return;
    }

    let tool_sdf = tool.swept_sdf(&start_local, &end_local, &tool_axis_local);
    let (bounds_min, bounds_max) = tool.swept_bounds(&start_local, &end_local, &tool_axis_local);
    workpiece.subtract(bounds_min, bounds_max, tool_sdf);
}
