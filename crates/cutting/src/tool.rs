//! Tool geometry definitions for CNC cutting tools.

use nalgebra::Vector3;

/// Tool type enumeration.
#[derive(Debug, Clone, Copy)]
pub enum ToolType {
    /// Ball nose end mill — hemispherical tip.
    BallNose,
    /// Flat end mill — cylindrical with flat bottom.
    FlatEnd,
    /// Tapered ball nose — conical with spherical tip.
    TaperedBall {
        /// Half-angle of the taper (rad).
        taper_angle: f64,
    },
}

/// Cutting tool geometry and state.
#[derive(Debug, Clone)]
pub struct Tool {
    /// Tool type.
    pub tool_type: ToolType,
    /// Tool cutting radius (m).
    pub radius: f64,
    /// Effective cutting length (m).
    pub cutting_length: f64,
    /// Shank diameter (m).
    pub shank_diameter: f64,
    /// Spindle RPM.
    pub rpm: f64,
    /// Accumulated wear (dimensionless, 0 = new).
    pub wear: f64,
}

impl Tool {
    /// Create a ball-nose end mill.
    pub fn ball_nose(radius_mm: f64, cutting_length_mm: f64) -> Self {
        Self {
            tool_type: ToolType::BallNose,
            radius: radius_mm * 0.001,
            cutting_length: cutting_length_mm * 0.001,
            shank_diameter: radius_mm * 2.0 * 0.001,
            rpm: 10000.0,
            wear: 0.0,
        }
    }

    /// Create a flat end mill.
    pub fn flat_end(radius_mm: f64, cutting_length_mm: f64) -> Self {
        Self {
            tool_type: ToolType::FlatEnd,
            radius: radius_mm * 0.001,
            cutting_length: cutting_length_mm * 0.001,
            shank_diameter: radius_mm * 2.0 * 0.001,
            rpm: 10000.0,
            wear: 0.0,
        }
    }

    /// Compute the SDF of the tool swept volume between two positions.
    /// Returns a closure suitable for OctreeSdf::subtract().
    ///
    /// Positions are in world space (meters).
    pub fn swept_sdf(
        &self,
        start: &Vector3<f64>,
        end: &Vector3<f64>,
    ) -> impl Fn(f32, f32, f32) -> f32 {
        let ax = start.x as f32;
        let ay = start.y as f32;
        let az = start.z as f32;
        let bx = end.x as f32;
        let by = end.y as f32;
        let bz = end.z as f32;
        let r = self.radius as f32;

        move |px: f32, py: f32, pz: f32| -> f32 {
            simuforge_material_sdf_capsule(px, py, pz, ax, ay, az, bx, by, bz, r)
        }
    }

    /// Bounding box of the swept volume between two positions, with padding.
    pub fn swept_bounds(
        &self,
        start: &Vector3<f64>,
        end: &Vector3<f64>,
    ) -> ([f32; 3], [f32; 3]) {
        let r = self.radius as f32;
        let pad = r + 0.002; // extra padding for SDF margin
        let min = [
            (start.x.min(end.x) as f32) - pad,
            (start.y.min(end.y) as f32) - pad,
            (start.z.min(end.z) as f32) - pad,
        ];
        let max = [
            (start.x.max(end.x) as f32) + pad,
            (start.y.max(end.y) as f32) + pad,
            (start.z.max(end.z) as f32) + pad,
        ];
        (min, max)
    }

    /// Effective cutting speed (m/s) at the tool tip.
    pub fn cutting_speed(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.radius * self.rpm / 60.0
    }

    /// Accumulate tool wear based on material removed volume.
    pub fn accumulate_wear(&mut self, volume_removed: f64, material_hardness: f64) {
        // Simplified Taylor tool life model
        self.wear += volume_removed * material_hardness * 1e-6;
    }
}

/// Capsule SDF (inline, to avoid cross-crate dependency on material).
fn simuforge_material_sdf_capsule(
    px: f32,
    py: f32,
    pz: f32,
    ax: f32,
    ay: f32,
    az: f32,
    bx: f32,
    by: f32,
    bz: f32,
    radius: f32,
) -> f32 {
    let pax = px - ax;
    let pay = py - ay;
    let paz = pz - az;
    let bax = bx - ax;
    let bay = by - ay;
    let baz = bz - az;

    let ba_dot = bax * bax + bay * bay + baz * baz;
    let t = if ba_dot > 1e-12 {
        ((pax * bax + pay * bay + paz * baz) / ba_dot).clamp(0.0, 1.0)
    } else {
        0.0
    };

    let cx = ax + t * bax - px;
    let cy = ay + t * bay - py;
    let cz = az + t * baz - pz;

    (cx * cx + cy * cy + cz * cz).sqrt() - radius
}

/// Tool position and orientation state.
#[derive(Debug, Clone)]
pub struct ToolState {
    /// Tool center point position in world frame (m).
    pub position: Vector3<f64>,
    /// Previous position (for swept volume computation).
    pub prev_position: Vector3<f64>,
    /// Tool axis direction (unit vector, typically Z in tool frame).
    pub axis: Vector3<f64>,
    /// Whether the spindle is running.
    pub spindle_on: bool,
    /// Start-of-frame position for accumulated swept path.
    pub frame_start_position: Vector3<f64>,
    /// Whether any material removal happened this frame.
    pub frame_dirty: bool,
}

impl ToolState {
    pub fn new() -> Self {
        Self {
            position: Vector3::zeros(),
            prev_position: Vector3::zeros(),
            axis: Vector3::z(),
            spindle_on: false,
            frame_start_position: Vector3::zeros(),
            frame_dirty: false,
        }
    }

    /// Update position (called each physics step).
    pub fn update_position(&mut self, new_pos: Vector3<f64>) {
        self.prev_position = self.position;
        self.position = new_pos;
    }

    /// Total displacement accumulated since frame start.
    pub fn frame_displacement(&self) -> f64 {
        (self.position - self.frame_start_position).norm()
    }

    /// Mark the start of a new render frame. Returns (start, end) if
    /// the tool moved enough for material removal.
    pub fn begin_frame(&mut self) -> Option<(Vector3<f64>, Vector3<f64>)> {
        let disp = self.frame_displacement();
        let start = self.frame_start_position;
        let end = self.position;
        self.frame_start_position = self.position;
        self.frame_dirty = false;
        if disp > 0.0003 {
            // Tool moved at least 0.3mm (sub-voxel threshold)
            Some((start, end))
        } else {
            None
        }
    }

    /// Distance moved since last physics step.
    pub fn displacement(&self) -> f64 {
        (self.position - self.prev_position).norm()
    }
}

impl Default for ToolState {
    fn default() -> Self {
        Self::new()
    }
}
