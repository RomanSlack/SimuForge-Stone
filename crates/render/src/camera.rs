//! Orbit camera with projection matrices.

use glam::{Mat4, Vec3};

/// Orbit camera that revolves around a target point.
pub struct OrbitCamera {
    /// Target point (world space).
    pub target: Vec3,
    /// Distance from target.
    pub distance: f32,
    /// Azimuth angle (rad) — horizontal rotation.
    pub yaw: f32,
    /// Elevation angle (rad) — vertical rotation.
    pub pitch: f32,
    /// Vertical FOV (rad).
    pub fov: f32,
    /// Near clipping plane.
    pub near: f32,
    /// Far clipping plane.
    pub far: f32,
}

impl OrbitCamera {
    pub fn new() -> Self {
        Self {
            // Look at midpoint between arm and workpiece.
            // Arm at origin, workpiece center at render (0.85, -0.10, 0.0).
            target: Vec3::new(0.45, 0.0, 0.0),
            distance: 2.2,
            yaw: 0.7,
            pitch: 0.35,
            fov: std::f32::consts::FRAC_PI_4,
            near: 0.01,
            far: 100.0,
        }
    }

    /// Camera position in world space.
    pub fn eye(&self) -> Vec3 {
        let x = self.distance * self.pitch.cos() * self.yaw.sin();
        let y = self.distance * self.pitch.sin();
        let z = self.distance * self.pitch.cos() * self.yaw.cos();
        self.target + Vec3::new(x, y, z)
    }

    /// View matrix (world → camera).
    pub fn view_matrix(&self) -> Mat4 {
        Mat4::look_at_rh(self.eye(), self.target, Vec3::Y)
    }

    /// Projection matrix.
    pub fn projection_matrix(&self, aspect: f32) -> Mat4 {
        Mat4::perspective_rh(self.fov, aspect, self.near, self.far)
    }

    /// Combined view-projection matrix.
    pub fn view_projection(&self, aspect: f32) -> Mat4 {
        self.projection_matrix(aspect) * self.view_matrix()
    }

    /// Rotate camera by delta yaw/pitch (from mouse drag).
    pub fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.yaw += delta_yaw;
        self.pitch = (self.pitch + delta_pitch).clamp(-1.5, 1.5);
    }

    /// Zoom (change distance).
    pub fn zoom(&mut self, delta: f32) {
        self.distance = (self.distance * (1.0 - delta * 0.1)).clamp(0.1, 50.0);
    }

    /// Pan (move target).
    pub fn pan(&mut self, dx: f32, dy: f32) {
        let right = Vec3::new(self.yaw.cos(), 0.0, -self.yaw.sin());
        let up = Vec3::Y;
        self.target += right * dx * self.distance * 0.002;
        self.target += up * dy * self.distance * 0.002;
    }

    /// Snap to look at the workpiece (render Y-up space).
    /// Workpiece center DH (0.85, 0, -0.10) → render (0.85, -0.10, 0.0).
    pub fn snap_to_workpiece(&mut self) {
        self.target = Vec3::new(0.85, -0.10, 0.0);
        self.distance = 0.7;
        self.yaw = 0.5;
        self.pitch = 0.3;
    }

    /// Snap to look at robot arm (render Y-up space).
    pub fn snap_to_arm(&mut self) {
        self.target = Vec3::new(0.45, 0.0, 0.0);
        self.distance = 2.2;
        self.yaw = 0.7;
        self.pitch = 0.35;
    }
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self::new()
    }
}

/// GPU-uploadable camera uniform.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    pub view_proj: [[f32; 4]; 4],
    pub view: [[f32; 4]; 4],
    pub proj: [[f32; 4]; 4],
    pub eye_pos: [f32; 4], // w unused, for alignment
}

impl CameraUniform {
    pub fn from_camera(camera: &OrbitCamera, aspect: f32) -> Self {
        let view = camera.view_matrix();
        let proj = camera.projection_matrix(aspect);
        let vp = proj * view;
        let eye = camera.eye();

        Self {
            view_proj: vp.to_cols_array_2d(),
            view: view.to_cols_array_2d(),
            proj: proj.to_cols_array_2d(),
            eye_pos: [eye.x, eye.y, eye.z, 1.0],
        }
    }
}
