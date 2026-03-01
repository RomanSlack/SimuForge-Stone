//! 5-DOF ping pong arm configuration: DH params, motors, PID, gearboxes.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Matrix3, Vector3};

use simuforge_core::DhParams;
use simuforge_motors::gearbox::Gearbox;
use simuforge_motors::pid::PidController;
use simuforge_motors::servo::{ServoMotor, ServoState};
use simuforge_physics::arm::RobotArm;
use simuforge_physics::joint::RevoluteJoint;
use simuforge_physics::spatial::spatial_inertia;

/// Table height in meters (standard ping pong table).
pub const TABLE_HEIGHT: f64 = 0.76;

/// Arm base offset from table center along X (m).
/// Player 1 at -1.5m, Player 2 at +1.5m.
pub const ARM_X_OFFSET: f64 = 1.5;

/// Paddle half-extents for collision (m). Standard paddle is ~15cm diameter.
pub const PADDLE_RADIUS: f64 = 0.075;
/// Paddle thickness (m).
pub const PADDLE_THICKNESS: f64 = 0.008;

/// Arm link rendering radii (m).
pub const ARM_LINK_RADII: [f32; 5] = [0.05, 0.04, 0.035, 0.03, 0.025];

/// Arm link colors — Player 1 (blue team).
pub const P1_LINK_COLORS: [[f32; 4]; 5] = [
    [0.15, 0.15, 0.20, 1.0], // J1: dark blue-grey base
    [0.20, 0.40, 0.80, 1.0], // J2: blue upper arm
    [0.20, 0.40, 0.80, 1.0], // J3: blue forearm
    [0.15, 0.15, 0.20, 1.0], // J4: dark wrist
    [0.55, 0.55, 0.60, 1.0], // J5: aluminum paddle mount
];

/// Arm link colors — Player 2 (red team).
pub const P2_LINK_COLORS: [[f32; 4]; 5] = [
    [0.20, 0.15, 0.15, 1.0], // J1: dark red-grey base
    [0.80, 0.25, 0.20, 1.0], // J2: red upper arm
    [0.80, 0.25, 0.20, 1.0], // J3: red forearm
    [0.20, 0.15, 0.15, 1.0], // J4: dark wrist
    [0.55, 0.55, 0.60, 1.0], // J5: aluminum paddle mount
];

/// Paddle color — Player 1 (red rubber side).
pub const P1_PADDLE_COLOR: [f32; 4] = [0.75, 0.10, 0.10, 1.0];
/// Paddle color — Player 2 (black rubber side).
pub const P2_PADDLE_COLOR: [f32; 4] = [0.12, 0.12, 0.12, 1.0];

/// Create a 5-DOF ping pong arm.
///
/// The arm is positioned at the given base_position in DH world space.
/// `mirror` = true for Player 2 (adds pi to J1 theta_offset so it faces the opposite direction).
pub fn create_pingpong_arm(mirror: bool) -> RobotArm {
    let theta_offset_j1 = if mirror { std::f64::consts::PI } else { 0.0 };

    let dh_params = vec![
        DhParams {
            a: 0.0,
            d: 0.15,
            alpha: -FRAC_PI_2,
            theta_offset: theta_offset_j1,
        }, // J1: shoulder yaw, 15cm pedestal
        DhParams::new(0.30, 0.0, 0.0),    // J2: shoulder pitch, 30cm upper arm
        DhParams::new(0.25, 0.0, 0.0),    // J3: elbow, 25cm forearm
        DhParams {
            a: 0.0,
            d: 0.0,
            alpha: -FRAC_PI_2,
            theta_offset: 0.0,
        }, // J4: wrist pitch coupling
        DhParams::new(0.0, 0.12, 0.0),    // J5: wrist roll, 12cm to paddle
    ];

    let joints = vec![
        // Shoulder: fast, wide range
        RevoluteJoint::new(-180.0, 180.0).with_friction(1.0, 0.3).with_velocity_limit(12.0),
        // Shoulder pitch
        RevoluteJoint::new(-90.0, 135.0).with_friction(1.0, 0.3).with_velocity_limit(10.0),
        // Elbow
        RevoluteJoint::new(-150.0, 150.0).with_friction(0.5, 0.2).with_velocity_limit(14.0),
        // Wrist pitch
        RevoluteJoint::new(-120.0, 120.0).with_friction(0.2, 0.1).with_velocity_limit(18.0),
        // Wrist roll
        RevoluteJoint::new(-180.0, 180.0).with_friction(0.1, 0.05).with_velocity_limit(20.0),
    ];

    let link_masses = vec![3.0, 2.0, 1.5, 0.8, 0.5];

    let link_spatial_inertias = dh_params
        .iter()
        .zip(link_masses.iter())
        .map(|(dh, &mass)| {
            let length = (dh.a.abs() + dh.d.abs()).max(0.05);
            let radius = 0.03;
            let ixx = mass * (3.0 * radius * radius + length * length) / 12.0;
            let izz = mass * radius * radius / 2.0;
            let inertia = Matrix3::new(ixx, 0.0, 0.0, 0.0, ixx, 0.0, 0.0, 0.0, izz);
            let com = Vector3::new(dh.a / 2.0, 0.0, dh.d / 2.0);
            spatial_inertia(mass, &com, &inertia)
        })
        .collect();

    // Reflected motor inertia: servo_inertia * gear_ratio^2
    let motor_reflected_inertias = vec![
        0.0003 * 10.0_f64.powi(2), // J1: 0.03
        0.0003 * 10.0_f64.powi(2), // J2: 0.03
        0.0002 * 8.0_f64.powi(2),  // J3: 0.013 (rounded)
        0.0001 * 5.0_f64.powi(2),  // J4: 0.0025
        0.0001 * 5.0_f64.powi(2),  // J5: 0.0025
    ];

    let link_names = vec![
        "Shoulder Yaw".into(),
        "Upper Arm".into(),
        "Forearm".into(),
        "Wrist Pitch".into(),
        "Wrist Roll/Paddle".into(),
    ];

    RobotArm {
        dh_params,
        joints,
        link_spatial_inertias,
        link_masses,
        motor_reflected_inertias,
        link_names,
        tool_offset: 0.0, // paddle is at the end-effector, no extra offset
    }
}

/// Create servo motors for the 5-DOF arm.
pub fn create_servo_motors() -> Vec<ServoState> {
    vec![
        ServoState::new(ServoMotor::new(8.4, 600.0, 0.0003)),  // J1: shoulder
        ServoState::new(ServoMotor::new(8.4, 600.0, 0.0003)),  // J2: shoulder pitch
        ServoState::new(ServoMotor::new(4.0, 800.0, 0.0002)),  // J3: elbow
        ServoState::new(ServoMotor::new(2.0, 1000.0, 0.0001)), // J4: wrist pitch
        ServoState::new(ServoMotor::new(2.0, 1000.0, 0.0001)), // J5: wrist roll
    ]
}

/// Create gearboxes for the 5-DOF arm (low ratio for speed).
pub fn create_gearboxes() -> Vec<Gearbox> {
    vec![
        Gearbox::planetary(10.0), // J1
        Gearbox::planetary(10.0), // J2
        Gearbox::planetary(8.0),  // J3
        Gearbox::planetary(5.0),  // J4
        Gearbox::planetary(5.0),  // J5
    ]
}

/// Create PID controllers for the 5-DOF arm.
/// Slightly underdamped (zeta ~ 0.8) for fast response.
pub fn create_pid_controllers() -> Vec<PidController> {
    vec![
        PidController::new(120.0, 20.0, 4.0, 200.0),  // J1: shoulder
        PidController::new(120.0, 20.0, 4.0, 200.0),  // J2: shoulder pitch
        PidController::new(80.0, 15.0, 2.5, 150.0),   // J3: elbow
        PidController::new(40.0, 8.0, 1.0, 80.0),     // J4: wrist pitch
        PidController::new(40.0, 8.0, 1.0, 80.0),     // J5: wrist roll
    ]
}

/// Paddle head dimensions for collision (keep original for physics).
pub const PADDLE_HEAD_WIDTH: f32 = 0.15;  // 15cm wide
pub const PADDLE_HEAD_HEIGHT: f32 = 0.17; // 17cm tall (slightly taller than wide)
pub const PADDLE_HEAD_THICKNESS: f32 = 0.007; // 7mm (wood + rubber)
/// Handle dimensions.
pub const PADDLE_HANDLE_LENGTH: f32 = 0.10; // 10cm handle
pub const PADDLE_HANDLE_WIDTH: f32 = 0.028; // 2.8cm wide (flat grip)
pub const PADDLE_HANDLE_DEPTH: f32 = 0.022; // 2.2cm deep

/// Generate a paddle mesh: egg-shaped head + flat rectangular handle.
/// Oriented along Y-axis: handle bottom at Y=0 (grip point), head above.
/// Head center at Y = handle_length + head_height/2.
pub fn generate_paddle_mesh() -> (Vec<simuforge_core::Vertex>, Vec<u32>) {
    use simuforge_core::Vertex;
    use std::f32::consts::TAU;

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let segments = 24u32;

    let hlen = PADDLE_HANDLE_LENGTH;
    let hw = PADDLE_HANDLE_WIDTH / 2.0;
    let hd = PADDLE_HANDLE_DEPTH / 2.0;

    // --- Handle: flat rectangular prism from Y=0 to Y=hlen ---
    // Front face (Z = +hd)
    let add_quad = |verts: &mut Vec<Vertex>, idxs: &mut Vec<u32>,
                    p0: [f32;3], p1: [f32;3], p2: [f32;3], p3: [f32;3], n: [f32;3]| {
        let b = verts.len() as u32;
        verts.push(Vertex { position: p0, normal: n });
        verts.push(Vertex { position: p1, normal: n });
        verts.push(Vertex { position: p2, normal: n });
        verts.push(Vertex { position: p3, normal: n });
        idxs.extend_from_slice(&[b, b+1, b+2, b, b+2, b+3]);
    };

    // Front (+Z)
    add_quad(&mut vertices, &mut indices,
        [-hw, 0.0, hd], [hw, 0.0, hd], [hw, hlen, hd], [-hw, hlen, hd],
        [0.0, 0.0, 1.0]);
    // Back (-Z)
    add_quad(&mut vertices, &mut indices,
        [hw, 0.0, -hd], [-hw, 0.0, -hd], [-hw, hlen, -hd], [hw, hlen, -hd],
        [0.0, 0.0, -1.0]);
    // Right (+X)
    add_quad(&mut vertices, &mut indices,
        [hw, 0.0, hd], [hw, 0.0, -hd], [hw, hlen, -hd], [hw, hlen, hd],
        [1.0, 0.0, 0.0]);
    // Left (-X)
    add_quad(&mut vertices, &mut indices,
        [-hw, 0.0, -hd], [-hw, 0.0, hd], [-hw, hlen, hd], [-hw, hlen, -hd],
        [-1.0, 0.0, 0.0]);
    // Bottom (Y=0)
    add_quad(&mut vertices, &mut indices,
        [-hw, 0.0, -hd], [hw, 0.0, -hd], [hw, 0.0, hd], [-hw, 0.0, hd],
        [0.0, -1.0, 0.0]);

    // --- Head: egg-shaped disc (ellipse wider at top, narrower at bottom near handle) ---
    let head_w = PADDLE_HEAD_WIDTH / 2.0;
    let head_h = PADDLE_HEAD_HEIGHT / 2.0;
    let pt = PADDLE_HEAD_THICKNESS / 2.0;
    // Head center Y
    let cy = hlen + head_h;

    // Egg shape: radius varies around the ellipse.
    // At angle=π/2 (top) = full width, at angle=-π/2 (bottom/handle junction) = narrower
    let egg_radius = |angle: f32| -> (f32, f32) {
        let (sin_a, cos_a) = angle.sin_cos();
        // Egg shape: top wider, bottom narrower where it meets handle
        let narrow = 1.0 + 0.12 * sin_a; // sin=1 (top) → 1.12, sin=-1 (bottom) → 0.88
        let rx = head_w * narrow;
        let ry = head_h;
        (rx * cos_a, ry * sin_a)
    };

    // Front face of head (+Z = pt)
    let base_v = vertices.len() as u32;
    vertices.push(Vertex { position: [0.0, cy, pt], normal: [0.0, 0.0, 1.0] }); // center
    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * TAU;
        let (ex, ey) = egg_radius(angle);
        vertices.push(Vertex { position: [ex, cy + ey, pt], normal: [0.0, 0.0, 1.0] });
    }
    for i in 0..segments {
        indices.extend_from_slice(&[base_v, base_v + 1 + i, base_v + 2 + i]);
    }

    // Back face of head (-Z = -pt)
    let base_v = vertices.len() as u32;
    vertices.push(Vertex { position: [0.0, cy, -pt], normal: [0.0, 0.0, -1.0] });
    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * TAU;
        let (ex, ey) = egg_radius(angle);
        vertices.push(Vertex { position: [ex, cy + ey, -pt], normal: [0.0, 0.0, -1.0] });
    }
    for i in 0..segments {
        indices.extend_from_slice(&[base_v, base_v + 2 + i, base_v + 1 + i]); // reversed winding
    }

    // Edge rim of head (connects front to back around perimeter)
    let base_v = vertices.len() as u32;
    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * TAU;
        let (ex, ey) = egg_radius(angle);
        // Outward-pointing normal for the rim
        let nx = ex;
        let ny = ey;
        let len = (nx * nx + ny * ny).sqrt();
        let (nnx, nny) = if len > 1e-6 { (nx / len, ny / len) } else { (1.0, 0.0) };
        vertices.push(Vertex { position: [ex, cy + ey, pt], normal: [nnx, nny, 0.0] });
        vertices.push(Vertex { position: [ex, cy + ey, -pt], normal: [nnx, nny, 0.0] });
    }
    for i in 0..segments {
        let b = base_v + i * 2;
        indices.extend_from_slice(&[b, b+2, b+1, b+1, b+2, b+3]);
    }

    (vertices, indices)
}

/// Ready position joint angles for Player 1 (arm hovering over their side).
pub fn ready_position_p1() -> Vec<f64> {
    vec![
        0.0,                            // J1: facing forward
        0.5,                            // J2: shoulder slightly raised
        -0.8,                           // J3: elbow bent
        0.3,                            // J4: wrist angled
        0.0,                            // J5: paddle neutral
    ]
}

/// Ready position joint angles for Player 2 (mirrored arm).
pub fn ready_position_p2() -> Vec<f64> {
    vec![
        0.0,                            // J1: facing forward (theta_offset handles mirroring)
        0.5,                            // J2: shoulder slightly raised
        -0.8,                           // J3: elbow bent
        0.3,                            // J4: wrist angled
        0.0,                            // J5: paddle neutral
    ]
}
