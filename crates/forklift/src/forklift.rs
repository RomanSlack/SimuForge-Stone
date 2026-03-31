use nalgebra::Vector3;

// Toyota 8FBN25 constants
const WHEELBASE: f64 = 1.525;
const MASS_UNLADEN: f64 = 4740.0;
const MAX_STEER_ANGLE: f64 = 1.396; // ±80°
const STEER_RATE: f64 = 0.5; // rad/s (EPS motor)
const MAX_SPEED_UNLOADED: f64 = 4.4; // m/s (16 km/h)
const MAX_SPEED_LOADED: f64 = 3.9; // m/s (14 km/h)
const MAX_DRAWBAR_PULL: f64 = 10_000.0; // N
const ROLLING_RESISTANCE: f64 = 0.02;
const MAX_BRAKE_DECEL: f64 = 3.0; // m/s²
const FRONT_WHEEL_RADIUS: f64 = 0.265; // m
const REAR_WHEEL_RADIUS: f64 = 0.20; // m

// Mast constants
const FORK_MIN_HEIGHT: f64 = 0.03; // m (forks nearly touching ground)
const FORK_MAX_HEIGHT: f64 = 3.0; // m
const LIFT_SPEED_LOADED: f64 = 0.34; // m/s
const LIFT_SPEED_UNLOADED: f64 = 0.60; // m/s
const LOWER_SPEED_LOADED: f64 = 0.50; // m/s
const LOWER_SPEED_UNLOADED: f64 = 0.45; // m/s
const TILT_SPEED: f64 = 0.087; // rad/s (~5°/s)
const TILT_FORWARD_MAX: f64 = -0.1047; // -6° (forward tilt is negative)
const TILT_BACKWARD_MAX: f64 = 0.2094; // +12°

pub struct LoadState {
    pub mass: f64,
    pub cg_height: f64,
}

pub struct ForkliftState {
    // Chassis rigid body (Z-up DH frame)
    pub position: Vector3<f64>,
    pub heading: f64,
    pub velocity: f64,
    pub angular_velocity: f64,

    // Steering
    pub steer_angle: f64,
    pub steer_cmd: f64,

    // Drive
    pub throttle: f64,
    pub brake: f64,

    // Mast
    pub mast_tilt: f64,
    pub mast_tilt_cmd: f64,
    pub fork_height: f64,
    pub fork_height_cmd: f64,

    // Wheel rotation (visual)
    pub front_wheel_angle: f64,
    pub rear_wheel_angle: f64,

    // Load
    pub carried_load: Option<LoadState>,
}

impl ForkliftState {
    pub fn new() -> Self {
        Self {
            position: Vector3::new(0.0, 0.0, 0.0),
            heading: 0.0,
            velocity: 0.0,
            angular_velocity: 0.0,
            steer_angle: 0.0,
            steer_cmd: 0.0,
            throttle: 0.0,
            brake: 0.0,
            mast_tilt: 0.0,
            mast_tilt_cmd: 0.0,
            fork_height: FORK_MIN_HEIGHT,
            fork_height_cmd: FORK_MIN_HEIGHT,
            front_wheel_angle: 0.0,
            rear_wheel_angle: 0.0,
            carried_load: None,
        }
    }

    pub fn total_mass(&self) -> f64 {
        MASS_UNLADEN + self.carried_load.as_ref().map_or(0.0, |l| l.mass)
    }

    pub fn max_speed(&self) -> f64 {
        if self.carried_load.is_some() {
            MAX_SPEED_LOADED
        } else {
            MAX_SPEED_UNLOADED
        }
    }

    pub fn step(&mut self, dt: f64) {
        let mass = self.total_mass();
        let g = 9.81;

        // --- Steering ---
        let steer_error = self.steer_cmd - self.steer_angle;
        let max_steer_delta = STEER_RATE * dt;
        self.steer_angle += steer_error.clamp(-max_steer_delta, max_steer_delta);
        self.steer_angle = self.steer_angle.clamp(-MAX_STEER_ANGLE, MAX_STEER_ANGLE);

        // --- Drive force ---
        let drive_force = self.throttle * MAX_DRAWBAR_PULL;
        let rolling_force = -self.velocity.signum() * ROLLING_RESISTANCE * mass * g;
        let brake_force = if self.brake > 0.0 {
            -self.velocity.signum() * self.brake * MAX_BRAKE_DECEL * mass
        } else {
            0.0
        };

        // Damping when no throttle and no brake (slow to a stop)
        let idle_damping = if self.throttle.abs() < 0.01 && self.brake < 0.01 {
            -self.velocity * 2.0 * mass
        } else {
            0.0
        };

        let net_force = drive_force + rolling_force + brake_force + idle_damping;
        let accel = net_force / mass;
        self.velocity += accel * dt;

        // Speed limiter
        let max_v = self.max_speed();
        self.velocity = self.velocity.clamp(-max_v * 0.5, max_v); // reverse at half speed

        // Stop completely at very low speeds when braking
        if self.brake > 0.0 && self.velocity.abs() < 0.05 {
            self.velocity = 0.0;
        }

        // --- Bicycle model kinematics (rear-wheel steering) ---
        // Reference point: rear axle center
        // For rear-wheel steer: heading_rate = v * tan(steer) / L
        // BUT the rear wheels steer, so the geometry is:
        //   d_heading = v * tan(steer_angle) / wheelbase
        // Position update uses the front axle as the instantaneous center reference
        let cos_h = self.heading.cos();
        let sin_h = self.heading.sin();

        if self.steer_angle.abs() > 0.001 {
            let tan_steer = self.steer_angle.tan();
            self.angular_velocity = self.velocity * tan_steer / WHEELBASE;
        } else {
            self.angular_velocity = 0.0;
        }

        self.heading += self.angular_velocity * dt;
        // Normalize heading to [-PI, PI]
        while self.heading > std::f64::consts::PI {
            self.heading -= std::f64::consts::TAU;
        }
        while self.heading < -std::f64::consts::PI {
            self.heading += std::f64::consts::TAU;
        }

        self.position.x += self.velocity * cos_h * dt;
        self.position.y += self.velocity * sin_h * dt;

        // --- Wheel rotation (visual) ---
        self.front_wheel_angle += self.velocity * dt / FRONT_WHEEL_RADIUS;
        self.rear_wheel_angle += self.velocity * dt / REAR_WHEEL_RADIUS;

        // --- Mast lift ---
        let loaded = self.carried_load.is_some();
        let lift_error = self.fork_height_cmd - self.fork_height;
        let lift_speed = if lift_error > 0.0 {
            if loaded { LIFT_SPEED_LOADED } else { LIFT_SPEED_UNLOADED }
        } else {
            if loaded { LOWER_SPEED_LOADED } else { LOWER_SPEED_UNLOADED }
        };
        let max_lift_delta = lift_speed * dt;
        self.fork_height += lift_error.clamp(-max_lift_delta, max_lift_delta);
        self.fork_height = self.fork_height.clamp(FORK_MIN_HEIGHT, FORK_MAX_HEIGHT);

        // --- Mast tilt ---
        let tilt_error = self.mast_tilt_cmd - self.mast_tilt;
        let max_tilt_delta = TILT_SPEED * dt;
        self.mast_tilt += tilt_error.clamp(-max_tilt_delta, max_tilt_delta);
        self.mast_tilt = self.mast_tilt.clamp(TILT_FORWARD_MAX, TILT_BACKWARD_MAX);
    }

    pub fn speed_kmh(&self) -> f64 {
        self.velocity.abs() * 3.6
    }

    /// Fork tip center position in DH world frame (Z-up).
    /// The forklift origin is at the rear axle. Front axle is at x = +wheelbase forward.
    /// The mast is ~0.175m beyond the front axle. Fork tips extend 1.07m further.
    pub fn fork_tip_center(&self) -> Vector3<f64> {
        let cos_h = self.heading.cos();
        let sin_h = self.heading.sin();
        // Distance from rear axle to fork tips along forward axis
        let fork_reach = WHEELBASE + 0.175 + 1.07;
        Vector3::new(
            self.position.x + cos_h * fork_reach,
            self.position.y + sin_h * fork_reach,
            self.fork_height + 0.02, // fork surface height
        )
    }

    /// Fork base center (at carriage) in DH world frame.
    pub fn fork_base_center(&self) -> Vector3<f64> {
        let cos_h = self.heading.cos();
        let sin_h = self.heading.sin();
        let mast_dist = WHEELBASE + 0.175;
        Vector3::new(
            self.position.x + cos_h * mast_dist,
            self.position.y + sin_h * mast_dist,
            self.fork_height + 0.02,
        )
    }
}
