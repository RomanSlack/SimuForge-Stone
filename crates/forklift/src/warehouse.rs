use nalgebra::Vector3;

/// A pallet sitting in the warehouse.
pub struct Pallet {
    /// Position in DH frame (Z-up), bottom-center of pallet.
    pub position: Vector3<f64>,
    /// Heading angle (rad) in DH frame.
    pub heading: f64,
    /// Whether this pallet has been picked up.
    pub picked_up: bool,
    /// Cargo boxes stacked on this pallet (height offsets in render Y).
    pub cargo_heights: Vec<f64>,
    /// Total mass of pallet + cargo.
    pub mass: f64,
}

impl Pallet {
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        Self {
            position: Vector3::new(x, y, 0.0),
            heading,
            picked_up: false,
            cargo_heights: vec![0.144, 0.544], // 2 boxes stacked
            mass: 500.0,
        }
    }

    pub fn empty(x: f64, y: f64, heading: f64) -> Self {
        Self {
            position: Vector3::new(x, y, 0.0),
            heading,
            picked_up: false,
            cargo_heights: vec![],
            mass: 25.0,
        }
    }
}

/// Static wall in render space (Y-up).
pub struct Wall {
    /// Center position in render space.
    pub render_center: [f32; 3],
    /// Half-extents in render space [hx, hy, hz].
    pub render_half_extents: [f32; 3],
}

/// Shelf rack (simplified).
pub struct ShelfRack {
    pub render_center: [f32; 3],
    pub heading: f64,
    pub length: f64,
    pub levels: usize,
}

/// The warehouse environment.
pub struct Warehouse {
    pub pallets: Vec<Pallet>,
    pub walls: Vec<Wall>,
    pub shelves: Vec<ShelfRack>,
    pub floor_half_size: f64,
}

impl Warehouse {
    pub fn new() -> Self {
        let floor_half_size = 25.0;
        let wall_h = 6.0;
        let wall_thick = 0.3;
        let hs = floor_half_size as f32;

        // Walls in render space (X=right, Y=up, Z=backward)
        let walls = vec![
            // Back wall (at +Z edge)
            Wall {
                render_center: [0.0, wall_h / 2.0, hs - wall_thick / 2.0],
                render_half_extents: [hs, wall_h / 2.0, wall_thick / 2.0],
            },
            // Left wall (at -X edge)
            Wall {
                render_center: [-hs + wall_thick / 2.0, wall_h / 2.0, hs / 2.0],
                render_half_extents: [wall_thick / 2.0, wall_h / 2.0, hs / 2.0],
            },
            // Right wall (at +X edge)
            Wall {
                render_center: [hs - wall_thick / 2.0, wall_h / 2.0, hs / 2.0],
                render_half_extents: [wall_thick / 2.0, wall_h / 2.0, hs / 2.0],
            },
        ];

        let shelves = vec![
            ShelfRack {
                render_center: [0.0, 0.0, -10.0],
                heading: 0.0,
                length: 12.0,
                levels: 3,
            },
            ShelfRack {
                render_center: [8.0, 0.0, -10.0],
                heading: 0.0,
                length: 12.0,
                levels: 3,
            },
            ShelfRack {
                render_center: [-8.0, 0.0, -10.0],
                heading: 0.0,
                length: 12.0,
                levels: 3,
            },
        ];

        // Pallets in DH coordinates (will be converted to render by to_render)
        let pallets = vec![
            Pallet::new(3.0, 3.0, 0.0),
            Pallet::new(5.0, 3.0, 0.0),
            Pallet::new(7.0, 3.0, 0.0),
            Pallet::new(-3.0, 5.0, std::f64::consts::FRAC_PI_2),
            Pallet::new(-3.0, 7.0, std::f64::consts::FRAC_PI_2),
            Pallet::empty(10.0, 5.0, 0.3),
            Pallet::empty(12.0, 5.0, -0.1),
            Pallet::new(-6.0, 15.0, 0.0),
            Pallet::new(-4.0, 15.0, 0.0),
        ];

        Warehouse {
            pallets,
            walls,
            shelves,
            floor_half_size,
        }
    }
}
