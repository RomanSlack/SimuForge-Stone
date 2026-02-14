//! Simple TOML-subset config loader for SimuForge.
//! Supports [sections] with key = value pairs (strings, floats, ints).

use std::collections::HashMap;
use std::path::Path;

/// Parsed configuration values, keyed by "section.key".
pub struct Config {
    values: HashMap<String, String>,
}

impl Config {
    /// Load config from a TOML file. Returns empty config if file doesn't exist.
    pub fn load(path: &Path) -> Self {
        let text = match std::fs::read_to_string(path) {
            Ok(t) => t,
            Err(_) => {
                eprintln!("No config file at {:?}, using defaults", path);
                return Self { values: HashMap::new() };
            }
        };
        eprintln!("Loaded config from {:?}", path);
        Self::parse(&text)
    }

    fn parse(text: &str) -> Self {
        let mut values = HashMap::new();
        let mut section = String::new();

        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if line.starts_with('[') && line.ends_with(']') {
                section = line[1..line.len() - 1].trim().to_string();
                continue;
            }
            if let Some(eq_pos) = line.find('=') {
                let key = line[..eq_pos].trim();
                let val = line[eq_pos + 1..].trim();
                // Strip inline comments
                let val = if let Some(hash) = val.find('#') {
                    val[..hash].trim()
                } else {
                    val
                };
                // Strip quotes from string values
                let val = val.trim_matches('"');
                let full_key = if section.is_empty() {
                    key.to_string()
                } else {
                    format!("{}.{}", section, key)
                };
                values.insert(full_key, val.to_string());
            }
        }

        Self { values }
    }

    pub fn get_f64(&self, key: &str, default: f64) -> f64 {
        self.values.get(key)
            .and_then(|v| v.parse().ok())
            .unwrap_or(default)
    }

    pub fn get_u32(&self, key: &str, default: u32) -> u32 {
        self.values.get(key)
            .and_then(|v| v.parse().ok())
            .unwrap_or(default)
    }

    pub fn get_str<'a>(&'a self, key: &str, default: &'a str) -> &'a str {
        self.values.get(key)
            .map(|s| s.as_str())
            .unwrap_or(default)
    }
}

/// All configurable simulation parameters with defaults.
pub struct SimConfig {
    // Tool
    pub tool_type: String,
    pub tool_radius_mm: f64,
    pub tool_cutting_length_mm: f64,
    pub tool_offset: f64,
    // Workpiece
    pub workpiece_center: [f64; 3],
    pub workpiece_half_extent: f64,
    pub workpiece_resolution: f64,
    // Carving
    pub default_speed: f64,
    pub max_physics_steps: u32,
    pub tracking_threshold: f64,
    pub orientation_weight: f64,
    // Track
    pub track_default_position: f64,
}

impl SimConfig {
    pub fn from_file(path: &Path) -> Self {
        let cfg = Config::load(path);
        Self {
            tool_type: cfg.get_str("tool.type", "flat_end").to_string(),
            tool_radius_mm: cfg.get_f64("tool.radius_mm", 6.0),
            tool_cutting_length_mm: cfg.get_f64("tool.cutting_length_mm", 25.0),
            tool_offset: cfg.get_f64("tool.tool_offset", 0.175),
            workpiece_center: [
                cfg.get_f64("workpiece.center_x", 0.65),
                cfg.get_f64("workpiece.center_y", 0.0),
                cfg.get_f64("workpiece.center_z", -0.2),
            ],
            workpiece_half_extent: cfg.get_f64("workpiece.half_extent", 0.1525),
            workpiece_resolution: cfg.get_f64("workpiece.resolution", 0.0005),
            default_speed: cfg.get_f64("carving.default_speed", 1.0),
            max_physics_steps: cfg.get_u32("carving.max_physics_steps", 5000),
            tracking_threshold: cfg.get_f64("carving.tracking_threshold", 0.01),
            orientation_weight: cfg.get_f64("carving.orientation_weight", 0.6),
            track_default_position: cfg.get_f64("track.default_position", 0.0),
        }
    }
}
