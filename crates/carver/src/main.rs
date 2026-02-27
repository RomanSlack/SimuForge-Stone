//! SimuForge Direct Carver — headless G-code to carved workpiece.
//!
//! Applies CSG subtraction directly from G-code commands without physics simulation.
//! Produces OBJ mesh and binary SDF output.

use std::path::Path;
use std::time::Instant;

use nalgebra::Vector3;
use simuforge_control::gcode::{GCommand, GCodeInterpreter};
use simuforge_cutting::tool::Tool;
use simuforge_material::octree::OctreeSdf;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: simuforge-carve <gcode-file> [options]");
        eprintln!();
        eprintln!("Options:");
        eprintln!("  --config <path>      Config file (default: config.toml)");
        eprintln!("  --resolution <m>     Voxel size in meters (default: from config or 0.0005)");
        eprintln!("  --export <path>      Output OBJ path (default: <gcode>.obj)");
        eprintln!("  --no-sdf             Skip saving binary SDF file");
        std::process::exit(1);
    }

    let gcode_path = &args[1];
    let config_path = find_arg(&args, "--config");
    let resolution_override: f64 = find_arg(&args, "--resolution")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    let output_path = find_arg(&args, "--export").unwrap_or_else(|| {
        Path::new(gcode_path)
            .with_extension("obj")
            .to_string_lossy()
            .into_owned()
    });
    let save_sdf = !args.iter().any(|a| a == "--no-sdf");

    // Load config
    let cfg = CarverConfig::load(config_path.as_deref(), resolution_override);

    eprintln!("SimuForge Direct Carver");
    eprintln!(
        "  G-code:     {}",
        gcode_path
    );
    eprintln!(
        "  Tool:       {} r={:.1}mm cl={:.1}mm",
        cfg.tool_type, cfg.tool_radius_mm, cfg.tool_cutting_length_mm
    );
    eprintln!(
        "  Workpiece:  {:.1}mm cube, {:.4}mm resolution",
        cfg.half_extent * 2000.0,
        cfg.cell_size * 1000.0
    );
    eprintln!("  Output:     {}", output_path);

    // Create tool
    let tool = match cfg.tool_type.as_str() {
        "ball_nose" => Tool::ball_nose(cfg.tool_radius_mm, cfg.tool_cutting_length_mm),
        _ => Tool::flat_end(cfg.tool_radius_mm, cfg.tool_cutting_length_mm),
    };

    // Create workpiece SDF
    let he = cfg.half_extent as f32;
    let t0 = Instant::now();
    let mut workpiece = OctreeSdf::new_block([he, he, he], cfg.cell_size as f32);
    eprintln!(
        "  Workpiece created: {} chunks ({:.2}s)",
        workpiece.chunk_count(),
        t0.elapsed().as_secs_f64()
    );

    // Load G-code
    let t0 = Instant::now();
    let interp = GCodeInterpreter::load_file(gcode_path).unwrap_or_else(|e| {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    });
    eprintln!(
        "  G-code loaded: {} commands ({:.2}s)",
        interp.commands.len(),
        t0.elapsed().as_secs_f64()
    );

    // Walk commands directly — no CarvingSession, no TrajectoryPlanner, no physics
    let mut position = Vector3::<f64>::zeros();
    let mut a_axis = 0.0_f64;
    let mut spindle_on = false;
    let mut cut_count = 0u64;
    let total_cmds = interp.commands.len();

    let t0 = Instant::now();

    for (i, cmd) in interp.commands.iter().enumerate() {
        match cmd {
            GCommand::SpindleOn { .. } => {
                spindle_on = true;
            }
            GCommand::SpindleOff => {
                spindle_on = false;
            }
            GCommand::Rapid { x, y, z, a, u: _ } => {
                if let Some(v) = x { position.x = *v; }
                if let Some(v) = y { position.y = *v; }
                if let Some(v) = z { position.z = *v; }
                if let Some(v) = a { a_axis = *v; }
            }
            GCommand::LinearFeed { x, y, z, a, u: _, f: _ } => {
                let prev_pos = position;
                let prev_a = a_axis;
                if let Some(v) = x { position.x = *v; }
                if let Some(v) = y { position.y = *v; }
                if let Some(v) = z { position.z = *v; }
                if let Some(v) = a { a_axis = *v; }

                // Cut when spindle on and A-axis hasn't changed (rotation = repositioning)
                if spindle_on && (a_axis - prev_a).abs() < 1e-6 {
                    cut_segment(
                        &tool,
                        &mut workpiece,
                        &prev_pos,
                        &position,
                        a_axis,
                        cfg.half_extent,
                    );
                    cut_count += 1;
                }
            }
            GCommand::ArcCW { x, y, z, f: _, .. }
            | GCommand::ArcCCW { x, y, z, f: _, .. } => {
                // Treat arcs as linear moves (same as GCodeInterpreter::next_target)
                let prev_pos = position;
                if let Some(v) = x { position.x = *v; }
                if let Some(v) = y { position.y = *v; }
                if let Some(v) = z { position.z = *v; }

                if spindle_on {
                    cut_segment(
                        &tool,
                        &mut workpiece,
                        &prev_pos,
                        &position,
                        a_axis,
                        cfg.half_extent,
                    );
                    cut_count += 1;
                }
            }
            GCommand::Comment(_) => {}
        }

        if (i + 1) % 10000 == 0 || i + 1 == total_cmds {
            eprint!(
                "\r  Carving: {}/{} commands, {} cuts, {:.1}s",
                i + 1,
                total_cmds,
                cut_count,
                t0.elapsed().as_secs_f64()
            );
        }
    }
    eprintln!();

    let carve_time = t0.elapsed().as_secs_f64();
    eprintln!(
        "  Carving complete: {} cuts in {:.2}s",
        cut_count, carve_time
    );

    // Export OBJ mesh
    let t0 = Instant::now();
    let (verts, tris) =
        simuforge_material::mesher::export_obj(&workpiece, Path::new(&output_path));
    eprintln!(
        "  OBJ exported: {} ({} verts, {} tris, {:.2}s)",
        output_path,
        verts,
        tris,
        t0.elapsed().as_secs_f64()
    );

    // Save binary SDF
    if save_sdf {
        let sdf_path = Path::new(&output_path).with_extension("sdf");
        let t0 = Instant::now();
        workpiece.save_binary(&sdf_path);
        eprintln!(
            "  SDF saved: {} ({:.2}s)",
            sdf_path.display(),
            t0.elapsed().as_secs_f64()
        );
    }
}

/// Apply a single cut segment in SDF-local coordinates.
///
/// Coordinate transform: gcode_pos + (0,0,half_extent) → rotated by -a_angle around X.
fn cut_segment(
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

/// Find a command-line argument value by flag name.
fn find_arg(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|a| a == flag)
        .and_then(|i| args.get(i + 1).cloned())
}

/// Carver-specific configuration (subset of SimConfig, no sim crate dependency).
struct CarverConfig {
    tool_type: String,
    tool_radius_mm: f64,
    tool_cutting_length_mm: f64,
    half_extent: f64,
    cell_size: f64,
}

impl CarverConfig {
    fn load(config_path: Option<&str>, resolution_override: f64) -> Self {
        let path = config_path.unwrap_or("config.toml");
        let values = parse_toml(path);

        let tool_type = get_str(&values, "tool.type", "flat_end").to_string();
        let tool_radius_mm = get_f64(&values, "tool.radius_mm", 6.0);
        let tool_cutting_length_mm = get_f64(&values, "tool.cutting_length_mm", 25.0);
        let half_extent = get_f64(&values, "workpiece.half_extent", 0.1525);
        let cell_size = if resolution_override > 0.0 {
            resolution_override
        } else {
            get_f64(&values, "workpiece.resolution", 0.0005)
        };

        Self {
            tool_type,
            tool_radius_mm,
            tool_cutting_length_mm,
            half_extent,
            cell_size,
        }
    }
}

// --- Minimal TOML parser (matches sim crate's Config) ---

fn parse_toml(path: &str) -> std::collections::HashMap<String, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(_) => return std::collections::HashMap::new(),
    };

    let mut values = std::collections::HashMap::new();
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
            let val = if let Some(hash) = val.find('#') {
                val[..hash].trim()
            } else {
                val
            };
            let val = val.trim_matches('"');
            let full_key = if section.is_empty() {
                key.to_string()
            } else {
                format!("{}.{}", section, key)
            };
            values.insert(full_key, val.to_string());
        }
    }

    values
}

fn get_f64(values: &std::collections::HashMap<String, String>, key: &str, default: f64) -> f64 {
    values
        .get(key)
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn get_str<'a>(
    values: &'a std::collections::HashMap<String, String>,
    key: &str,
    default: &'a str,
) -> &'a str {
    values.get(key).map(|s| s.as_str()).unwrap_or(default)
}
