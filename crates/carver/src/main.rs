//! SimuForge Direct Carver — headless G-code to carved workpiece.
//!
//! Applies CSG subtraction directly from G-code commands without physics simulation.
//! Produces OBJ mesh and binary SDF output.

use std::path::Path;
use std::time::Instant;

use simuforge_control::gcode::GCodeInterpreter;
use simuforge_cutting::tool::Tool;
use simuforge_carver::DirectCarver;

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
    eprintln!("  G-code:     {}", gcode_path);
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

    // Create DirectCarver and carve everything
    let mut carver = DirectCarver::new(interp.commands, tool, cfg.half_extent, cfg.cell_size);
    eprintln!(
        "  Workpiece created: {} chunks",
        carver.workpiece.chunk_count(),
    );

    let t0 = Instant::now();
    let total = carver.total_commands();

    // Carve in batches of 10000 for progress reporting
    while carver.current_command() < total {
        let before = carver.current_command();
        carver.carve_batch(10000);
        let after = carver.current_command();
        if after == total || (after / 10000) != (before / 10000) {
            eprint!(
                "\r  Carving: {}/{} commands, {} cuts, {:.1}s",
                after, total, carver.cut_count(), t0.elapsed().as_secs_f64()
            );
        }
    }
    eprintln!();

    let carve_time = t0.elapsed().as_secs_f64();
    eprintln!(
        "  Carving complete: {} cuts in {:.2}s",
        carver.cut_count(), carve_time
    );

    // Export OBJ mesh
    let t0 = Instant::now();
    let (verts, tris) =
        simuforge_material::mesher::export_obj(&carver.workpiece, Path::new(&output_path));
    eprintln!(
        "  OBJ exported: {} ({} verts, {} tris, {:.2}s)",
        output_path, verts, tris, t0.elapsed().as_secs_f64()
    );

    // Save binary SDF
    if save_sdf {
        let sdf_path = Path::new(&output_path).with_extension("sdf");
        let t0 = Instant::now();
        carver.workpiece.save_binary(&sdf_path);
        eprintln!(
            "  SDF saved: {} ({:.2}s)",
            sdf_path.display(),
            t0.elapsed().as_secs_f64()
        );
    }
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
