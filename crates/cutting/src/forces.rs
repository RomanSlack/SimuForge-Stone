//! Cutting force model for stone machining.
//!
//! Uses specific cutting energy approach: F = Ks * A * doc
//! where Ks = specific cutting energy (N/mm²), A = chip cross-section area,
//! doc = depth of cut.

use nalgebra::Vector3;

/// Material properties for cutting force calculation.
#[derive(Debug, Clone, Copy)]
pub struct MaterialProps {
    /// Specific cutting energy (N/mm² = MPa).
    pub specific_energy: f64,
    /// Hardness (Mohs scale, for wear calculation).
    pub hardness: f64,
    /// Density (kg/m³).
    pub density: f64,
}

impl MaterialProps {
    /// Carrara marble properties.
    pub fn marble() -> Self {
        Self {
            specific_energy: 20.0, // N/mm² — typical for marble
            hardness: 3.0,         // Mohs
            density: 2710.0,       // kg/m³
        }
    }

    /// Granite properties (harder).
    pub fn granite() -> Self {
        Self {
            specific_energy: 45.0,
            hardness: 6.5,
            density: 2750.0,
        }
    }

    /// Limestone (softer).
    pub fn limestone() -> Self {
        Self {
            specific_energy: 12.0,
            hardness: 3.0,
            density: 2500.0,
        }
    }
}

/// Compute cutting force vector given machining parameters.
///
/// # Arguments
/// * `material` - Material properties
/// * `tool_radius` - Tool cutting radius (m)
/// * `feed_direction` - Unit vector of feed direction
/// * `depth_of_cut` - Radial depth of cut (m)
/// * `feed_per_tooth` - Feed per tooth (m)
/// * `num_flutes` - Number of cutting flutes
///
/// # Returns
/// Force vector in world frame (N). Opposes the feed direction.
pub fn cutting_force(
    material: &MaterialProps,
    tool_radius: f64,
    feed_direction: &Vector3<f64>,
    depth_of_cut: f64,
    feed_per_tooth: f64,
    num_flutes: u32,
) -> Vector3<f64> {
    if depth_of_cut <= 0.0 || feed_per_tooth <= 0.0 {
        return Vector3::zeros();
    }

    // Convert to mm for the specific energy formula
    let doc_mm = depth_of_cut * 1000.0;
    let fpt_mm = feed_per_tooth * 1000.0;
    let radius_mm = tool_radius * 1000.0;

    // Chip cross-section area (mm²) — simplified for ball nose
    // Average chip thickness for ball nose ≈ fpt * doc / (π * R)
    let avg_chip_thickness = fpt_mm * doc_mm / (std::f64::consts::PI * radius_mm);

    // Engagement arc length (mm)
    let engagement_angle = (doc_mm / radius_mm).min(1.0).acos();
    let arc_length = radius_mm * engagement_angle;

    // Chip area (mm²)
    let chip_area = avg_chip_thickness * arc_length;

    // Force magnitude (N) = Ks * A * number_of_flutes_engaged
    // Typically ~1 flute engaged at a time for ball nose
    let flutes_engaged = (num_flutes as f64 * engagement_angle / (2.0 * std::f64::consts::PI)).max(1.0);
    let force_magnitude = material.specific_energy * chip_area * flutes_engaged;

    // Force direction: primarily opposing feed, with radial component
    let feed_norm = if feed_direction.norm() > 1e-9 {
        feed_direction.normalize()
    } else {
        return Vector3::zeros();
    };

    // Tangential force (opposing feed): ~60% of total
    // Normal force (radial, pushing tool away from workpiece): ~40%
    let tangential = -feed_norm * force_magnitude * 0.6;

    // Radial direction: perpendicular to feed in the cutting plane
    // For simplicity, use the cross product with tool axis (Z-up)
    let z_axis = Vector3::z();
    let radial_dir = feed_norm.cross(&z_axis);
    let radial = if radial_dir.norm() > 1e-9 {
        radial_dir.normalize() * force_magnitude * 0.4
    } else {
        Vector3::zeros()
    };

    tangential + radial
}

/// Estimate volume of material removed per time step.
///
/// Based on feed rate, depth of cut, and step over width.
pub fn material_removal_rate(
    depth_of_cut: f64,
    step_over: f64,
    feed_rate: f64,
) -> f64 {
    depth_of_cut * step_over * feed_rate
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_marble_cutting_force() {
        let marble = MaterialProps::marble();
        let force = cutting_force(
            &marble,
            0.003,                              // 3mm radius ball nose
            &Vector3::new(1.0, 0.0, 0.0),      // feed in X
            0.001,                               // 1mm depth of cut
            0.0001,                              // 0.1mm feed per tooth
            2,                                   // 2 flutes
        );

        let magnitude = force.norm();
        // For marble with these parameters, expect force in range 1-50N
        assert!(
            magnitude > 0.1 && magnitude < 100.0,
            "Unexpected cutting force magnitude: {} N",
            magnitude
        );

        // Force should primarily oppose feed (negative X component)
        assert!(
            force.x < 0.0,
            "Force should oppose feed direction: {:?}",
            force
        );
    }

    #[test]
    fn test_zero_doc_no_force() {
        let marble = MaterialProps::marble();
        let force = cutting_force(
            &marble,
            0.003,
            &Vector3::new(1.0, 0.0, 0.0),
            0.0, // zero depth
            0.0001,
            2,
        );
        assert!(force.norm() < 1e-10);
    }
}
