//! Spatial audio utilities: stereo panning, distance attenuation.

/// Equal-power stereo panning.
/// `pan` ranges from -1.0 (full left) to 1.0 (full right).
/// Returns (left_gain, right_gain).
pub fn stereo_pan(pan: f32) -> (f32, f32) {
    let p = pan.clamp(-1.0, 1.0);
    // Equal-power: gains sum to 1 in power (sqrt(L²+R²) = const)
    let angle = (p + 1.0) * 0.25 * std::f32::consts::PI; // 0..π/2
    let left = angle.cos();
    let right = angle.sin();
    (left, right)
}

/// Distance-based attenuation (inverse distance, clamped).
/// `distance` in meters, `ref_distance` is the distance at which gain = 1.0.
pub fn distance_attenuation(distance: f32, ref_distance: f32) -> f32 {
    if distance <= ref_distance {
        1.0
    } else {
        (ref_distance / distance).min(1.0)
    }
}

/// Compute stereo pan value from source position relative to listener.
/// Uses the dot product of source direction with listener right vector.
/// `source_x` is the source position along the listener's left-right axis.
/// For our setup: positive X = Player 2 side (right), negative X = Player 1 side (left).
pub fn position_to_pan(source_x: f32, max_distance: f32) -> f32 {
    (source_x / max_distance).clamp(-1.0, 1.0)
}
