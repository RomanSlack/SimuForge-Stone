//! Coordinate conversions: WGS84 ↔ ENU ↔ render-space, Web Mercator tile math.
//!
//! The drone simulation uses a local East-North-Up (ENU) frame for physics,
//! which maps to render-space as: rx = east, ry = up, rz = -north.

use std::f64::consts::PI;

/// Geographic origin for local ENU coordinate frame.
pub struct GeoOrigin {
    /// Reference latitude (radians).
    pub lat0: f64,
    /// Reference longitude (radians).
    pub lon0: f64,
    /// Meters per degree latitude at the reference point.
    pub meters_per_deg_lat: f64,
    /// Meters per degree longitude at the reference point.
    pub meters_per_deg_lon: f64,
}

impl GeoOrigin {
    /// Create a new origin from latitude/longitude in degrees.
    pub fn from_degrees(lat_deg: f64, lon_deg: f64) -> Self {
        let lat0 = lat_deg.to_radians();
        let lon0 = lon_deg.to_radians();
        // WGS84 ellipsoid approximation
        let meters_per_deg_lat = 111_132.92;
        let meters_per_deg_lon = 111_132.92 * lat0.cos();
        Self {
            lat0,
            lon0,
            meters_per_deg_lat,
            meters_per_deg_lon,
        }
    }

    /// Convert ENU (east, north) meters to lat/lon in degrees.
    pub fn enu_to_latlon(&self, east: f64, north: f64) -> (f64, f64) {
        let lat_deg = self.lat0.to_degrees() + north / self.meters_per_deg_lat;
        let lon_deg = self.lon0.to_degrees() + east / self.meters_per_deg_lon;
        (lat_deg, lon_deg)
    }

    /// Convert lat/lon in degrees to ENU (east, north) meters.
    pub fn latlon_to_enu(&self, lat_deg: f64, lon_deg: f64) -> (f64, f64) {
        let east = (lon_deg - self.lon0.to_degrees()) * self.meters_per_deg_lon;
        let north = (lat_deg - self.lat0.to_degrees()) * self.meters_per_deg_lat;
        (east, north)
    }

    /// Convert ENU (east, north, up) to render-space (rx, ry, rz).
    /// Render uses Y-up: rx = east, ry = up, rz = -north.
    pub fn enu_to_render(&self, east: f64, north: f64, up: f64) -> (f32, f32, f32) {
        (east as f32, up as f32, -north as f32)
    }

    /// Convert render-space (rx, ry, rz) back to ENU (east, north, up).
    pub fn render_to_enu(&self, rx: f32, ry: f32, rz: f32) -> (f64, f64, f64) {
        (rx as f64, -(rz as f64), ry as f64)
    }

    /// Convert render-space (rx, rz) to lat/lon in degrees.
    pub fn render_to_latlon(&self, rx: f32, rz: f32) -> (f64, f64) {
        let (east, north, _up) = self.render_to_enu(rx, 0.0, rz);
        self.enu_to_latlon(east, north)
    }
}

/// Convert lat/lon (degrees) to Web Mercator tile coordinates at given zoom level.
/// Returns (tile_x, tile_y) as integers.
pub fn latlon_to_tile(lat_deg: f64, lon_deg: f64, zoom: u32) -> (u32, u32) {
    let n = (1u64 << zoom) as f64;
    let lat_rad = lat_deg.to_radians();
    let x = ((lon_deg + 180.0) / 360.0 * n).floor() as u32;
    let y = ((1.0 - lat_rad.tan().asinh() / PI) / 2.0 * n).floor() as u32;
    (x, y)
}

/// Convert lat/lon (degrees) to fractional Web Mercator tile coordinates.
/// Returns (tile_x, tile_y) as f64 for sub-tile positioning.
pub fn latlon_to_tile_frac(lat_deg: f64, lon_deg: f64, zoom: u32) -> (f64, f64) {
    let n = (1u64 << zoom) as f64;
    let lat_rad = lat_deg.to_radians();
    let x = (lon_deg + 180.0) / 360.0 * n;
    let y = (1.0 - lat_rad.tan().asinh() / PI) / 2.0 * n;
    (x, y)
}

/// Get the geographic bounds of a tile in degrees: (lat_min, lon_min, lat_max, lon_max).
pub fn tile_bounds(tx: u32, ty: u32, zoom: u32) -> (f64, f64, f64, f64) {
    let n = (1u64 << zoom) as f64;
    let lon_min = tx as f64 / n * 360.0 - 180.0;
    let lon_max = (tx + 1) as f64 / n * 360.0 - 180.0;
    let lat_max = (PI * (1.0 - 2.0 * ty as f64 / n)).sinh().atan().to_degrees();
    let lat_min = (PI * (1.0 - 2.0 * (ty + 1) as f64 / n)).sinh().atan().to_degrees();
    (lat_min, lon_min, lat_max, lon_max)
}

/// Approximate tile width in meters at a given latitude and zoom level.
pub fn tile_width_meters(lat_deg: f64, zoom: u32) -> f64 {
    let circumference = 40_075_016.686; // Earth circumference at equator (meters)
    let n = (1u64 << zoom) as f64;
    circumference * lat_deg.to_radians().cos() / n
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enu_roundtrip() {
        // Damascus area: 33.5°N, 36.3°E
        let origin = GeoOrigin::from_degrees(33.5, 36.3);
        let (lat, lon) = origin.enu_to_latlon(1000.0, 2000.0);
        let (east, north) = origin.latlon_to_enu(lat, lon);
        assert!((east - 1000.0).abs() < 0.01, "east roundtrip: {east}");
        assert!((north - 2000.0).abs() < 0.01, "north roundtrip: {north}");
    }

    #[test]
    fn test_render_enu_roundtrip() {
        let origin = GeoOrigin::from_degrees(33.5, 36.3);
        let (rx, ry, rz) = origin.enu_to_render(100.0, 200.0, 50.0);
        assert!((rx - 100.0).abs() < 0.01);
        assert!((ry - 50.0).abs() < 0.01);
        assert!((rz - (-200.0)).abs() < 0.01);
        let (e, n, u) = origin.render_to_enu(rx, ry, rz);
        assert!((e - 100.0).abs() < 0.01);
        assert!((n - 200.0).abs() < 0.01);
        assert!((u - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_latlon_to_tile() {
        // Known: at zoom 14, (0°N, 0°E) should be tile (8192, 8192)
        let (x, y) = latlon_to_tile(0.0, 0.0, 14);
        assert_eq!(x, 8192);
        assert_eq!(y, 8192);
    }

    #[test]
    fn test_tile_bounds_cover_point() {
        let lat = 33.5;
        let lon = 36.3;
        let zoom = 14;
        let (tx, ty) = latlon_to_tile(lat, lon, zoom);
        let (lat_min, lon_min, lat_max, lon_max) = tile_bounds(tx, ty, zoom);
        assert!(lat >= lat_min && lat <= lat_max, "lat {lat} not in [{lat_min}, {lat_max}]");
        assert!(lon >= lon_min && lon <= lon_max, "lon {lon} not in [{lon_min}, {lon_max}]");
    }

    #[test]
    fn test_tile_width() {
        // At equator, zoom 14: ~2.4km
        let w = tile_width_meters(0.0, 14);
        assert!(w > 2000.0 && w < 3000.0, "tile width at equator z14: {w}");
        // At lat 33: narrower
        let w33 = tile_width_meters(33.0, 14);
        assert!(w33 < w, "tile at lat 33 should be narrower");
        assert!(w33 > 1500.0 && w33 < 2500.0, "tile width at 33°N z14: {w33}");
    }
}
