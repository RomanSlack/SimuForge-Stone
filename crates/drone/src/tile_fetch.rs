//! Mapbox tile downloading with disk cache and background thread pool.
//!
//! Fetches Terrain-RGB elevation tiles and satellite imagery tiles from Mapbox,
//! caches them to `~/.cache/simuforge/tiles/`, and decodes on background threads.

use std::collections::HashSet;
use std::fs;
use std::path::PathBuf;
use std::sync::mpsc;
use std::thread;

/// Mapbox API configuration.
pub struct MapboxConfig {
    pub access_token: String,
    cache_dir: PathBuf,
}

impl MapboxConfig {
    /// Create config from environment variable `MAPBOX_ACCESS_TOKEN`.
    /// Returns None if the variable is not set.
    pub fn from_env() -> Option<Self> {
        let token = std::env::var("MAPBOX_ACCESS_TOKEN").ok()?;
        if token.is_empty() {
            return None;
        }
        let cache_dir = dirs_cache().join("simuforge").join("tiles");
        Some(Self {
            access_token: token,
            cache_dir,
        })
    }

    fn elevation_url(&self, z: u32, x: u32, y: u32) -> String {
        format!(
            "https://api.mapbox.com/v4/mapbox.terrain-rgb/{z}/{x}/{y}@2x.pngraw?access_token={}",
            self.access_token
        )
    }

    fn satellite_url(&self, z: u32, x: u32, y: u32) -> String {
        format!(
            "https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg90?access_token={}",
            self.access_token
        )
    }

    fn cache_path(&self, layer: &str, z: u32, x: u32, y: u32, ext: &str) -> PathBuf {
        self.cache_dir.join(format!("{layer}/{z}/{x}/{y}.{ext}"))
    }
}

/// Get the platform cache directory (~/.cache on Linux).
fn dirs_cache() -> PathBuf {
    if let Ok(dir) = std::env::var("XDG_CACHE_HOME") {
        return PathBuf::from(dir);
    }
    if let Ok(home) = std::env::var("HOME") {
        return PathBuf::from(home).join(".cache");
    }
    PathBuf::from("/tmp")
}

/// Decoded elevation tile: 256×256 grid of height values in meters.
pub struct ElevationTile {
    pub heights: Vec<f32>,
    pub width: u32,
    pub height: u32,
}

impl ElevationTile {
    /// Decode a Mapbox Terrain-RGB PNG into height values.
    /// Formula: height = -10000 + (R * 65536 + G * 256 + B) * 0.1
    pub fn from_png(data: &[u8]) -> Option<Self> {
        let img = image::load_from_memory_with_format(data, image::ImageFormat::Png).ok()?;
        let rgba = img.to_rgba8();
        let w = rgba.width();
        let h = rgba.height();
        let mut heights = Vec::with_capacity((w * h) as usize);
        for pixel in rgba.pixels() {
            let r = pixel[0] as f32;
            let g = pixel[1] as f32;
            let b = pixel[2] as f32;
            let height = -10000.0 + (r * 65536.0 + g * 256.0 + b) * 0.1;
            heights.push(height);
        }
        Some(Self {
            heights,
            width: w,
            height: h,
        })
    }

    /// Bilinear sample height at fractional pixel coordinates.
    /// `fx`, `fy` are in [0, 1] range within the tile.
    pub fn sample(&self, fx: f32, fy: f32) -> f32 {
        let px = fx * (self.width - 1) as f32;
        let py = fy * (self.height - 1) as f32;
        let x0 = (px.floor() as u32).min(self.width - 2);
        let y0 = (py.floor() as u32).min(self.height - 2);
        let x1 = x0 + 1;
        let y1 = y0 + 1;
        let sx = px - x0 as f32;
        let sy = py - y0 as f32;
        let h00 = self.heights[(y0 * self.width + x0) as usize];
        let h10 = self.heights[(y0 * self.width + x1) as usize];
        let h01 = self.heights[(y1 * self.width + x0) as usize];
        let h11 = self.heights[(y1 * self.width + x1) as usize];
        let h0 = h00 + (h10 - h00) * sx;
        let h1 = h01 + (h11 - h01) * sx;
        h0 + (h1 - h0) * sy
    }
}

/// Raw tile data ready for GPU upload.
pub struct RawTile {
    pub tx: u32,
    pub ty: u32,
    pub zoom: u32,
    pub elevation: ElevationTile,
    /// Satellite image as RGBA8 pixels.
    pub satellite_rgba: Vec<u8>,
    pub satellite_width: u32,
    pub satellite_height: u32,
}

/// Tile coordinate key.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TileCoord {
    pub z: u32,
    pub x: u32,
    pub y: u32,
}

/// Background tile loader with thread pool and disk cache.
pub struct TileLoader {
    /// Channel to receive completed tiles on the main thread.
    pub receiver: mpsc::Receiver<RawTile>,
    sender: mpsc::Sender<TileCoord>,
    /// Track in-flight requests to avoid duplicates.
    pending: HashSet<TileCoord>,
}

impl TileLoader {
    /// Create a new tile loader with `num_threads` background fetch threads.
    pub fn new(config: MapboxConfig, num_threads: usize, elev_zoom: u32, sat_zoom: u32) -> Self {
        let (coord_tx, coord_rx) = mpsc::channel::<TileCoord>();
        let (tile_tx, tile_rx) = mpsc::channel::<RawTile>();
        let coord_rx = std::sync::Arc::new(std::sync::Mutex::new(coord_rx));
        let config = std::sync::Arc::new(config);

        for _ in 0..num_threads {
            let coord_rx = coord_rx.clone();
            let tile_tx = tile_tx.clone();
            let config = config.clone();
            thread::Builder::new()
                .name("tile-fetch".into())
                .spawn(move || {
                    loop {
                        let coord = {
                            let rx = coord_rx.lock().unwrap();
                            match rx.recv() {
                                Ok(c) => c,
                                Err(_) => return, // channel closed
                            }
                        };

                        if let Some(tile) = fetch_tile(&config, coord, elev_zoom, sat_zoom) {
                            let _ = tile_tx.send(tile);
                        }
                    }
                })
                .expect("Failed to spawn tile fetch thread");
        }

        Self {
            receiver: tile_rx,
            sender: coord_tx,
            pending: HashSet::new(),
        }
    }

    /// Request a tile to be loaded. Deduplicates against in-flight requests.
    pub fn request(&mut self, coord: TileCoord) {
        if self.pending.insert(coord) {
            let _ = self.sender.send(coord);
        }
    }

    /// Mark a tile as no longer pending (called when received or evicted).
    pub fn mark_complete(&mut self, coord: &TileCoord) {
        self.pending.remove(coord);
    }

    /// Check if a tile request is already in flight.
    pub fn is_pending(&self, coord: &TileCoord) -> bool {
        self.pending.contains(coord)
    }
}

/// Fetch a single tile: check disk cache first, then HTTP.
fn fetch_tile(
    config: &MapboxConfig,
    coord: TileCoord,
    elev_zoom: u32,
    sat_zoom: u32,
) -> Option<RawTile> {
    // For elevation, use elev_zoom; for satellite, use sat_zoom.
    // The coord.z is the display zoom. We fetch elevation at elev_zoom
    // and satellite at sat_zoom. For simplicity, we use the same tile coords
    // but potentially different zoom levels.
    let elev_data = fetch_cached_or_download(
        config,
        "terrain-rgb",
        elev_zoom,
        coord.x,
        coord.y,
        "png",
        &config.elevation_url(elev_zoom, coord.x, coord.y),
    )?;

    let elevation = ElevationTile::from_png(&elev_data)?;

    // Satellite: fetch at higher zoom for more detail.
    // Each elevation tile at zoom Z maps to 4 satellite tiles at zoom Z+1:
    //   (2x, 2y), (2x+1, 2y), (2x, 2y+1), (2x+1, 2y+1)
    // Stitch them into one texture for 4× pixel density.
    let sat_rgba;
    let sat_w;
    let sat_h;

    if sat_zoom > elev_zoom {
        let scale = 1u32 << (sat_zoom - elev_zoom); // 2 for +1 zoom
        let base_x = coord.x * scale;
        let base_y = coord.y * scale;

        // Fetch all sub-tiles
        let mut sub_images: Vec<Option<image::RgbaImage>> = Vec::new();
        for dy in 0..scale {
            for dx in 0..scale {
                let sx = base_x + dx;
                let sy = base_y + dy;
                let data = fetch_cached_or_download(
                    config, "satellite", sat_zoom, sx, sy, "jpg",
                    &config.satellite_url(sat_zoom, sx, sy),
                );
                let img = data.and_then(|d| {
                    image::load_from_memory_with_format(&d, image::ImageFormat::Jpeg)
                        .ok()
                        .map(|i| i.to_rgba8())
                });
                sub_images.push(img);
            }
        }

        // Determine sub-tile size from first available image
        let tile_px = sub_images.iter().find_map(|i| i.as_ref()).map(|i| i.width()).unwrap_or(512);
        sat_w = tile_px * scale;
        sat_h = tile_px * scale;
        let mut stitched = vec![0u8; (sat_w * sat_h * 4) as usize];

        for dy in 0..scale {
            for dx in 0..scale {
                let idx = (dy * scale + dx) as usize;
                if let Some(img) = &sub_images[idx] {
                    let src = img.as_raw();
                    let src_w = img.width().min(tile_px);
                    let src_h = img.height().min(tile_px);
                    let dst_x0 = dx * tile_px;
                    let dst_y0 = dy * tile_px;
                    for row in 0..src_h {
                        let src_off = (row * img.width() * 4) as usize;
                        let dst_off = ((dst_y0 + row) * sat_w * 4 + dst_x0 * 4) as usize;
                        let count = (src_w * 4) as usize;
                        stitched[dst_off..dst_off + count]
                            .copy_from_slice(&src[src_off..src_off + count]);
                    }
                }
            }
        }
        sat_rgba = stitched;
    } else {
        // Same zoom: single tile as before
        let sat_data = fetch_cached_or_download(
            config, "satellite", sat_zoom, coord.x, coord.y, "jpg",
            &config.satellite_url(sat_zoom, coord.x, coord.y),
        )?;
        let sat_img = image::load_from_memory_with_format(&sat_data, image::ImageFormat::Jpeg).ok()?;
        let rgba_img = sat_img.to_rgba8();
        sat_w = rgba_img.width();
        sat_h = rgba_img.height();
        sat_rgba = rgba_img.to_vec();
    }

    Some(RawTile {
        tx: coord.x,
        ty: coord.y,
        zoom: coord.z,
        elevation,
        satellite_rgba: sat_rgba,
        satellite_width: sat_w,
        satellite_height: sat_h,
    })
}

/// Check disk cache, download if missing.
fn fetch_cached_or_download(
    config: &MapboxConfig,
    layer: &str,
    z: u32,
    x: u32,
    y: u32,
    ext: &str,
    url: &str,
) -> Option<Vec<u8>> {
    let cache_path = config.cache_path(layer, z, x, y, ext);

    // Try disk cache first
    if cache_path.exists() {
        return fs::read(&cache_path).ok();
    }

    // Download
    let response = ureq::get(url).call().ok()?;
    let mut data = Vec::new();
    response.into_reader().read_to_end(&mut data).ok()?;

    // Write to cache
    if let Some(parent) = cache_path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    let _ = fs::write(&cache_path, &data);

    Some(data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_elevation_decode_formula() {
        // Test the terrain-RGB decode formula with known values
        // R=1, G=134, B=160 should give: -10000 + (1*65536 + 134*256 + 160) * 0.1
        //   = -10000 + (65536 + 34304 + 160) * 0.1 = -10000 + 10000.0 = 0.0
        let r = 1u8;
        let g = 134u8;
        let b = 160u8;
        let h = -10000.0 + (r as f32 * 65536.0 + g as f32 * 256.0 + b as f32) * 0.1;
        assert!((h - 0.0).abs() < 0.1, "sea level decode: {h}");
    }

    #[test]
    fn test_elevation_sample() {
        // Create a simple 2x2 elevation tile
        let tile = ElevationTile {
            heights: vec![0.0, 10.0, 20.0, 30.0],
            width: 2,
            height: 2,
        };
        // Corner samples
        assert!((tile.sample(0.0, 0.0) - 0.0).abs() < 0.01);
        assert!((tile.sample(1.0, 0.0) - 10.0).abs() < 0.01);
        assert!((tile.sample(0.0, 1.0) - 20.0).abs() < 0.01);
        assert!((tile.sample(1.0, 1.0) - 30.0).abs() < 0.01);
        // Center: average of all 4
        assert!((tile.sample(0.5, 0.5) - 15.0).abs() < 0.01);
    }

    #[test]
    fn test_cache_dir() {
        let dir = dirs_cache();
        // Should return a path (not crash)
        assert!(!dir.as_os_str().is_empty());
    }
}
