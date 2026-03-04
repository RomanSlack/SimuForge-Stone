//! Real-world terrain manager using Mapbox elevation + satellite tiles.
//!
//! Streams tiles around the camera, generates textured meshes for GPU rendering,
//! and provides terrain_height() for physics queries via bilinear elevation sampling.

use std::collections::HashMap;

use simuforge_core::TexturedVertex;
use simuforge_render::pipelines::pbr::PbrTexturedPipeline;
use wgpu::util::DeviceExt;

use crate::geo::{self, GeoOrigin};
use crate::tile_fetch::{ElevationTile, MapboxConfig, RawTile, TileCoord, TileLoader};

/// Mesh resolution: vertices per tile edge. 128×128 quads = 16641 verts, 98304 indices.
const MESH_RES: u32 = 128;
/// Tile streaming radius (in tiles). 5 → 11×11 = 121 tiles.
const TILE_RADIUS: i32 = 5;
/// Eviction radius (tiles beyond this are unloaded). Hysteresis prevents thrashing.
const EVICT_RADIUS: i32 = 7;
/// Zoom level for elevation tiles.
const ELEV_ZOOM: u32 = 14;
/// Zoom level for satellite tiles (one level higher than elevation for 4× detail).
const SAT_ZOOM: u32 = 15;

/// Per-tile GPU resources.
pub struct LoadedTile {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub num_indices: u32,
    pub texture: wgpu::Texture,
    pub texture_view: wgpu::TextureView,
    pub texture_bind_group: wgpu::BindGroup,
    /// Keep elevation data for physics height queries.
    pub elevation: ElevationTile,
    /// Tile bounds in render-space for quick lookups.
    pub render_min_x: f32,
    pub render_max_x: f32,
    pub render_min_z: f32,
    pub render_max_z: f32,
}

/// Real-world terrain manager.
pub struct RealTerrainManager {
    pub origin: GeoOrigin,
    loader: TileLoader,
    pub tiles: HashMap<TileCoord, LoadedTile>,
    /// Sampler shared by all tile textures.
    pub sampler: wgpu::Sampler,
    /// Current camera tile position for streaming.
    last_cam_tile: (i32, i32),
    /// Material bind group for textured terrain (shared camera/light/material).
    pub material_buffer: wgpu::Buffer,
    pub material_bind_group: wgpu::BindGroup,
    /// Elevation offset: subtracted from all heights so origin ground = 0.
    /// Set from the first tile loaded that covers the origin.
    pub elevation_offset: f32,
    elevation_offset_set: bool,
}

impl RealTerrainManager {
    /// Create a new terrain manager centered on a geographic origin.
    pub fn new(
        device: &wgpu::Device,
        config: MapboxConfig,
        origin: GeoOrigin,
        pbr: &simuforge_render::pipelines::pbr::PbrPipeline,
    ) -> Self {
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Satellite Sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        let loader = TileLoader::new(config, 4, ELEV_ZOOM, SAT_ZOOM);

        let (material_buffer, material_bind_group) = pbr.create_material_bind_group(device);

        Self {
            origin,
            loader,
            tiles: HashMap::new(),
            sampler,
            last_cam_tile: (i32::MAX, i32::MAX),
            material_buffer,
            material_bind_group,
            elevation_offset: 0.0,
            elevation_offset_set: false,
        }
    }

    /// Update tile streaming based on camera position (render-space).
    /// Call once per frame.
    pub fn update(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        cam_rx: f32,
        cam_rz: f32,
        textured_pipeline: &PbrTexturedPipeline,
    ) {
        // Receive completed tiles from background threads
        while let Ok(raw) = self.loader.receiver.try_recv() {
            let coord = TileCoord {
                z: raw.zoom,
                x: raw.tx,
                y: raw.ty,
            };
            self.loader.mark_complete(&coord);

            // Auto-detect elevation offset from the first tile covering the origin
            if !self.elevation_offset_set {
                // Sample elevation at the origin (render 0,0 = ENU 0,0 = our lat/lon origin)
                let origin_height = raw.elevation.sample(0.5, 0.5);
                self.elevation_offset = origin_height;
                self.elevation_offset_set = true;
                eprintln!(
                    "[terrain] Elevation offset: {:.0}m (ground level at origin)",
                    self.elevation_offset
                );
            }

            let loaded = self.upload_tile(device, queue, raw, textured_pipeline);
            self.tiles.insert(coord, loaded);
        }

        // Convert camera render-space to lat/lon
        let (lat, lon) = self.origin.render_to_latlon(cam_rx, cam_rz);
        let (cam_tx, cam_ty) = geo::latlon_to_tile(lat, lon, ELEV_ZOOM);
        let cam_tile = (cam_tx as i32, cam_ty as i32);

        if cam_tile == self.last_cam_tile {
            return;
        }
        self.last_cam_tile = cam_tile;

        // Request tiles in ring around camera, nearest first
        let mut requests: Vec<(i32, TileCoord)> = Vec::new();
        for dx in -TILE_RADIUS..=TILE_RADIUS {
            for dy in -TILE_RADIUS..=TILE_RADIUS {
                let tx = cam_tile.0 + dx;
                let ty = cam_tile.1 + dy;
                if tx < 0 || ty < 0 {
                    continue;
                }
                let coord = TileCoord {
                    z: ELEV_ZOOM,
                    x: tx as u32,
                    y: ty as u32,
                };
                if !self.tiles.contains_key(&coord) && !self.loader.is_pending(&coord) {
                    let dist = dx * dx + dy * dy;
                    requests.push((dist, coord));
                }
            }
        }
        requests.sort_by_key(|r| r.0);
        for (_, coord) in requests {
            self.loader.request(coord);
        }

        // Evict distant tiles
        let evict: Vec<TileCoord> = self
            .tiles
            .keys()
            .filter(|c| {
                let dx = (c.x as i32 - cam_tile.0).abs();
                let dy = (c.y as i32 - cam_tile.1).abs();
                dx > EVICT_RADIUS || dy > EVICT_RADIUS
            })
            .copied()
            .collect();
        for coord in evict {
            self.tiles.remove(&coord);
        }
    }

    /// Upload a raw tile to GPU: create mesh and texture.
    fn upload_tile(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        raw: RawTile,
        textured_pipeline: &PbrTexturedPipeline,
    ) -> LoadedTile {
        // Compute tile geographic bounds
        let (lat_min, lon_min, lat_max, lon_max) =
            geo::tile_bounds(raw.tx, raw.ty, ELEV_ZOOM);

        // Convert corners to render-space
        let (e_min, n_min) = self.origin.latlon_to_enu(lat_min, lon_min);
        let (e_max, n_max) = self.origin.latlon_to_enu(lat_max, lon_max);
        let (rx_min, _ry_min, rz_max) = self.origin.enu_to_render(e_min, n_min, 0.0);
        let (rx_max, _ry_max, rz_min) = self.origin.enu_to_render(e_max, n_max, 0.0);

        // Generate mesh (subtract elevation_offset so origin ground ≈ 0)
        let (vertices, indices) =
            mesh_tile(&raw.elevation, rx_min, rx_max, rz_min, rz_max, self.elevation_offset);

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Terrain Tile VB"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Terrain Tile IB"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        // Upload satellite texture
        let tex_size = wgpu::Extent3d {
            width: raw.satellite_width,
            height: raw.satellite_height,
            depth_or_array_layers: 1,
        };
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Satellite Tile"),
            size: tex_size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &raw.satellite_rgba,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(4 * raw.satellite_width),
                rows_per_image: Some(raw.satellite_height),
            },
            tex_size,
        );
        let texture_view = texture.create_view(&Default::default());
        let texture_bind_group =
            textured_pipeline.create_texture_bind_group(device, &texture_view, &self.sampler);

        LoadedTile {
            vertex_buffer,
            index_buffer,
            num_indices: indices.len() as u32,
            texture,
            texture_view,
            texture_bind_group,
            elevation: raw.elevation,
            render_min_x: rx_min,
            render_max_x: rx_max,
            render_min_z: rz_min,
            render_max_z: rz_max,
        }
    }

    /// Query terrain height at a render-space position.
    /// Returns Some(height_ry) if a tile is loaded at that position, None otherwise.
    pub fn terrain_height(&self, rx: f32, rz: f32) -> Option<f32> {
        // Find the tile that contains this position
        for tile in self.tiles.values() {
            if rx >= tile.render_min_x
                && rx <= tile.render_max_x
                && rz >= tile.render_min_z
                && rz <= tile.render_max_z
            {
                // Compute fractional position within tile
                let fx = (rx - tile.render_min_x) / (tile.render_max_x - tile.render_min_x);
                let fz = (rz - tile.render_min_z) / (tile.render_max_z - tile.render_min_z);
                // Note: rz maps to tile Y inversely (rz = -north, tile Y increases southward)
                let fy = fz;
                let height = tile.elevation.sample(fx, fy) - self.elevation_offset;
                return Some(height);
            }
        }
        None
    }

    /// Check if real terrain has any tiles loaded (i.e., is active).
    pub fn is_active(&self) -> bool {
        !self.tiles.is_empty()
    }
}

/// Generate a mesh grid from elevation data with UVs.
/// `elev_offset` is subtracted from all heights so the origin ground level ≈ 0.
fn mesh_tile(
    elevation: &ElevationTile,
    rx_min: f32,
    rx_max: f32,
    rz_min: f32,
    rz_max: f32,
    elev_offset: f32,
) -> (Vec<TexturedVertex>, Vec<u32>) {
    let n = MESH_RES + 1;
    let num_verts = (n * n) as usize;
    let num_indices = (MESH_RES * MESH_RES * 6) as usize;
    let mut verts = Vec::with_capacity(num_verts);
    let mut idxs = Vec::with_capacity(num_indices);

    let dx = rx_max - rx_min;
    let dz = rz_max - rz_min;

    // Generate vertices
    for iy in 0..n {
        for ix in 0..n {
            let u = ix as f32 / MESH_RES as f32;
            let v = iy as f32 / MESH_RES as f32;
            let rx = rx_min + u * dx;
            let rz = rz_min + v * dz;
            let ry = elevation.sample(u, v) - elev_offset;

            verts.push(TexturedVertex {
                position: [rx, ry, rz],
                normal: [0.0, 1.0, 0.0], // placeholder, computed below
                uv: [u, v],
            });
        }
    }

    // Compute normals via finite differences
    for iy in 0..n {
        for ix in 0..n {
            let idx = (iy * n + ix) as usize;

            let h_left = if ix > 0 {
                verts[idx - 1].position[1]
            } else {
                verts[idx].position[1]
            };
            let h_right = if ix < n - 1 {
                verts[idx + 1].position[1]
            } else {
                verts[idx].position[1]
            };
            let h_down = if iy > 0 {
                verts[(idx as u32 - n) as usize].position[1]
            } else {
                verts[idx].position[1]
            };
            let h_up = if iy < n - 1 {
                verts[(idx as u32 + n) as usize].position[1]
            } else {
                verts[idx].position[1]
            };

            let step_x = dx / MESH_RES as f32;
            let step_z = dz / MESH_RES as f32;

            let nx = (h_left - h_right) / (2.0 * step_x);
            let nz = (h_down - h_up) / (2.0 * step_z);
            let ny = 1.0_f32;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            verts[idx].normal = [nx / len, ny / len, nz / len];
        }
    }

    // Generate indices
    for iy in 0..MESH_RES {
        for ix in 0..MESH_RES {
            let i00 = iy * n + ix;
            let i10 = i00 + 1;
            let i01 = i00 + n;
            let i11 = i01 + 1;
            idxs.extend_from_slice(&[i00, i01, i10, i10, i01, i11]);
        }
    }

    (verts, idxs)
}
