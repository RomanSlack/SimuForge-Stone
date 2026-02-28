# Floating Material / Stale Mesh Bug — SOLVED
**Date**: 2026-02-27 (late night, last attempt before bed)
**Time spent**: ~5+ hours across multiple AI sessions, 10+ failed attempts

## The Problem
During timelapse mode, floating planes/shells of material appeared all over the
carved workpiece. They looked like grid-aligned sheets and random chunks of old
geometry hovering in space. Normal carve mode worked perfectly — only timelapse
was broken.

## Failed Approaches (DO NOT REPEAT)
Every single one of these was tried and failed. They all targeted the SDF when
the bug was actually in the mesh pipeline:

1. **Thin shell removal** (morphological ±1 neighbor check) — removes legit geometry
2. **Isosurface bias increase** (0.4*cs) — erodes real geometry, doesn't fix root cause
3. **Per-chunk connected component filtering** — chunks split components, misses cross-chunk
4. **Boundary face removal** — removes legitimate outer faces of the workpiece
5. **Degenerate triangle detection** (< 4 tris) — too crude, misses real artifacts
6. **Ground truth SDF clamping** (pre-carve full result, clamp intermediates) — the ground
   truth itself has the same corruption because it's produced by the same max() CSG chain
7. **i16 quantization upgrade** (i8 → i16) — problem is topological, not precision
8. **SDF redistancing / Fast Sweeping Method** — fixes SDF quality but doesn't fix the
   actual rendering bug (kept as secondary defense layer)
9. **Connectivity-based pocket removal** (global BFS flood-fill) — good for SDF quality
   but doesn't fix the rendering bug either

## THE ACTUAL ROOT CAUSE
**Stale mesh data in ChunkMeshManager for empty chunks.**

When `mesh_chunk()` returns `None` (chunk is fully carved away / all air), the code
did NOTHING — the old GPU mesh for that chunk was never removed from `ChunkMeshManager`.

```rust
// THE BUG — both sync and async paths had this pattern:
if let Some(mesh) = mesh_chunk(&sdf, coord) {
    chunk_meshes.upload_chunk(..., &mesh.positions, &mesh.normals, &mesh.indices);
}
// When mesh_chunk returns None → OLD MESH STAYS FOREVER
```

### Why normal carve mode worked fine:
- Chunks carved slowly (1 command per physics step)
- Chunks almost never go from "has geometry" to "completely empty" in a single frame
- Stale data is never visibly different from current state

### Why timelapse mode broke:
- 17+ commands batch-carved per frame
- Entire chunks go from "has geometry" to "all air" in one batch
- Old mesh persists in GPU buffer → renders as floating planes of stale geometry
- Accumulates over time as more chunks are carved away

## THE FIX (2 logic changes)

### 1. Sync mesh path (`crates/sim/src/main.rs`):
```rust
// BEFORE (bug):
if let Some(mesh) = mesh_chunk(&sdf, *coord) {
    chunk_meshes.upload_chunk(...);
}

// AFTER (fix):
if let Some(mesh) = mesh_chunk(&sdf, *coord) {
    chunk_meshes.upload_chunk(..., &mesh.positions, &mesh.normals, &mesh.indices);
} else {
    chunk_meshes.upload_chunk(..., &[], &[], &[]); // triggers removal
}
```

### 2. Async mesh path (`crates/material/src/async_mesher.rs`):
```rust
// BEFORE (bug):
if let Some(mesh) = mesh_from_snapshot(&req) {
    let _ = tx.send(mesh);
}

// AFTER (fix):
if let Some(mesh) = mesh_from_snapshot(&req) {
    let _ = tx.send(mesh);
} else {
    let _ = tx.send(ChunkMesh { coord, positions: vec![], normals: vec![], indices: vec![] });
}
```

Both paths go through `upload_chunk`'s existing cleanup logic:
```rust
if indices.is_empty() {
    self.chunks.remove(&coord); // removes stale mesh
}
```

## Lesson Learned
When the rendered output looks wrong, don't assume the data (SDF) is wrong.
Trace the FULL pipeline: data → mesh extraction → GPU upload → rendering.
The bug was in the mesh upload step, not the data step. Every SDF fix was
correct but invisible because the renderer was showing stale cached geometry.

## Also in these commits:
- SDF redistancing module (`redistance.rs`) — connectivity cleanup + Fast Sweeping
  as a secondary defense layer for SDF quality
- i16 quantization (Sd8 = i16) for better distance precision
- Isosurface bias (0.15*cs) in mesher + async_mesher for chunk boundary gaps
