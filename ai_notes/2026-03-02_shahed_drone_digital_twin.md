# Shahed-136 Digital Twin — Educational Visualization

**Date:** 2026-03-02
**Purpose:** Educational YouTube video — flight profile visualization and digital twin
**Branch:** TBD (new branch off main)

## Concept

Render a Shahed-136 loitering munition flying a realistic trajectory to a target.
No explosion or destructive effects — drone impacts and stops. Pure aerospace
visualization for educational content.

## What We Have (from ping pong engine)

| Component | Reuse | Notes |
|-----------|-------|-------|
| wgpu renderer | 100% | PBR, shadows, SSAO, SSS, composite |
| Camera system | 100% | Orbit camera, could add tracking mode |
| Physics integration | ~80% | Semi-implicit Euler, gravity + drag |
| Collision detection | ~50% | Need ground/building impact instead of paddle |
| Coordinate system | 100% | DH Z-up physics, Y-up render, coord swap |
| Audio engine | ~30% | Could add engine hum, wind, impact thud |

## New Work

### 1. Drone Mesh
- Delta wing planform: ~2.5m wingspan, ~3.5m length
- Simple triangular wing + cylindrical fuselage + V-tail
- Propeller disc at the rear (pusher config)
- Generate procedurally like the paddle mesh, or load OBJ

### 2. Flight Dynamics (simplified)
- Not full 6DOF aero — just a guided point mass with:
  - Thrust (constant ~50 kg, Shahed cruises at ~185 km/h)
  - Drag (Cd ~ 0.03-0.05 for clean delta wing)
  - Lift (L = 0.5 * rho * v^2 * S * Cl, maintain altitude)
  - Gravity
- Guidance: waypoint following with bank-to-turn
- Terminal phase: dive to target at ~30° angle

### 3. Terrain / Environment
- Flat ground plane (reuse floor mesh, scale up)
- Target building: simple box (reuse generate_box)
- Optional: a few scattered buildings for context
- Optional: launch rail/ramp at the start

### 4. Trajectory Phases
1. **Launch**: angled rail at ~15°, accelerates via booster
2. **Climb**: climb to cruise altitude (~100-200m)
3. **Cruise**: level flight along waypoints at ~185 km/h (~51 m/s)
4. **Terminal**: dive toward target GPS coordinate
5. **Impact**: drone stops, sticks to target surface

### 5. Camera Modes
- Orbit (existing) — overview of the scene
- Tracking — camera follows drone, smooth lerp
- Side profile — fixed position, drone flies past
- Terminal POV — behind the drone during dive

### 6. HUD / Overlay
- Altitude, speed, distance to target
- Flight phase label
- Trajectory line (reuse LinePipeline from ball trail)

## Architecture

```
crates/shahed/
  src/
    main.rs         -- App struct, window, render loop (copy from pingpong)
    drone.rs        -- Drone struct: position, velocity, orientation, flight model
    guidance.rs     -- Waypoint following, terminal guidance
    terrain.rs      -- Ground plane + buildings
    mesh.rs         -- Procedural drone mesh generation
    hud.rs          -- egui overlay with flight data
```

## Estimated Effort

| Task | Time |
|------|------|
| New crate scaffold + copy renderer setup | 30 min |
| Drone mesh (procedural delta wing) | 1-2 hr |
| Flight dynamics (point mass + guidance) | 1-2 hr |
| Terrain + target building | 30 min |
| Trajectory phases + waypoints | 1 hr |
| Camera tracking mode | 30 min |
| HUD overlay | 30 min |
| Polish + recording setup | 1 hr |
| **Total** | **~6-8 hours** |

## Key Specs (Shahed-136)

- Wingspan: 2.5m
- Length: 3.5m
- Weight: ~200 kg
- Engine: ~50 hp pusher prop (Mado MD-550)
- Cruise speed: ~185 km/h (51 m/s)
- Range: ~2,500 km
- Cruise altitude: 60-4000m (typically 100-500m for terrain following)
- Navigation: GPS/INS
- Launch: rail-launched with rocket booster
