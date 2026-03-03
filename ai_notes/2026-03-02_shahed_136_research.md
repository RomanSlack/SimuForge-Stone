# Shahed-136 Digital Twin — Research & Implementation Notes

## Date: 2026-03-02

## Overview
Educational flight simulation of a Shahed-136 (Geran-2) delta-wing loitering munition.
Implemented as `crates/drone/` in the SimuForge workspace.

## Key Specs Used in Simulation

| Parameter | Value | Source |
|-----------|-------|--------|
| Total mass | 200 kg | Confirmed |
| Fuel mass | 80 kg | Derived |
| Wing area | 2.75 m^2 | Derived from geometry |
| Aspect ratio | 2.27 | Derived |
| Cd0 | 0.030 | Engineering estimate |
| Oswald e | 0.65 | Typical for low-AR delta |
| CL_max | 1.05 | Vortex-augmented stall |
| Cruise speed | 51.4 m/s (185 km/h) | Confirmed |
| Cruise alt | 300 m | Typical operational |
| Engine | 37 kW (50 hp) | Limbach L550E confirmed |
| Cruise thrust | ~400 N | Derived from drag balance |
| Booster thrust | ~10,000 N | Engineering estimate |
| Booster burn | 3.0 s | Confirmed |
| Fuel rate | 0.0023 kg/s | Derived from BSFC |
| Range | ~50 km (sim) | Shortened for demo |

## Flight Model
- Point-mass with orientation (heading, pitch, bank)
- Bank-to-turn coordinated flight model
- Lift/drag from angle of attack with linear Cl and parabolic drag polar
- Barometric air density model
- 200 Hz physics timestep

## Guidance
- 5-phase state machine: PreLaunch → Launch → Climb → Cruise → Terminal → Impact
- Waypoint navigation with proportional heading control via bank-to-turn
- Altitude hold via pitch command
- Terminal dive with progressive steepening

## Files Created
- `crates/drone/Cargo.toml` — dependencies
- `crates/drone/src/main.rs` — App, render, camera, HUD, time control
- `crates/drone/src/drone.rs` — Procedural delta-wing mesh
- `crates/drone/src/flight.rs` — Aerodynamic point-mass model
- `crates/drone/src/guidance.rs` — Waypoint following + phase state machine
- `crates/drone/src/terrain.rs` — Desert ground + buildings + rail
- `crates/drone/src/sound.rs` — Engine voice (sawtooth + noise)

## Controls
- Space: Launch
- 1-5: Time scale (1x, 10x, 50x, 100x, 200x)
- C: Cycle camera (Orbit/Chase/Side)
- P: Pause
- Mouse: Orbit camera
- Escape: Quit
