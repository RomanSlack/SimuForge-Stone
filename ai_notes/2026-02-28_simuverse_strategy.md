n# SimuVerse Strategy Notes
**Date**: 2026-02-28

## The Vision

An AI-native digital reality platform where agentic systems can autonomously
construct, simulate, and experiment with digital twins of anything — from a
single weld joint to an entire factory floor. Agents build worlds from
primitives, run experiments, observe results, and iterate. A programmable,
queryable, fast-forwardable version of physical reality.

SimuForge-Stone was the proof of concept: a full physics + material + rendering
stack built from scratch in Rust, entirely by AI, in weeks. SimuVerse is the
generalization.

---

## Why Build From Scratch (Not Unity/Unreal)

Unity and Unreal are built for humans. GUI editors, drag-and-drop, visual
scripting, asset pipelines. An AI agent needs the exact opposite:

| What AI Needs | Unity/Unreal | Custom Rust |
|--------------|-------------|-------------|
| Headless (no render) | Possible but fights you | Native — rendering is optional |
| 10K parallel sims | Not designed for this | Trivial — just spawn processes |
| Programmatic world construction | Hacky, undocumented APIs | First-class, it's the only way |
| Reset in microseconds | Scene teardown is slow | Memcpy a state struct |
| Full state observability | Reflection/serialization hell | Every field is just a Rust struct |
| Deterministic replay | Floating point chaos across platforms | Controlled, reproducible |
| Sub-millisecond physics steps | 16ms minimum tick | 0.1ms steps, already proven |

You'd spend more time working around engine assumptions than building features.
The SimuForge-Stone experience proved a full stack can be built from scratch
faster than learning an engine's workarounds.

**The rendering is the easy part.** PBR, shadows, SSAO — that's a few thousand
lines. The hard part is the simulation fidelity and the agent interface layer,
which no existing engine provides.

---

## The Actual Value Prop

Not "another simulation engine." The value is the **interface layer between
AI agents and physics**. MCP gave AI agents access to tools and APIs.
SimuVerse gives AI agents access to reality itself.

The platform is a set of composable physics primitives + an agent-facing API:
- Rigid body dynamics
- Deformable solids (FEM)
- Signed distance fields (material add/remove)
- Thermal simulation
- Fluid dynamics (SPH or grid-based)
- Electrical/signal simulation
- Chemical reaction modeling

Agents compose these primitives to model anything. They don't need pre-built
"welding simulation" or "CNC simulation" — they construct those from the
primitives themselves, the same way a programmer builds applications from
language primitives.

---

## Use Cases (Where Real Money Lives)

### 1. Sim-to-Real Robotics
Train robot control policies in simulation, transfer to physical hardware.
The sim needs to be fast (millions of episodes), deterministic (reproducible
results), and physically accurate (forces, contacts, dynamics).

**Who pays:** Robotics companies, warehouse automation, manufacturing
**Competitors:** NVIDIA Isaac Sim, MuJoCo, PyBullet
**Our edge:** Rust performance, AI-native API, headless-first design

### 2. Manufacturing Process Optimization
AI finds optimal parameters by running thousands of variants:
- Welding: voltage, wire speed, travel speed, torch angle → bead quality
- CNC: feed rate, stepover, depth of cut, tool selection → surface finish + cycle time
- 3D printing: layer height, temperature, speed, infill → strength + print time

**Who pays:** Manufacturing companies, job shops, machine tool OEMs
**Our edge:** Already have the CNC sim working. Welding/additive are close.

### 3. Factory Digital Twins
AI agent lays out a production line: machine placement, conveyor routing,
buffer sizing, worker paths. Simulates throughput, finds bottlenecks,
optimizes. Changes one machine → instantly re-simulates.

**Who pays:** Automotive, aerospace, electronics manufacturers
**Competitors:** Siemens Tecnomatix, Dassault DELMIA (expensive, human-operated)
**Our edge:** AI operates it autonomously, iterates 1000x faster than human engineer

### 4. Autonomous R&D / Generative Engineering
"Design a bracket that survives 10,000 load cycles under 50N."
AI generates candidate geometry → simulates stress (FEM) → iterates on design →
converges on optimized part → outputs manufacturing instructions.

**Who pays:** Engineering firms, aerospace, medical devices
**Our edge:** Closed loop from design → simulation → manufacturing plan, all AI-driven

### 5. Training Data Generation
Generate synthetic sensor data for perception models:
- Camera images of robot workspaces (for vision models)
- Force/torque profiles for contact-rich tasks (for policy learning)
- Failure mode scenarios (for safety validation)

**Who pays:** AI/ML teams building robot perception and control
**Competitors:** NVIDIA Omniverse (enterprise, heavy)
**Our edge:** Lightweight, fast, easy to script millions of variations

### 6. Education / Rapid Prototyping
Interactive environment where students or engineers can:
- Prototype robot cells before buying hardware
- Test G-code on a virtual machine before running on a real one
- Experiment with manufacturing processes risk-free

**Who pays:** Universities, makerspaces, individual engineers
**Our edge:** Open source, runs on commodity hardware, no license fees

---

## Architecture Direction

### Core Principles
1. **Headless first, rendering optional** — the default mode is no window, pure computation
2. **Agent API is the primary interface** — humans get a viewer, but agents get the real API
3. **Composable primitives** — don't build "a welding sim," build primitives agents compose
4. **Deterministic by default** — same seed + same actions = same result, always
5. **State is serializable** — snapshot, fork, rewind any simulation at any point
6. **Embarrassingly parallel** — 1000 sims on 1000 cores with no shared state

### Agent Interface (the key differentiator)
The agent talks to SimuVerse the way it talks to any tool — structured
function calls in, structured observations out:

```
Agent → create_body(shape=box, size=[1,1,1], material=steel)
Agent → create_robot(urdf="kuka_kr6.urdf")
Agent → set_joint_targets([0.5, -0.3, 1.2, 0, 0.8, 0])
Agent → step(dt=0.001, n=1000)
Agent ← observation { joint_angles, forces, contacts, temperature_field }
Agent → query_stress(body_id=1, point=[0.5, 0.3, 0.1])
Agent ← { von_mises: 45.2e6, principal: [50e6, 30e6, -5e6] }
```

This could be an MCP server. Claude (or any agent) connects to SimuVerse
as a tool, constructs experiments, runs them, and reasons about results.

### Tech Stack
- **Rust** — performance, safety, no GC pauses during simulation
- **wgpu** — optional rendering when visualization is needed
- **Headless compute** — simulation runs without GPU when doing batch experiments
- **MCP or similar** — agent-facing API protocol
- **WebSocket/gRPC** — for remote agent connections

---

## What To Build Next (Suggested Order)

1. **Additive SDF operations** (union/deposit) — already 90% done from SimuForge-Stone
2. **Headless mode** — strip rendering, expose simulation as a library/API
3. **State serialization** — snapshot/restore any simulation state
4. **MCP server wrapper** — let Claude call simulation functions as tools
5. **Rigid body dynamics** — beyond articulated arms, general multi-body
6. **Basic FEM** — stress/strain on simple geometries
7. **Thermal model** — heat input from welding/printing, conduction, cooling
8. **Scene composition API** — agents build worlds from primitives programmatically

The first four are days of work each. That gets you to a demo where Claude
can autonomously design and simulate a manufacturing process end-to-end.

---

## Open Questions

- Open source vs. commercial? Open core (free engine, paid cloud/enterprise)?
- Solo project vs. seek collaborators early?
- Target audience first: robotics researchers, manufacturing engineers, or AI labs?
- How much physical accuracy is needed vs. "good enough for AI training"?
- GPU compute for parallel sims or keep it CPU-only for simplicity?