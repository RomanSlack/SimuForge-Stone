#!/usr/bin/env python3
"""STL to G-code for SimuForge-Stone CNC simulator.

Converts a 3D mesh into indexed 4-axis G-code: heightmap-based raster
toolpaths with A-axis rotation between faces. Roughing removes bulk
material in waterline layers; finishing follows the actual surface.

Usage:
    python stl_to_gcode.py bust.stl -o alexander.gcode
    python stl_to_gcode.py bust.stl --stepover 2 --stepdown 1.5
    python stl_to_gcode.py bust.stl --faces 0,45,90,135,180,-135,-90,-45

Requires: pip install numpy trimesh
"""

import argparse
import sys
import time

import numpy as np

try:
    import trimesh
except ImportError:
    sys.exit("Install: pip install numpy trimesh")

# Heights in mm
SAFE_Z = 5.0        # retract during cutting
RETRACT_Z = 25.0    # retract for face transitions


def load_and_fit(path, size, rotate_x=90):
    """Load mesh, orient to Z-up, center, scale to fit workpiece."""
    mesh = trimesh.load(path, force="mesh")

    # Pre-rotate model (default: +90 around X converts Y-up OBJ to Z-up)
    if rotate_x != 0:
        rot = trimesh.transformations.rotation_matrix(
            np.radians(rotate_x), [1, 0, 0]
        )
        mesh.apply_transform(rot)

    mesh.vertices -= mesh.centroid
    mesh.apply_scale(0.85 * size / mesh.extents.max())
    return mesh


def heightmap(mesh, half, step):
    """Build heightmap by rasterizing triangles onto a grid.

    For each triangle, projects it to XY, finds grid cells within its
    bounding box, tests point-in-triangle via barycentric coords, and
    records the highest Z value. No spatial index libraries needed.
    """
    xs = np.arange(-half, half + step / 2, step)
    ys = np.arange(-half, half + step / 2, step)
    zmap = np.full((len(ys), len(xs)), np.nan)

    tris = mesh.triangles  # (N, 3, 3)

    for tri in tris:
        v0, v1, v2 = tri
        e1 = v1 - v0
        e2 = v2 - v0

        # Cross product Z component (2D area of triangle in XY)
        cross_z = e1[0] * e2[1] - e1[1] * e2[0]
        if abs(cross_z) < 1e-10:
            continue  # degenerate / vertical triangle

        # XY bounding box -> grid index range
        xmin = min(v0[0], v1[0], v2[0])
        xmax = max(v0[0], v1[0], v2[0])
        ymin = min(v0[1], v1[1], v2[1])
        ymax = max(v0[1], v1[1], v2[1])

        ix0 = max(0, int(np.searchsorted(xs, xmin)) - 1)
        ix1 = min(len(xs), int(np.searchsorted(xs, xmax)) + 1)
        iy0 = max(0, int(np.searchsorted(ys, ymin)) - 1)
        iy1 = min(len(ys), int(np.searchsorted(ys, ymax)) + 1)

        if ix0 >= ix1 or iy0 >= iy1:
            continue

        # Vectorized barycentric test for all grid points in bbox
        gx = xs[ix0:ix1]
        gy = ys[iy0:iy1]
        px, py = np.meshgrid(gx, gy)
        dx = px - v0[0]
        dy = py - v0[1]

        d00 = e1[0] ** 2 + e1[1] ** 2
        d01 = e1[0] * e2[0] + e1[1] * e2[1]
        d11 = e2[0] ** 2 + e2[1] ** 2
        denom = d00 * d11 - d01 * d01
        if abs(denom) < 1e-10:
            continue

        u = (d11 * (dx * e1[0] + dy * e1[1]) - d01 * (dx * e2[0] + dy * e2[1])) / denom
        v = (d00 * (dx * e2[0] + dy * e2[1]) - d01 * (dx * e1[0] + dy * e1[1])) / denom

        inside = (u >= -1e-6) & (v >= -1e-6) & (u + v <= 1 + 1e-6)
        z = v0[2] + u * e1[2] + v * e2[2]

        region = zmap[iy0:iy1, ix0:ix1]
        update = inside & (np.isnan(region) | (z > region))
        region[update] = z[update]

    return xs, ys, zmap


def raster(xs, ys, zmap, z_level, finish, feed):
    """One zigzag raster pass across the heightmap."""
    out = []
    plunge_f = max(int(feed) // 3, 100)

    for iy in range(len(ys)):
        order = range(len(xs)) if iy % 2 == 0 else range(len(xs) - 1, -1, -1)
        cutting = False

        for ix in order:
            zs = zmap[iy, ix]
            if np.isnan(zs):
                if cutting:
                    out.append(f"G0 Z{SAFE_Z:.1f}")
                    cutting = False
                continue

            z = zs if finish else max(z_level, zs)
            if not finish and z >= -0.05:
                if cutting:
                    out.append(f"G0 Z{SAFE_Z:.1f}")
                    cutting = False
                continue

            x, y = float(xs[ix]), float(ys[iy])
            if not cutting:
                out.append(f"G0 X{x:.2f} Y{y:.2f}")
                out.append(f"G1 Z{z:.2f} F{plunge_f}")
                cutting = True
            else:
                out.append(f"G1 X{x:.2f} Y{y:.2f} Z{z:.2f} F{int(feed)}")

        if cutting:
            out.append(f"G0 Z{SAFE_Z:.1f}")

    return out


def gen_face(mesh, a_deg, half, stepover, stepdown, max_depth, feed):
    """Roughing + finishing G-code for one face orientation."""
    rot = trimesh.transformations.rotation_matrix(np.radians(a_deg), [1, 0, 0])
    m = mesh.copy()
    m.apply_transform(rot)

    xs, ys, zraw = heightmap(m, half, stepover)
    surface = zraw - half  # mesh Z -> gcode Z (0 = top, negative = into)

    # Clamp to max cutting depth (can't reach deeper from this face)
    valid = ~np.isnan(surface)
    surface[valid] = np.maximum(surface[valid], -max_depth)

    geo = valid
    if not np.any(geo):
        return []

    floor = max(float(np.nanmin(surface)), -max_depth)

    # Fill ALL empty grid cells with floor depth so surrounding material
    # is fully removed (no walls left around the bust)
    clearance = surface.copy()
    clearance[np.isnan(clearance)] = floor

    lines = [
        "",
        f"(=== Face A={a_deg:.0f} ===)",
        "M5",
        f"G0 Z{RETRACT_Z:.0f}",
        "G0 X0 Y0",
        f"G0 A{a_deg:.0f}",
        "M3 S10000",
        f"G0 Z{SAFE_Z:.0f}",
    ]

    # Roughing: waterline layers
    for z in np.arange(-stepdown, floor - stepdown, -stepdown):
        lines += raster(xs, ys, clearance, z, False, feed)

    # Finishing: follow actual surface
    lines += raster(xs, ys, surface, None, True, feed)

    return lines


def main():
    ap = argparse.ArgumentParser(description="STL to G-code for SimuForge-Stone")
    ap.add_argument("stl", help="Input mesh file (STL/OBJ/PLY)")
    ap.add_argument("-o", "--output", default="output.gcode")
    ap.add_argument("--workpiece", type=float, default=305, help="Cube side mm (305)")
    ap.add_argument("--stepover", type=float, default=4, help="XY spacing mm (4)")
    ap.add_argument("--stepdown", type=float, default=3, help="Z per roughing pass mm (3)")
    ap.add_argument("--max-depth", type=float, default=0,
                    help="Max cut depth mm (0 = auto: workpiece/2 + 5)")
    ap.add_argument("--feed", type=float, default=800, help="Feed rate mm/min (800)")
    ap.add_argument("--faces", default="0,90,-90,180",
                    help="A-axis angles, comma-separated (0,90,-90,180)")
    ap.add_argument("--rotate-x", type=float, default=90,
                    help="Pre-rotate model around X axis (deg). 90 = Y-up to Z-up (default)")
    args = ap.parse_args()

    half = args.workpiece / 2
    max_depth = args.max_depth if args.max_depth > 0 else half + 5
    faces = [float(f) for f in args.faces.split(",")]

    print(f"Loading {args.stl}...")
    mesh = load_and_fit(args.stl, args.workpiece, rotate_x=args.rotate_x)
    print(f"  {len(mesh.vertices):,} verts, {len(mesh.faces):,} tris")
    print(f"  Fitted extents: {mesh.extents.round(1)} mm")
    print(f"  Max depth: {max_depth:.0f}mm (half={half:.0f}mm)")

    gc = [
        "(STL-to-GCode for SimuForge-Stone)",
        f"(Model: {args.stl})",
        f"(Workpiece: {args.workpiece:.0f}mm cube)",
        f"(Stepover: {args.stepover}mm, Stepdown: {args.stepdown}mm, Feed: {args.feed:.0f}mm/min)",
        f"(Max depth: {max_depth:.0f}mm, Rotate-X: {args.rotate_x:.0f}deg)",
        "",
        f"G0 Z{RETRACT_Z:.0f}",
    ]

    t0 = time.time()
    for a in faces:
        print(f"  A={a:+.0f}°...", end=" ", flush=True)
        face = gen_face(mesh, a, half, args.stepover, args.stepdown,
                        max_depth, args.feed)
        gc.extend(face)
        print(f"{sum(1 for l in face if l.startswith('G1')):,} cuts")

    gc += ["", "M5", f"G0 Z{RETRACT_Z:.0f}", "G0 X0 Y0", "G0 A0", "%"]

    with open(args.output, "w") as f:
        f.write("\n".join(gc) + "\n")

    total = sum(1 for l in gc if l.startswith("G1"))
    elapsed = time.time() - t0
    print(f"\n-> {args.output}: {len(gc):,} lines, {total:,} cutting moves ({elapsed:.1f}s)")


if __name__ == "__main__":
    main()
