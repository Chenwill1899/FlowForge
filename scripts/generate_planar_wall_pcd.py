#!/usr/bin/env python3
"""Generate the deterministic obstacle PCDs used by the ROS regressions.

Historically this produced a single full-height planar wall (`wall:`). The 3D
fluid-guidance scenarios extend the schema with purely optional keys -- a
scenario without them produces byte-identical output to the original script:

  wall:
    ...existing keys...
    windows:            # optional rectangular cutouts in the (y, z) plane
      - {y_min: -0.7, y_max: 0.7, z_min: 1.2, z_max: 2.4}
  boxes:                # optional extra boxes, same keys as wall; z_min > 0
    - {center_x: 4.0, center_y: 0.0, thickness_x: 0.2, half_length_y: 4.8,
       z_min: 1.3, z_max: 3.3, point_spacing: 0.1}
  ground:               # optional single-plane floor grid. Needed by the 3D
                        # maps: with an in-bounds virtual ceiling the ESDF has
                        # no accidental ground layer, so the floor must come
                        # from real points.
    {x_min: -3.0, x_max: 12.0, y_min: -5.5, y_max: 5.5, z: 0.0,
     point_spacing: 0.1}

`wall` becomes optional once `boxes` is given.

Further optional keys for the 3D scenarios:

  pillars:              # explicit (optionally tilted) cylindrical pillars
    - {center_x: 6.0, center_y: 1.0, radius: 0.35, z_min: 0.05, z_max: 3.35,
       point_spacing: 0.1, tilt_deg: 20, tilt_azimuth_deg: 90}
  random:               # seeded procedural field -- SAME SEED, SAME BYTES.
                        # Vertical blockers (walls/pillars/pillar_clusters)
                        # are placed with a minimum mutual gap so the field
                        # stays flyable; sills and overhangs are vertically
                        # passable and place freely. pillar_clusters is a
                        # full-height center pillar flanked by two low
                        # pillars at a random azimuth (dodge past the flanks
                        # or hop over them), placed as one blocker. Optional
                        # corridor_y/corridor_jitter narrow the y sampling
                        # (not x) for window_walls/sills/overhangs/
                        # pillar_clusters ONLY -- forces those four onto a
                        # flight line at y~corridor_y (their own half-length/
                        # radius, always >> the jitter, then guarantees
                        # coverage) while their x position stays random over
                        # the full region and plain pillars/tilted_pillars
                        # stay free over the full region on both axes. See
                        # expand_random() for every knob.
    {seed: 1, region: {...}, keepout: [...], min_gap: 1.4,
     corridor_y: -1.0, corridor_jitter: 0.6, ...}
  swap_xy: true          # transpose every point's x and y as a final step,
                        # after everything else (wall/boxes/pillars/random/
                        # ground/replicas/absolute_*) is built in the
                        # schema's native x-thin/y-long, x-travel
                        # orientation. Use when the target world frame's
                        # long/travel axis is y instead of x.
"""

import argparse
import math
import random
from pathlib import Path

import yaml


def inclusive_values(start, stop, step):
    if step <= 0.0 or stop < start:
        raise ValueError("invalid inclusive range")
    count = int(round((stop - start) / step))
    if abs(start + count * step - stop) > 1e-8:
        raise ValueError("range endpoints must be an integer number of steps apart")
    return [start + index * step for index in range(count + 1)]


def box_points(spec):
    spacing = float(spec["point_spacing"])
    center_x = float(spec["center_x"])
    center_y = float(spec["center_y"])
    half_x = 0.5 * float(spec["thickness_x"])
    half_y = float(spec["half_length_y"])
    tilt = math.radians(float(spec.get("tilt_deg", 0.0)))
    azimuth = math.radians(float(spec.get("tilt_azimuth_deg", 0.0)))
    yaw = math.radians(float(spec.get("yaw_deg", 0.0)))
    drift = math.tan(tilt)
    cyaw, syaw = math.cos(yaw), math.sin(yaw)

    xs = inclusive_values(center_x - half_x, center_x + half_x, spacing)
    ys = inclusive_values(center_y - half_y, center_y + half_y, spacing)
    zs = inclusive_values(float(spec["z_min"]), float(spec["z_max"]), spacing)

    windows = spec.get("windows") or []

    def inside_any_window(y, z):
        for window in windows:
            if (float(window["y_min"]) < y < float(window["y_max"])
                    and float(window["z_min"]) < z < float(window["z_max"])):
                return True
        return False

    # Ordering (x outer, y middle, z inner) is load-bearing: legacy scenarios
    # must reproduce the original file byte for byte.
    points = []
    for x in xs:
        for y in ys:
            for z in zs:
                if inside_any_window(y, z):
                    continue
                dz = z - float(spec["z_min"])
                local_x, local_y = x - center_x, y - center_y
                rotated_x = center_x + cyaw * local_x - syaw * local_y
                rotated_y = center_y + syaw * local_x + cyaw * local_y
                points.append((rotated_x + drift * dz * math.cos(azimuth + yaw),
                               rotated_y + drift * dz * math.sin(azimuth + yaw), z))
    return points


def wall_points(config):
    return box_points(config["wall"])


def ground_points(spec):
    spacing = float(spec["point_spacing"])
    z = float(spec.get("z", 0.0))
    xs = inclusive_values(float(spec["x_min"]), float(spec["x_max"]), spacing)
    ys = inclusive_values(float(spec["y_min"]), float(spec["y_max"]), spacing)
    return [(x, y, z) for x in xs for y in ys]


def cylinder_points(spec):
    """Solid (optionally tilted) pillar: a filled horizontal disc per z level.

    Tilt is modelled by shifting each disc's centre along the azimuth by
    tan(tilt) * (z - z_min) -- the vertical-section approximation, accurate
    for the moderate tilts (< ~35 deg) these scenes use and trivially
    deterministic."""
    spacing = float(spec["point_spacing"])
    cx = float(spec["center_x"])
    cy = float(spec["center_y"])
    radius = float(spec["radius"])
    tilt = math.radians(float(spec.get("tilt_deg", 0.0)))
    azimuth = math.radians(float(spec.get("tilt_azimuth_deg", 0.0)))
    drift = math.tan(tilt)
    points = []
    for z in inclusive_values(float(spec["z_min"]), float(spec["z_max"]), spacing):
        dx = drift * (z - float(spec["z_min"])) * math.cos(azimuth)
        dy = drift * (z - float(spec["z_min"])) * math.sin(azimuth)
        rr = 0.0
        while rr <= radius + 1e-9:
            if rr < 1e-9:
                points.append((cx + dx, cy + dy, z))
            else:
                n = max(6, int(math.ceil(2.0 * math.pi * rr / spacing)))
                for k in range(n):
                    theta = 2.0 * math.pi * k / n
                    points.append((cx + dx + rr * math.cos(theta),
                                   cy + dy + rr * math.sin(theta), z))
            rr += spacing
    return points


def expand_random(block):
    """Expand a seeded `random:` block into primitive specs (deterministic).

    Returns {"boxes": [...], "pillars": [...]}. Placement: vertical blockers
    (window walls, pillars, tilted pillars) are rejection-sampled with at
    least `min_gap` horizontal clearance between bounding circles, so every
    corridor between blockers stays flyable (min_gap should comfortably
    exceed 2*d_s). Sills and overhangs are crossed vertically, so they only
    respect the keepout zones, not the gap graph. The expansion order and
    every RNG draw are fixed by the code path: one seed, one scene, forever.
    """
    rng = random.Random(int(block["seed"]))
    spacing = float(block.get("point_spacing", 0.1))
    region = block["region"]
    x_lo, x_hi = float(region["x_min"]), float(region["x_max"])
    y_lo, y_hi = float(region["y_min"]), float(region["y_max"])
    keepouts = [(float(k["x"]), float(k["y"]), float(k["radius"]))
                for k in block.get("keepout") or []]
    min_gap = float(block.get("min_gap", 1.4))
    z_top = float(block.get("z_top", 3.35))
    z_base = float(block.get("z_base", 0.05))
    # Optional flight corridor: when set, "corridor" placements (see the
    # corridor=True calls below) draw y from a narrow +/-corridor_jitter
    # band around corridor_y instead of the full region -- so an obstacle
    # too far off to one side can never be placed, and every corridor
    # obstacle's own half-length/radius (always well over the jitter) then
    # guarantees its footprint spans y=corridor_y. x is still drawn from the
    # full region, so along-crossing position/order/spacing stays random.
    # Omitted (None): unchanged legacy behavior, full-region y sampling.
    corridor_y = block.get("corridor_y")
    corridor_y = None if corridor_y is None else float(corridor_y)
    corridor_jitter = float(block.get("corridor_jitter", 0.8))
    corridor_gap = float(block.get("corridor_gap", 1.0))
    window_centers_y = block.get("window_centers_y")
    window_tilts_deg = block.get("window_tilts_deg")
    window_tilt_azimuth_deg = float(block.get("window_tilt_azimuth_deg", 0.0))
    window_yaws_deg = block.get("window_yaws_deg")
    overhang_centers_x = block.get("overhang_centers_x")
    sill_centers_x = block.get("sill_centers_x")
    route_clearance = float(block.get("route_clearance", 0.0))
    corridor_boxes = []  # (travel-axis center, half thickness)

    def snap(value):
        return round(round(value / spacing) * spacing, 6)

    def z_range(lo, hi):
        # Endpoints may sit anywhere (z_base is 0.05 on the 3D maps); only
        # the SPAN must be an integer number of point_spacing steps
        # (inclusive_values' lattice rule), and it must not poke past z_top.
        span = max(spacing, round((hi - lo) / spacing) * spacing)
        while lo + span > z_top + 1e-9:
            span -= spacing
        return round(lo, 6), round(lo + span, 6)

    def rand_range(pair):
        return rng.uniform(float(pair[0]), float(pair[1]))

    blockers = []   # (x, y, bounding_radius) of vertical blockers
    field_blockers = []

    def place(bound_radius, blocking, corridor=False, axial_half=None,
              forced_y=None, forced_x=None, avoid_blockers=False,
              blocker_gap=None, field_blocking=False, field_radius=None):
        for _ in range(300):
            if forced_x is not None:
                x = float(forced_x)
                if x < x_lo + bound_radius or x > x_hi - bound_radius:
                    return None
            else:
                x = rng.uniform(x_lo + bound_radius, x_hi - bound_radius)
            if forced_y is not None:
                y = float(forced_y)
            elif corridor and corridor_y is not None:
                y = rng.uniform(corridor_y - corridor_jitter, corridor_y + corridor_jitter)
            else:
                y = rng.uniform(y_lo + bound_radius, y_hi - bound_radius)
            if any(math.hypot(x - kx, y - ky) < bound_radius + kr
                   for kx, ky, kr in keepouts):
                continue
            if (blocking and not corridor and route_clearance > 0.0
                    and corridor_y is not None
                    and abs(y - corridor_y) < route_clearance + bound_radius):
                continue
            clearance_gap = min_gap if blocker_gap is None else blocker_gap
            if blocking and any(
                    math.hypot(x - bx, y - by) < bound_radius + br + clearance_gap
                    for bx, by, br in blockers):
                continue
            if avoid_blockers and axial_half is not None and any(
                    abs(x - bx) < axial_half + br + clearance_gap
                    and abs(y - by) < bound_radius + br + clearance_gap
                    for bx, by, br in field_blockers):
                continue
            if axial_half is not None and any(
                    abs(x - bx) < axial_half + bh + corridor_gap
                    for bx, bh in corridor_boxes):
                continue
            if blocking:
                blockers.append((x, y, bound_radius))
                if field_blocking:
                    field_blockers.append((x, y, bound_radius
                                           if field_radius is None
                                           else field_radius))
            if axial_half is not None:
                corridor_boxes.append((x, axial_half))
            return x, y
        return None

    boxes, pillars = [], []

    for spec in [block.get("window_walls") or {}]:
        for i in range(int(spec.get("count", 0))):
            half = rand_range(spec.get("half_length", [2.0, 3.0]))
            forced_y = None
            if window_centers_y is not None and i < len(window_centers_y):
                forced_y = window_centers_y[i]
            spot = place(half + 0.2, blocking=True, corridor=True,
                         axial_half=0.1, forced_y=forced_y)
            if spot is None:
                continue
            x, y = spot
            width = float(spec.get("window_width", 1.6))
            wz = spec.get("window_z", [1.1, 2.5])
            window_box = {
                "center_x": snap(x), "center_y": snap(y),
                "thickness_x": 0.2, "half_length_y": snap(half),
                "z_min": z_base, "z_max": z_top, "point_spacing": spacing,
                "windows": [{"y_min": snap(y - 0.5 * width),
                             "y_max": snap(y + 0.5 * width),
                             "z_min": float(wz[0]), "z_max": float(wz[1])}],
            }
            if window_tilts_deg is not None and i < len(window_tilts_deg):
                window_box["tilt_deg"] = float(window_tilts_deg[i])
                window_box["tilt_azimuth_deg"] = window_tilt_azimuth_deg
            if window_yaws_deg is not None and i < len(window_yaws_deg):
                window_box["yaw_deg"] = float(window_yaws_deg[i])
            boxes.append(window_box)

    # pillar_clusters runs before the plain pillars/tilted_pillars below:
    # its bound_radius (center to flank tip) is the largest footprint any
    # random primitive claims here, so giving it first pick of open space
    # avoids it losing every placement retry to smaller, more flexible
    # pillars that ran first and fragmented the region.
    for spec in [block.get("pillar_clusters") or {}]:
        # Full-height center pillar flanked by two low pillars at a random
        # azimuth -- open beyond the flanks costs a detour, so the flyable
        # move is a lateral dodge past the flanks or a hop over them. The
        # whole 3-pillar group is one blocker (bound_radius spans center to
        # flank tip) so it keeps min_gap from every other placement.
        center_x_offsets = spec.get("center_x_offsets") or []
        center_y_offsets = spec.get("center_y_offsets") or []
        for cluster_index in range(int(spec.get("count", 0))):
            center_radius = rand_range(spec.get("center_radius", [0.5, 0.7]))
            flank_radius = rand_range(spec.get("flank_radius", [0.4, 0.6]))
            flank_offset = rand_range(spec.get("flank_offset", [1.5, 2.2]))
            bound = center_radius + flank_offset + flank_radius
            spot = place(bound, blocking=True, corridor=True,
                         field_blocking=True)
            if spot is None:
                continue
            cx, cy = spot
            # A scenario may apply a post-placement native-y offset to an
            # individual cluster.  This preserves all RNG draws and every
            # other obstacle position while translating the center and both
            # flanks as one rigid group.  With swap_xy, native y is world x.
            if cluster_index < len(center_x_offsets):
                cx += float(center_x_offsets[cluster_index])
            if cluster_index < len(center_y_offsets):
                cy += float(center_y_offsets[cluster_index])
            azimuth = math.radians(rng.uniform(0.0, 360.0))
            _, center_top = z_range(z_base, z_top)
            pillars.append({"center_x": cx, "center_y": cy, "radius": center_radius,
                            "z_min": z_base, "z_max": center_top,
                            "point_spacing": spacing})
            flank_height = spec.get("flank_height", [0.8, 1.2])
            for sign in (-1.0, 1.0):
                fx = cx + sign * flank_offset * math.cos(azimuth)
                fy = cy + sign * flank_offset * math.sin(azimuth)
                _, flank_top = z_range(z_base, rand_range(flank_height))
                pillars.append({"center_x": fx, "center_y": fy, "radius": flank_radius,
                                "z_min": z_base, "z_max": flank_top,
                                "point_spacing": spacing})

    for spec in [block.get("pillars") or {}]:
        for _ in range(int(spec.get("count", 0))):
            radius = rand_range(spec.get("radius", [0.25, 0.45]))
            spot = place(radius, blocking=True, field_blocking=True,
                         field_radius=radius)
            if spot is None:
                continue
            x, y = spot
            full = rng.random() < float(spec.get("full_height_ratio", 0.7))
            _, top = z_range(z_base, z_top if full
                             else rand_range(spec.get("low_height", [1.4, 2.2])))
            pillars.append({"center_x": x, "center_y": y, "radius": radius,
                            "z_min": z_base, "z_max": top,
                            "point_spacing": spacing})

    for spec in [block.get("tilted_pillars") or {}]:
        for _ in range(int(spec.get("count", 0))):
            radius = rand_range(spec.get("radius", [0.2, 0.32]))
            tilt = rand_range(spec.get("tilt_deg", [12.0, 28.0]))
            azimuth = rng.uniform(0.0, 360.0)
            height = z_top - 0.3 - z_base
            bound = radius + math.tan(math.radians(tilt)) * height
            spot = place(bound, blocking=True, field_blocking=True,
                         field_radius=radius)
            if spot is None:
                continue
            x, y = spot
            _, tilt_top = z_range(z_base, z_base + height)
            pillars.append({"center_x": x, "center_y": y, "radius": radius,
                            "z_min": z_base, "z_max": tilt_top,
                            "point_spacing": spacing,
                            "tilt_deg": tilt, "tilt_azimuth_deg": azimuth})

    for spec in [block.get("sills") or {}]:
        sill_tilts_deg = block.get("sill_tilts_deg")
        sill_tilt_azimuth_deg = float(block.get("sill_tilt_azimuth_deg", 0.0))
        sill_yaws_deg = block.get("sill_yaws_deg")
        for i in range(int(spec.get("count", 0))):
            half = rand_range(spec.get("half_length", [1.5, 2.8]))
            forced_x = None
            if sill_centers_x is not None and i < len(sill_centers_x):
                forced_x = sill_centers_x[i]
            spot = place(half, blocking=False, corridor=True,
                         axial_half=0.2,
                         forced_x=forced_x)
            if spot is None:
                continue
            x, y = spot
            _, sill_top = z_range(0.0, rand_range(spec.get("height", [0.3, 0.6])))
            sill_box = {
                "center_x": snap(x), "center_y": snap(y),
                "thickness_x": snap(rand_range(spec.get("thickness", [0.2, 0.4]))),
                "half_length_y": snap(half),
                "z_min": 0.0, "z_max": sill_top,
                "point_spacing": spacing,
            }
            if sill_tilts_deg is not None and i < len(sill_tilts_deg):
                sill_box["tilt_deg"] = float(sill_tilts_deg[i])
                sill_box["tilt_azimuth_deg"] = sill_tilt_azimuth_deg
            if sill_yaws_deg is not None and i < len(sill_yaws_deg):
                sill_box["yaw_deg"] = float(sill_yaws_deg[i])
            boxes.append(sill_box)

    for spec in [block.get("overhangs") or {}]:
        overhang_tilts_deg = block.get("overhang_tilts_deg")
        overhang_tilt_azimuth_deg = float(block.get("overhang_tilt_azimuth_deg", 0.0))
        overhang_yaws_deg = block.get("overhang_yaws_deg")
        for i in range(int(spec.get("count", 0))):
            half = rand_range(spec.get("half_length", [1.0, 2.2]))
            forced_x = None
            if overhang_centers_x is not None and i < len(overhang_centers_x):
                forced_x = overhang_centers_x[i]
            spot = place(half, blocking=False, corridor=True,
                         axial_half=0.3, forced_x=forced_x,
                         avoid_blockers=True, blocker_gap=0.3)
            if spot is None:
                continue
            x, y = spot
            z_lo = round(rand_range(spec.get("z_min", [1.6, 2.0])), 6)
            z_lo, z_hi = z_range(z_lo, z_lo + rand_range(spec.get("depth", [0.6, 1.2])))
            overhang_box = {
                "center_x": snap(x), "center_y": snap(y),
                "thickness_x": snap(rand_range(spec.get("thickness", [0.3, 0.6]))),
                "half_length_y": snap(half),
                "z_min": z_lo, "z_max": z_hi,
                "point_spacing": spacing,
            }
            if overhang_tilts_deg is not None and i < len(overhang_tilts_deg):
                overhang_box["tilt_deg"] = float(overhang_tilts_deg[i])
                overhang_box["tilt_azimuth_deg"] = overhang_tilt_azimuth_deg
            if overhang_yaws_deg is not None and i < len(overhang_yaws_deg):
                overhang_box["yaw_deg"] = float(overhang_yaws_deg[i])
            boxes.append(overhang_box)

    # Add a second, side-only obstacle layer. These obstacles are sampled in
    # the two transverse bands outside the nominal crossing strip, so they add
    # scene richness without changing the guaranteed two-window route.
    side = block.get("side_obstacles") or {}
    side_bands = side.get("bands", [[-7.0, -4.2], [2.2, 7.0]])
    side_gap = float(side.get("gap", 1.2))
    side_boxes = []  # (band index, travel-axis center, half thickness)

    def place_side(band_index, half_thickness):
        lo, hi = [float(v) for v in side_bands[band_index]]
        for _ in range(300):
            x = rng.uniform(x_lo + half_thickness, x_hi - half_thickness)
            y = rng.uniform(lo + 0.3, hi - 0.3)
            if any(bi == band_index
                   and abs(x - bx) < half_thickness + bh + side_gap
                   for bi, bx, bh in side_boxes):
                continue
            side_boxes.append((band_index, x, half_thickness))
            return x, y
        return None

    side_window = side.get("window_walls") or {}
    for band_index in range(len(side_bands)):
        for _ in range(int(side_window.get("count_per_side", 0))):
            half = rand_range(side_window.get("half_length", [1.5, 2.5]))
            spot = place_side(band_index, 0.1)
            if spot is None:
                continue
            x, y = spot
            width = float(side_window.get("window_width", 1.5))
            wz = side_window.get("window_z", [1.0, 2.4])
            boxes.append({
                "center_x": snap(x), "center_y": snap(y),
                "thickness_x": 0.2, "half_length_y": snap(half),
                "z_min": z_base, "z_max": z_top,
                "point_spacing": spacing,
                "windows": [{"y_min": snap(y - 0.5 * width),
                             "y_max": snap(y + 0.5 * width),
                             "z_min": float(wz[0]), "z_max": float(wz[1])}],
                "tilt_deg": rand_range(side_window.get("tilt_deg", [-18.0, 18.0])),
                "yaw_deg": rand_range(side_window.get("yaw_deg", [-25.0, 25.0])),
            })

    side_sill = side.get("sills") or {}
    for band_index in range(len(side_bands)):
        for _ in range(int(side_sill.get("count_per_side", 0))):
            half = rand_range(side_sill.get("half_length", [1.2, 2.2]))
            spot = place_side(band_index, 0.2)
            if spot is None:
                continue
            x, y = spot
            _, top = z_range(0.0, rand_range(side_sill.get("height", [0.7, 1.4])))
            boxes.append({
                "center_x": snap(x), "center_y": snap(y),
                "thickness_x": snap(rand_range(side_sill.get("thickness", [0.2, 0.4]))),
                "half_length_y": snap(half), "z_min": 0.0, "z_max": top,
                "point_spacing": spacing,
                "tilt_deg": rand_range(side_sill.get("tilt_deg", [-18.0, 18.0])),
                "yaw_deg": rand_range(side_sill.get("yaw_deg", [-25.0, 25.0])),
            })

    side_overhang = side.get("overhangs") or {}
    for band_index in range(len(side_bands)):
        for _ in range(int(side_overhang.get("count_per_side", 0))):
            half = rand_range(side_overhang.get("half_length", [1.0, 2.0]))
            spot = place_side(band_index, 0.3)
            if spot is None:
                continue
            x, y = spot
            z_lo = rand_range(side_overhang.get("z_min", [1.1, 1.7]))
            z_lo, z_hi = z_range(z_lo, z_lo + rand_range(side_overhang.get("depth", [0.5, 1.0])))
            boxes.append({
                "center_x": snap(x), "center_y": snap(y),
                "thickness_x": snap(rand_range(side_overhang.get("thickness", [0.3, 0.6]))),
                "half_length_y": snap(half), "z_min": z_lo, "z_max": z_hi,
                "point_spacing": spacing,
                "tilt_deg": rand_range(side_overhang.get("tilt_deg", [-18.0, 18.0])),
                "yaw_deg": rand_range(side_overhang.get("yaw_deg", [-25.0, 25.0])),
            })

    return {"boxes": boxes, "pillars": pillars}


def _random_filler_cylinder(base, rng, jitter, r_lo, r_hi, z_lo, z_hi,
                            full_ratio, tilt_deg, pts):
    """A single cylinder with randomized position/radius/height/tilt.

    Position is jittered by up to `jitter` around the base center; radius is
    uniform in [r_lo, r_hi]; the top is full height with probability
    `full_ratio`, else a random lattice-snapped lower height; tilt (when
    requested) is a random angle with a random azimuth. Deterministic for a
    given seed.
    """
    cx = float(base["center_x"]) + rng.uniform(-jitter, jitter)
    cy = float(base["center_y"]) + rng.uniform(-jitter, jitter)
    r = rng.uniform(r_lo, r_hi)
    if rng.random() < full_ratio:
        top = z_hi
    else:
        top = rng.uniform(z_lo + pts, z_hi)
        top = max(z_lo + pts, round(round(top / pts) * pts, 6))
    spec = {"center_x": cx, "center_y": cy, "radius": r,
            "z_min": z_lo, "z_max": top, "point_spacing": pts}
    lo, hi = (float(tilt_deg[0]), float(tilt_deg[1]))
    if hi > lo:
        t = rng.uniform(lo, hi)
        if t > 0.1:
            spec["tilt_deg"] = t
            spec["tilt_azimuth_deg"] = rng.uniform(0.0, 360.0)
    return spec


def scenario_points(config):
    points = []
    if "wall" in config:
        points.extend(wall_points(config))
    for spec in config.get("boxes") or []:
        points.extend(box_points(spec))
    for spec in config.get("pillars") or []:
        points.extend(cylinder_points(spec))
    if "random" in config:
        expanded = expand_random(config["random"])
        for spec in expanded["boxes"]:
            points.extend(box_points(spec))
        for spec in expanded["pillars"]:
            points.extend(cylinder_points(spec))
    if "ground" in config:
        points.extend(ground_points(config["ground"]))
    if not points:
        raise ValueError(
            "scenario produced no points (need wall/boxes/pillars/random/ground)")
    # Optional whole-scene copies (joystick free-flight playgrounds): each
    # {dx, dy, rot_deg} appends a copy of EVERYTHING above, rotated by
    # rot_deg about the random tile's own center (default (0,0) when the
    # scene has no random block) and then shifted by (dx, dy) -- so the
    # tile's center lands at (tile_c + (dx, dy)). Point-level, so windows,
    # tilts and random content replicate exactly; a rotated tile's bounding
    # box grows with |rot|, and the min_gap corridor guarantee only holds
    # WITHIN one copy, not between copies. Keep the copies inside the SDF
    # map bounds (+-50 m on the 100 m map).
    replicas = config.get("replicas") or []
    if replicas:
        base = list(points)
        tile_c = [0.0, 0.0]
        if "random" in config:
            region = config["random"]["region"]
            tile_c = [0.5 * (region["x_min"] + region["x_max"]),
                      0.5 * (region["y_min"] + region["y_max"])]
        for shift in replicas:
            dx, dy = float(shift["dx"]), float(shift["dy"])
            rot = math.radians(float(shift.get("rot_deg", 0.0)))
            if abs(rot) < 1e-9:
                points.extend((x + dx, y + dy, z) for x, y, z in base)
            else:
                cr, sr = math.cos(rot), math.sin(rot)
                cx, cy = tile_c
                for x, y, z in base:
                    ux, uy = x - cx, y - cy
                    points.append((cr * ux - sr * uy + cx + dx,
                                   sr * ux + cr * uy + cy + dy, z))
    # Absolute filler obstacles (world coordinates), appended AFTER the
    # replicas so they appear exactly once: used to plug the gaps/holes at
    # the edges of a replicated tile block without more point copies. Each
    # spec may carry jitter / radius_min / radius_max / full_height_ratio /
    # tilt_deg to randomize the filler so it does not look like a grid.
    for i, spec in enumerate(config.get("absolute_pillars") or []):
        rng = random.Random(int(spec.get("seed", 7000 + i)))
        jitter = float(spec.get("jitter", 0.0))
        r_lo = float(spec.get("radius_min", spec.get("radius", 0.35)))
        r_hi = float(spec.get("radius_max", spec.get("radius", 0.35)))
        z_lo = float(spec.get("z_min", 0.0))
        z_hi = float(spec.get("z_max", 3.2))
        full = float(spec.get("full_height_ratio", 1.0))
        tilt = spec.get("tilt_deg", [0.0, 0.0])
        pts = float(spec.get("point_spacing", 0.2))
        points.extend(cylinder_points(_random_filler_cylinder(
            spec, rng, jitter, r_lo, r_hi, z_lo, z_hi, full, tilt, pts)))
    for spec in config.get("absolute_boxes") or []:
        points.extend(box_points(spec))
    # absolute_pillar_grid: a rectangular lattice of pillars (world coords)
    # for filling large empty patches cheaply. spacing is the center-to-
    # center gap; the grid runs from x_min/y_min in spacing steps while the
    # coordinate stays within x_max/y_max. Each lattice point is randomized
    # (jitter/radius/height/tilt) so the fill looks organic.
    for g, spec in enumerate(config.get("absolute_pillar_grid") or []):
        spacing = float(spec["spacing"])
        rng = random.Random(int(spec.get("seed", 8000 + g)))
        jitter = float(spec.get("jitter", 0.0))
        r_lo = float(spec.get("radius_min", spec.get("radius", 0.35)))
        r_hi = float(spec.get("radius_max", spec.get("radius", 0.35)))
        z_lo = float(spec.get("z_min", 0.0))
        z_hi = float(spec.get("z_max", 3.2))
        full = float(spec.get("full_height_ratio", 1.0))
        tilt = spec.get("tilt_deg", [0.0, 0.0])
        pts = float(spec.get("point_spacing", 0.2))
        x = float(spec["x_min"])
        while x <= float(spec["x_max"]) + 1e-9:
            y = float(spec["y_min"])
            while y <= float(spec["y_max"]) + 1e-9:
                points.extend(cylinder_points(_random_filler_cylinder(
                    {"center_x": x, "center_y": y}, rng, jitter, r_lo, r_hi,
                    z_lo, z_hi, full, tilt, pts)))
                y += spacing
            x += spacing
    # swap_xy: transpose every point's x and y as a final global step. Lets a
    # scenario be authored in the schema's native orientation (walls/window
    # cuts are thin-in-x/long-in-y, i.e. built for x-direction travel, same
    # as pillar_3d_course.yaml) while actually being laid out along world y
    # -- e.g. to match an arena whose long axis is y in the target world
    # frame. A transpose, not a rotation (det -1): fine for a point cloud,
    # and window/keepout math above already ran in the pre-swap frame, so it
    # composes with every other key unmodified.
    if config.get("swap_xy"):
        points = [(y, x, z) for x, y, z in points]
    return points


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("\n".join(header) + "\n")
        for x, y, z in points:
            stream.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    config = yaml.safe_load(args.scenario.read_text(encoding="utf-8"))
    points = scenario_points(config)
    write_pcd(args.output, points)
    print(f"generated {len(points)} points: {args.output}")


if __name__ == "__main__":
    main()
