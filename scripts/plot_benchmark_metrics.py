#!/usr/bin/env python3
"""Compute benchmark metrics and render a summary figure for a baseline run.

Reads the CSVs recorded by record_benchmark_trajectory.py plus the GVF node log,
then writes three artifacts into the run directory:
  - metrics.json        machine-readable metrics
  - summary.txt         one-page human-readable summary (also echoed by the runner)
  - benchmark_plot.png  multi-panel figure

The "planned" stream is the /position_cmd message: in human-input mode its position
just echoes odometry, but its *velocity* is the commanded v_cmd -- the quantity we
tune against (speed cap, lateral oscillation), so it is treated as the command signal.
"""

import argparse
import csv
import json
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover - yaml ships with the ROS python env
    yaml = None


COLORS = {
    "actual": "#176B87",
    "command": "#7A5195",
    "intent": "#EF8354",
    "accent": "#2E7D32",
    "warn": "#C62828",
    "dark": "#263238",
    "muted": "#607D8B",
    "grid": "#DCE4E8",
    "panel": "#FFFFFF",
    "background": "#F4F7F9",
}

# Thresholds for the derived metrics.
MOVING_SPEED = 0.05       # m/s below this the platform is considered stationary
INTENT_ACTIVE = 0.05      # m/s intent magnitude above this counts as an active command
SPEED_CAP_EPS = 0.05      # m/s tolerance before a command counts as over the ceiling
HEADING_SPEED = 0.20      # m/s gate for a well-defined heading (avoids near-zero noise)
MAX_STEP_DT = 0.10        # s ignore heading-rate over gaps longer than this


# --------------------------------------------------------------------------- IO


def find_file(run_dir, name, subdirs=("", "data", "config", "raw")):
    for sub in subdirs:
        candidate = run_dir / sub / name if sub else run_dir / name
        if candidate.is_file():
            return candidate
    return run_dir / name


def load_csv(path, fields):
    rows = []
    try:
        stream = open(path, newline="", encoding="utf-8")
    except OSError:
        return np.empty((0, len(fields)))
    with stream:
        for row in csv.DictReader(stream):
            try:
                rows.append([float(row[field]) for field in fields])
            except (KeyError, TypeError, ValueError):
                continue
    return np.asarray(rows, dtype=float) if rows else np.empty((0, len(fields)))


def read_metadata(run_dir):
    meta = {}
    path = run_dir / "metadata.txt"
    if path.is_file():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if "=" in line:
                key, _, value = line.partition("=")
                meta[key.strip()] = value.strip()
    return meta


def read_intent_ceiling(run_dir, intent):
    """Prefer the configured human_intent_max_speed; fall back to observed intent."""
    params_path = find_file(run_dir, "rosparams.yaml", subdirs=("config", "", "raw"))
    if yaml is not None and params_path.is_file():
        try:
            data = yaml.safe_load(params_path.read_text(encoding="utf-8")) or {}
            gvf = (data.get("gvf") or {}) if isinstance(data, dict) else {}
            value = gvf.get("human_intent_max_speed")
            if value is not None and math.isfinite(float(value)):
                return float(value)
        except (yaml.YAMLError, ValueError, TypeError):
            pass
    if len(intent):
        observed = float(np.max(intent[:, 4]))
        if observed > 0:
            return observed
    return float("nan")


def count_log_matches(path, pattern):
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return 0
    return len(re.findall(pattern, text))


def extract_log_floats(path, pattern):
    """Return every capture-group-1 match of `pattern` in the log, parsed as float."""
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return np.array([])
    return np.array([float(v) for v in re.findall(pattern, text)])


# ---------------------------------------------------------------------- metrics


def relative_time(values):
    if len(values) == 0:
        return values
    return values - values[0]


def path_length(data):
    if len(data) < 2:
        return 0.0
    delta = np.diff(data[:, 1:4], axis=0)
    return float(np.linalg.norm(delta, axis=1).sum())


def _percentile(values, q):
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return float("nan")
    return float(np.percentile(values, q))


def heading_series(vx, vy, speed, min_speed=HEADING_SPEED):
    """Unwrapped heading (rad) of a velocity signal, masked where nearly still."""
    mask = speed > min_speed
    heading = np.full(len(vx), np.nan)
    heading[mask] = np.arctan2(vy[mask], vx[mask])
    finite = np.isfinite(heading)
    if finite.sum() >= 2:
        heading[finite] = np.unwrap(heading[finite])
    return heading, mask


def intent_alignment(actual, intent):
    active = intent[intent[:, 4] > INTENT_ACTIVE]
    if len(active) == 0 or len(actual) == 0:
        return float("nan")
    actual_vx = np.interp(active[:, 0], actual[:, 0], actual[:, 4])
    actual_vy = np.interp(active[:, 0], actual[:, 0], actual[:, 5])
    actual_norm = np.hypot(actual_vx, actual_vy)
    intent_norm = np.hypot(active[:, 1], active[:, 2])
    valid = actual_norm > 0.02
    if not np.any(valid):
        return float("nan")
    cosine = (actual_vx * active[:, 1] + actual_vy * active[:, 2]) / (
        actual_norm * intent_norm + 1e-9
    )
    return float(np.mean(np.clip(cosine[valid], -1.0, 1.0)))


def lateral_oscillation(command, intent):
    """Count command-velocity swings lateral to the intent direction.

    Projects the command velocity onto the frame of the (interpolated) intent
    direction and counts sign changes of the lateral component while an intent is
    active. A clean tracker holds one sign per maneuver; the observed left-right
    sway shows up as many reversals.
    """
    if len(command) < 3 or len(intent) == 0:
        return 0, float("nan")
    t = command[:, 0]
    intent_vx = np.interp(t, intent[:, 0], intent[:, 1])
    intent_vy = np.interp(t, intent[:, 0], intent[:, 2])
    intent_mag = np.hypot(intent_vx, intent_vy)
    cmd_mag = command[:, 7]
    active = (intent_mag > INTENT_ACTIVE) & (cmd_mag > MOVING_SPEED)
    if active.sum() < 3:
        return 0, float("nan")
    ix = intent_vx / (intent_mag + 1e-9)
    iy = intent_vy / (intent_mag + 1e-9)
    # z-component of intent x command = lateral (cross-track) command velocity.
    lateral = ix * command[:, 5] - iy * command[:, 4]
    lateral = lateral[active]
    signs = np.sign(lateral)
    signs = signs[signs != 0]
    reversals = int(np.sum(np.abs(np.diff(signs)) > 0)) if len(signs) > 1 else 0
    lateral_rms = float(np.sqrt(np.mean(lateral ** 2)))
    return reversals, lateral_rms


def heading_rate_rms(command):
    """RMS turn rate of the command velocity heading (deg/s).

    Gated on a well-defined heading (speed > HEADING_SPEED) and on adjacent samples
    (dt <= MAX_STEP_DT) so the statistic reflects sustained sway rather than atan2
    noise across near-zero-speed gaps.
    """
    if len(command) < 3:
        return float("nan")
    t = command[:, 0]
    heading, _ = heading_series(command[:, 4], command[:, 5], command[:, 7])
    dh = np.diff(heading)
    dt = np.diff(t)
    ok = np.isfinite(dh) & (dt > 1e-3) & (dt <= MAX_STEP_DT)
    if ok.sum() < 2:
        return float("nan")
    rate = np.degrees(dh[ok] / dt[ok])
    return float(np.sqrt(np.mean(rate ** 2)))


def lateral_accel_rms(actual):
    """RMS lateral acceleration of the actual trajectory (m/s^2)."""
    if len(actual) < 3:
        return float("nan")
    t = actual[:, 0]
    dt = np.diff(t)
    ok = dt > 1e-3
    if ok.sum() < 2:
        return float("nan")
    ax = np.diff(actual[:, 4])[ok] / dt[ok]
    ay = np.diff(actual[:, 5])[ok] / dt[ok]
    vx = actual[:-1, 4][ok]
    vy = actual[:-1, 5][ok]
    speed = np.hypot(vx, vy)
    moving = speed > MOVING_SPEED
    if moving.sum() < 2:
        return float("nan")
    # lateral accel = component of accel perpendicular to velocity
    lateral = (vx[moving] * ay[moving] - vy[moving] * ax[moving]) / (speed[moving] + 1e-9)
    return float(np.sqrt(np.mean(lateral ** 2)))


def velocity_tracking_rms(actual, command):
    """RMS error between actual velocity and commanded velocity (m/s)."""
    if len(actual) == 0 or len(command) == 0:
        return float("nan")
    t = actual[:, 0]
    cmd_vx = np.interp(t, command[:, 0], command[:, 4])
    cmd_vy = np.interp(t, command[:, 0], command[:, 5])
    err = np.hypot(actual[:, 4] - cmd_vx, actual[:, 5] - cmd_vy)
    return float(np.sqrt(np.mean(err ** 2)))


def stationary_time(actual, intent):
    """Seconds the platform is stalled (barely moving) while intent is active."""
    if len(actual) < 2 or len(intent) == 0:
        return 0.0
    t = actual[:, 0]
    intent_mag = np.interp(t, intent[:, 0], intent[:, 4])
    stalled = (actual[:, 7] < MOVING_SPEED) & (intent_mag > INTENT_ACTIVE)
    dt = np.diff(t)
    return float(np.sum(dt[stalled[:-1]]))


def heading_error_series(command, intent):
    """|cmd - intent| heading error (deg), wrapped to [0, 180], gated on active intent.

    Shared by the metrics (mean/p90) and the heading-error panel so both report the
    same signal -- see the panel's docstring-comment for why this is wrapped rather
    than diffed between two independently np.unwrap-ed traces.
    """
    if len(command) == 0 or len(intent) == 0:
        empty = np.empty(0)
        return empty, empty, empty
    t = command[:, 0]
    ivx = np.interp(t, intent[:, 0], intent[:, 1])
    ivy = np.interp(t, intent[:, 0], intent[:, 2])
    imag = np.hypot(ivx, ivy)
    active = (imag > INTENT_ACTIVE) & (command[:, 7] > HEADING_SPEED)
    ch = np.arctan2(command[:, 5], command[:, 4])
    ih = np.arctan2(ivy, ivx)
    err = np.abs(np.degrees((ch - ih + np.pi) % (2 * np.pi) - np.pi))
    return t, err, active


def over_cap_fraction(command, ceiling):
    if len(command) < 2 or not math.isfinite(ceiling) or ceiling <= 0:
        return float("nan")
    t = command[:, 0]
    dt = np.diff(t)
    over = command[:-1, 7] > (ceiling + SPEED_CAP_EPS)
    total = float(np.sum(dt))
    if total <= 0:
        return float("nan")
    return float(100.0 * np.sum(dt[over]) / total)


def make_metrics(run_dir):
    fields = ["time", "x", "y", "z", "vx", "vy", "vz", "speed"]
    actual = load_csv(find_file(run_dir, "actual_trajectory.csv"), fields)
    command = load_csv(find_file(run_dir, "planned_command.csv"), fields)
    intent = load_csv(
        find_file(run_dir, "human_intent.csv"), ["time", "vx", "vy", "vz", "speed"]
    )

    ceiling = read_intent_ceiling(run_dir, intent)
    reversals, lateral_rms = lateral_oscillation(command, intent)
    heading_rms = heading_rate_rms(command)
    _, heading_err, heading_err_active = heading_error_series(command, intent)
    heading_err_active_vals = heading_err[heading_err_active] if len(heading_err) else heading_err

    straight = 0.0
    if len(actual) > 1:
        straight = float(np.linalg.norm(actual[-1, 1:4] - actual[0, 1:4]))
    length = path_length(actual)

    gvf_log = find_file(run_dir, "gvf.log", subdirs=("raw", ""))
    # Per-tick compute time of the local Darcy fluid solve -- this is the
    # actual hot-path cost in the current lifted-GVF + fluid fusion (see
    # gvf::calcFluidGuidance2D); it replaces the old corridor-planner
    # "plan time" metric, which the human-input benchmark path never
    # exercises (updateIntentState/KinoPathCallback/FSMCallback's replan
    # logging have no live caller / never leave WAIT_TARGET in this mode).
    # Two accepted shapes, because the log line has changed twice as the solve
    # was restructured and old benchmark runs must stay readable:
    #   legacy  "[GVF][FLUID] harmonic solve 7.0 ms (80x67 grid, 21 components)"
    #   nested  "[GVF][FLUID] 2.63 ms | coarse 40x33 ... | fine 80x80 ... | quiver ..."
    # Both report the same quantity: total wall time for one rebuild of the
    # local field, so they go into the same metric.
    fluid_solve_times_ms = extract_log_floats(
        gvf_log,
        r"\[GVF\]\[FLUID\] (?:local|harmonic) solve ([\d.]+) ms "
        r"\(\d+x\d+ grid(?:, \d+ components)?\)",
    )
    if len(fluid_solve_times_ms) == 0:
        fluid_solve_times_ms = extract_log_floats(
            gvf_log, r"\[GVF\]\[FLUID\] ([\d.]+) ms \| coarse ")
    # Per-level times, nested-format runs only (empty list on legacy runs).
    esdf_update_times_ms = extract_log_floats(
        gvf_log, r"\[SDF\]\[ESDF\] update ([\d.]+) ms ")
    esdf_update_totals = extract_log_floats(
        gvf_log, r"\[SDF\]\[ESDF\] update [\d.]+ ms \| updates (\d+) ")
    esdf_skip_totals = extract_log_floats(
        gvf_log, r"\[SDF\]\[ESDF\][^|]*\| updates \d+ \| skipped (\d+) ")
    fluid_coarse_ms = extract_log_floats(
        gvf_log, r"\| coarse \d+x\d+ ([\d.]+) ms ")
    fluid_fine_ms = extract_log_floats(
        gvf_log, r"\| fine \d+x\d+ ([\d.]+) ms ")
    # 3D potential-flow solve ("[GVF][FLUID3D] 9.8 ms | coarse 40x33x8 ...").
    # Tolerant fallback so a reshaped log line still yields the total.
    fluid3d_solve_times_ms = extract_log_floats(
        gvf_log, r"\[GVF\]\[FLUID3D\] ([\d.]+) ms \| coarse ")
    if len(fluid3d_solve_times_ms) == 0:
        fluid3d_solve_times_ms = extract_log_floats(
            gvf_log, r"\[GVF\]\[FLUID3D\] ([\d.]+) ms")

    metrics = {
        # --- geometry ---
        "duration_s": float(actual[-1, 0] - actual[0, 0]) if len(actual) > 1 else 0.0,
        "actual_path_length_m": length,
        "straight_line_distance_m": straight,
        "path_efficiency": float(straight / length) if length > 1e-6 else float("nan"),
        # --- speed ---
        "actual_speed_max_mps": float(np.max(actual[:, 7])) if len(actual) else 0.0,
        "actual_speed_mean_mps": float(np.mean(actual[:, 7])) if len(actual) else 0.0,
        "cmd_speed_max_mps": float(np.max(command[:, 7])) if len(command) else 0.0,
        "cmd_speed_mean_mps": float(np.mean(command[:, 7])) if len(command) else 0.0,
        "cmd_speed_p95_mps": _percentile(command[:, 7], 95) if len(command) else float("nan"),
        "intent_ceiling_mps": ceiling,
        "cmd_over_ceiling_pct": over_cap_fraction(command, ceiling),
        # --- tracking / intent ---
        "velocity_tracking_rms_mps": velocity_tracking_rms(actual, command),
        "intent_alignment": intent_alignment(actual, intent),
        "stationary_while_active_s": stationary_time(actual, intent),
        # --- smoothness / oscillation ---
        "cmd_lateral_reversals": reversals,
        "cmd_lateral_rms_mps": lateral_rms,
        "cmd_heading_rate_rms_dps": heading_rms,
        "actual_lateral_accel_rms_mps2": lateral_accel_rms(actual),
        "heading_error_mean_deg": float(np.mean(heading_err_active_vals))
        if len(heading_err_active_vals) else float("nan"),
        "heading_error_p90_deg": _percentile(heading_err_active_vals, 90)
        if len(heading_err_active_vals) else float("nan"),
        # --- fluid solve activity (from the GVF log; the current live
        #     control path is lifted-GVF + local Darcy fluid fusion, see
        #     gvf::calcFluidGuidance2D / gvf::fuseLiftedGvfFluid) ---
        "fluid_solve_count": int(len(fluid_solve_times_ms)),
        "fluid_solve_time_mean_ms": float(np.mean(fluid_solve_times_ms))
        if len(fluid_solve_times_ms) else float("nan"),
        "fluid_solve_time_p90_ms": _percentile(fluid_solve_times_ms, 90)
        if len(fluid_solve_times_ms) else float("nan"),
        "fluid_coarse_time_mean_ms": float(np.mean(fluid_coarse_ms))
        if len(fluid_coarse_ms) else float("nan"),
        "fluid_fine_time_mean_ms": float(np.mean(fluid_fine_ms))
        if len(fluid_fine_ms) else float("nan"),
        "fluid_solve_time_max_ms": float(np.max(fluid_solve_times_ms))
        if len(fluid_solve_times_ms) else float("nan"),
        "fluid_solve_failed_count": count_log_matches(
            gvf_log, r"\[GVF\]\[FLUID\] sparse Darcy solve failed"
        ),
        "fluid_encounter_count": count_log_matches(
            gvf_log, r"\[GVF\]\[FLUID\] encounter enter side="
        ),
        "fluid_route_blocked_count": count_log_matches(
            gvf_log, r"\[GVF\]\[FLUID\] route blocked: no complete bypass"
        ),
        "fluid_hard_stop_count": count_log_matches(
            gvf_log, r"\[GVF\]\[FLUID(?:3D)?\] hard stop: no ESDF normal"
        ),
        "fluid_nonfinite_count": count_log_matches(
            gvf_log, r"\[GVF\]\[FLUID(?:3D)?\] non-finite guidance"
        ),
        # --- 3D potential-flow solve (gvf::calcFluidGuidance3D; empty/zero on
        #     2D runs, so every key below stays a strict metrics superset) ---
        "fluid3d_solve_count": int(len(fluid3d_solve_times_ms)),
        "fluid3d_solve_time_mean_ms": float(np.mean(fluid3d_solve_times_ms))
        if len(fluid3d_solve_times_ms) else float("nan"),
        "fluid3d_solve_time_p90_ms": _percentile(fluid3d_solve_times_ms, 90)
        if len(fluid3d_solve_times_ms) else float("nan"),
        "fluid3d_solve_time_max_ms": float(np.max(fluid3d_solve_times_ms))
        if len(fluid3d_solve_times_ms) else float("nan"),
        "fluid3d_crossflow_latch_count": count_log_matches(
            gvf_log, r"\[GVF\]\[FLUID3D\] crossflow latch ON"
        ),
        # --- altitude (recorded since the first benchmarks; consumed here
        #     once vertical avoidance became a behavior under test) ---
        "actual_z_start_m": float(actual[0, 3]) if len(actual) else float("nan"),
        "actual_z_final_m": float(actual[-1, 3]) if len(actual) else float("nan"),
        "actual_z_max_m": float(np.max(actual[:, 3])) if len(actual) else float("nan"),
        "actual_z_min_m": float(np.min(actual[:, 3])) if len(actual) else float("nan"),
        "actual_vz_max_abs_mps": float(np.max(np.abs(actual[:, 6])))
        if len(actual) else float("nan"),
        "cmd_vz_max_abs_mps": float(np.max(np.abs(command[:, 6])))
        if len(command) else float("nan"),
        "climb_event": bool(len(actual)
                            and float(np.max(actual[:, 3]) - actual[0, 3]) > 0.3),
        # --- SDFMap 3D ESDF maintenance (collision-checking map; runs on its
        #     own 10 Hz timer, so it never shows up in the fluid solve stats).
        #     "update" times are 1 Hz throttled samples; updates/skipped are
        #     cumulative counters, so the last logged value is the run total.
        "esdf_update_time_mean_ms": float(np.mean(esdf_update_times_ms))
        if len(esdf_update_times_ms) else float("nan"),
        "esdf_update_time_p90_ms": _percentile(esdf_update_times_ms, 90)
        if len(esdf_update_times_ms) else float("nan"),
        "esdf_update_time_max_ms": float(np.max(esdf_update_times_ms))
        if len(esdf_update_times_ms) else float("nan"),
        "esdf_update_count": int(esdf_update_totals[-1])
        if len(esdf_update_totals) else 0,
        "esdf_skip_count": int(esdf_skip_totals[-1])
        if len(esdf_skip_totals) else 0,
    }
    metrics = {
        key: (None if isinstance(value, float) and not math.isfinite(value) else value)
        for key, value in metrics.items()
    }
    return actual, command, intent, ceiling, metrics


# ------------------------------------------------------------------------- plot


def legend_if_labeled(axis, **kwargs):
    """Add a legend only when the axis actually has labeled artists."""
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(**kwargs)


def _find_nested_key(node, key):
    """First finite numeric value under `key` anywhere in a YAML tree."""
    if isinstance(node, dict):
        value = node.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            return float(value)
        for child in node.values():
            found = _find_nested_key(child, key)
            if found is not None:
                return found
    elif isinstance(node, list):
        for child in node:
            found = _find_nested_key(child, key)
            if found is not None:
                return found
    return None


def scenario_z_annotations(run_dir):
    """Obstacle z bands + virtual-ceiling height from the run's config/ snapshot.

    The 3D-wall runners copy their scenario YAML into config/, and the harness
    snapshots the resolved rosparams there too; both are optional, so this
    degrades to no annotations on runs that lack them.
    """
    bands = []
    ceil_z = None
    config_dir = run_dir / "config"
    if yaml is None or not config_dir.is_dir():
        return bands, ceil_z
    for path in sorted(config_dir.glob("*.yaml")):
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(data, dict):
            continue
        wall = data.get("wall")
        if isinstance(wall, dict) and "z_min" in wall and "z_max" in wall:
            bands.append((float(wall["z_min"]), float(wall["z_max"])))
        for box in data.get("boxes") or []:
            if isinstance(box, dict) and "z_min" in box and "z_max" in box:
                bands.append((float(box["z_min"]), float(box["z_max"])))
        if ceil_z is None:
            ceil_z = _find_nested_key(data, "virtual_ceil_height")
    return bands, ceil_z


def setup_axis(axis):
    axis.set_facecolor(COLORS["panel"])
    axis.grid(True, color=COLORS["grid"], linewidth=0.8)
    axis.set_axisbelow(True)
    for spine in ("top", "right"):
        axis.spines[spine].set_visible(False)
    axis.spines["left"].set_color(COLORS["grid"])
    axis.spines["bottom"].set_color(COLORS["grid"])


def _fmt(value, unit="", nd=2):
    if value is None:
        return "n/a"
    if isinstance(value, int):
        return f"{value}{unit}"
    return f"{value:.{nd}f}{unit}"


def plot_run(run_dir, actual, command, intent, ceiling, metrics):
    plt.rcParams.update({"font.size": 10, "axes.titleweight": "bold"})
    # A third, full-width altitude row appears only when the run actually
    # moved vertically (or ran the 3D solver); pure-2D runs keep the exact
    # historical two-row figure.
    z_span = float(np.max(actual[:, 3]) - np.min(actual[:, 3])) if len(actual) else 0.0
    show_altitude = z_span > 0.25 or (metrics.get("fluid3d_solve_count") or 0) > 0
    if show_altitude:
        fig = plt.figure(figsize=(17, 12.5), facecolor=COLORS["background"])
        grid = fig.add_gridspec(
            3, 3, left=0.05, right=0.985, top=0.90, bottom=0.05,
            hspace=0.40, wspace=0.24, height_ratios=[1.0, 1.0, 0.62]
        )
    else:
        fig = plt.figure(figsize=(17, 10), facecolor=COLORS["background"])
        grid = fig.add_gridspec(
            2, 3, left=0.05, right=0.985, top=0.88, bottom=0.07, hspace=0.32, wspace=0.24
        )
    ax_xy = fig.add_subplot(grid[0, 0])
    ax_speed = fig.add_subplot(grid[0, 1])
    ax_heading = fig.add_subplot(grid[0, 2])
    ax_vel = fig.add_subplot(grid[1, 0])
    ax_osc = fig.add_subplot(grid[1, 1])
    ax_summary = fig.add_subplot(grid[1, 2])
    ax_alt = fig.add_subplot(grid[2, :]) if show_altitude else None

    for axis in (ax_xy, ax_speed, ax_heading, ax_vel, ax_osc):
        setup_axis(axis)

    # (0,0) Trajectory ------------------------------------------------------
    if len(actual):
        ax_xy.plot(actual[:, 1], actual[:, 2], color=COLORS["actual"], linewidth=2.3, label="actual")
        ax_xy.scatter(actual[0, 1], actual[0, 2], s=70, color=COLORS["accent"],
                      edgecolor="white", linewidth=1.2, zorder=5, label="start")
        ax_xy.scatter(actual[-1, 1], actual[-1, 2], s=70, color=COLORS["warn"],
                      edgecolor="white", linewidth=1.2, zorder=5, label="end")
    ax_xy.set_title("Trajectory (XY)", loc="left")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_aspect("equal", adjustable="datalim")
    legend_if_labeled(ax_xy, frameon=True, facecolor="white", edgecolor="none", ncol=3, loc="best", fontsize=8)

    # (0,1) Speed profile ---------------------------------------------------
    if len(actual):
        ax_speed.plot(relative_time(actual[:, 0]), actual[:, 7], color=COLORS["actual"],
                      linewidth=2.0, label="actual")
    if len(command):
        ax_speed.plot(relative_time(command[:, 0]), command[:, 7], color=COLORS["command"],
                      linewidth=1.4, alpha=0.85, label="commanded |v_cmd|")
    if len(intent):
        ax_speed.plot(relative_time(intent[:, 0]), intent[:, 4], color=COLORS["intent"],
                      linewidth=1.6, linestyle="--", label="human intent")
    if math.isfinite(ceiling):
        ax_speed.axhline(ceiling, color=COLORS["warn"], linewidth=1.1, linestyle=":",
                         label=f"intent ceiling {ceiling:.2f}")
    ax_speed.set_title("Speed profile", loc="left")
    ax_speed.set_xlabel("time [s]")
    ax_speed.set_ylabel("speed [m/s]")
    legend_if_labeled(ax_speed, frameon=True, facecolor="white", edgecolor="none", loc="best", fontsize=8)

    # (0,2) Heading error vs intent, wrapped onto the circle --------------------
    # Both headings live on a circle, so plotting them as two independently np.unwrap-ed
    # lines drifts them apart by multiples of 360 deg -- a pure artifact. The honest signal
    # is the absolute angular difference |cmd - intent| wrapped to [0, 180]: near 0 when the
    # command tracks the human, spiking only during avoidance maneuvers.
    if len(command) and len(intent):
        t, err, active = heading_error_series(command, intent)
        err_plot = np.where(active, err, np.nan)
        rel = relative_time(t)
        ax_heading.fill_between(rel, 0, err_plot, color=COLORS["command"], alpha=0.15)
        ax_heading.plot(rel, err_plot, color=COLORS["command"], linewidth=1.3, label="|cmd − intent|")
        if np.any(active):
            mean_err = float(np.mean(err[active]))
            ax_heading.axhline(mean_err, color=COLORS["muted"], linewidth=1.0, linestyle=":",
                               label=f"mean {mean_err:.0f}°")
    ax_heading.set_ylim(0, 180)
    ax_heading.set_title("Heading error vs intent (wrapped)", loc="left")
    ax_heading.set_xlabel("time [s]")
    ax_heading.set_ylabel("|Δ heading| [deg]")
    legend_if_labeled(ax_heading, frameon=True, facecolor="white", edgecolor="none", loc="best", fontsize=8)

    # (1,0) Velocity components --------------------------------------------
    if len(actual):
        ax_vel.plot(relative_time(actual[:, 0]), actual[:, 4], color=COLORS["actual"],
                    linewidth=1.7, label="actual vx")
        ax_vel.plot(relative_time(actual[:, 0]), actual[:, 5], color=COLORS["actual"],
                    linewidth=1.7, alpha=0.5, label="actual vy")
    if len(command):
        ax_vel.plot(relative_time(command[:, 0]), command[:, 4], color=COLORS["command"],
                    linewidth=1.2, alpha=0.8, label="cmd vx")
        ax_vel.plot(relative_time(command[:, 0]), command[:, 5], color=COLORS["command"],
                    linewidth=1.2, alpha=0.45, label="cmd vy")
    if len(intent):
        ax_vel.step(relative_time(intent[:, 0]), intent[:, 1], color=COLORS["intent"],
                    linewidth=1.2, linestyle="--", where="post", label="intent vx")
        ax_vel.step(relative_time(intent[:, 0]), intent[:, 2], color=COLORS["intent"],
                    linewidth=1.2, alpha=0.6, linestyle=":", where="post", label="intent vy")
    ax_vel.axhline(0.0, color=COLORS["muted"], linewidth=0.8)
    ax_vel.set_title("Velocity components", loc="left")
    ax_vel.set_xlabel("time [s]")
    ax_vel.set_ylabel("velocity [m/s]")
    legend_if_labeled(ax_vel, frameon=True, facecolor="white", edgecolor="none", ncol=3, loc="best", fontsize=7)

    # (1,1) Command lateral (cross-track) velocity -- the oscillation signal -
    if len(command) and len(intent):
        t = command[:, 0]
        ivx = np.interp(t, intent[:, 0], intent[:, 1])
        ivy = np.interp(t, intent[:, 0], intent[:, 2])
        imag = np.hypot(ivx, ivy)
        active = imag > INTENT_ACTIVE
        ix = np.where(active, ivx / (imag + 1e-9), np.nan)
        iy = np.where(active, ivy / (imag + 1e-9), np.nan)
        forward = ix * command[:, 4] + iy * command[:, 5]
        lateral = ix * command[:, 5] - iy * command[:, 4]
        rel = relative_time(t)
        ax_osc.plot(rel, forward, color=COLORS["accent"], linewidth=1.4, label="along intent")
        ax_osc.plot(rel, lateral, color=COLORS["warn"], linewidth=1.4, label="lateral (sway)")
        ax_osc.fill_between(rel, 0, lateral, color=COLORS["warn"], alpha=0.15)
    ax_osc.axhline(0.0, color=COLORS["muted"], linewidth=0.8)
    ax_osc.set_title("Command velocity vs intent frame", loc="left")
    ax_osc.set_xlabel("time [s]")
    ax_osc.set_ylabel("velocity [m/s]")
    legend_if_labeled(ax_osc, frameon=True, facecolor="white", edgecolor="none", loc="best", fontsize=8)

    # (1,2) Metrics panel ---------------------------------------------------
    ax_summary.set_facecolor(COLORS["panel"])
    ax_summary.axis("off")
    ax_summary.text(0.0, 1.0, "Metrics", fontsize=13, fontweight="bold",
                    color=COLORS["dark"], transform=ax_summary.transAxes, va="top")

    groups = [
        ("motion", [
            ("Duration", _fmt(metrics["duration_s"], " s", 1)),
            ("Path length", _fmt(metrics["actual_path_length_m"], " m")),
            ("Path efficiency", _fmt(metrics["path_efficiency"])),
            ("Stalled (intent on)", _fmt(metrics["stationary_while_active_s"], " s", 1)),
        ]),
        ("speed", [
            ("Actual max / mean", f"{_fmt(metrics['actual_speed_max_mps'])} / {_fmt(metrics['actual_speed_mean_mps'])}"),
            ("Cmd max / p95", f"{_fmt(metrics['cmd_speed_max_mps'])} / {_fmt(metrics['cmd_speed_p95_mps'])}"),
            ("Cmd over ceiling", _fmt(metrics["cmd_over_ceiling_pct"], " %", 1)),
            ("Intent alignment", _fmt(metrics["intent_alignment"])),
        ]),
        ("smoothness", [
            ("Lateral reversals", _fmt(metrics["cmd_lateral_reversals"])),
            ("Heading rate rms", _fmt(metrics["cmd_heading_rate_rms_dps"], " °/s", 0)),
            ("Heading err mean/p90", f"{_fmt(metrics['heading_error_mean_deg'], '°', 0)} / {_fmt(metrics['heading_error_p90_deg'], '°', 0)}"),
            ("Lateral accel rms", _fmt(metrics["actual_lateral_accel_rms_mps2"], " m/s²")),
            ("Vel track rms", _fmt(metrics["velocity_tracking_rms_mps"], " m/s")),
        ]),
        ("fluid solve", [
            ("Solve time mean/p90/max", f"{_fmt(metrics['fluid_solve_time_mean_ms'], ' ms', 1)} / {_fmt(metrics['fluid_solve_time_p90_ms'], ' ms', 1)} / {_fmt(metrics['fluid_solve_time_max_ms'], ' ms', 1)}"),
            ("Solves (n) / failed", f"{_fmt(metrics['fluid_solve_count'])} / {_fmt(metrics['fluid_solve_failed_count'])}"),
            ("Encounters / blocked", f"{_fmt(metrics['fluid_encounter_count'])} / {_fmt(metrics['fluid_route_blocked_count'])}"),
            ("Hard stops / non-finite", f"{_fmt(metrics['fluid_hard_stop_count'])} / {_fmt(metrics['fluid_nonfinite_count'])}"),
        ]),
    ]

    y = 0.93
    for title, rows in groups:
        ax_summary.text(0.0, y, title.upper(), fontsize=8.5, fontweight="bold",
                        color=COLORS["muted"], transform=ax_summary.transAxes, va="top")
        y -= 0.045
        for label, value in rows:
            ax_summary.text(0.02, y, label, fontsize=9, color=COLORS["muted"],
                            transform=ax_summary.transAxes, va="top")
            ax_summary.text(1.0, y, value, fontsize=9.5, color=COLORS["dark"],
                            fontweight="bold", ha="right", transform=ax_summary.transAxes, va="top")
            y -= 0.045
        y -= 0.02

    # (2,:) Altitude profile (conditional) -----------------------------------
    if ax_alt is not None:
        setup_axis(ax_alt)
        if len(actual):
            ax_alt.plot(relative_time(actual[:, 0]), actual[:, 3],
                        color=COLORS["actual"], linewidth=2.2, label="actual z")
            ax_alt.axhline(actual[0, 3], color=COLORS["muted"], linewidth=1.0,
                           linestyle=":", label=f"start z {actual[0, 3]:.2f}")
        bands, ceil_z = scenario_z_annotations(run_dir)
        for index, (lo, hi) in enumerate(bands):
            ax_alt.axhspan(lo, hi, color=COLORS["warn"], alpha=0.10,
                           label="obstacle z band" if index == 0 else None)
        if ceil_z is not None:
            ax_alt.axhline(ceil_z, color=COLORS["warn"], linewidth=1.2,
                           linestyle="--", label=f"virtual ceiling {ceil_z:.1f}")
        ax_twin = ax_alt.twinx()
        if len(command):
            ax_twin.plot(relative_time(command[:, 0]), command[:, 6],
                         color=COLORS["command"], linewidth=1.1, alpha=0.7,
                         label="cmd vz")
        ax_twin.set_ylabel("vz [m/s]", color=COLORS["command"])
        ax_twin.tick_params(axis="y", labelcolor=COLORS["command"])
        for spine in ("top",):
            ax_twin.spines[spine].set_visible(False)
        ax_alt.set_title("Altitude profile", loc="left")
        ax_alt.set_xlabel("time [s]")
        ax_alt.set_ylabel("z [m]")
        handles_a, labels_a = ax_alt.get_legend_handles_labels()
        handles_t, labels_t = ax_twin.get_legend_handles_labels()
        if handles_a or handles_t:
            ax_alt.legend(handles_a + handles_t, labels_a + labels_t,
                          frameon=True, facecolor="white", edgecolor="none",
                          ncol=5, loc="best", fontsize=8)

    run_id = run_dir.name
    fig.suptitle("GVF-Nav · Human Discrete-Control Benchmark", x=0.05, y=0.955,
                 ha="left", fontsize=20, fontweight="bold", color=COLORS["dark"])
    fig.text(0.05, 0.915,
             f"run {run_id}  ·  actual vs. commanded vs. intent — motion, speed cap, and lateral oscillation",
             color=COLORS["muted"], fontsize=10)
    fig.savefig(run_dir / "benchmark_plot.png", dpi=170, facecolor=fig.get_facecolor())
    plt.close(fig)


# ---------------------------------------------------------------------- summary


def write_summary(run_dir, meta, metrics):
    def line(label, value):
        return f"  {label:<29}{value}"

    ceiling = metrics["intent_ceiling_mps"]
    lines = [
        "GVF-Nav baseline benchmark summary",
        f"  run                       {run_dir.name}",
        f"  git_commit                {meta.get('git_commit', 'n/a')}"
        + ("  (dirty)" if meta.get("git_dirty") == "true" else ""),
        f"  duration                  {_fmt(metrics['duration_s'], ' s', 1)}",
        "",
        "  motion",
        line("path length", _fmt(metrics["actual_path_length_m"], " m")),
        line("path efficiency", _fmt(metrics["path_efficiency"])),
        line("stalled (intent on)", _fmt(metrics["stationary_while_active_s"], " s", 1)),
        "",
        "  speed / command",
        line("actual max / mean", f"{_fmt(metrics['actual_speed_max_mps'])} / {_fmt(metrics['actual_speed_mean_mps'])} m/s"),
        line("cmd max / p95", f"{_fmt(metrics['cmd_speed_max_mps'])} / {_fmt(metrics['cmd_speed_p95_mps'])} m/s"),
        line("intent ceiling", _fmt(ceiling, " m/s")),
        line("cmd over ceiling", _fmt(metrics["cmd_over_ceiling_pct"], " %", 1)),
        line("intent alignment", _fmt(metrics["intent_alignment"])),
        line("vel tracking rms", _fmt(metrics["velocity_tracking_rms_mps"], " m/s")),
        "",
        "  smoothness / oscillation",
        line("lateral reversals", _fmt(metrics["cmd_lateral_reversals"])),
        line("cmd lateral rms", _fmt(metrics["cmd_lateral_rms_mps"], " m/s")),
        line("heading rate rms", _fmt(metrics["cmd_heading_rate_rms_dps"], " deg/s", 0)),
        line("heading error mean / p90", f"{_fmt(metrics['heading_error_mean_deg'], ' deg', 0)} / {_fmt(metrics['heading_error_p90_deg'], ' deg', 0)}"),
        line("lateral accel rms", _fmt(metrics["actual_lateral_accel_rms_mps2"], " m/s^2")),
        "",
        "  fluid solve (local Darcy avoidance)",
        line("solve time mean/p90/max", f"{_fmt(metrics['fluid_solve_time_mean_ms'], ' ms', 2)} / {_fmt(metrics['fluid_solve_time_p90_ms'], ' ms', 2)} / {_fmt(metrics['fluid_solve_time_max_ms'], ' ms', 2)}"),
        line("solves (n) / failed", f"{_fmt(metrics['fluid_solve_count'])} / {_fmt(metrics['fluid_solve_failed_count'])}"),
        line("encounters / blocked", f"{_fmt(metrics['fluid_encounter_count'])} / {_fmt(metrics['fluid_route_blocked_count'])}"),
        line("hard stops / non-finite", f"{_fmt(metrics['fluid_hard_stop_count'])} / {_fmt(metrics['fluid_nonfinite_count'])}"),
        "",
        "  sdf map 3d esdf (collision-checking map)",
        line("update time mean/p90/max", f"{_fmt(metrics['esdf_update_time_mean_ms'], ' ms', 2)} / {_fmt(metrics['esdf_update_time_p90_ms'], ' ms', 2)} / {_fmt(metrics['esdf_update_time_max_ms'], ' ms', 2)}"),
        line("updates / skipped", f"{_fmt(metrics['esdf_update_count'])} / {_fmt(metrics['esdf_skip_count'])}"),
    ]
    # Extra groups only for runs that exercised the vertical axis, so the
    # historical 2D summary layout stays byte-identical.
    z_max = metrics.get("actual_z_max_m")
    z_min = metrics.get("actual_z_min_m")
    z_span = (z_max - z_min) if (z_max is not None and z_min is not None) else 0.0
    if (metrics.get("fluid3d_solve_count") or 0) > 0 or z_span > 0.25:
        lines += [
            "",
            "  fluid solve 3d (potential flow)",
            line("solve time mean/p90/max", f"{_fmt(metrics['fluid3d_solve_time_mean_ms'], ' ms', 2)} / {_fmt(metrics['fluid3d_solve_time_p90_ms'], ' ms', 2)} / {_fmt(metrics['fluid3d_solve_time_max_ms'], ' ms', 2)}"),
            line("solves (n) / crossflow latches", f"{_fmt(metrics['fluid3d_solve_count'])} / {_fmt(metrics['fluid3d_crossflow_latch_count'])}"),
            "",
            "  altitude",
            line("z start / final", f"{_fmt(metrics['actual_z_start_m'], ' m')} / {_fmt(metrics['actual_z_final_m'], ' m')}"),
            line("z max / min", f"{_fmt(metrics['actual_z_max_m'], ' m')} / {_fmt(metrics['actual_z_min_m'], ' m')}"),
            line("vz max |actual| / |cmd|", f"{_fmt(metrics['actual_vz_max_abs_mps'], ' m/s')} / {_fmt(metrics['cmd_vz_max_abs_mps'], ' m/s')}"),
            line("climb event", str(metrics.get("climb_event"))),
        ]
    (run_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True, type=Path)
    args = parser.parse_args()

    actual, command, intent, ceiling, metrics = make_metrics(args.run_dir)
    meta = read_metadata(args.run_dir)

    (args.run_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    write_summary(args.run_dir, meta, metrics)
    plot_run(args.run_dir, actual, command, intent, ceiling, metrics)


if __name__ == "__main__":
    main()
