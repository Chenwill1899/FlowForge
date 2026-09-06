#!/usr/bin/env bash
# Paper demo: fly the 3D flow-GVF planner (test_gvf_3d.launch) across a
# left-to-right crossing (as shown in the main/near-top-down RViz window)
# of pillar_forest_mixed -- a variant of the stock pillar forest
# (map_generator/resource/pillar.pcd) that swaps five of its random pillars
# for the five obstacle types from pillar_3d_course.yaml (climb / dive /
# thread-a-window / dodge-and-hop a pillar cluster / pass under a floating
# block) and lets the remaining pillars lean, so one crossing exercises
# every flow-GVF 3D avoidance behavior, not just generic pillar dodging.
# Under a single constant joystick push, no direction changes, so the
# resulting trajectory shows pure flow-GVF obstacle deformation with no
# human-steering artifacts.
# Uses the same run_baseline_benchmark.sh machinery via env overrides, so it
# does not touch the default baseline (2D) run. Generates its map on every
# invocation rather than committing a static pcd.
set -Eeuo pipefail
readonly ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

readonly SCENARIO="${SIM_PAPER_SCENARIO:-${ROOT}/scripts/benchmark_scenarios/pillar_forest_mixed.yaml}"
readonly SCENARIO_ID="$(basename "${SCENARIO%.yaml}")"
readonly ASSET_DIR="${BENCHMARK_GENERATED_ASSET_DIR:-${ROOT}/logs/sim_paper_assets}"
readonly GENERATED_MAP="${ASSET_DIR}/${SCENARIO_ID}.pcd"

mkdir -p "${ASSET_DIR}"
python3 "${ROOT}/scripts/generate_planar_wall_pcd.py" \
  --scenario "${SCENARIO}" \
  --output "${GENERATED_MAP}"

export BENCHMARK_MAP="${BENCHMARK_MAP:-${GENERATED_MAP}}"
export BENCHMARK_GVF_LAUNCH_FILE="test_gvf_3d.launch"
export BENCHMARK_LOG_DIR="${BENCHMARK_LOG_DIR:-${ROOT}/logs/sim_paper}"
# The arena (see diff_drive_v1.yaml's header) spans roughly x in [-7.2, 7.3],
# y in [-13.7, 13.7], with a virtual boundary wall just past those edges --
# pillar_forest_mixed.yaml's region/swap_xy match this exactly. The main
# RViz window renders this map left-right wide (world y, ~27 m) and up-down
# narrow (world x, ~14.5 m), with world +y toward screen-left (see
# pillar_forest_crossing_v1.yaml's header for how that was confirmed). So
# "leftmost" is the +y edge, not the +/-x edge. Start near (x=-1.0, y=13.4)
# -- clear of the nearest obstacle there (see the scenario's spawn keepout)
# -- at cruise height 1.0, matching the fluid_cruise_z override below (see
# pillar_forest_mixed.yaml's header for why this scenario needs 1.0 instead
# of the launch's 2.0 default).
export BENCHMARK_SIM_EXTRA_ARGS="${BENCHMARK_SIM_EXTRA_ARGS:-init_x:=-1.0 init_y:=13.4 init_z:=1.0}"
# max_speed:=3.0 raises the planar intent ceiling so the measured average
# speed lands at ~2.0 m/s on this course (at the old 2.0 ceiling the average
# was ~1.4 m/s; the obstacle D-ramps and altitude homing pull the mean down).
export BENCHMARK_GVF_EXTRA_ARGS="${BENCHMARK_GVF_EXTRA_ARGS:-fluid_cruise_z:=1.0 max_speed:=3.0}"
# pillar_forest_crossing_v1.yaml: a single constant lateral (world -y, i.e.
# left-to-right on screen) stick push held for 20 s then released to zero --
# no forward input and no direction changes. Including the replay hold and
# benchmark startup/shutdown buffers, the recorded run is approximately 25 s.
export BENCHMARK_TRACE_FILE="${BENCHMARK_TRACE_FILE:-${ROOT}/src/Interface/human_input_sim/config/pillar_forest_crossing_v1.yaml}"
# Interactive runs open RViz automatically; set BENCHMARK_ENABLE_RVIZ=false
# for unattended/headless runs.
export BENCHMARK_SIM_ENABLE_RVIZ="${BENCHMARK_ENABLE_RVIZ:-true}"

# Experiment II, on the same mixed pillar course. `noise` is retained as the
# command-line compatibility name, but now applies a deterministic constant
# world-frame force. The force marker is rendered only in the FPV RViz window
# as an amber arrow at the quadrotor.
#
# The default is deliberately disturbance-free.  `gvf-nav.sh sim-paper noise`
# applies two short horizontal force pulses (1.5 N by default): 6--8 s and 14--16 s.  At
# each pulse start the replay node evaluates the generated PCD around the
# current odometry position and points the force into the locally clearest
# horizontal sector.  The force is zero between pulses and after the second.
# A `pulse` branch remains available to the harness for recovery experiments.
#
# A caller can always provide BENCHMARK_PRE_GVF_CMD directly for a custom
# force schedule; that has priority over these reproducible defaults.
readonly DISTURBANCE_MODE="${SIM_PAPER_DISTURBANCE_MODE:-none}"
readonly EXPERIMENT_MODE="${SIM_PAPER_EXPERIMENT_MODE:-online}"
if [[ -n "${SIM_PAPER_FORCE_MAGNITUDE:-}" ]]; then
  readonly FORCE_MAGNITUDE="${SIM_PAPER_FORCE_MAGNITUDE}"
elif [[ "${EXPERIMENT_MODE}" == "fixed" ]]; then
  readonly FORCE_MAGNITUDE="0.4"
else
  readonly FORCE_MAGNITUDE="1.5"
fi
case "${EXPERIMENT_MODE}" in
  online)
    ;;
  fixed)
    # Experiment I: retain the first valid online streamline and remove the
    # altitude-homing correction, leaving the horizontal eq. (41) law as the
    # only lateral feedback term.  The fixed pulse is intentionally a single
    # 0.4 N, one-second event; the remaining record is the recovery window.
    export BENCHMARK_GVF_EXTRA_ARGS="${BENCHMARK_GVF_EXTRA_ARGS} fluid_3d_freeze_streamline:=true fluid_3d_homing_in_track:=false fluid_3d_k_n:=1.0"
    export BENCHMARK_TRACE_FILE="${ROOT}/src/Interface/human_input_sim/config/pillar_forest_fixed_reference_v1.yaml"
    export BENCHMARK_MAP="${ROOT}/logs/sim_paper_assets/fixed_reference_clear_corridor.pcd"
    # Generate the experiment-I map separately: the normal mixed pillar
    # course remains untouched for online `noise` runs.
    python3 "${ROOT}/scripts/generate_planar_wall_pcd.py" \
      --scenario "${ROOT}/scripts/benchmark_scenarios/fixed_reference_clear_corridor.yaml" \
      --output "${BENCHMARK_MAP}"
    ;;
  *)
    echo "unknown SIM_PAPER_EXPERIMENT_MODE: ${EXPERIMENT_MODE} (use online or fixed)" >&2
    exit 2
    ;;
esac
if [[ -z "${BENCHMARK_PRE_GVF_CMD:-}" ]]; then
  case "${DISTURBANCE_MODE}" in
    none)
      ;;
    pulse)
      export BENCHMARK_PRE_GVF_CMD="python3 ${ROOT}/scripts/scenario_replay.py --force-event 7.0:1.5,0,0 --force-event 8.5:0,0,0 --force-event 18.0:-1.5,0,0 --force-event 19.5:0,0,0 --duration 32.0"
      ;;
    noise)
      if [[ "${EXPERIMENT_MODE}" == "fixed" ]]; then
        export BENCHMARK_PRE_GVF_CMD="python3 ${ROOT}/scripts/scenario_replay.py --open-force-event 6.0:${FORCE_MAGNITUDE} --force-event 7.0:0,0,0 --obstacle-pcd ${BENCHMARK_MAP} --open-force-vertical-window 0.65 --duration 32.0"
      else
        export BENCHMARK_PRE_GVF_CMD="python3 ${ROOT}/scripts/scenario_replay.py --open-force-event 6.0:${FORCE_MAGNITUDE} --force-event 8.0:0,0,0 --open-force-event 14.0:${FORCE_MAGNITUDE} --force-event 16.0:0,0,0 --obstacle-pcd ${GENERATED_MAP} --open-force-vertical-window 0.65 --duration 32.0"
      fi
      ;;
    *)
      echo "unknown SIM_PAPER_DISTURBANCE_MODE: ${DISTURBANCE_MODE} (use none, pulse, or noise)" >&2
      exit 2
      ;;
  esac
fi

exec "${ROOT}/scripts/run_baseline_benchmark.sh"
