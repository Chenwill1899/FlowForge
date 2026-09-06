#!/usr/bin/env bash
# One closed-loop benchmark runner: 3D quadrotor or 2D differential drive.
set -Eeuo pipefail
readonly ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<EOF
Usage: $0 [3d|2d]
  3d  Quadrotor, mixed 3D pillar course (default)
  2d  Differential-drive robot, pillar map and constant left-to-right intent

Runs in the project's ROS Noetic Docker container; builds on first use.
Set BENCHMARK_ENABLE_RVIZ=false for headless runs.
3D experiments: SIM_PAPER_DISTURBANCE_MODE=none|pulse|noise,
                SIM_PAPER_EXPERIMENT_MODE=online|fixed.
See README.md for map, trace, container and build overrides.
EOF
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then usage; exit 0; fi
readonly MODE="${1:-3d}"
if [[ $# -gt 1 || ( "${MODE}" != 3d && "${MODE}" != 2d ) ]]; then
  usage >&2
  exit 2
fi
if [[ "${MODE}" == 2d && ( "${SIM_PAPER_DISTURBANCE_MODE:-none}" != none || "${SIM_PAPER_EXPERIMENT_MODE:-online}" != online ) ]]; then
  echo "2d does not support the quadrotor force/fixed-streamline experiments" >&2
  exit 2
fi

if [[ "${FLOWFORGE_CONTAINER:-0}" != 1 ]]; then
  exec "${ROOT}/scripts/ros1_docker.sh" exec "${ROOT}/scripts/run_sim_paper.sh" "${MODE}"
fi
mkdir -p "${ROOT}/logs"
exec 9>"${ROOT}/logs/.benchmark.lock"
if ! flock -n 9; then
  echo "another benchmark/build owns this workspace; wait for it to finish" >&2
  exit 1
fi
if [[ ! -f "${ROOT}/devel/setup.bash" ]]; then
  "${ROOT}/scripts/ros1_docker.sh" compile
fi

export BENCHMARK_LOG_DIR="${BENCHMARK_LOG_DIR:-${ROOT}/logs/sim_paper/${MODE}}"
export BENCHMARK_SIM_ENABLE_RVIZ="${BENCHMARK_SIM_ENABLE_RVIZ:-${BENCHMARK_ENABLE_RVIZ:-true}}"
if [[ "${MODE}" == 2d ]]; then
  export BENCHMARK_SIM_PKG=diff_drive_gvf_sim
  export BENCHMARK_SIM_LAUNCH_FILE=diff_drive_benchmark_sim.launch
  export BENCHMARK_SIM_WAIT_NODE=/diff_drive_sim
  export BENCHMARK_GVF_LAUNCH_FILE=test_gvf_diff_drive.launch
  # Use a clear screen-left spawn in the stock pillar map and the same -Y intent as 3D.
  export BENCHMARK_TRACE_FILE="${BENCHMARK_TRACE_FILE:-${ROOT}/src/Interface/human_input_sim/config/diff_drive_crossing_v1.yaml}"
  export BENCHMARK_MAP="${BENCHMARK_MAP:-${ROOT}/src/Interface/uav_simulator/dynamic_map_generator/resource/pillar.pcd}"
  export BENCHMARK_SIM_EXTRA_ARGS="${BENCHMARK_SIM_EXTRA_ARGS:-init_x:=2.0 init_y:=13.4 init_yaw:=-1.5707963267948966}"
  if [[ -n "${SIM_PAPER_SCENARIO:-}" ]]; then
    asset_dir="${BENCHMARK_GENERATED_ASSET_DIR:-${ROOT}/logs/sim_paper_assets}"
    export BENCHMARK_MAP="${asset_dir}/$(basename "${SIM_PAPER_SCENARIO%.yaml}").pcd"
    python3 "${ROOT}/scripts/generate_planar_wall_pcd.py" --scenario "${SIM_PAPER_SCENARIO}" --output "${BENCHMARK_MAP}"
    export BENCHMARK_SCENARIO_FILE="${SIM_PAPER_SCENARIO}"
  fi
else
  export BENCHMARK_SIM_PKG=so3_quadrotor_simulator
  export BENCHMARK_SIM_LAUNCH_FILE=simulator.launch
  export BENCHMARK_SIM_WAIT_NODE=/quadrotor_simulator_so3
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
  export BENCHMARK_SCENARIO_FILE="${SCENARIO}"
  # Cross the mixed pillar arena at cruise height, world +y toward -y.
  export BENCHMARK_SIM_EXTRA_ARGS="${BENCHMARK_SIM_EXTRA_ARGS:-init_x:=-1.0 init_y:=13.4 init_z:=1.0}"
  export BENCHMARK_GVF_EXTRA_ARGS="${BENCHMARK_GVF_EXTRA_ARGS:-fluid_cruise_z:=1.0 max_speed:=3.0}"
  export BENCHMARK_TRACE_FILE="${BENCHMARK_TRACE_FILE:-${ROOT}/src/Interface/human_input_sim/config/pillar_forest_crossing_v1.yaml}"
  # Keep the established 3D force-pulse and fixed-reference experiments.
  # `noise` is the compatibility name for deterministic horizontal force pulses.
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
      export BENCHMARK_SCENARIO_FILE="${ROOT}/scripts/benchmark_scenarios/fixed_reference_clear_corridor.yaml"
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
    scenario_args=()
    case "${DISTURBANCE_MODE}" in
      none) ;;
      pulse)
        scenario_args=(--force-event 7.0:1.5,0,0 --force-event 8.5:0,0,0
                       --force-event 18.0:-1.5,0,0 --force-event 19.5:0,0,0)
        ;;
      noise)
        if [[ "${EXPERIMENT_MODE}" == fixed ]]; then
          scenario_args=(--open-force-event "6.0:${FORCE_MAGNITUDE}" --force-event 7.0:0,0,0)
        else
          scenario_args=(--open-force-event "6.0:${FORCE_MAGNITUDE}" --force-event 8.0:0,0,0
                         --open-force-event "14.0:${FORCE_MAGNITUDE}" --force-event 16.0:0,0,0)
        fi
        scenario_args+=(--obstacle-pcd "${BENCHMARK_MAP}" --open-force-vertical-window 0.65)
        ;;
      *)
        echo "unknown SIM_PAPER_DISTURBANCE_MODE: ${DISTURBANCE_MODE} (use none, pulse, or noise)" >&2
        exit 2
        ;;
    esac
    if [[ ${#scenario_args[@]} -gt 0 ]]; then
      printf -v BENCHMARK_PRE_GVF_CMD '%q ' python3 "${ROOT}/scripts/scenario_replay.py" "${scenario_args[@]}" --duration 32.0
      export BENCHMARK_PRE_GVF_CMD
    fi
  fi
fi

# Both platforms share the recording, shutdown and reporting lifecycle below.
readonly TRACE_FILE="${BENCHMARK_TRACE_FILE}"
readonly SIM_MAP="${BENCHMARK_MAP}"
readonly SCENARIO_FILE="${BENCHMARK_SCENARIO_FILE:-}"
readonly LOG_ROOT="${BENCHMARK_LOG_DIR}"
readonly SIM_PKG="${BENCHMARK_SIM_PKG}"
readonly SIM_LAUNCH_FILE="${BENCHMARK_SIM_LAUNCH_FILE}"
readonly SIM_WAIT_NODE="${BENCHMARK_SIM_WAIT_NODE}"
readonly SIM_ENABLE_RVIZ="${BENCHMARK_SIM_ENABLE_RVIZ}"
readonly PRE_GVF_CMD="${BENCHMARK_PRE_GVF_CMD:-}"
# Extra launch arguments are space-separated name:=value pairs.
readonly SIM_EXTRA_ARGS="${BENCHMARK_SIM_EXTRA_ARGS}"
readonly GVF_LAUNCH_FILE="${BENCHMARK_GVF_LAUNCH_FILE}"
readonly GVF_EXTRA_ARGS="${BENCHMARK_GVF_EXTRA_ARGS:-}"
readonly RUN_ID="$(date +%Y%m%d_%H%M%S_%N)"
readonly RUN_DIR="${LOG_ROOT}/${RUN_ID}"

# Keep the run directory tidy: process logs, recorded data, and config dumps each
# live in their own subdirectory so the top level only holds the artifacts a human
# actually opens (summary.txt, metrics.json, benchmark_plot.png, benchmark.bag).
readonly RAW_DIR="${RUN_DIR}/raw"          # roscore/sim/gvf/rosbag/replay stdout+stderr
readonly DATA_DIR="${RUN_DIR}/data"        # recorded CSV time series
readonly CONFIG_DIR="${RUN_DIR}/config"    # trace + resolved rosparams
readonly ROS_LOG_DIR_LOCAL="${RAW_DIR}/roslog"
readonly ROS_HOME_LOCAL="${RAW_DIR}/roshome"

mkdir -p "${RAW_DIR}" "${DATA_DIR}" "${CONFIG_DIR}" "${ROS_LOG_DIR_LOCAL}" "${ROS_HOME_LOCAL}"

# Export before sourcing: ROS's own catkin profile.d hooks (e.g. 10.roslaunch.sh) reference
# $ROS_MASTER_URI unguarded, which trips `set -u` when it isn't already in the environment.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
set +u  # ROS setup hooks may read unset variables.
source /opt/ros/noetic/setup.bash
source "${ROOT}/devel/setup.bash"
set -u

export ROS_LOG_DIR="${ROS_LOG_DIR_LOCAL}"
export ROS_HOME="${ROS_HOME_LOCAL}"

if [[ ! -f "${SIM_MAP}" ]]; then
  echo "benchmark map not found: ${SIM_MAP}" >&2
  exit 1
fi
if [[ ! -f "${TRACE_FILE}" ]]; then
  echo "intent trace not found: ${TRACE_FILE}" >&2
  exit 1
fi

cp "${TRACE_FILE}" "${CONFIG_DIR}/$(basename "${TRACE_FILE}")"
if [[ "$(basename "${TRACE_FILE}")" != "baseline_v1.yaml" ]]; then
  # Keep the historical artifact name for downstream readers while retaining
  # the actual trace basename for paper scenario provenance.
  cp "${TRACE_FILE}" "${CONFIG_DIR}/baseline_v1.yaml"
fi
if [[ -n "${SCENARIO_FILE}" && -f "${SCENARIO_FILE}" ]]; then
  cp "${SCENARIO_FILE}" "${CONFIG_DIR}/$(basename "${SCENARIO_FILE}")"
fi
if [[ -n "${SIM_MAP}" && -f "${SIM_MAP}" ]]; then
  cp "${SIM_MAP}" "${CONFIG_DIR}/$(basename "${SIM_MAP}")"
fi
{
  echo "run_id=${RUN_ID}"
  echo "started_at=$(date --iso-8601=seconds)"
  echo "workspace=${ROOT}"
  echo "mode=${MODE}"
  echo "simulator=${SIM_PKG}/${SIM_LAUNCH_FILE}"
  echo "planner=${GVF_LAUNCH_FILE}"
  echo "trace_file=${TRACE_FILE}"
  [[ -n "${SCENARIO_FILE}" ]] && echo "scenario_file=${SCENARIO_FILE}"
  [[ -n "${SIM_MAP}" ]] && echo "map_file=${SIM_MAP}"
  [[ -n "${PAPER_CASE_ID:-}" ]] && echo "paper_case_id=${PAPER_CASE_ID}"
  [[ -n "${PAPER_REPETITION:-}" ]] && echo "paper_repetition=${PAPER_REPETITION}"
  [[ -n "${PRE_GVF_CMD}" ]] && echo "scenario_replay_command=${PRE_GVF_CMD}"
  echo "ros_master_uri=${ROS_MASTER_URI}"
  if command -v git >/dev/null 2>&1; then
    git -C "${ROOT}" rev-parse HEAD | sed 's/^/git_commit=/'
    git -C "${ROOT}" diff --quiet HEAD 2>/dev/null && echo "git_dirty=false" || echo "git_dirty=true"
  else
    echo "git_commit=unavailable"
    echo "git_dirty=unavailable"
  fi
} > "${RUN_DIR}/metadata.txt"

# A benchmark owns its ROS graph. Reusing a master that is still shutting down
# from a previous run can make the node/topic probes succeed against stale
# registrations and produce an empty, falsely successful recording.
if rosnode list >/dev/null 2>&1; then
  echo "another ROS master is already running at ${ROS_MASTER_URI}; wait for it to exit" >&2
  exit 1
fi

roscore > "${RAW_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!

stop_process() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return 0
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 80); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  local plot_status=0
  trap - EXIT INT TERM
  stop_process "${REPLAY_PID:-}"
  stop_process "${TRAJECTORY_RECORDER_PID:-}"
  stop_process "${PRE_GVF_PID:-}"
  stop_process "${BAG_PID:-}"
  stop_process "${GVF_PID:-}"
  stop_process "${SIM_PID:-}"
  stop_process "${ROSCORE_PID:-}"

  {
    echo "finished_at=$(date --iso-8601=seconds)"
  } >> "${RUN_DIR}/metadata.txt"

  if [[ -s "${DATA_DIR}/actual_trajectory.csv" ]]; then
    python3 "${ROOT}/scripts/plot_benchmark_metrics.py" \
      --run-dir "${RUN_DIR}" \
      > "${RAW_DIR}/metrics_plot.log" 2>&1 || plot_status=$?
  else
    echo "actual trajectory is empty; skip metrics plot" > "${RAW_DIR}/metrics_plot.log"
    plot_status=1
  fi
  echo "plot_exit_code=${plot_status}" >> "${RUN_DIR}/metadata.txt"

  # Echo the one-page summary the plotter produced, so the terminal shows the
  # headline numbers without the operator having to open any file.
  echo
  if [[ -s "${RUN_DIR}/summary.txt" ]]; then
    cat "${RUN_DIR}/summary.txt"
  fi
  echo
  echo "benchmark run: ${RUN_DIR}"
  echo "  plot    : ${RUN_DIR}/benchmark_plot.png"
  echo "  metrics : ${RUN_DIR}/metrics.json"
  echo "  raw logs: ${RAW_DIR}"
  if [[ "${status}" -eq 0 && "${plot_status}" -ne 0 ]]; then status="${plot_status}"; fi
  echo "exit_code=${status}" >> "${RUN_DIR}/metadata.txt"
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for_node() {
  local node="$1"
  for _ in $(seq 1 120); do
    if rosnode list 2>/dev/null | grep -Fqx "${node}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "timed out waiting for ROS node: ${node}" >&2
  return 1
}

wait_for_topic() {
  local topic="$1"
  for _ in $(seq 1 120); do
    if rostopic list 2>/dev/null | grep -Fqx "${topic}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "timed out waiting for ROS topic: ${topic}" >&2
  return 1
}

for _ in $(seq 1 120); do
  if rosnode list >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

SIM_LAUNCH_ARGS=()
[[ -n "${SIM_MAP}" ]] && SIM_LAUNCH_ARGS+=("map:=${SIM_MAP}")
[[ -n "${SIM_ENABLE_RVIZ}" ]] && SIM_LAUNCH_ARGS+=("enable_rviz:=${SIM_ENABLE_RVIZ}")
sim_extra=()
[[ -n "${SIM_EXTRA_ARGS}" ]] && read -r -a sim_extra <<< "${SIM_EXTRA_ARGS}"
SIM_LAUNCH_ARGS+=("${sim_extra[@]}")
roslaunch "${SIM_PKG}" "${SIM_LAUNCH_FILE}" "${SIM_LAUNCH_ARGS[@]}" > "${RAW_DIR}/simulator.log" 2>&1 &
SIM_PID=$!
wait_for_node "${SIM_WAIT_NODE}"
wait_for_topic /sim/odom
if [[ "${MODE}" == 2d ]]; then
  wait_for_node /gvf_cmd_bridge
  wait_for_node /robot_state_publisher
  wait_for_topic /cmd_vel
fi

GVF_LAUNCH_ARGS=(enable_joy_node:=false)
gvf_extra=()
[[ -n "${GVF_EXTRA_ARGS}" ]] && read -r -a gvf_extra <<< "${GVF_EXTRA_ARGS}"
GVF_LAUNCH_ARGS+=("${gvf_extra[@]}")
roslaunch fluid "${GVF_LAUNCH_FILE}" "${GVF_LAUNCH_ARGS[@]}" > "${RAW_DIR}/gvf.log" 2>&1 &
GVF_PID=$!
wait_for_node /formation_planning
wait_for_topic /human_intent
wait_for_topic /position_cmd

# Optional paper scenario replay starts after the planner has subscribed to
# the map and just before trajectory recording/replay. Its event timestamps
# therefore have a stable relationship to the benchmark trace.
if [[ -n "${PRE_GVF_CMD}" ]]; then
  bash -lc "${PRE_GVF_CMD}" > "${RAW_DIR}/scenario_replay.log" 2>&1 &
  PRE_GVF_PID=$!
  sleep 1
fi

rosparam dump "${CONFIG_DIR}/rosparams.yaml" /formation_planning
rosparam dump "${CONFIG_DIR}/all_rosparams.yaml"

python3 "${ROOT}/scripts/record_benchmark_trajectory.py" \
  --actual "${DATA_DIR}/actual_trajectory.csv" \
  --planned "${DATA_DIR}/planned_command.csv" \
  --intent "${DATA_DIR}/human_intent.csv" \
  --force "${DATA_DIR}/force_disturbance.csv" \
  --events "${DATA_DIR}/scenario_events.csv" \
  --field "${DATA_DIR}/field_diagnostics.csv" \
  --reanchors "${DATA_DIR}/reanchor_events.csv" \
  > "${RAW_DIR}/trajectory_recorder.log" 2>&1 &
TRAJECTORY_RECORDER_PID=$!
sleep 1

rosbag record --quiet -O "${RUN_DIR}/benchmark.bag" \
  /sim/odom \
  /mock_map \
  /sim/local_map \
  /quadrotor_simulator_so3/force_disturbance \
  /paper/scenario_events \
  /paper/force_disturbance_marker \
  /paper/gvf_field_diagnostics \
  /paper/gvf_reanchor_events \
  /human_intent \
  /position_cmd \
  /cmd_vel \
  /tf \
  /tf_static \
  /rosout > "${RAW_DIR}/rosbag.log" 2>&1 &
BAG_PID=$!

rosrun human_input_sim joy_trace_replay.py \
  "_trace_file:=${TRACE_FILE}" \
  "__name:=baseline_replay" > "${RAW_DIR}/baseline_replay.log" 2>&1 &
REPLAY_PID=$!

REPLAY_STATUS=0
wait "${REPLAY_PID}" || REPLAY_STATUS=$?
if [[ "${REPLAY_STATUS}" -ne 0 ]]; then
  echo "baseline replay failed with exit code ${REPLAY_STATUS}" >&2
  exit "${REPLAY_STATUS}"
fi

sleep 1

for process_spec in \
  "${SIM_PID}:simulator" \
  "${GVF_PID}:formation_planning" \
  "${TRAJECTORY_RECORDER_PID}:trajectory_recorder" \
  "${BAG_PID}:rosbag" \
  "${PRE_GVF_PID:-}:scenario_replay"; do
  process_pid="${process_spec%%:*}"
  process_name="${process_spec#*:}"
  [[ -n "${process_pid}" ]] || continue
  if ! kill -0 "${process_pid}" 2>/dev/null; then
    echo "benchmark process exited before replay completed: ${process_name}" >&2
    exit 1
  fi
done

for node in "${SIM_WAIT_NODE}" /map_pub /formation_planning /map_generator; do
  rosnode ping -c 1 "${node}" > /dev/null 2>&1 || { echo "required node stopped: ${node}" >&2; exit 1; }
done
if [[ "${MODE}" == 2d ]]; then
  rosnode ping -c 1 /gvf_cmd_bridge > /dev/null 2>&1
fi
exit 0
