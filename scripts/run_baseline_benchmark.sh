#!/usr/bin/env bash
set -Eeuo pipefail

readonly ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Trace and base map default to the baseline; override with env vars for dedicated tests
# (e.g. the forced-channel-switch scene) without disturbing the default baseline run.
readonly TRACE_FILE="${BENCHMARK_TRACE_FILE:-${ROOT}/src/Interface/human_input_sim/config/baseline_v1.yaml}"
readonly SIM_MAP="${BENCHMARK_MAP:-}"
readonly SCENARIO_FILE="${BENCHMARK_SCENARIO_FILE:-}"
readonly LOG_ROOT="${BENCHMARK_LOG_DIR:-${ROOT}/logs/baseline_benchmark}"
# Platform side of the loop (odom + cmd_vel/so3_cmd producer) is swappable so the same
# unmodified GVF planner config can be smoke-tested against ground-robot simulators; the
# quadrotor is the default so the existing baseline/forced-switch scripts are unaffected.
readonly SIM_PKG="${BENCHMARK_SIM_PKG:-so3_quadrotor_simulator}"
readonly SIM_LAUNCH_FILE="${BENCHMARK_SIM_LAUNCH_FILE:-simulator.launch}"
readonly SIM_WAIT_NODE="${BENCHMARK_SIM_WAIT_NODE:-/quadrotor_simulator_so3}"
readonly SIM_ENABLE_RVIZ="${BENCHMARK_SIM_ENABLE_RVIZ:-}"
readonly PRE_GVF_CMD="${BENCHMARK_PRE_GVF_CMD:-}"
# Extra roslaunch args appended to the SIMULATOR launch (e.g. a non-default
# start pose for the off-streamline initial-condition test:
# "init_x:=0.0 init_y:=1.5 init_z:=1.0"). Space-separated "name:=value" pairs.
readonly SIM_EXTRA_ARGS="${BENCHMARK_SIM_EXTRA_ARGS:-}"
# GVF planner launch file is swappable so platform-specific parameter sets (e.g. the
# diff-drive car's vehicle-footprint-aware config) can replace the default quadrotor config.
readonly GVF_LAUNCH_FILE="${BENCHMARK_GVF_LAUNCH_FILE:-test_gvf.launch}"
# Extra roslaunch args appended to the GVF launch, so a scenario can override a
# planner arg (e.g. local_update_range_x:=6.0) without editing the launch file.
# Space-separated "name:=value" pairs.
readonly GVF_EXTRA_ARGS="${BENCHMARK_GVF_EXTRA_ARGS:-}"
readonly RUN_ID="$(date +%Y%m%d_%H%M%S)"
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
source /opt/ros/noetic/setup.bash
source "${ROOT}/devel/setup.bash"

export ROS_LOG_DIR="${ROS_LOG_DIR_LOCAL}"
export ROS_HOME="${ROS_HOME_LOCAL}"

if [[ ! -f "${TRACE_FILE}" ]]; then
  echo "baseline trace not found: ${TRACE_FILE}" >&2
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
  echo "trace_file=${TRACE_FILE}"
  [[ -n "${SCENARIO_FILE}" ]] && echo "scenario_file=${SCENARIO_FILE}"
  [[ -n "${SIM_MAP}" ]] && echo "map_file=${SIM_MAP}"
  [[ -n "${PAPER_CASE_ID:-}" ]] && echo "paper_case_id=${PAPER_CASE_ID}"
  [[ -n "${PAPER_REPETITION:-}" ]] && echo "paper_repetition=${PAPER_REPETITION}"
  [[ -n "${PRE_GVF_CMD}" ]] && echo "scenario_replay_command=${PRE_GVF_CMD}"
  echo "ros_master_uri=${ROS_MASTER_URI}"
  if command -v git >/dev/null 2>&1; then
    git -C "${ROOT}" rev-parse HEAD | sed 's/^/git_commit=/'
    git -C "${ROOT}" diff --quiet 2>/dev/null && echo "git_dirty=false" || echo "git_dirty=true"
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
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  local plot_status=0
  trap - EXIT INT TERM
  stop_process "${TRAJECTORY_RECORDER_PID:-}"
  stop_process "${PRE_GVF_PID:-}"
  stop_process "${BAG_PID:-}"
  stop_process "${REPLAY_PID:-}"
  stop_process "${GVF_PID:-}"
  stop_process "${SIM_PID:-}"
  stop_process "${ROSCORE_PID:-}"

  {
    echo "finished_at=$(date --iso-8601=seconds)"
    echo "exit_code=${status}"
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
  exit "${status}"
}
trap cleanup EXIT INT TERM

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
# shellcheck disable=SC2206  # deliberate word-splitting: space-separated name:=value pairs
[[ -n "${SIM_EXTRA_ARGS}" ]] && SIM_LAUNCH_ARGS+=(${SIM_EXTRA_ARGS})
roslaunch "${SIM_PKG}" "${SIM_LAUNCH_FILE}" "${SIM_LAUNCH_ARGS[@]}" > "${RAW_DIR}/simulator.log" 2>&1 &
SIM_PID=$!
wait_for_node "${SIM_WAIT_NODE}"
wait_for_topic /sim/odom

GVF_LAUNCH_ARGS=(enable_joy_node:=false)
# shellcheck disable=SC2206  # deliberate word-splitting: space-separated name:=value pairs
[[ -n "${GVF_EXTRA_ARGS}" ]] && GVF_LAUNCH_ARGS+=(${GVF_EXTRA_ARGS})
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

rosparam dump "${CONFIG_DIR}/rosparams.yaml" /formation_planning || true

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
  /rosout > "${RAW_DIR}/rosbag.log" 2>&1 &
BAG_PID=$!

rosrun human_input_sim joy_trace_replay.py \
  "_trace_file:=${TRACE_FILE}" \
  "__name:=baseline_replay" > "${RAW_DIR}/baseline_replay.log" 2>&1 &
REPLAY_PID=$!

wait "${REPLAY_PID}"
REPLAY_STATUS=$?
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

exit 0
