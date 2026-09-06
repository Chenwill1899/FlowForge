#!/usr/bin/env bash
# Project-local Noetic build/runtime, adapted from ros1_docker/gvf-nav.sh.
set -Eeuo pipefail
readonly ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly IMAGE="${FLOWFORGE_IMAGE:-flowforge:noetic-vnc}"
readonly CONTAINER="${FLOWFORGE_DOCKER_CONTAINER:-flowforge-noetic}"
readonly VNC_PORT="${FLOWFORGE_VNC_PORT:-5902}"
action="${1:-help}"
shift || true

case "${action}" in
  build|start|compile|exec|shell|stop|logs) ;;
  help|-h|--help)
    echo "Usage: $0 {build|start|compile|exec COMMAND...|shell|stop|logs}"
    echo "Benchmark: ./scripts/run_sim_paper.sh [3d|2d]"
    exit 0 ;;
  *) echo "Unknown Docker action: ${action}" >&2; exit 2 ;;
esac

# Called both by the host helper and the benchmark's first-run build.
if [[ "${FLOWFORGE_CONTAINER:-0}" == 1 && "${action}" == compile ]]; then
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
  cd "${ROOT}"
  exec catkin_make -DCMAKE_BUILD_TYPE=Release -j"${FLOWFORGE_BUILD_JOBS:-4}" "$@"
fi

# A login can acquire the docker group before the current desktop session does.
if [[ " $(id -nG) " != *" docker "* && " $(id -nG "$(id -un)") " == *" docker "* ]]; then
  printf -v command '%q ' "$0" "${action}" "$@"
  exec sg docker -c "${command}"
fi
if ! docker info >/dev/null 2>&1; then
  echo "Cannot connect to Docker; check the service and your docker group access." >&2
  exit 1
fi

build_image() {
  docker build --build-arg "USER_UID=$(id -u)" --build-arg "USER_GID=$(id -g)" \
    -t "${IMAGE}" "${ROOT}/docker"
}

start_container() {
  if docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
    mounted_root="$(docker inspect --format '{{.Config.WorkingDir}}' "${CONTAINER}")"
    if [[ "${mounted_root}" != "${ROOT}" ]]; then
      echo "${CONTAINER} belongs to ${mounted_root}; set FLOWFORGE_DOCKER_CONTAINER to a different name." >&2
      exit 1
    fi
    if [[ "$(docker inspect --format '{{.State.Running}}' "${CONTAINER}")" != true ]]; then
      docker start "${CONTAINER}" >/dev/null
    fi
  else
    if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then build_image; fi
    # Private Docker networking isolates this ROS master from old GVF-Nav runs.
    # Mount at the same path so caller-provided map/trace paths remain valid.
    docker run -d --init --name "${CONTAINER}" --shm-size 1g \
      -p "127.0.0.1:${VNC_PORT}:5901" \
      -v "${ROOT}:${ROOT}:rw" -w "${ROOT}" \
      "${IMAGE}" >/dev/null
  fi
  for _ in $(seq 1 50); do
    if docker exec "${CONTAINER}" xdpyinfo >/dev/null 2>&1; then return; fi
    sleep 0.1
  done
  docker logs "${CONTAINER}" >&2
  echo "FlowForge desktop failed to start." >&2
  exit 1
}

case "${action}" in
  build) build_image ;;
  start) start_container; echo "VNC: 127.0.0.1:${VNC_PORT}" ;;
  stop) docker stop "${CONTAINER}" ;;
  logs) docker logs "${CONTAINER}" ;;
  compile|exec|shell)
    if [[ "${action}" == exec && $# -eq 0 ]]; then echo "exec requires a command" >&2; exit 2; fi
    start_container
    forwarded_env=()
    # docker exec does not inherit the host environment. Forward benchmark knobs
    # as individual arguments; never eval caller values or shell-escape them by hand.
    while IFS= read -r name; do
      case "${name}" in
        BENCHMARK_*|SIM_PAPER_*|PAPER_CASE_ID|PAPER_REPETITION|FLOWFORGE_BUILD_JOBS)
          forwarded_env+=(--env "${name}=${!name}") ;;
      esac
    done < <(compgen -e)
    case "${action}" in
      compile) set -- "${ROOT}/scripts/ros1_docker.sh" compile "$@" ;;
      exec) ;;
      shell) exec docker exec -it "${CONTAINER}" bash -c 'source /opt/ros/noetic/setup.bash; [[ ! -f devel/setup.bash ]] || source devel/setup.bash; exec bash -i' ;;
    esac
    # docker exec itself does not proxy host signals to the container command.
    # Keep its client alive until the benchmark has flushed bags and stopped ROS.
    pid_file="/tmp/flowforge-exec-$$-${RANDOM}.pid"
    forward_signal() {
      docker exec "${CONTAINER}" bash -c 'kill -TERM "$(cat "$1")"' _ "${pid_file}" 2>/dev/null || true
    }
    trap forward_signal INT TERM
    (
      trap '' INT TERM
      exec docker exec "${forwarded_env[@]}" "${CONTAINER}" bash -c \
        'pid_file="$1"; shift; echo "$$" > "$pid_file"; exec "$@"' _ "${pid_file}" "$@"
    ) &
    client_pid=$!
    status=0
    wait "${client_pid}" || status=$?
    # A trapped host signal interrupts wait; wait again for container cleanup.
    if kill -0 "${client_pid}" 2>/dev/null; then wait "${client_pid}" || status=$?; fi
    trap - INT TERM
    docker exec "${CONTAINER}" rm -f "${pid_file}" 2>/dev/null || true
    exit "${status}"
    ;;
esac
