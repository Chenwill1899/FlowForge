#!/usr/bin/env bash
# A private software-rendered desktop; only the host's loopback VNC port is exposed.
set -eo pipefail
mkdir -p "${HOME}/.vnc"
Xtigervnc "${DISPLAY}" -geometry "${VNC_GEOMETRY:-1600x900}" -depth 24 \
  -localhost=0 -SecurityTypes None -AlwaysShared -rfbport 5901 \
  >"${HOME}/.vnc/Xtigervnc.log" 2>&1 &
vnc_pid=$!
trap 'kill "${vnc_pid}" 2>/dev/null || true' EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
for _ in $(seq 1 50); do
  if xdpyinfo >/dev/null 2>&1; then
    dbus-run-session -- fluxbox >"${HOME}/.vnc/fluxbox.log" 2>&1 &
    echo "FlowForge desktop ready on container port 5901"
    wait "${vnc_pid}"
    exit $?
  fi
  if ! kill -0 "${vnc_pid}" 2>/dev/null; then break; fi
  sleep 0.1
done
cat "${HOME}/.vnc/Xtigervnc.log" >&2
exit 1
