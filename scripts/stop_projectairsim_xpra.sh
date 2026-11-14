#!/usr/bin/env bash
# Stop the ProjectAirSim + xpra session started by
# scripts/launch_projectairsim_xpra.sh.
# Usage: scripts/stop_projectairsim_xpra.sh [DISPLAY] [--view-only | --sim-only]
#
#   --view-only   only stop xpra shadow; leave the simulator and Xvfb running
#   --sim-only    only stop the simulator; leave Xvfb and xpra running (used
#                 for map switches so the console picture stays connected)
#
# The simulator and Xvfb are stopped only when pidfiles written by the launch
# script still point at processes owned by this project. A simulator that was
# started by hand (attach mode) is never touched.
set -euo pipefail

DISPLAY_NUM="${1:-:104}"
if [[ "${DISPLAY_NUM}" == --* ]]; then
  DISPLAY_NUM=":104"
else
  shift || true
fi
VIEW_ONLY=no
SIM_ONLY=no
for arg in "$@"; do
  case "$arg" in
    --view-only) VIEW_ONLY=yes ;;
    --sim-only) SIM_ONLY=yes ;;
    *)
      echo "unknown argument '$arg'" >&2
      exit 1
      ;;
  esac
done
if [[ "$VIEW_ONLY" == yes && "$SIM_ONLY" == yes ]]; then
  echo "--view-only and --sim-only are mutually exclusive" >&2
  exit 1
fi

if [[ ! "${DISPLAY_NUM}" =~ ^:[1-9][0-9]{0,4}$ ]]; then
  echo "display must look like :104" >&2
  exit 1
fi
DISPLAY_NAME="${DISPLAY_NUM#:}"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${PROJECTAIRSIM_XPRA_RUNTIME:-$PROJECT_DIR/output/xpra}"

# Match /proc/<pid>/cmdline against a marker so a recycled PID is never killed.
owned_process() {
  local pid="$1" marker="$2"
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  tr '\0' ' ' <"/proc/$pid/cmdline" | grep -qF "$marker"
}

stop_owned() {
  local label="$1" pidfile="$2" marker="$3"
  [[ -f "$pidfile" ]] || return 0
  local pid
  pid="$(tr -d '[:space:]' <"$pidfile" 2>/dev/null || true)"
  if [[ ! "$pid" =~ ^[0-9]+$ ]]; then
    echo "ignoring malformed pidfile $pidfile" >&2
    rm -f "$pidfile"
    return 0
  fi
  if ! owned_process "$pid" "$marker"; then
    echo "ignoring stale $label pidfile (PID $pid is gone or reused)"
    rm -f "$pidfile"
    return 0
  fi
  echo "stopping $label (PID $pid)"
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.2
  done
  if kill -0 "$pid" 2>/dev/null; then
    echo "$label did not exit; sending SIGKILL" >&2
    kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$pidfile"
}

if [[ "$SIM_ONLY" == yes ]]; then
  stop_owned "simulator" "$RUNTIME/sim${DISPLAY_NAME}.pid" "$PROJECT_DIR/sim/"
  echo "simulator stopped; display $DISPLAY_NUM and xpra kept running"
  exit 0
fi

endpoint="socket://$RUNTIME/sessions/${DISPLAY_NAME}/socket"
if [[ -S "$RUNTIME/sessions/${DISPLAY_NAME}/socket" ]]; then
  /usr/bin/xpra stop "$endpoint" \
    --socket-dirs="$RUNTIME/sockets" \
    --socket-dir="$RUNTIME/sockets" \
    --sessions-dir="$RUNTIME/sessions" || true
else
  echo "no xpra session for display $DISPLAY_NUM under $RUNTIME" >&2
fi

if [[ "$VIEW_ONLY" == yes ]]; then
  rm -f "$RUNTIME/shadow${DISPLAY_NAME}.pid"
  exit 0
fi

stop_owned "simulator" "$RUNTIME/sim${DISPLAY_NAME}.pid" "$PROJECT_DIR/sim/"
stop_owned "Xvfb" "$RUNTIME/xvfb${DISPLAY_NAME}.pid" "Xvfb $DISPLAY_NUM"
rm -f "$RUNTIME/shadow${DISPLAY_NAME}.pid" "$RUNTIME/xpra${DISPLAY_NAME}.pid"

echo "display $DISPLAY_NUM stopped"
