#!/usr/bin/env bash
# Launch ProjectAirSim in a window on a bare Xvfb and serve it via xpra shadow.
# Usage: scripts/launch_projectairsim_xpra.sh [--sim-only] [MAP] [DISPLAY] [XPRA_PORT] [TOPICS_PORT] [SERVICES_PORT]
#
#   --sim-only    only (re)start the windowed UE binary on a display whose
#                 Xvfb and xpra shadow are already running; used for map
#                 switches so the console picture never reconnects
#   MAP           cityenviron (default) | neighborhood | blocks
#   DISPLAY       X display number, e.g. :104 (default)
#   XPRA_PORT     HTTP/TCP port for the xpra HTML5 client (default 10010)
#   TOPICS_PORT   ProjectAirSim topic port (default 8991)
#   SERVICES_PORT ProjectAirSim service port (default 8992)
#
# If DISPLAY already shows a simulator window (e.g. the user started it by
# hand), only xpra shadow is started and the running window is reused.
# Otherwise this script starts bare Xvfb, then the windowed UE binary, then
# xpra shadow. Logs and pidfiles live under output/ (override with
# PROJECTAIRSIM_XPRA_RUNTIME).
set -euo pipefail

SIM_ONLY=no
ARGS=()
for arg in "$@"; do
  case "$arg" in
    --sim-only) SIM_ONLY=yes ;;
    *) ARGS+=("$arg") ;;
  esac
done
if (( ${#ARGS[@]} )); then set -- "${ARGS[@]}"; else set --; fi

MAP="${1:-cityenviron}"
DISPLAY_NUM="${2:-:104}"
XPRA_PORT="${3:-10010}"
TOPICS_PORT="${4:-8991}"
SERVICES_PORT="${5:-8992}"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${PROJECTAIRSIM_XPRA_RUNTIME:-$PROJECT_DIR/output/xpra}"
LOG_DIR="${PROJECTAIRSIM_LOG_DIR:-$PROJECT_DIR/output/logs}"
mkdir -p "$RUNTIME/logs" "$RUNTIME/sockets" "$RUNTIME/sessions" "$LOG_DIR"

DISPLAY_NAME="${DISPLAY_NUM#:}"
SCREEN_GEOMETRY="1280x720x24"

case "$MAP" in
  cityenviron)
    SIM_BINARY="$PROJECT_DIR/sim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping"
    SIM_ARG="CityEnviron"
    WINDOW_TITLE="CityEnviron"
    ;;
  neighborhood)
    SIM_BINARY="$PROJECT_DIR/sim/Neighborhood/AirSimNH/Binaries/Linux/AirSimNH-Linux-Shipping"
    SIM_ARG="AirSimNH"
    WINDOW_TITLE="AirSimNH"
    ;;
  blocks)
    SIM_BINARY="$PROJECT_DIR/sim/Blocks-Linux-1.0.1/Blocks/Binaries/Linux/Blocks-Linux-Shipping"
    SIM_ARG="Blocks"
    WINDOW_TITLE="Blocks"
    ;;
  *)
    echo "unknown map '$MAP' (choose cityenviron, neighborhood or blocks)" >&2
    exit 1
    ;;
esac

if [[ ! "${DISPLAY_NUM}" =~ ^:[1-9][0-9]{0,4}$ ]]; then
  echo "display must look like :104" >&2
  exit 1
fi
if [[ ! -x "$SIM_BINARY" ]]; then
  echo "simulator binary not found: $SIM_BINARY" >&2
  exit 1
fi

port_listening() {
  ss -ltn "sport = :$1" 2>/dev/null | grep -q LISTEN
}

x_display_alive() {
  xwininfo -display "$DISPLAY_NUM" -root >/dev/null 2>&1
}

window_present() {
  xwininfo -display "$DISPLAY_NUM" -root -tree 2>/dev/null \
    | grep -q "\"$WINDOW_TITLE"
}

reclaim_stale_display() {
  local lock="/tmp/.X${DISPLAY_NAME}-lock"
  local socket="/tmp/.X11-unix/X${DISPLAY_NAME}"
  [[ -e "$lock" ]] || return 0
  local holder
  holder="$(tr -d '[:space:]' < "$lock" 2>/dev/null || true)"
  if [[ "$holder" =~ ^[0-9]+$ ]] && kill -0 "$holder" 2>/dev/null; then
    return 0  # A live X server still owns this display.
  fi
  if [[ ! -O "$lock" ]]; then
    echo "display $DISPLAY_NUM has a stale lock not owned by this user: $lock" >&2
    exit 1
  fi
  echo "removing stale X lock for display $DISPLAY_NUM (holder '${holder:-?}' is gone)"
  rm -f "$lock" "$socket"
}

wait_for() {
  local description="$1" deadline="$2"
  shift 2
  local start=$SECONDS
  while (( SECONDS - start < deadline )); do
    if "$@"; then
      return 0
    fi
    sleep 0.3
  done
  echo "timed out waiting for $description" >&2
  return 1
}

start_simulator() {
  echo "starting $SIM_ARG on $DISPLAY_NUM (topics $TOPICS_PORT, services $SERVICES_PORT)"
  (
    cd "$PROJECT_DIR/sim"
    setsid nohup env DISPLAY="$DISPLAY_NUM" "$SIM_BINARY" "$SIM_ARG" \
      -windowed -ResX=1280 -ResY=720 -nosound -log \
      -topicsport="$TOPICS_PORT" -servicesport="$SERVICES_PORT" \
      >>"$LOG_DIR/${MAP}_${TOPICS_PORT}.log" 2>&1 </dev/null &
    echo $! >"$RUNTIME/sim${DISPLAY_NAME}.pid"
  )
  if ! wait_for "'$WINDOW_TITLE' window" 120 window_present; then
    echo "see $LOG_DIR/${MAP}_${TOPICS_PORT}.log" >&2
    exit 1
  fi
}

if [[ "$SIM_ONLY" == yes ]]; then
  echo "restarting only the simulator on $DISPLAY_NUM; keeping Xvfb and xpra"
  if ! x_display_alive; then
    echo "display $DISPLAY_NUM is not up; a full launch is required" >&2
    exit 1
  fi
  if ! port_listening "$XPRA_PORT"; then
    echo "xpra port $XPRA_PORT is not listening; a full launch is required" >&2
    exit 1
  fi
  if window_present; then
    echo "a '$WINDOW_TITLE' window is already on $DISPLAY_NUM; stop it first" >&2
    exit 1
  fi
  for port in "$TOPICS_PORT" "$SERVICES_PORT"; do
    if port_listening "$port"; then
      echo "TCP port $port is already in use; stop the old simulator first" >&2
      exit 1
    fi
  done
  start_simulator
  echo "$MAP" >"$RUNTIME/current_map"
  echo
  echo "ProjectAirSim $SIM_ARG ready on display $DISPLAY_NUM (xpra kept)"
  echo "  xpra view: http://<host>:${XPRA_PORT}/   (console stays connected)"
  echo "  ports:     topics $TOPICS_PORT, services $SERVICES_PORT"
  echo "  logs:      $LOG_DIR and $RUNTIME/logs"
  exit 0
fi

if port_listening "$XPRA_PORT"; then
  echo "TCP port $XPRA_PORT is already in use; stop the old xpra session first" >&2
  exit 1
fi

ATTACH=no
if x_display_alive; then
  if window_present; then
    ATTACH=yes
  else
    echo "display $DISPLAY_NUM is up but no '$WINDOW_TITLE' window was found" >&2
    echo "start the simulator there yourself, or pass a free display" >&2
    exit 1
  fi
else
  reclaim_stale_display
  for port in "$TOPICS_PORT" "$SERVICES_PORT"; do
    if port_listening "$port"; then
      echo "TCP port $port is already in use but display $DISPLAY_NUM is down" >&2
      exit 1
    fi
  done
fi

if [[ "$ATTACH" == no ]]; then
  echo "starting Xvfb $DISPLAY_NUM ($SCREEN_GEOMETRY)"
  setsid nohup Xvfb "$DISPLAY_NUM" \
    -screen 0 "$SCREEN_GEOMETRY" \
    +extension GLX +extension RANDR +extension RENDER +extension Composite \
    -nolisten tcp -noreset \
    >>"$LOG_DIR/xvfb${DISPLAY_NAME}.log" 2>&1 </dev/null &
  XVFB_PID=$!
  if ! wait_for "display $DISPLAY_NUM" 20 x_display_alive; then
    echo "see $LOG_DIR/xvfb${DISPLAY_NAME}.log" >&2
    exit 1
  fi
  echo "$XVFB_PID" >"$RUNTIME/xvfb${DISPLAY_NAME}.pid"

  start_simulator
else
  echo "reusing the '$WINDOW_TITLE' window already running on $DISPLAY_NUM"
  if ! port_listening "$TOPICS_PORT" || ! port_listening "$SERVICES_PORT"; then
    echo "warning: ports $TOPICS_PORT/$SERVICES_PORT are not listening; check that" >&2
    echo "         config/base.yaml points at the running simulator" >&2
  fi
fi

echo "starting xpra shadow on $DISPLAY_NUM (TCP $XPRA_PORT)"
# Record which map the running window actually shows. The planner reads this
# marker on startup to pick the matching scene JSONC; without it a manual
# launch leaves the previous map's marker in place and the planner loads the
# wrong spawn coordinates into this binary.
echo "$MAP" >"$RUNTIME/current_map"
setsid nohup /usr/bin/xpra shadow "${DISPLAY_NUM},multi-window=no" \
  --backend=x11 \
  --daemon=no \
  --sharing=yes \
  --bind-tcp="0.0.0.0:${XPRA_PORT}" \
  --html=on \
  --dbus=no \
  --dbus-launch= \
  --pulseaudio=no \
  --notifications=no \
  --mdns=no \
  --printing=no \
  --file-transfer=no \
  --log-dir="$RUNTIME/logs" \
  --pidfile="$RUNTIME/xpra${DISPLAY_NAME}.pid" \
  --socket-dirs="$RUNTIME/sockets" \
  --socket-dir="$RUNTIME/sockets" \
  --sessions-dir="$RUNTIME/sessions" \
  >>"$RUNTIME/logs/shadow${DISPLAY_NAME}.log" 2>&1 </dev/null &
echo $! >"$RUNTIME/shadow${DISPLAY_NAME}.pid"

if ! wait_for "xpra port $XPRA_PORT" 20 port_listening "$XPRA_PORT"; then
  echo "see $RUNTIME/logs/shadow${DISPLAY_NAME}.log" >&2
  exit 1
fi

echo
echo "ProjectAirSim $SIM_ARG ready on display $DISPLAY_NUM"
echo "  xpra view: http://<host>:${XPRA_PORT}/   (console embeds this URL)"
echo "  ports:     topics $TOPICS_PORT, services $SERVICES_PORT"
echo "  logs:      $LOG_DIR and $RUNTIME/logs"
echo "  stop:      scripts/stop_projectairsim_xpra.sh $DISPLAY_NUM"