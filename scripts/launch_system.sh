#!/usr/bin/env bash
#
# launch_system.sh - one-command launcher for the macro_micro_robot (Macro-Micro Surgical
# Robot) stack and all of its runtime dependencies (Touch driver, Arduino serial
# bridge, UR16e driver, control nodes, RViz).
#
# This replaces manually opening N terminals, sourcing ROS 2 in each one, and
# running the commands from MICRO_MODULE_STARTUP.md / README.md by hand.
#
# Usage:
#   ./launch_system.sh <mode> [options]
#
# Modes:
#   display     RViz + robot/joint state publishers only. No hardware needed.
#   touch       Touch stylus teleop of the micro-manipulator (no UR16e).
#               Touch driver (touch_server + touch_bridge) + serial bridge + micro_module_control.
#   keyboard    Keyboard teleop of the micro-manipulator. No Touch device
#               needed - only the Arduino. Good for bench-testing the servos.
#   ur16e-keyboard  UR16e + MoveIt Servo + trajectory recorder; this terminal
#               then jogs the arm from the keyboard.
#   ur16e-touch UR16e + MoveIt Servo + trajectory recorder + Touch driver;
#               hold the stylus WHITE button to drive the arm. With
#               --with-micro also the micro module: hold GREY 1 s to switch.
#               Type a tool number + Enter in this terminal to switch tools
#               (config/tools.yaml; not in RCM mode, robot stationary).
#   ur16e       Same as ur16e-touch.
#   full        Same as ur16e-touch --with-micro: arm, Touch and micro module.
#               Needs the Arduino, see --arduino-port.
#   stop        Stop every process started by a previous run of this script.
#   help        Show this message.
#
# Options:
#   --robot-ip=IP            Robot IP address (default: 192.168.0.100)
#   --calibration-file=PATH  Robot kinematics calibration YAML, from
#                            ur_calibration_extract.launch.py (default: nominal
#                            UR16e model, NOT this arm's calibration;
#                            see docs/UR_CALIBRATION_FIX.md)
#   --arduino-port=PORT      Serial port for the micro-module Arduino
#                            (default: /dev/ttyACM0)
#   --baud=RATE              Serial baud rate for the Arduino (default: 9600)
#   --mock                   Simulate the UR16e (no robot needed) in the
#                            'ur16e-keyboard' / 'ur16e-touch' modes.
#   --with-micro             'ur16e-touch' mode: also start the serial bridge +
#                            micro_module_control. Hold GREY 1 s (no WHITE)
#                            to switch the stylus between arm and micro module.
#   --with-rviz              Also start RViz + robot_state_publisher for
#                            every mode except 'display'.
#   --no-gui                 Use joint_state_publisher instead of the GUI
#                            version in 'display' mode.
#   --distro=NAME            ROS 2 distro to source (default: $ROS_DISTRO or
#                            "jazzy").
#   --dry-run                Print the commands that would run and exit.
#   -h, --help               Show this message.
#
# Every mode except 'display' and 'keyboard' runs its nodes in the background,
# tees their output to per-node log files under log/launch_system/<run>/, and
# tears everything down together on Ctrl+C (or via './launch_system.sh stop').

set -euo pipefail

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"
PID_FILE="${WS_DIR}/log/launch_system/pids"

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
MODE="${1:-help}"
[ "$#" -gt 0 ] && shift || true

ROS_DISTRO_ARG="${ROS_DISTRO:-jazzy}"
ROBOT_IP="192.168.0.100"
# The lab arm is a UR16e; the lab_ur5e_1 calibration belongs to the old UR5e.
UR_TYPE="ur16e"
CALIBRATION_FILE=""
ARDUINO_PORT="/dev/ttyACM0"
BAUD="9600"
WITH_RVIZ="false"
WITH_MICRO="false"
GUI="true"
DRY_RUN="false"
MOCK="false"

for arg in "$@"; do
  case "$arg" in
    --robot-ip=*) ROBOT_IP="${arg#*=}" ;;
    --calibration-file=*) CALIBRATION_FILE="${arg#*=}" ;;
    --arduino-port=*) ARDUINO_PORT="${arg#*=}" ;;
    --baud=*) BAUD="${arg#*=}" ;;
    --distro=*) ROS_DISTRO_ARG="${arg#*=}" ;;
    --with-rviz) WITH_RVIZ="true" ;;
    --with-micro) WITH_MICRO="true" ;;
    --no-gui) GUI="false" ;;
    --dry-run) DRY_RUN="true" ;;
    --mock) MOCK="true" ;;
    -h|--help) MODE="help" ;;
    *)
      echo "Unknown option: $arg" >&2
      exit 1
      ;;
  esac
done

# Empty = let the launch files pick the nominal UR16e model
KIN_ARG=()
[ -n "$CALIBRATION_FILE" ] && KIN_ARG=(kinematics_params_file:="$CALIBRATION_FILE")

usage() {
  sed -n '2,53p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

if [ "$MODE" = "help" ] || [ "$MODE" = "-h" ] || [ "$MODE" = "--help" ]; then
  usage
  exit 0
fi

# ---------------------------------------------------------------------------
# Stopping: every run_bg command leads its own process group (setsid), so the
# whole tree is stopped, not just `ros2 launch`. Killing only the top PID left
# ros2_control_node running; those orphans kept publishing stale /joint_states
# (poses an hour old) into every later session, which the teleop and Servo then
# acted on.
# ---------------------------------------------------------------------------
proc_alive() {
  kill -0 -- "-$1" 2>/dev/null || kill -0 "$1" 2>/dev/null
}

stop_groups() {
  # stop_groups <pid>... : SIGINT, then SIGTERM after 15 s, then SIGKILL after 5 s
  # more. A PID that is not a group leader (older PID files, tail) gets a plain kill.
  local step sig wait_s pid alive
  for step in "INT 15" "TERM 5" "KILL 2"; do
    sig=${step% *}; wait_s=${step#* }
    alive=()
    for pid in "$@"; do proc_alive "$pid" && alive+=("$pid"); done
    [ "${#alive[@]}" -eq 0 ] && return 0
    [ "$sig" != "INT" ] && echo "   still running: ${alive[*]} - sending SIG${sig}"
    for pid in "${alive[@]}"; do
      kill -"$sig" -- "-$pid" 2>/dev/null || kill -"$sig" "$pid" 2>/dev/null || true
    done
    for _ in $(seq $((wait_s * 10))); do
      alive=()
      for pid in "$@"; do proc_alive "$pid" && alive+=("$pid"); done
      [ "${#alive[@]}" -eq 0 ] && return 0
      sleep 0.1
    done
  done
}

# ---------------------------------------------------------------------------
# stop mode - doesn't need the ROS environment
# ---------------------------------------------------------------------------
if [ "$MODE" = "stop" ]; then
  if [ ! -f "$PID_FILE" ]; then
    echo "No recorded launch_system.sh run found (${PID_FILE} missing)."
    exit 0
  fi
  echo "Stopping processes from the last launch_system.sh run..."
  mapfile -t STOP_PIDS < "$PID_FILE"
  stop_groups "${STOP_PIDS[@]}"
  rm -f "$PID_FILE"
  echo "Done."
  exit 0
fi

case "$MODE" in
  display|touch|keyboard|ur16e|ur16e-keyboard|ur16e-touch|full) ;;
  *)
    echo "Unknown mode: $MODE" >&2
    usage
    exit 1
    ;;
esac

case "$MODE" in
  ur16e) MODE="ur16e-touch" ;;
  full) MODE="ur16e-touch"; WITH_MICRO="true" ;;
esac

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
ROS_SETUP="/opt/ros/${ROS_DISTRO_ARG}/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"

if [ "$DRY_RUN" = "false" ]; then
  if [ ! -f "$ROS_SETUP" ]; then
    echo "ERROR: ${ROS_SETUP} not found. Pass --distro=<name> if you're not on ROS 2 ${ROS_DISTRO_ARG}." >&2
    exit 1
  fi
  if [ ! -f "$WS_SETUP" ]; then
    echo "ERROR: ${WS_SETUP} not found. Build the workspace first:" >&2
    echo "  cd ${WS_DIR} && colcon build --symlink-install" >&2
    exit 1
  fi
  # ROS 2's setup.bash files reference variables (e.g. AMENT_TRACE_SETUP_FILES)
  # without guarding for `set -u`, so relax nounset just for sourcing them.
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$WS_SETUP"
  set -u
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${WS_DIR}/log/launch_system/${RUN_ID}"
PIDS=()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
run_bg() {
  # run_bg <name> <command...>
  local name="$1"; shift
  echo ">> [$name] $*"
  if [ "$DRY_RUN" = "true" ]; then
    return 0
  fi
  # Own process group (and session), so cleanup can stop the whole tree and a Ctrl+C
  # here reaches only this script, which then shuts everything down in order
  setsid "$@" > "${LOG_DIR}/${name}.log" 2>&1 < /dev/null &
  local pid=$!
  PIDS+=("$pid")
  echo "$pid" >> "$PID_FILE"
  echo "   started as PID $pid, log: ${LOG_DIR}/${name}.log"
}

wait_for_topic() {
  local topic="$1" timeout="${2:-15}" waited=0
  [ "$DRY_RUN" = "true" ] && return 0
  echo ">> waiting up to ${timeout}s for topic ${topic} ..."
  until ros2 topic list 2>/dev/null | grep -qx "$topic"; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge "$timeout" ]; then
      echo "   WARNING: timed out waiting for ${topic} - continuing anyway." >&2
      return 0
    fi
  done
  echo "   ${topic} is up."
}

wait_for_service() {
  local service="$1" timeout="${2:-30}" waited=0
  [ "$DRY_RUN" = "true" ] && return 0
  echo ">> waiting up to ${timeout}s for service ${service} ..."
  until ros2 service list 2>/dev/null | grep -qx "$service"; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge "$timeout" ]; then
      echo "   WARNING: timed out waiting for ${service} - continuing anyway." >&2
      return 0
    fi
  done
  echo "   ${service} is up."
}

check_serial_port() {
  local port="$1"
  if [ "$DRY_RUN" = "true" ]; then
    return 0
  fi
  if [ ! -e "$port" ]; then
    echo "ERROR: ${port} does not exist." >&2
    local candidates
    candidates="$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)"
    if [ -n "$candidates" ]; then
      echo "Devices found instead:" >&2
      echo "$candidates" >&2
      echo "Retry with --arduino-port=<one of the above>." >&2
    fi
    exit 1
  fi
  echo ">> chmod 666 ${port}"
  sudo chmod 666 "$port"
}

start_touch_driver() {
  run_bg touch_driver ros2 launch macro_micro_robot touch_driver.launch.py
  wait_for_topic /phantom/state 15
}

start_serial_bridge() {
  check_serial_port "$ARDUINO_PORT"
  run_bg serial_bridge python3 "${WS_DIR}/src/Serial_Control/ros2_bridge/serial_bridge.py" \
    --ros-args -p port:="$ARDUINO_PORT" -p baud:="$BAUD"
  wait_for_topic /micro_module_motor_states 15
}

start_rviz_if_requested() {
  if [ "$WITH_RVIZ" = "true" ]; then
    run_bg rviz ros2 launch macro_micro_robot display.launch.py gui:=false
  fi
}

cleanup() {
  if [ "${#PIDS[@]}" -eq 0 ]; then
    return
  fi
  echo
  echo ">> Shutting down..."
  stop_groups "${PIDS[@]}"
  wait 2>/dev/null || true
  rm -f "$PID_FILE"
  echo ">> Stopped."
}

check_no_running_driver() {
  # A second ros2_control_node on the same ROS domain publishes its own (stale)
  # /joint_states next to the live one. Orphans (their `ros2 launch` is gone, so the
  # parent is systemd / init) are left over from an earlier run: stop them. One that
  # still belongs to a running launch is someone's live session: refuse instead.
  [ "$DRY_RUN" = "true" ] && return 0
  local mine="${ROS_DOMAIN_ID:-0}" pid domain parent orphans=() live=()
  for pid in $(pgrep -f "lib/controller_manager/ros2_control_node" || true); do
    domain=$(tr '\0' '\n' < "/proc/${pid}/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p')
    [ "${domain:-0}" = "$mine" ] || continue
    parent=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [ "$parent" = "1" ] || [ "$(ps -o comm= -p "$parent" 2>/dev/null)" = "systemd" ]; then
      orphans+=("$pid")
    else
      live+=("$pid")
    fi
  done
  if [ "${#live[@]}" -gt 0 ]; then
    echo "ERROR: a robot driver from a running session is already up on ROS_DOMAIN_ID ${mine}" >&2
    echo "       (ros2_control_node PID ${live[*]}). Two drivers publish two /joint_states streams." >&2
    echo "       Stop that session first (Ctrl+C in its terminal, or ./scripts/launch_system.sh stop)." >&2
    exit 1
  fi
  if [ "${#orphans[@]}" -gt 0 ]; then
    echo ">> Stopping robot driver(s) left over from an earlier run: PID ${orphans[*]}"
    stop_groups "${orphans[@]}"
    for pid in "${orphans[@]}"; do
      if proc_alive "$pid"; then
        echo "ERROR: could not stop leftover ros2_control_node PID ${pid} - kill it by hand." >&2
        exit 1
      fi
    done
  fi
}

if [ "$DRY_RUN" = "false" ]; then
  # Before the PID file is replaced, so `launch_system.sh stop` can still stop the
  # previous run if that is what is holding the driver
  case "$MODE" in
    ur16e-keyboard|ur16e-touch) check_no_running_driver ;;
  esac
  mkdir -p "$(dirname "$PID_FILE")"
  rm -f "$PID_FILE"
  mkdir -p "$LOG_DIR"
  trap cleanup EXIT INT TERM
  echo "Logs: ${LOG_DIR}"
fi

# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------
case "$MODE" in

  display)
    echo ">> [rviz] ros2 launch macro_micro_robot display.launch.py gui:=${GUI}"
    if [ "$DRY_RUN" = "true" ]; then
      exit 0
    fi
    exec ros2 launch macro_micro_robot display.launch.py gui:="${GUI}"
    ;;

  touch)
    start_touch_driver

    start_serial_bridge

    run_bg micro_module_control ros2 run macro_micro_robot micro_module_control

    start_rviz_if_requested
    ;;

  keyboard)
    start_serial_bridge

    start_rviz_if_requested

    echo ">> [keyboard_control] running in foreground - this terminal now reads your keypresses."
    echo "   q/a proximal pan, w/s proximal tilt, e/d distal pan, r/f distal tilt. Ctrl+C to quit."
    if [ "$DRY_RUN" = "true" ]; then
      exit 0
    fi
    python3 "${WS_DIR}/src/Serial_Control/ros2_bridge/keyboard_control.py" \
      --ros-args -p port:="$ARDUINO_PORT"
    ;;

  ur16e-keyboard|ur16e-touch)
    INPUT="keyboard"
    [ "$MODE" = "ur16e-touch" ] && INPUT="touch"
    if [ "$MOCK" = "false" ]; then
      echo ">> Make sure the External Control program is running on the UR16e Teach Pendant."
    fi
    run_bg ur16e_teleop ros2 launch macro_micro_robot ur16e_teleop.launch.py \
      ur_type:="$UR_TYPE" robot_ip:="$ROBOT_IP" use_mock_hardware:="$MOCK" "${KIN_ARG[@]}" \
      input:="$INPUT" launch_rviz:="$WITH_RVIZ"
    wait_for_service /servo_node/switch_command_type 90

    if [ "$WITH_MICRO" = "true" ] && [ "$MODE" = "ur16e-touch" ]; then
      start_serial_bridge
      run_bg micro_module_control ros2 run macro_micro_robot micro_module_control
      echo ">> Stylus starts on the arm (MACRO). Hold GREY 1 s without WHITE to switch to MICRO and back."
    fi

    echo ">> Move to a safe start pose any time with:"
    echo "     ros2 run macro_micro_robot trajectory_player.py home"
    echo ">> Replay the last recording with:"
    echo "     ros2 run macro_micro_robot trajectory_player.py latest --speed 0.5"

    if [ "$MODE" = "ur16e-keyboard" ]; then
      echo ">> [ur16e_keyboard_teleop] running in foreground - this terminal now reads your keypresses."
      if [ "$DRY_RUN" = "true" ]; then
        exit 0
      fi
      ros2 run macro_micro_robot ur16e_keyboard_teleop.py
      exit 0
    fi
    ;;

esac

if [ "$DRY_RUN" = "true" ]; then
  exit 0
fi

echo
echo ">> System is up (mode: ${MODE}). Press Ctrl+C to stop everything."
echo ">> Tailing logs from ${LOG_DIR} - Ctrl+C here stops all nodes."
if [ "$MODE" = "ur16e-touch" ]; then
  # Logs in the background; this terminal reads tool numbers (tool library) instead
  tail -n +1 -f "${LOG_DIR}"/*.log &
  PIDS+=("$!")
  ros2 run macro_micro_robot tool_library.py console || true
  wait
else
  tail -n +1 -f "${LOG_DIR}"/*.log
fi
