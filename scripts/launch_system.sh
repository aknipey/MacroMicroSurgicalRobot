#!/usr/bin/env bash
#
# launch_system.sh - one-command launcher for the ls_thesis (Macro-Micro Surgical
# Robot) stack and all of its runtime dependencies (Touch driver, Arduino serial
# bridge, UR5e driver, control nodes, RViz).
#
# This replaces manually opening N terminals, sourcing ROS 2 in each one, and
# running the commands from MICRO_MODULE_STARTUP.md / README.md by hand.
#
# Usage:
#   ./launch_system.sh <mode> [options]
#
# Modes:
#   display     RViz + robot/joint state publishers only. No hardware needed.
#   touch       Touch stylus teleop of the micro-manipulator (no UR5e).
#               omni_state driver + serial bridge + micro_module_control.
#   keyboard    Keyboard teleop of the micro-manipulator. No Touch device
#               needed - only the Arduino. Good for bench-testing the servos.
#   ur5e-keyboard  UR5e + MoveIt Servo + trajectory recorder; this terminal
#               then jogs the arm from the keyboard.
#   ur5e-touch  UR5e + MoveIt Servo + trajectory recorder + Touch driver;
#               hold the stylus WHITE button to drive the arm.
#   ur5e        UR5e driver + legacy ur5e_control (ROS 1 port - needs a
#               Cartesian controller the ROS 2 UR driver doesn't have).
#   full        Full system: UR5e driver + Touch driver + serial bridge +
#               micro_module_control + ur5e_control.
#   stop        Stop every process started by a previous run of this script.
#   help        Show this message.
#
# Options:
#   --robot-ip=IP            UR5e IP address (default: 192.168.0.100)
#   --calibration-file=PATH  UR5e kinematics calibration YAML
#                            (default: <repo>/lab_ur5e_1_calibration.yaml)
#   --arduino-port=PORT      Serial port for the micro-module Arduino
#                            (default: /dev/ttyACM0)
#   --baud=RATE              Serial baud rate for the Arduino (default: 9600)
#   --mock                   Simulate the UR5e (no robot needed) in the
#                            'ur5e-keyboard' / 'ur5e-touch' modes.
#   --with-rviz              Also start RViz + robot_state_publisher for
#                            'touch' / 'keyboard' / 'ur5e' / 'full' modes.
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
CALIBRATION_FILE="${PKG_DIR}/lab_ur5e_1_calibration.yaml"
ARDUINO_PORT="/dev/ttyACM0"
BAUD="9600"
WITH_RVIZ="false"
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

usage() {
  sed -n '2,50p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

if [ "$MODE" = "help" ] || [ "$MODE" = "-h" ] || [ "$MODE" = "--help" ]; then
  usage
  exit 0
fi

# ---------------------------------------------------------------------------
# stop mode - doesn't need the ROS environment
# ---------------------------------------------------------------------------
if [ "$MODE" = "stop" ]; then
  if [ ! -f "$PID_FILE" ]; then
    echo "No recorded launch_system.sh run found (${PID_FILE} missing)."
    exit 0
  fi
  echo "Stopping processes from the last launch_system.sh run..."
  while read -r pid; do
    if kill -0 "$pid" 2>/dev/null; then
      echo "  killing PID $pid"
      kill "$pid" 2>/dev/null || true
    fi
  done < "$PID_FILE"
  rm -f "$PID_FILE"
  echo "Done."
  exit 0
fi

case "$MODE" in
  display|touch|keyboard|ur5e|ur5e-keyboard|ur5e-touch|full) ;;
  *)
    echo "Unknown mode: $MODE" >&2
    usage
    exit 1
    ;;
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
  ( "$@" > "${LOG_DIR}/${name}.log" 2>&1 ) &
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
      return 1
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
      return 1
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

cleanup() {
  if [ "${#PIDS[@]}" -eq 0 ]; then
    return
  fi
  echo
  echo ">> Shutting down..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  rm -f "$PID_FILE"
  echo ">> Stopped."
}

if [ "$DRY_RUN" = "false" ]; then
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
    echo ">> [rviz] ros2 launch ls_thesis display.launch.py gui:=${GUI}"
    if [ "$DRY_RUN" = "true" ]; then
      exit 0
    fi
    exec ros2 launch ls_thesis display.launch.py gui:="${GUI}"
    ;;

  touch)
    run_bg touch_driver ros2 launch omni_common omni_state.launch.py
    wait_for_topic /phantom/state 15

    check_serial_port "$ARDUINO_PORT"
    run_bg serial_bridge python3 "${WS_DIR}/src/Serial_Control/ros2_bridge/serial_bridge.py" \
      --ros-args -p port:="$ARDUINO_PORT" -p baud:="$BAUD"
    wait_for_topic /micro_module_motor_states 15

    run_bg micro_module_control ros2 run ls_thesis micro_module_control

    if [ "$WITH_RVIZ" = "true" ]; then
      run_bg rviz ros2 launch ls_thesis display.launch.py gui:=false
    fi
    ;;

  keyboard)
    check_serial_port "$ARDUINO_PORT"
    run_bg serial_bridge python3 "${WS_DIR}/src/Serial_Control/ros2_bridge/serial_bridge.py" \
      --ros-args -p port:="$ARDUINO_PORT" -p baud:="$BAUD"
    wait_for_topic /micro_module_motor_states 15

    if [ "$WITH_RVIZ" = "true" ]; then
      run_bg rviz ros2 launch ls_thesis display.launch.py gui:=false
    fi

    echo ">> [keyboard_control] running in foreground - this terminal now reads your keypresses."
    echo "   q/a proximal pan, w/s proximal tilt, e/d distal pan, r/f distal tilt. Ctrl+C to quit."
    if [ "$DRY_RUN" = "true" ]; then
      exit 0
    fi
    python3 "${WS_DIR}/src/Serial_Control/ros2_bridge/keyboard_control.py" \
      --ros-args -p port:="$ARDUINO_PORT"
    ;;

  ur5e)
    run_bg ur_driver ros2 launch ur_robot_driver ur_control.launch.py \
      ur_type:=ur5e robot_ip:="$ROBOT_IP" kinematics_params_file:="$CALIBRATION_FILE" \
      launch_rviz:=false
    wait_for_service /controller_manager/list_controllers 60

    run_bg ur5e_control ros2 run ls_thesis ur5e_control

    if [ "$WITH_RVIZ" = "true" ]; then
      run_bg rviz ros2 launch ls_thesis display.launch.py gui:=false
    fi
    ;;

  ur5e-keyboard|ur5e-touch)
    INPUT="keyboard"
    [ "$MODE" = "ur5e-touch" ] && INPUT="touch"
    if [ "$MOCK" = "false" ]; then
      echo ">> Make sure the External Control program is running on the UR5e Teach Pendant."
    fi
    run_bg ur5e_teleop ros2 launch ls_thesis ur5e_teleop.launch.py \
      robot_ip:="$ROBOT_IP" use_mock_hardware:="$MOCK" kinematics_params_file:="$CALIBRATION_FILE" \
      input:="$INPUT" launch_rviz:="$WITH_RVIZ"
    wait_for_service /servo_node/switch_command_type 90

    echo ">> Move to a safe start pose any time with:"
    echo "     ros2 run ls_thesis trajectory_player.py home"
    echo ">> Replay the last recording with:"
    echo "     ros2 run ls_thesis trajectory_player.py latest --speed 0.5"

    if [ "$MODE" = "ur5e-keyboard" ]; then
      echo ">> [ur5e_keyboard_teleop] running in foreground - this terminal now reads your keypresses."
      if [ "$DRY_RUN" = "true" ]; then
        exit 0
      fi
      ros2 run ls_thesis ur5e_keyboard_teleop.py
      exit 0
    fi
    ;;

  full)
    run_bg touch_driver ros2 launch omni_common omni_state.launch.py
    wait_for_topic /phantom/state 15

    check_serial_port "$ARDUINO_PORT"
    run_bg serial_bridge python3 "${WS_DIR}/src/Serial_Control/ros2_bridge/serial_bridge.py" \
      --ros-args -p port:="$ARDUINO_PORT" -p baud:="$BAUD"
    wait_for_topic /micro_module_motor_states 15

    run_bg ur_driver ros2 launch ur_robot_driver ur_control.launch.py \
      ur_type:=ur5e robot_ip:="$ROBOT_IP" kinematics_params_file:="$CALIBRATION_FILE" \
      launch_rviz:=false
    wait_for_service /controller_manager/list_controllers 60

    echo ">> Start the LS_ROS_CONTROL program on the UR5e Teach Pendant now, then press Enter."
    if [ "$DRY_RUN" = "false" ]; then
      read -r
    fi

    run_bg micro_module_control ros2 run ls_thesis micro_module_control
    run_bg ur5e_control ros2 run ls_thesis ur5e_control

    if [ "$WITH_RVIZ" = "true" ]; then
      run_bg rviz ros2 launch ls_thesis display.launch.py gui:=false
    fi
    ;;
esac

if [ "$DRY_RUN" = "true" ]; then
  exit 0
fi

echo
echo ">> System is up (mode: ${MODE}). Press Ctrl+C to stop everything."
echo ">> Tailing logs from ${LOG_DIR} - Ctrl+C here stops all nodes."
tail -n +1 -f "${LOG_DIR}"/*.log
