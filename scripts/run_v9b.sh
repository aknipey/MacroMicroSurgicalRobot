#!/usr/bin/env bash
#
# run_v9b.sh - walk through the V9b RCM runs with the wand + NDI: switches the mode, restarts
# the stack, replays, checks each recording, and stops to ask whenever a person is needed.
#
# RCM runs only. Scaled5_* and NoBuzzArea are left out: their wrist 3 angle (~135 deg) turns the
# markers ~110 deg away from a camera that sees the RCM runs (0 % NDI on the 11-03 Scaled5_Slow).
#
# Needs, in another terminal: python3 -m ndi_tracking run (leave it running).
# You still do: press Play on the pendant after each restart, answer y before anything moves,
# move the camera before step 5. Stay at the e-stop.
#
# Usage:
#   ./scripts/run_v9b.sh                 # from step 0
#   ./scripts/run_v9b.sh --from=3        # resume at step 3
#   ./scripts/run_v9b.sh --only=9
#   ./scripts/run_v9b.sh --dry-run       # print the plan, change nothing
#
# Ctrl+C stops the replay, then this script stops the stack (never Ctrl+Z). If a recording
# was cut off, logs/NEW_RUN is left behind: this script tells you where to move it.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS="$(cd "${PKG}/../.." && pwd)"
LOGS="${PKG}/logs"
CALIBRATION="${PKG}/config/calibrations/ur16e_kinematics_2026-11-03.yaml"
STACK_LOG_DIR="${LOGS}/V9b/stack_logs"
ROBOT_IP="192.168.0.100"
PROGRAM_TOPIC="/io_and_status_controller/robot_program_running"
LAUNCH_EXTRA="${LAUNCH_EXTRA:-}"    # extra launch arguments, for testing on mock hardware
# Bridge run: 11-03 values and how far off counts as "something moved"
BRIDGE_SLOW=1.29
BRIDGE_RCM=0.48
BRIDGE_TOLERANCE=0.15

FROM=0
ONLY=""
DRY_RUN=false
for arg in "$@"; do
  case "$arg" in
    --from=*) FROM="${arg#*=}" ;;
    --only=*) ONLY="${arg#*=}" ;;
    --dry-run) DRY_RUN=true ;;
    -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done

STACK_PID=""
PLAYER_PID=""
POSE_FILE=""

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
die() { printf '\n\033[1;31m!!! %s\033[0m\n' "$*" >&2; exit 1; }

ask_continue() {
  local answer
  read -r -p "$1 [Enter = continue, q = stop here] " answer
  [ "$answer" = "q" ] && exit 0
  return 0
}

# --- stack -------------------------------------------------------------------------------
stack_alive() { [ -n "$STACK_PID" ] && kill -0 "$STACK_PID" 2>/dev/null; }

stop_stack() {
  if stack_alive; then
    echo ">> Stopping the stack"
    kill -INT -- "-${STACK_PID}" 2>/dev/null
    for _ in $(seq 1 40); do
      pgrep -g "$STACK_PID" >/dev/null || break
      sleep 0.5
    done
    if pgrep -g "$STACK_PID" >/dev/null; then
      echo ">> Still running after 20 s: sending SIGKILL to what is left"
      kill -KILL -- "-${STACK_PID}" 2>/dev/null
      sleep 1
    fi
  fi
  STACK_PID=""
}

start_stack() {
  stop_stack
  if pgrep -f ros2_control_node >/dev/null; then
    pgrep -fa ros2_control_node
    die "A ros2_control_node is still running, and it would feed stale /joint_states into the new stack. Stop it first."
  fi
  mkdir -p "$STACK_LOG_DIR"
  local log
  log="${STACK_LOG_DIR}/$(date +%H%M%S)_step${STEP}.log"
  echo ">> Starting the stack (log: ${log})"
  # set -m: its own process group, so a Ctrl+C here reaches the replay and not the stack
  # shellcheck disable=SC2086
  set -m
  ros2 launch macro_micro_robot ur16e_teleop.launch.py input:=replay \
    kinematics_params_file:="$CALIBRATION" $LAUNCH_EXTRA >"$log" 2>&1 </dev/null &
  STACK_PID=$!
  set +m

  local up=false
  for _ in $(seq 1 45); do
    stack_alive || die "The stack exited during startup. See ${log}"
    if ros2 service list --no-daemon 2>/dev/null | grep -q "^/servo_node/switch_command_type$"; then
      up=true
      break
    fi
    sleep 2
  done
  $up || die "Servo did not come up in 90 s. See ${log}"
  say ">> Press Play on the pendant (External Control)"
  until timeout 4 ros2 topic echo --once --field data --qos-durability transient_local \
          --qos-reliability reliable "$PROGRAM_TOPIC" std_msgs/msg/Bool 2>/dev/null | grep -qi "^true"; do
    stack_alive || die "The stack exited. See ${log}"
  done
  echo ">> External Control is running"
  grep -q "calibration parameters of the connected robot don't match" "$log" \
    && die "The driver says the calibration doesn't match this arm. See ${log}"
  for _ in $(seq 1 15); do grep -q "V2_holder_only" "$log" && break; sleep 2; done
  grep -q "V2_holder_only" "$log" || die "Tool 3 V2_holder_only is not active. See ${log}"
}

# --- recordings --------------------------------------------------------------------------
check_new_run_free() {
  [ -e "${LOGS}/NEW_RUN" ] && die "logs/NEW_RUN exists (a cut-off recording?). Move it, e.g. mv ${LOGS}/NEW_RUN ${LOGS}/V9b/<name>_INCOMPLETE, then run with --from=${STEP}"
  return 0
}

count_replays() {
  find "${LOGS}/$1" -mindepth 1 -maxdepth 1 -type d -name '*_Replay[0-9]*' ! -name '*_NO_NDI' 2>/dev/null | wc -l
}

# One line per run: NDI RCM rms, NDI visibility, code_version
summarise() {
  python3 - "${LOGS}" "$@" <<'EOF'
import os, sys, yaml
logs, runs = sys.argv[1], sys.argv[2:]
for run in runs:
    path = os.path.join(logs, run)
    try:
        ndi = yaml.safe_load(open(os.path.join(path, "NDI_META.yaml"))) or {}
        meta = yaml.safe_load(open(os.path.join(path, "META.yaml"))) or {}
    except OSError as exc:
        print(f"  {run}: {exc}")
        continue
    rms = [s["error_rms_mm"] for s in ndi.get("rcm_segments") or [] if "error_rms_mm" in s]
    seen = 100.0 * ndi.get("visible_rows", 0) / max(ndi.get("rows", 0), 1)
    version = meta.get("code_version", "?")
    flags = ("  !!! dirty" if "dirty" in str(version) else "") + ("  !!! NDI < 95 %" if seen < 95 else "")
    print(f"  {os.path.basename(run):48s} rms {', '.join(f'{r:.2f}' for r in rms) or '-':12s} NDI {seen:5.1f} %  {version}{flags}")
EOF
}

# batch <dest> <repeats> <run...>
batch() {
  local dest="$1" repeats="$2"
  shift 2
  local args=("--dest=${dest}" "--repeats=${repeats}" "$@") wanted
  wanted=$(( repeats * ($# > 0 ? $# : 4) ))
  if $DRY_RUN; then echo "   replay_batch.sh ${args[*]}"; return 0; fi
  check_new_run_free
  local before after
  before=$(count_replays "$dest")
  "${SCRIPT_DIR}/replay_batch.sh" "${args[@]}"
  after=$(count_replays "$dest")
  local names
  mapfile -t names < <(cd "$LOGS" && ls -1dtr "${dest}"/*_Replay[0-9]* 2>/dev/null | grep -v '_NO_NDI$' | tail -n $(( after - before > 0 ? after - before : 1 )))
  say "${dest}: $(( after - before )) of ${wanted} replays saved"
  [ "$after" -gt "$before" ] && summarise "${names[@]}"
  [ $(( after - before )) -eq "$wanted" ] \
    || die "Batch incomplete. Fix the cause, then run with --from=${STEP} (replay numbers carry on from what exists)."
}

mode() {
  if $DRY_RUN; then echo "   set_v9_mode.sh $1, restart stack, wait for Play"; return 0; fi
  check_new_run_free
  "${SCRIPT_DIR}/set_v9_mode.sh" "$1" || die "set_v9_mode.sh failed"
  start_stack
}

recorder() {
  ros2 service call "/trajectory_recorder/$1" std_srvs/srv/Trigger 2>&1 | tail -1
}

keep_recording() {
  local name="$1" target="${LOGS}/V9b/$1"
  [ -e "$target" ] && die "logs/V9b/${name} already exists: rename it, then run with --from=${STEP}"
  [ -d "${LOGS}/NEW_RUN" ] || die "No logs/NEW_RUN: nothing was recorded."
  if [ ! -s "${LOGS}/NEW_RUN/NDI_PATH.csv" ]; then
    mv "${LOGS}/NEW_RUN" "${target}_NO_NDI"
    die "No NDI data (kept as logs/V9b/${name}_NO_NDI). Check the tracker sees the wand."
  fi
  mv "${LOGS}/NEW_RUN" "$target"
  summarise "V9b/${name}"
}

program_stopped() {
  local state
  state="$(timeout 4 ros2 topic echo --once --field data --qos-durability transient_local \
           --qos-reliability reliable "$PROGRAM_TOPIC" std_msgs/msg/Bool 2>/dev/null | head -1)"
  [ "${state,,}" = "false" ]
}

# The recorder start can stall the control loop long enough to stop External Control, and a
# trajectory left waiting then jumps to catch up when Play is pressed (2026-11-06: 170 mm in
# 2.5 s). So check after the stall, and stop everything if the program drops while it plays.
stop_player() {
  [ -n "$PLAYER_PID" ] || return 0
  kill -INT -- "-${PLAYER_PID}" 2>/dev/null
  sleep 2
  kill -KILL -- "-${PLAYER_PID}" 2>/dev/null
  PLAYER_PID=""
}

pose_failed() {
  stop_player
  recorder stop
  mv "${LOGS}/NEW_RUN" "${LOGS}/V9b/$1_INCOMPLETE_$(date +%H%M%S)" 2>/dev/null
  die "$2 Kept as logs/V9b/$1_INCOMPLETE_*. The stack stops now: don't press Play, run with --from=${STEP}."
}

pose_run() {
  local name="$1"
  if $DRY_RUN; then echo "   record logs/V9b/${name}: 112 s between the latch and working poses"; return 0; fi
  check_new_run_free
  [ -n "$POSE_FILE" ] || POSE_FILE="$(python3 "${SCRIPT_DIR}/pose_offset_motion.py")" || die "pose_offset_motion.py failed"
  stack_alive || start_stack
  recorder start
  sleep 8
  program_stopped && pose_failed "$name" "External Control stopped when the recording started."
  ask_continue "The arm moves now: 15 s approach, then 112 s."
  program_stopped && pose_failed "$name" "External Control stopped before the motion."

  local out="${STACK_LOG_DIR}/${name}_player_$(date +%H%M%S).log"
  mkdir -p "$STACK_LOG_DIR"
  set -m
  ros2 run macro_micro_robot trajectory_player.py "$POSE_FILE" --approach-time 15 --yes >"$out" 2>&1 </dev/null &
  PLAYER_PID=$!
  set +m
  while kill -0 "$PLAYER_PID" 2>/dev/null; do
    program_stopped && pose_failed "$name" "External Control stopped during the motion."
    sleep 1
  done
  wait "$PLAYER_PID"
  PLAYER_PID=""
  tail -2 "$out"
  # trajectory_player.py exits 0 even when the trajectory fails
  grep -q "Trajectory complete" "$out" || pose_failed "$name" "The motion did not finish."
  sleep 2
  recorder stop
  keep_recording "$name"
}

# --- steps -------------------------------------------------------------------------------
step_0() {
  mode position
  batch V9b/PositionBridge 2 V7/SlowCheckingRCMSimpleHolder V7/RCMOnlySimpleHolder:--rcm
  $DRY_RUN && return 0
  python3 - "${LOGS}/V9b/PositionBridge" "$BRIDGE_SLOW" "$BRIDGE_RCM" "$BRIDGE_TOLERANCE" <<'EOF' \
    || ask_continue "Bridge run differs from 11-03. Check RCMOnly first: if it still matches, this is the camera (SlowChecking changes with it) and the hand-eye is fine. Carry on?"
import glob, os, sys, yaml
folder, slow, rcm, tolerance = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
ok = True
for pattern, expected in (("SlowChecking*", slow), ("RCMOnly*", rcm)):
    for path in sorted(glob.glob(os.path.join(folder, pattern))):
        meta = yaml.safe_load(open(os.path.join(path, "NDI_META.yaml")))
        for segment in meta.get("rcm_segments") or []:
            if abs(segment["error_rms_mm"] - expected) > tolerance:
                print(f"  {os.path.basename(path)}: {segment['error_rms_mm']:.2f} mm vs {expected} on 11-03")
                ok = False
sys.exit(0 if ok else 1)
EOF
}

step_1() {
  say "Unfiltered velocity: 1:5 runs only. Ctrl+C or the pendant stop if it shakes badly."
  mode velocity-unfiltered
  batch V9b/VelocityUnfiltered 2 V7/RCMOnlySimpleHolder:--rcm
}

step_2() {
  mode velocity-optimised
  batch V9b/VelocityOptimised 3
}

step_3() {
  if $DRY_RUN; then echo "   30 s recording with the arm still -> logs/V9b/NDIStaticFloor"; return 0; fi
  check_new_run_free
  stack_alive || start_stack
  ask_continue "Static NDI floor: nobody touches the arm, wand or table for 30 s."
  recorder start
  sleep 30
  recorder stop
  keep_recording NDIStaticFloor
}

step_4() {
  # RCMOnly left RCM mode on, so this needs a fresh stack
  $DRY_RUN || start_stack
  say "Pose-offset test, camera position 1. Clear the area: this joint move doesn't hold any RCM point."
  pose_run PoseOffset_Camera1
}

step_5() {
  if ! $DRY_RUN; then
    say "Move the Vega about 100 mm, at a slightly different angle, with the wand still in view."
    ask_continue "Camera moved and settled?"
  fi
  pose_run PoseOffset_Camera2
}

step_6() {
  if $DRY_RUN; then echo "   stop stack, set_v9_mode.sh final"; return 0; fi
  stop_stack
  "${SCRIPT_DIR}/set_v9_mode.sh" final || die "set_v9_mode.sh failed"
  say "RCM runs done, settings are at final. The wand can come off."
}

# --- main --------------------------------------------------------------------------------
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${WS}/install/setup.bash"
set -u

steps=()
for n in 0 1 2 3 4 5 6; do
  if [ -n "$ONLY" ]; then
    [ "$n" = "$ONLY" ] && steps+=("$n")
  elif [ "$n" -ge "$FROM" ]; then
    steps+=("$n")
  fi
done
[ "${#steps[@]}" -gt 0 ] || die "No steps to run."

if $DRY_RUN; then
  for STEP in "${steps[@]}"; do
    echo "Step ${STEP}:"
    "step_${STEP}"
  done
  exit 0
fi

trap 'stop_player; stop_stack' EXIT
trap 'echo; echo ">> Interrupted"; exit 130' INT TERM

STEP="${steps[0]}"
timeout 3 bash -c "</dev/tcp/${ROBOT_IP}/30001" 2>/dev/null || [ -n "$LAUNCH_EXTRA" ] || die "Can't reach the arm at ${ROBOT_IP}: check the cable and that it is powered on."
[ -z "$(git -C "$PKG" status --porcelain --untracked-files=no | grep -v 'macro_micro_robot/teleop_settings.py' || true)" ] \
  || die "Uncommitted changes: commit them first, or set_v9_mode.sh refuses and code_version says -dirty."
check_new_run_free
ros2 node list 2>/dev/null | grep -q "/ndi_tracking" \
  || die "NDI tracker not seen. Start it: cd ~/ros2_ws/src/ndi_tracking && python3 -m ndi_tracking run"

say "Running steps ${steps[*]}. Wand and holder must be untouched; don't move the camera until asked."
for STEP in "${steps[@]}"; do
  say "=== Step ${STEP}"
  "step_${STEP}"
  if [ "$STEP" != "${steps[-1]}" ]; then ask_continue "Step ${STEP} done."; fi
done
say "All done. Back up logs/V9b, and tell Claude the runs are in for the analysis."
