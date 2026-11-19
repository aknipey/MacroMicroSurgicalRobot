#!/usr/bin/env bash
#
# run_v9c.sh - the final lab session (thesis/planning/09_final_lab_session.md, revised 11-19).
#
# Phase A (steps 1-10): bare flange, no NDI, runs unattended. Output mode x filter cutoff grid at
#   1:5 (Scaled5_Fast, Scaled5_Slow x4 per cell), position 5 Hz anchor at both ends, then the 1:1
#   inputs at position 20 Hz. Tool 3 stays configured, as in session C, so paths match V9b/NoTool.
#   Order: P5 V5 P10 V10 P20 V20 Poff Voff P5_end P20(1:1). Velocity cells are 1:5 only.
#
# Phase B (steps 20+): the wand and NDI (RCM, tip vibration, calibration). Not written yet.
#
# Unattended needs the pendant in REMOTE CONTROL with the External Control program loaded: the
# script then presses Play itself over the dashboard port after each restart. In local mode it
# beeps and waits for you to press Play. Stay within reach of the e-stop either way.
#
# Usage:
#   ./scripts/run_v9c.sh                 # phase A from step 1
#   ./scripts/run_v9c.sh --from=6        # resume at step 6 (replay numbers carry on)
#   ./scripts/run_v9c.sh --only=10
#   ./scripts/run_v9c.sh --dry-run       # print the plan, change nothing
#   LAUNCH_EXTRA="use_mock_hardware:=true" ROS_DOMAIN_ID=77 V9C=V9c_mock REPEATS_1TO5=1 \
#     ./scripts/run_v9c.sh --only=1                                         # mock test
#
# Any failed batch stops the stack and the script, and prints the --from= to resume with.
# Ctrl+C stops the replay, then the stack (never Ctrl+Z).

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS="$(cd "${PKG}/../.." && pwd)"
LOGS="${PKG}/logs"
CALIBRATION="${PKG}/config/calibrations/ur16e_kinematics_2026-11-03.yaml"
TOOLS="${PKG}/config/tools.yaml"
V9C="${V9C:-V9c}"                  # logs/ subfolder; V9c_mock for mock tests
SESSION_DIR="${LOGS}/${V9C}"
STACK_LOG_DIR="${SESSION_DIR}/stack_logs"
SESSION_LOG="${SESSION_DIR}/REPLAYS_$(date +%F).md"
ROBOT_IP="192.168.0.100"
LAUNCH_EXTRA="${LAUNCH_EXTRA:-}"    # extra launch arguments, for testing on mock hardware
REPEATS_1TO5="${REPEATS_1TO5:-4}"
REPEATS_1TO1="${REPEATS_1TO1:-2}"

FROM=1
ONLY=""
DRY_RUN=false
for arg in "$@"; do
  case "$arg" in
    --from=*) FROM="${arg#*=}" ;;
    --only=*) ONLY="${arg#*=}" ;;
    --dry-run) DRY_RUN=true ;;
    -h|--help) sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done

NDI_OPT=""
STACK_PID=""
STACK_LOG=""
TOOL_NAME=""
STEP=""

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
note() { mkdir -p "$SESSION_DIR"; printf -- '- %s step %s: %s\n' "$(date +%H:%M:%S)" "$STEP" "$*" >>"$SESSION_LOG"; }
die() {
  printf '\n\033[1;31m!!! %s\033[0m\n' "$*" >&2
  $DRY_RUN || note "STOPPED: $*"
  printf '\a'
  exit 1
}

# --- settings (each change committed, so every recording's code_version names it) --------
set_mode() {
  "${SCRIPT_DIR}/set_v9_mode.sh" "$1" | grep -v "Now restart" || die "set_v9_mode.sh $1 failed"
}

set_tool() {
  local current
  current=$(sed -n 's/^default: *//p' "$TOOLS")
  if [ "$current" != "$1" ]; then
    sed -i "s/^default: .*/default: $1/" "$TOOLS"
    git -C "$PKG" commit -q -m "V9c: default tool $1" -- config/tools.yaml || die "commit of tools.yaml failed"
    echo ">> tools.yaml default: $1 (committed)"
  fi
  TOOL_NAME=$(python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['tools'][int(sys.argv[2])]['name'])" "$TOOLS" "$1") \
    || die "Tool $1 not in tools.yaml"
}

# --- UR dashboard (port 29999). Plain TCP, no ros2 CLI. ----------------------------------
dashboard() {
  local reply=""
  exec 3<>"/dev/tcp/${ROBOT_IP}/29999" 2>/dev/null || return 1
  read -r -t 3 _ <&3
  printf '%s\n' "$1" >&3
  read -r -t 5 reply <&3
  exec 3>&- 3<&-
  printf '%s' "$reply"
}
remote_control() { [ -z "$LAUNCH_EXTRA" ] && dashboard "is in remote control" | grep -qi "true"; }

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

# From the stack log, never by subscribing (a new subscriber lined up with the 11-06 stalls)
program_state() { grep -E "Robot program (running|stopped)" "$STACK_LOG" 2>/dev/null | tail -1; }

start_stack() {
  stop_stack
  for _ in $(seq 1 10); do pgrep -f ros2_control_node >/dev/null || break; sleep 1; done
  if pgrep -f ros2_control_node >/dev/null; then
    pgrep -fa ros2_control_node
    die "A ros2_control_node is still running and would feed stale /joint_states into the new stack. Kill it, then --from=${STEP}."
  fi
  mkdir -p "$STACK_LOG_DIR"
  STACK_LOG="${STACK_LOG_DIR}/$(date +%H%M%S)_step${STEP}.log"
  echo ">> Starting the stack (log: ${STACK_LOG})"
  set -m
  # shellcheck disable=SC2086
  ros2 launch macro_micro_robot ur16e_teleop.launch.py input:=replay \
    kinematics_params_file:="$CALIBRATION" $LAUNCH_EXTRA >"$STACK_LOG" 2>&1 </dev/null &
  STACK_PID=$!
  set +m

  local up=false
  for _ in $(seq 1 45); do
    stack_alive || die "The stack exited during startup. See ${STACK_LOG}"
    if ros2 service list --no-daemon 2>/dev/null | grep -q "^/servo_node/switch_command_type$"; then
      up=true
      break
    fi
    sleep 2
  done
  $up || die "Servo did not come up in 90 s. See ${STACK_LOG}"

  if [ -z "$LAUNCH_EXTRA" ]; then
    sleep 3   # let the driver open its reverse interface before the program connects to it
    if remote_control; then
      echo ">> Pressing Play over the dashboard: $(dashboard play)"
    else
      printf '\a'
      say ">> Press Play on the pendant (External Control). [Pendant in Remote Control = no need next time]"
    fi
  fi
  local waited=0
  until program_state | grep -q "running"; do
    stack_alive || die "The stack exited. See ${STACK_LOG}"
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -eq 30 ] && remote_control; then
      echo ">> Not running after 30 s: pressing Play again: $(dashboard play)"
    fi
    [ $((waited % 60)) -eq 0 ] && printf '\a'
  done
  echo ">> External Control is running"
  grep -q "calibration parameters of the connected robot don't match" "$STACK_LOG" \
    && die "The driver says the calibration doesn't match this arm. See ${STACK_LOG}"
  for _ in $(seq 1 15); do grep -q "$TOOL_NAME" "$STACK_LOG" && break; sleep 2; done
  grep -q "$TOOL_NAME" "$STACK_LOG" || die "Tool '${TOOL_NAME}' is not active. See ${STACK_LOG}"
}

# --- recordings --------------------------------------------------------------------------
check_new_run_free() {
  [ -e "${LOGS}/NEW_RUN" ] && die "logs/NEW_RUN exists (a cut-off recording?). Move it, e.g. mv ${LOGS}/NEW_RUN ${SESSION_DIR}/<name>_INCOMPLETE, then --from=${STEP}"
  return 0
}

count_replays() {
  find "${LOGS}/$1" -mindepth 1 -maxdepth 1 -type d -name '*_Replay[0-9]*' ! -name '*_NO_NDI' 2>/dev/null | wc -l
}

# One line per run: duration, code_version, NDI visibility if any
summarise() {
  python3 - "${LOGS}" "$@" <<'EOF'
import os, sys, yaml
logs, runs = sys.argv[1], sys.argv[2:]
for run in runs:
    path = os.path.join(logs, run)
    try:
        meta = yaml.safe_load(open(os.path.join(path, "META.yaml"))) or {}
    except OSError as exc:
        print(f"  {run}: {exc}")
        continue
    version = str(meta.get("code_version", "?"))
    flags = "  !!! dirty" if "dirty" in version else ""
    ndi = ""
    ndi_path = os.path.join(path, "NDI_META.yaml")
    if os.path.exists(ndi_path):
        n = yaml.safe_load(open(ndi_path)) or {}
        seen = 100.0 * n.get("visible_rows", 0) / max(n.get("rows", 0), 1)
        ndi = f"NDI {seen:5.1f} %"
        flags += "  !!! NDI < 95 %" if seen < 95 else ""
    found = {}
    def walk(d):
        if isinstance(d, dict):
            for k, v in d.items():
                if k in ("SERVO_OUTPUT", "TOUCH_COMMAND_FILTER", "TOUCH_KALMAN", "SERVO_SMOOTH_SPEEDJ"):
                    found.setdefault(k, v)
                walk(v)
    walk(meta)
    settings = (f"{found.get('SERVO_OUTPUT', '?')} {found.get('TOUCH_COMMAND_FILTER', '?')} Hz "
                f"Kalman {found.get('TOUCH_KALMAN', '?')} smooth {found.get('SERVO_SMOOTH_SPEEDJ', '?')}")
    print(f"  {os.path.basename(run):28s} {settings:42s} {ndi:12s} {version}{flags}")
EOF
}

# batch <dest> <repeats> <run...>
batch() {
  local dest="$1" repeats="$2"
  shift 2
  local args=("--dest=${dest}" "--repeats=${repeats}" "--yes" ${NDI_OPT:+"$NDI_OPT"} "$@")
  local wanted=$(( repeats * $# ))
  if $DRY_RUN; then echo "   replay_batch.sh ${args[*]}"; return 0; fi
  check_new_run_free
  stack_alive || die "No stack running."
  local before after lines_before
  before=$(count_replays "$dest")
  lines_before=$(wc -l <"$STACK_LOG")
  "${SCRIPT_DIR}/replay_batch.sh" "${args[@]}"
  after=$(count_replays "$dest")
  local names=()
  [ "$after" -gt "$before" ] && mapfile -t names < <(cd "$LOGS" && ls -1dtr "${dest}"/*_Replay[0-9]* 2>/dev/null | grep -v '_NO_NDI$' | tail -n $(( after - before )))
  say "${dest}: $(( after - before )) of ${wanted} replays saved"
  [ "${#names[@]}" -gt 0 ] && summarise "${names[@]}"
  local faults
  faults=$(tail -n +"$((lines_before + 1))" "$STACK_LOG" | grep -i -c -E "protective|Robot program stopped|program drop|overspeed")
  note "$(git -C "$PKG" log --oneline -1 | cut -c1-60) | ${dest}: $(( after - before ))/${wanted} replays, ${faults} fault lines in the stack log"
  [ "$faults" -eq 0 ] || echo "!!! ${faults} fault lines in the stack log during this batch: see ${STACK_LOG}"
  [ $(( after - before )) -eq "$wanted" ] \
    || die "Batch incomplete in ${dest}. Check the pendant (protective stop? program stopped?), then --from=${STEP}."
}

# cell <mode> <dest> <repeats> <run...>: switch mode, restart, replay
cell() {
  local m="$1" dest="$2" repeats="$3"
  shift 3
  say "=== Step ${STEP}: mode ${m} -> ${dest}"
  if $DRY_RUN; then echo "   set_v9_mode.sh ${m}, restart stack, Play"; batch "$dest" "$repeats" "$@"; return 0; fi
  check_new_run_free
  stop_stack
  set_mode "$m"
  start_stack
  batch "$dest" "$repeats" "$@"
}

# --- phase A: bare flange, no NDI --------------------------------------------------------
S5=(V3/Scaled5_Fast V3/Scaled5_Slow)
ONE_TO_ONE=(V3/LongBenchmark V3/ShortBenchmark V7/NoBuzzArea V7/LargeVibrations)
G="${V9C}/NoTool"

step_1()  { cell p5   "$G/P5"     "$REPEATS_1TO5" "${S5[@]}"; }
step_2()  { cell v5   "$G/V5"     "$REPEATS_1TO5" "${S5[@]}"; }
step_3()  { cell p10  "$G/P10"    "$REPEATS_1TO5" "${S5[@]}"; }
step_4()  { cell v10  "$G/V10"    "$REPEATS_1TO5" "${S5[@]}"; }
step_5()  { cell p20  "$G/P20"    "$REPEATS_1TO5" "${S5[@]}"; }
step_6()  { cell v20  "$G/V20"    "$REPEATS_1TO5" "${S5[@]}"; }
step_7()  { cell poff "$G/Poff"   "$REPEATS_1TO5" "${S5[@]}"; }
step_8()  { cell voff "$G/Voff"   "$REPEATS_1TO5" "${S5[@]}"; }
step_9()  { cell p5   "$G/P5_end" "$REPEATS_1TO5" "${S5[@]}"; }
# 1:1 at 20 Hz, position only (LargeVibrations is allowed: the rule is velocity only). Last in
# phase A, so a fault here can't cost any of the 1:5 grid.
step_10() { cell p20  "$G/P20_1to1" "$REPEATS_1TO1" "${ONE_TO_ONE[@]}"; }

PHASE_A=(1 2 3 4 5 6 7 8 9 10)

# --- main --------------------------------------------------------------------------------
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${WS}/install/setup.bash"
set -u

steps=()
for n in "${PHASE_A[@]}"; do
  if [ -n "$ONLY" ]; then
    [ "$n" = "$ONLY" ] && steps+=("$n")
  elif [ "$n" -ge "$FROM" ]; then
    steps+=("$n")
  fi
done
[ "${#steps[@]}" -gt 0 ] || die "No steps to run."

if $DRY_RUN; then
  for STEP in "${steps[@]}"; do "step_${STEP}"; done
  exit 0
fi

trap 'stop_stack; "${SCRIPT_DIR}/set_v9_mode.sh" final >/dev/null 2>&1 || true' EXIT
trap 'echo; echo ">> Interrupted"; note "interrupted (Ctrl+C)"; exit 130' INT TERM

STEP="${steps[0]}"
if [ -z "$LAUNCH_EXTRA" ]; then
  timeout 3 bash -c "</dev/tcp/${ROBOT_IP}/30001" 2>/dev/null || die "Can't reach the arm at ${ROBOT_IP}: check the cable and that it is powered on."
fi
[ -z "$(git -C "$PKG" status --porcelain --untracked-files=no | grep -v -E 'macro_micro_robot/teleop_settings.py|config/tools.yaml' || true)" ] \
  || die "Uncommitted changes: commit them first, or code_version says -dirty."
check_new_run_free
NDI_OPT=--no-ndi
set_tool 3

if [ -z "$LAUNCH_EXTRA" ]; then
  if remote_control; then
    echo ">> Pendant is in Remote Control: Play is pressed automatically."
  else
    echo ">> Pendant is in LOCAL control: you'll have to press Play after each of ${#steps[@]} restarts."
    echo "   For unattended runs: pendant top-right -> Remote Control (External Control program loaded)."
  fi
fi
say "Phase A, steps ${steps[*]}: BARE FLANGE, nothing on the tool flange, workspace clear."
say "About $(( ${#steps[@]} * 7 )) min. The arm moves without asking from here on. Stay near the e-stop."
read -r -p "Bare flange, robot clear, start? [y/N] " answer
[ "$answer" = "y" ] || [ "$answer" = "Y" ] || exit 1
note "phase A start, steps ${steps[*]}"

for STEP in "${steps[@]}"; do
  "step_${STEP}"
done
stop_stack
note "phase A done"
say "Phase A done (settings go back to final on exit). Tell Claude: the grid is in logs/V9c/NoTool."
