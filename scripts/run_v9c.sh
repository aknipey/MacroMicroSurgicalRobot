#!/usr/bin/env bash
#
# run_v9c.sh - the final lab session (thesis/planning/09_final_lab_session.md, revised 11-19).
#
# Phase A (steps 1-10): bare flange, no NDI, runs unattended. Output mode x filter cutoff grid at
#   1:5 (Scaled5_Fast, Scaled5_Slow x4 per cell), position 5 Hz anchor at both ends, then the 1:1
#   inputs at position 20 Hz. Tool 3 stays configured, as in session C, so paths match V9b/NoTool.
#   Order: P5 V5 P10 V10 P20 V20 Poff Voff P5_end P20(1:1). Velocity cells are 1:5 only.
#
# Phase B (steps 20-35, --phase=B): NDI. Steps 20-32 with the WAND-ONLY mount (SimpleHolder straight on the
#   flange, no quick-change coupling, so no coupling error in the RCM results); steps 33-34 with the C3 coupling.
#   The script runs the tracker itself.
#   20 warm-up RCMOnly (tool 3): checks the camera sees the wand at the RCM poses
#   21 three hand-eye fits back to back (wrist jog + robot-tcp) -> tools 4, 5, 6
#   22 NDI static floor (30 s, arm still)
#   23 RCM baseline, P5, tool 4: SlowChecking x3, RCMOnly x3
#   24-28 RCMOnly x3 in V5 P20 V20 Poff Voff (tool 4): RCM error and tip vibration per mode
#   29-30 fits 2 and 3 (tools 5, 6), P5: SlowChecking x2 each
#   31-32 perturbed tool 4: +1 mm across the shaft (tool 7), +2 mm along it (tool 8), SlowChecking x2
#   33 YOU fit the C3 coupling + wand plate; C3 exchange redo (20 swaps; the script times the still periods)
#   34 hand-eye fit of the coupling mount -> tool 9, then 5x: YOU exchange, it replays RCMOnly with tool 9
#   35 settings back to final, default tool back to 2
#   Steps 20-32 run unattended except one check before the first wrist jog.
#
# Unattended needs the pendant in REMOTE CONTROL with the External Control program loaded: the
# script then presses Play itself over the dashboard port after each restart. In local mode it
# beeps and waits for you to press Play. Stay within reach of the e-stop either way.
#
# Usage:
#   ./scripts/run_v9c.sh                 # phase A from step 1
#   ./scripts/run_v9c.sh --phase=B       # phase B (wand) from step 20
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

FROM=""
ONLY=""
PHASE=A
DRY_RUN=false
for arg in "$@"; do
  case "$arg" in
    --from=*) FROM="${arg#*=}" ;;
    --only=*) ONLY="${arg#*=}" ;;
    --dry-run) DRY_RUN=true ;;
    --phase=*) PHASE="${arg#*=}" ;;
    -h|--help) sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
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

ask_continue() {
  local answer
  read -r -p "$1 [Enter = continue, q = stop here] " answer
  [ "$answer" = "q" ] && exit 0
  return 0
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
program_state() {
  if [ -n "$LAUNCH_EXTRA" ]; then   # mock hardware has no robot program
    grep -q "Touch teleop ready" "$STACK_LOG" 2>/dev/null && echo "running"
    return 0
  fi
  grep -E "Robot program (running|stopped)" "$STACK_LOG" 2>/dev/null | tail -1
}

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

# --- phase B: wand, NDI ------------------------------------------------------------------
NDI_DIR="${WS}/src/ndi_tracking"
NDI_PID=""
SHAFT_MM=158.6
R="${V9C}/RCM"
SLOW=V7/SlowCheckingRCMSimpleHolder
RCMONLY=V7/RCMOnlySimpleHolder:--rcm

ndi_running() { pgrep -f "ndi_tracking run" >/dev/null; }

ndi_stop() {
  if ndi_running; then
    echo ">> Stopping the NDI tracker"
    pkill -INT -f "ndi_tracking run"
    for _ in $(seq 1 20); do ndi_running || break; sleep 0.5; done
    ndi_running && pkill -KILL -f "ndi_tracking run"
    sleep 1
  fi
  NDI_PID=""
}

ndi_start() {
  ndi_running && return 0
  mkdir -p "$STACK_LOG_DIR"
  local log="${STACK_LOG_DIR}/$(date +%H%M%S)_ndi_run.log"
  echo ">> Starting the NDI tracker (log: ${log})"
  set -m
  (cd "$NDI_DIR" && exec python3 -m ndi_tracking run) >"$log" 2>&1 </dev/null &
  NDI_PID=$!
  set +m
  for _ in $(seq 1 30); do
    ros2 node list --no-daemon 2>/dev/null | grep -q "/ndi_tracking" && { echo ">> NDI tracker up"; return 0; }
    ndi_running || die "The NDI tracker exited. See ${log}"
    sleep 1
  done
  die "NDI tracker not on ROS after 30 s. See ${log}"
}

recorder() { ros2 service call "/trajectory_recorder/$1" std_srvs/srv/Trigger 2>&1 | tail -1; }

# trajectory_player.py <file>, stopped at once if External Control drops (11-06: a waiting
# trajectory jumps to catch up when Play is pressed again)
play_file() {
  local file="$1" out="${STACK_LOG_DIR}/$(date +%H%M%S)_player.log" pid
  set -m
  ros2 run macro_micro_robot trajectory_player.py "$file" --approach-time 5 --yes >"$out" 2>&1 </dev/null &
  pid=$!
  set +m
  while kill -0 "$pid" 2>/dev/null; do
    if program_state | grep -q stopped; then
      kill -INT -- "-${pid}" 2>/dev/null; sleep 2; kill -KILL -- "-${pid}" 2>/dev/null
      die "External Control stopped during ${file}. Don't press Play; resume with --phase=B --from=${STEP}."
    fi
    sleep 0.5
  done
  wait "$pid"
  tail -2 "$out"
  grep -q "Trajectory complete" "$out" || die "The motion did not finish (${out})."
}

# fit <tool number> <name>: one hand-eye calibration through the arm, added to tools.yaml
fit() {
  local num="$1" name="$2" jog_out jog_file dur tcp_log cal_out json
  say "--- Hand-eye fit -> tool ${num} (${name})"
  if $DRY_RUN; then echo "   wrist_jog.py; robot-tcp --wand-is-tool; wand_tool_calibration.py --tool 3; tool_library add ${num}"; return 0; fi
  ndi_stop   # robot-tcp needs the Vega to itself
  jog_out=$(python3 "${SCRIPT_DIR}/wrist_jog.py") || die "wrist_jog.py failed: ${jog_out}"
  echo "$jog_out" | head -3
  jog_file=$(echo "$jog_out" | sed -n 's/^Wrote //p')
  dur=$(echo "$jog_out" | sed -n 's/.*robot-tcp --wand-is-tool -d \([0-9.]*\).*/\1/p')
  [ -n "$jog_file" ] && [ -n "$dur" ] || die "Could not read wrist_jog.py output."
  tcp_log="${STACK_LOG_DIR}/$(date +%H%M%S)_robot_tcp_tool${num}.log"
  (cd "$NDI_DIR" && exec python3 -m ndi_tracking robot-tcp --wand-is-tool -d "$dur") >"$tcp_log" 2>&1 </dev/null &
  local tcp_pid=$!
  for _ in $(seq 1 30); do grep -q "recording" "$tcp_log" && break; kill -0 "$tcp_pid" 2>/dev/null || break; sleep 0.5; done
  grep -q "recording" "$tcp_log" || { kill "$tcp_pid" 2>/dev/null; die "robot-tcp did not start: ${tcp_log}"; }
  sleep 4
  play_file "$jog_file"
  echo ">> Waiting for robot-tcp to finish its ${dur} s"
  wait "$tcp_pid"
  grep -q "raw data saved" "$tcp_log" || die "robot-tcp saved nothing: ${tcp_log}"
  cal_out=$(python3 "${SCRIPT_DIR}/wand_tool_calibration.py" --tool 3 2>&1)
  echo "$cal_out" | grep -E "hand-eye|halves|tip in tool0|vs tool|NOT RECOMMENDED|saved"
  json=$(echo "$cal_out" | sed -n 's/^saved //p')
  [ -f "$json" ] || die "wand_tool_calibration.py wrote no JSON."
  if echo "$cal_out" | grep -q "NOT RECOMMENDED"; then
    note "fit ${num}: NOT RECOMMENDED ($(echo "$cal_out" | grep "NOT RECOMMENDED"))"
    [ "$num" = 4 ] && die "Fit 1 is not usable (see above): the RCM runs need it. Check the wand is in view, then --phase=B --from=21."
  fi
  if echo "$cal_out" | grep -q "NOT RECOMMENDED" && [ "$num" = 9 ]; then
    die "The coupling-mount fit is not usable (see above). Check the wand is in view, then --phase=B --from=34."
  fi
  ros2 run macro_micro_robot tool_library.py add "$num" "$name" --from "$json" --length "$SHAFT_MM" --force \
    --notes "V9c session D $(date +%F\ %H:%M)" || die "tool_library add failed"
  git -C "$PKG" add "$json" config/tools.yaml
  git -C "$PKG" commit -q -m "V9c: tool ${num} ${name} (hand-eye fit)" || die "commit failed"
  note "tool ${num} ${name}: $(echo "$cal_out" | grep "tip in tool0" | sed 's/^ *//')"
}

# perturbed copies of tool 4: across the shaft (perpendicular to direction, in its x-z plane) and along it
perturb_tools() {
  $DRY_RUN && { echo "   tools 7 (+1 mm across), 8 (+2 mm along) from tool 4"; return 0; }
  python3 - "$TOOLS" <<'EOF' >"${STACK_LOG_DIR}/perturb.txt" || die "Could not compute the perturbed tools"
import sys, yaml, numpy as np
t = yaml.safe_load(open(sys.argv[1]))["tools"][4]
tip, d = np.array(t["tip_tool0_mm"], float), np.array(t["direction_tool0"], float)
d /= np.linalg.norm(d)
x = np.array([1.0, 0, 0]); lat = x - x.dot(d) * d; lat /= np.linalg.norm(lat)
for v in (tip + 1.0 * lat, tip + 2.0 * d):
    print(" ".join(f"{c:.4f}" for c in v), " ".join(f"{c:.6f}" for c in d))
EOF
  local tip7 tip8 dir
  read -r -a tip7 < <(sed -n 1p "${STACK_LOG_DIR}/perturb.txt")
  read -r -a tip8 < <(sed -n 2p "${STACK_LOG_DIR}/perturb.txt")
  ros2 run macro_micro_robot tool_library.py add 7 "wand_D_plus1mm_lateral" --tip "${tip7[@]:0:3}" --direction "${tip7[@]:3:3}" \
    --length "$SHAFT_MM" --force --notes "tool 4 + 1.0 mm across the shaft (V9c T2d)" || die "tool_library add 7 failed"
  ros2 run macro_micro_robot tool_library.py add 8 "wand_D_plus2mm_axial" --tip "${tip8[@]:0:3}" --direction "${tip8[@]:3:3}" \
    --length "$SHAFT_MM" --force --notes "tool 4 + 2.0 mm along the shaft (V9c T2d control)" || die "tool_library add 8 failed"
  git -C "$PKG" commit -q -m "V9c: perturbed tools 7 (+1 mm lateral) and 8 (+2 mm axial) from tool 4" -- config/tools.yaml
  echo ">> tools 7, 8: $(cat "${STACK_LOG_DIR}/perturb.txt" | cut -d' ' -f1-3 | tr '\n' ' ')"
}

# cell_t <tool> <mode> <dest> <repeats> <run...>
cell_t() {
  local tool="$1"; shift
  $DRY_RUN || { stop_stack; set_tool "$tool"; }
  $DRY_RUN && echo "   tool ${tool}"
  cell "$@"
}

countdown() {
  local s="$1" msg="$2"
  while [ "$s" -gt 0 ]; do printf '\r   %s: %3d s ' "$msg" "$s"; sleep 1; s=$((s - 1)); done
  printf '\a\r   %s: done          \n' "$msg"
}

step_20() {
  say "=== Step 20: warm-up RCMOnly, tool 3, P5 -> ${V9C}/Warmup (checks the wand is seen at the RCM poses)"
  cell_t 3 p5 "${V9C}/Warmup" 1 "$RCMONLY"
}

step_21() {
  say "=== Step 21: three hand-eye fits, nothing touched in between"
  if ! $DRY_RUN; then
    stop_stack; set_mode p5; set_tool 3; start_stack   # fresh stack: RCM off after the warm-up
    printf '\a'
    ask_continue "Wrist jog next: wrists 1, 2, 3 swing +-30 deg one at a time (wand tip sweeps ~20 cm). Clear around the wand?"
  fi
  fit 4 wand_D_fit1
  fit 5 wand_D_fit2
  fit 6 wand_D_fit3
  $DRY_RUN || ndi_start
}

step_22() {
  say "=== Step 22: NDI static floor, 30 s -> ${V9C}/NDIStaticFloor"
  $DRY_RUN && return 0
  ndi_start
  stack_alive || { set_tool 4; start_stack; }
  check_new_run_free
  [ -e "${SESSION_DIR}/NDIStaticFloor" ] && die "${SESSION_DIR}/NDIStaticFloor exists"
  echo ">> Hands off the arm, wand and table"; sleep 5
  recorder start; sleep 30; recorder stop; sleep 2
  [ -d "${LOGS}/NEW_RUN" ] || die "Nothing recorded"
  mv "${LOGS}/NEW_RUN" "${SESSION_DIR}/NDIStaticFloor"
  summarise "${V9C}/NDIStaticFloor"
  note "NDI static floor recorded"
}

step_23() { $DRY_RUN || ndi_start; cell_t 4 p5   "$R/P5"   3 "$SLOW" "$RCMONLY"; }
step_24() { $DRY_RUN || ndi_start; cell_t 4 v5   "$R/V5"   3 "$RCMONLY"; }
step_25() { $DRY_RUN || ndi_start; cell_t 4 p20  "$R/P20"  3 "$RCMONLY"; }
step_26() { $DRY_RUN || ndi_start; cell_t 4 v20  "$R/V20"  3 "$RCMONLY"; }
step_27() { $DRY_RUN || ndi_start; cell_t 4 poff "$R/Poff" 3 "$RCMONLY"; }
step_28() { $DRY_RUN || ndi_start; cell_t 4 voff "$R/Voff" 3 "$RCMONLY"; }
step_29() { $DRY_RUN || ndi_start; cell_t 5 p5 "${V9C}/RCM_fit2" 2 "$SLOW"; }
step_30() { $DRY_RUN || ndi_start; cell_t 6 p5 "${V9C}/RCM_fit3" 2 "$SLOW"; }
step_31() { $DRY_RUN || ndi_start; perturb_tools; cell_t 7 p5 "${V9C}/RCM_perturb_lateral1" 2 "$SLOW"; }
step_32() { $DRY_RUN || ndi_start; cell_t 8 p5 "${V9C}/RCM_perturb_axial2" 2 "$SLOW"; }

exchange_set() {
  local name="$1" reply dir
  ndi_start
  reply=$(cd "$NDI_DIR" && python3 -m ndi_tracking ctl start "$name")
  echo ">> ${reply}"
  dir=$(echo "$reply" | sed -n 's/.*-> //p')
  [ -n "$dir" ] || die "The tracker did not start recording: ${reply}"
  countdown 60 "Seated and still, HANDS OFF the table"
  say "Now 20 exchanges: lever open, tool off and out of view, refit, lever fully closed, step back, >= 5 s seated."
  read -r -p "Press Enter when the 20th is seated and you've stepped back. " _
  countdown 60 "Seated and still, HANDS OFF the table"
  (cd "$NDI_DIR" && python3 -m ndi_tracking ctl stop) | tail -3
  sleep 2
  [ -d "$dir" ] || die "Recording folder ${dir} not found"
  [ -e "${LOGS}/Exchange/$(basename "$dir")" ] && die "logs/Exchange/$(basename "$dir") exists"
  mv "$dir" "${LOGS}/Exchange/"
  echo ">> Saved logs/Exchange/$(basename "$dir")"
  note "exchange set logs/Exchange/$(basename "$dir")"
}

step_33() {
  say "=== Step 33: C3 exchange redo (arm not commanded)"
  $DRY_RUN && { echo "   stop stack; NDI recording: 60 s still, 20 swaps, 60 s still -> logs/Exchange/<time>_QC_V3_redo"; return 0; }
  stop_stack
  printf '\a'
  ask_continue "Swap to the C3 COUPLING now: coupling on the flange, wand on its C3 tool plate, lever closed, in the camera's view. Nobody touches the table except to work the lever. Done?"
  exchange_set QC_V3_redo
  local a
  read -r -p "Also redo C2 now (swap to the C2 tool plate first, ~15 min)? [y/N] " a
  if [ "$a" = "y" ] || [ "$a" = "Y" ]; then
    ask_continue "C2 tool plate on, wand on it, in view?"
    exchange_set QC_V2_redo
    ask_continue "Put the C3 plate and wand back for step 34."
  fi
}

step_34() {
  say "=== Step 34: hand-eye fit of the coupling mount (tool 9), then exchange + RCMOnly, 5x -> ${V9C}/ExchangeRCM"
  $DRY_RUN && { echo "   fit 9 wand_D_C3_coupling; 5x: you exchange; replay_batch.sh --repeats=1 RCMOnly (tool 9, P5)"; return 0; }
  # The wand sits differently on the coupling than in the wand-only mount, so it needs its own calibration.
  # Fitted from where the arm stopped after step 32 (SlowChecking end pose, wand seen there); step 33 doesn't move it.
  ndi_start
  stop_stack; set_mode p5; set_tool 3; start_stack
  ask_continue "Wrist jog with the coupling fitted next (+-30 deg per wrist). Clear around the wand?"
  fit 9 wand_D_C3_coupling
  ndi_start
  stop_stack; set_tool 9; start_stack
  local i t0 t1
  for i in 1 2 3 4 5; do
    printf '\a'
    say "Exchange ${i}/5: lever open, tool off, refit, lever closed, step back. (Phone stopwatch too.)"
    t0=$(date +%s.%N)
    read -r -p "Press Enter the moment the lever is closed and you're clear. " _
    t1=$(date +%s.%N)
    note "exchange ${i}: $(python3 -c "print(f'{$t1 - $t0:.1f}')") s prompt-to-Enter"
    batch "${V9C}/ExchangeRCM" 1 "$RCMONLY"
  done
}

step_35() {
  say "=== Step 35: settings back to final, default tool 2"
  $DRY_RUN && return 0
  stop_stack
  set_mode final
  set_tool 2
  note "phase B done"
}

PHASE_B=(20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35)

# --- main --------------------------------------------------------------------------------
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${WS}/install/setup.bash"
set -u

steps=()
case "$PHASE" in
  A|a) PHASE_STEPS=("${PHASE_A[@]}"); FROM="${FROM:-1}" ;;
  B|b) PHASE_STEPS=("${PHASE_B[@]}"); FROM="${FROM:-20}" ;;
  *) die "--phase=A or --phase=B" ;;
esac
if [ -n "$ONLY" ] && [ "$ONLY" -ge 20 ]; then PHASE_STEPS=("${PHASE_B[@]}"); PHASE=B; fi
if [ -n "$FROM" ] && [ "$FROM" -ge 20 ]; then PHASE_STEPS=("${PHASE_B[@]}"); PHASE=B; fi
for n in "${PHASE_STEPS[@]}"; do
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

trap 'stop_stack; [ -n "$NDI_PID" ] && ndi_stop; "${SCRIPT_DIR}/set_v9_mode.sh" final >/dev/null 2>&1 || true' EXIT
trap 'echo; echo ">> Interrupted"; note "interrupted (Ctrl+C)"; exit 130' INT TERM

STEP="${steps[0]}"
if [ -z "$LAUNCH_EXTRA" ]; then
  timeout 3 bash -c "</dev/tcp/${ROBOT_IP}/30001" 2>/dev/null || die "Can't reach the arm at ${ROBOT_IP}: check the cable and that it is powered on."
fi
[ -z "$(git -C "$PKG" status --porcelain --untracked-files=no | grep -v -E 'macro_micro_robot/teleop_settings.py|config/tools.yaml' || true)" ] \
  || die "Uncommitted changes: commit them first, or code_version says -dirty."
check_new_run_free
if [ "$PHASE" = B ]; then
  NDI_OPT=""
else
  NDI_OPT=--no-ndi
  set_tool 3
fi

if [ -z "$LAUNCH_EXTRA" ]; then
  if remote_control; then
    echo ">> Pendant is in Remote Control: Play is pressed automatically."
  else
    echo ">> Pendant is in LOCAL control: you'll have to press Play after each of ${#steps[@]} restarts."
    echo "   For unattended runs: pendant top-right -> Remote Control (External Control program loaded)."
  fi
fi
if [ "$PHASE" = B ]; then
  say "Phase B, steps ${steps[*]}: WAND-ONLY mount (SimpleHolder on the flange, no coupling; steps 33-34 ask for the coupling later), camera placed and not touched again, workspace clear."
  say "The script starts and stops the NDI tracker itself: close any 'ndi_tracking run' you have open."
  read -r -p "Wand on, camera fixed, robot clear, start? [y/N] " answer
else
  say "Phase A, steps ${steps[*]}: BARE FLANGE, nothing on the tool flange, workspace clear."
  say "About $(( ${#steps[@]} * 7 )) min. The arm moves without asking from here on. Stay near the e-stop."
  read -r -p "Bare flange, robot clear, start? [y/N] " answer
fi
[ "$answer" = "y" ] || [ "$answer" = "Y" ] || exit 1
note "phase ${PHASE} start, steps ${steps[*]}"

for STEP in "${steps[@]}"; do
  "step_${STEP}"
done
stop_stack
note "phase ${PHASE} done"
if [ "$PHASE" = B ]; then
  say "Phase B done. Back up logs/V9c and logs/Exchange, and tell Claude the runs are in."
else
  say "Phase A done (settings go back to final on exit). Wand on, then: ./scripts/run_v9c.sh --phase=B"
fi
