#!/usr/bin/env bash
#
# rerun_v9c_gaps.sh - top up the phase A grid cells whose replays have a recording gap.
#
# Scans logs/V9c/NoTool/<cell>/*_Replay* for a gap > 50 ms in UR_PATH.csv or STYLUS_RAW.csv (11-19: ~335 ms stalls,
# 1.6-3.3 s into some recordings; session C had none). For each cell and input it counts the CLEAN
# replays and records new ones until there are 4 again (2 for P20_1to1), into the same folder (replay
# numbers carry on, gapped ones are kept and listed in the session log). Each new replay is checked at
# once and retried, up to 3 attempts. Cells that share a mode share one stack restart.
#
# Same as run_v9c.sh otherwise: bare flange, no NDI, tool 3, Play pressed automatically in Remote
# Control (else it beeps and waits). Uses run_v9c.sh's functions.
#
#   ./scripts/rerun_v9c_gaps.sh            # scan, show the plan, ask, run
#   ./scripts/rerun_v9c_gaps.sh --dry-run  # scan and show the plan only
#   ./scripts/rerun_v9c_gaps.sh --scan     # list every gapped replay and stop

set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE_ARG="${1:-}"

# run_v9c.sh's settings and functions, without its main part
set --
# shellcheck disable=SC1090
source <(sed -e '/^# --- main/,$d' -e "s#^SCRIPT_DIR=.*#SCRIPT_DIR=\"${HERE}\"#" "${HERE}/run_v9c.sh")
STEP="redo"
GAP_MS=50
MAX_TRIES=3

declare -A CELL_MODE=([P5]=p5 [P5_end]=p5 [V5]=v5 [P10]=p10 [V10]=v10 [P20]=p20 [V20]=v20
                      [Poff]=poff [Voff]=voff [P20_1to1]=p20)
declare -A CELL_TARGET=([P20_1to1]=2)
declare -A INPUT=([Scaled5_Fast]=V3/Scaled5_Fast [Scaled5_Slow]=V3/Scaled5_Slow [LongBenchmark]=V3/LongBenchmark
                  [ShortBenchmark]=V3/ShortBenchmark [NoBuzzArea]=V7/NoBuzzArea [LargeVibrations]=V7/LargeVibrations)

# max gap (ms) in a replay's UR_PATH.csv, from the recorder's own time column
max_gap() {
  python3 - "$1" <<'EOF'
import sys, numpy as np
g = 0
for f in ('UR_PATH.csv', 'STYLUS_RAW.csv'):
    t = np.loadtxt(sys.argv[1] + '/' + f, delimiter=',', skiprows=1, usecols=0)
    g = max(g, 1000 * np.diff(t).max() if len(t) > 1 else 99999)
print(int(round(g)))
EOF
}

# Lines "mode cell input needed gapped_list" for every cell/input short of clean replays
plan() {
  python3 - "${LOGS}/${V9C}/NoTool" "$GAP_MS" "$1" <<'EOF'
import glob, os, re, sys, numpy as np
root, gap_ms, show_all = sys.argv[1], float(sys.argv[2]), sys.argv[3] == 'scan'
mode = {'P5': 'p5', 'P5_end': 'p5', 'V5': 'v5', 'P10': 'p10', 'V10': 'v10', 'P20': 'p20', 'V20': 'v20',
        'Poff': 'poff', 'Voff': 'voff', 'P20_1to1': 'p20'}
target = {'P20_1to1': 2}
rows = {}
for path in sorted(glob.glob(f'{root}/*/*_Replay[0-9]*')):
    if path.endswith('_NO_NDI') or not os.path.exists(f'{path}/UR_PATH.csv'):
        continue
    cell, name = path.split('/')[-2], os.path.basename(path)
    if cell not in mode:
        continue
    inp = re.sub(r'_Replay\d+$', '', name)
    g = 0
    for f in ('UR_PATH.csv', 'STYLUS_RAW.csv'):   # arm path and replayed input (TOUCH_PATH breaks are in the input)
        t = np.loadtxt(f'{path}/{f}', delimiter=',', skiprows=1, usecols=0)
        g = max(g, 1000 * np.diff(t).max() if len(t) > 1 else 1e9)
    r = rows.setdefault((cell, inp), {'clean': 0, 'gapped': []})
    if g > gap_ms:
        r['gapped'].append(f'{name}({g:.0f}ms)')
        if show_all:
            print(f'  {cell}/{name}: {g:.0f} ms')
    else:
        r['clean'] += 1
if show_all:
    sys.exit(0)
order = ['p5', 'v5', 'p10', 'v10', 'p20', 'v20', 'poff', 'voff']
for (cell, inp), r in sorted(rows.items(), key=lambda kv: (order.index(mode[kv[0][0]]), kv[0])):
    need = target.get(cell, 4) - r['clean']
    if need > 0:
        print(mode[cell], cell, inp, need, ','.join(r['gapped']) or '-')
EOF
}

newest_replay() {
  (cd "$LOGS" && ls -1dt "${V9C}/NoTool/$1/$2"_Replay[0-9]* 2>/dev/null | head -1)
}

# --- main ----------------------------------------------------------------------------------
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${WS}/install/setup.bash"
set -u

if [ "$MODE_ARG" = "--scan" ]; then
  echo "Replays with a gap > ${GAP_MS} ms:"
  plan scan
  exit 0
fi

mapfile -t TODO < <(plan run)
if [ "${#TODO[@]}" -eq 0 ]; then
  say "Nothing to redo: every cell has its full count of clean replays."
  exit 0
fi
say "To redo (mode, cell, input, new replays needed, gapped ones kept):"
total=0
for line in "${TODO[@]}"; do
  read -r m c i n g <<<"$line"
  printf '  %-5s %-9s %-16s %d   %s\n' "$m" "$c" "$i" "$n" "$g"
  total=$((total + n))
done
modes=$(printf '%s\n' "${TODO[@]}" | awk '{print $1}' | uniq | wc -l)
say "${total} replays in ${modes} mode(s): about $(( modes * 2 + total / 2 + 1 )) min."
[ "$MODE_ARG" = "--dry-run" ] && exit 0

trap 'stop_stack; "${SCRIPT_DIR}/set_v9_mode.sh" final >/dev/null 2>&1 || true' EXIT
trap 'echo; echo ">> Interrupted"; note "redo interrupted (Ctrl+C)"; exit 130' INT TERM

pgrep -f "run_v9c.sh" >/dev/null && die "run_v9c.sh is still running: wait for phase A to finish."
timeout 3 bash -c "</dev/tcp/${ROBOT_IP}/30001" 2>/dev/null || die "Can't reach the arm at ${ROBOT_IP}."
[ -z "$(git -C "$PKG" status --porcelain --untracked-files=no | grep -v -E 'macro_micro_robot/teleop_settings.py|config/tools.yaml' || true)" ] \
  || die "Uncommitted changes: commit them first, or code_version says -dirty."
check_new_run_free
NDI_OPT=--no-ndi
set_tool 3
if remote_control; then echo ">> Pendant is in Remote Control: Play is pressed automatically."
else echo ">> Pendant is in LOCAL control: press Play after each of the ${modes} restarts."; fi
read -r -p "Bare flange, robot clear, start? [y/N] " answer
[ "$answer" = "y" ] || [ "$answer" = "Y" ] || exit 1
note "redo start: ${total} replays in ${modes} mode(s)"
for line in "${TODO[@]}"; do
  read -r _ c i _ g <<<"$line"
  [ "$g" != "-" ] && note "gapped (kept, exclude in analysis): ${c}: ${g}"
done

current=""
failed=()
for line in "${TODO[@]}"; do
  read -r m c i n _ <<<"$line"
  if [ "$m" != "$current" ]; then
    say "=== mode ${m}"
    stop_stack
    set_mode "$m"
    start_stack
    current="$m"
  fi
  for ((k = 1; k <= n; k++)); do
    ok=false
    for ((try = 1; try <= MAX_TRIES; try++)); do
      batch "${V9C}/NoTool/${c}" 1 "${INPUT[$i]}"
      r=$(newest_replay "$c" "$i")
      g=$(max_gap "${LOGS}/${r}")
      if [ "$g" -le "$GAP_MS" ]; then
        echo ">> ${r}: clean (max gap ${g} ms)"
        ok=true
        break
      fi
      echo "!!! ${r}: gap ${g} ms again (attempt ${try}/${MAX_TRIES})"
      note "redo gapped again (kept, exclude): ${r} ${g} ms"
    done
    $ok || failed+=("${c}/${i}")
  done
done
stop_stack
if [ "${#failed[@]}" -gt 0 ]; then
  note "redo done; still short of a clean replay: ${failed[*]}"
  say "Done, but still no clean replay after ${MAX_TRIES} tries for: ${failed[*]}"
else
  note "redo done: every cell back to its full clean count"
  say "Done: every cell has its full count of clean replays. Settings go back to final on exit."
fi
