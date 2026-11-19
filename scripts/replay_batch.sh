#!/usr/bin/env bash
#
# replay_batch.sh - replay several logged runs back to back with stylus_replay.py,
# moving each recording out of logs/NEW_RUN before the next one deletes it.
#
# Needs, in other terminals: the stack with input:=replay, and the NDI tracker
# (python3 -m ndi_tracking run). Every replay must come out with NDI data: the batch
# stops at the first one without it (LOG_REQUIRE_NDI is off, so nothing else would).
#
# Usage:
#   ./scripts/replay_batch.sh [--dest=V8] [--repeats=3] [--no-ndi] [--yes] [run[:--rcm] ...]
#
#   --no-ndi for runs with no markers on the tool: skips the tracker checks
#   --yes    skip the "robot clear?" question (for run_v9c.sh, which asks once per session)
#   run      folder relative to logs/, e.g. V7/FastCheckingRCMSimpleHolder;
#            append :--rcm for runs recorded with RCM mode already on
#   default  the four V7 SimpleHolder originals (as replayed on 2026-10-26)
#
# Each replay goes to logs/<dest>/<run name>_Replay<n> (n = next free number).
# The robot moves to each run's start pose without asking.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS="$(cd "${SCRIPT_DIR}/../logs" && pwd)"
DEST="V8"
REPEATS=3
REQUIRE_NDI=true
ASSUME_YES=false
RUNS=()

for arg in "$@"; do
  case "$arg" in
    --dest=*) DEST="${arg#*=}" ;;
    --repeats=*) REPEATS="${arg#*=}" ;;
    --no-ndi) REQUIRE_NDI=false ;;
    --yes) ASSUME_YES=true ;;
    -h|--help) sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "Unknown option: $arg" >&2; exit 1 ;;
    *) RUNS+=("$arg") ;;
  esac
done
if [ "${#RUNS[@]}" -eq 0 ]; then
  # RCMOnly last: at 1:5 it ends with the tool in and RCM mode on, which can only be left
  # back at its own RCM point, so a different run after it refuses to start (2026-11-03)
  RUNS=(V7/FastCheckingRCMSimpleHolder V7/SlowCheckingRCMSimpleHolder
        V7/WholeRunDrinkBottleSimpleHolder V7/RCMOnlySimpleHolder:--rcm)
fi

set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${SCRIPT_DIR}/../../../install/setup.bash"
set -u

# --- checks before anything moves
if [ -e "${LOGS}/NEW_RUN" ]; then
  echo "logs/NEW_RUN exists and the first replay would delete it. Rename it first, e.g.:" >&2
  echo "  mv ${LOGS}/NEW_RUN ${LOGS}/<name>" >&2
  exit 1
fi
for spec in "${RUNS[@]}"; do
  run="${spec%%:*}"
  [ -f "${LOGS}/${run}/STYLUS_RAW.csv" ] || { echo "No STYLUS_RAW.csv in logs/${run}" >&2; exit 1; }
done
if $REQUIRE_NDI && ! ros2 node list 2>/dev/null | grep -q "/ndi_tracking"; then
  echo "NDI tracker not seen on ROS (domain ${ROS_DOMAIN_ID:-0}). Start it first:" >&2
  echo "  cd ~/ros2_ws/src/ndi_tracking && python3 -m ndi_tracking run" >&2
  exit 1
fi
mkdir -p "${LOGS}/${DEST}"

echo "Replaying ${#RUNS[@]} run(s) x ${REPEATS} into logs/${DEST}/ - the robot will move without asking."
if ! $ASSUME_YES; then
  read -r -p "Robot clear and External Control running? [y/N] " answer
  [ "$answer" = "y" ] || [ "$answer" = "Y" ] || exit 1
fi

done_list=()
for spec in "${RUNS[@]}"; do
  run="${spec%%:*}"
  flags=()
  [ "$spec" != "$run" ] && flags=("${spec#*:}")
  name="$(basename "$run")"
  for ((i = 1; i <= REPEATS; i++)); do
    n=1
    while [ -e "${LOGS}/${DEST}/${name}_Replay${n}" ]; do n=$((n + 1)); done
    target="${LOGS}/${DEST}/${name}_Replay${n}"
    echo
    echo "=== ${run} ${flags[*]:-} -> ${DEST}/${name}_Replay${n}  ($i/${REPEATS})"
    ros2 run macro_micro_robot stylus_replay.py "$run" --yes "${flags[@]}" || true
    # REPLAY.yaml is only written once the recorder has stopped and saved
    if [ ! -f "${LOGS}/NEW_RUN/REPLAY.yaml" ]; then
      echo "!!! The replay did not finish with a saved recording - stopping the batch." >&2
      break 2
    fi
    # NDI_PATH.csv has a row per frame even when nothing is tracked (the 11-03 Scaled5_Slow
    # replay had 0 % visible): check the visible column, not just that the file has data
    visible_pct=0
    if [ -s "${LOGS}/NEW_RUN/NDI_PATH.csv" ]; then
      visible_pct=$(awk -F, 'NR > 1 { n++; v += $4 } END { print (n ? int(100 * v / n) : 0) }' "${LOGS}/NEW_RUN/NDI_PATH.csv")
    fi
    if $REQUIRE_NDI && [ "$visible_pct" -lt 95 ]; then
      # a free _NO_NDI name: mv into an existing one would nest NEW_RUN inside it (11-19)
      m=$n
      while [ -e "${LOGS}/${DEST}/${name}_Replay${m}_NO_NDI" ]; do m=$((m + 1)); done
      mv "${LOGS}/NEW_RUN" "${LOGS}/${DEST}/${name}_Replay${m}_NO_NDI"
      echo "!!! Wand tracked for only ${visible_pct} % of this replay (kept as ${DEST}/${name}_Replay${m}_NO_NDI) - stopping the batch." >&2
      echo "    Check the tracker sees the wand, then run the batch again." >&2
      break 2
    fi
    mv "${LOGS}/NEW_RUN" "$target"
    done_list+=("${DEST}/${name}_Replay${n}")
  done
done

echo
echo "Saved ${#done_list[@]} replay(s):"
printf '  %s\n' "${done_list[@]}"
