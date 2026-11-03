#!/usr/bin/env bash
#
# set_v9_mode.sh - switch teleop_settings.py between the V9 comparison modes and commit,
# so every recording's code_version names the mode it ran in. Restart the stack after.
#
#   ./scripts/set_v9_mode.sh trajectory | position | velocity-unfiltered | velocity-optimised | final
#
# All V9 modes keep TOUCH_SCALE_ROTATION_EXPONENT = 0.5 (as on 2026-10-26), so the 1:5 runs compare.
#   trajectory           SERVO_OUTPUT trajectory, stock speedj, filter 5 Hz, Kalman on
#   position             SERVO_OUTPUT position,   stock speedj, filter 5 Hz, Kalman on
#   velocity-unfiltered  SERVO_OUTPUT velocity,   stock speedj, filter off,  Kalman off  (1:5 runs only!)
#   velocity-optimised   SERVO_OUTPUT velocity,   smooth speedj, filter 5 Hz, Kalman on
#   final                the defaults after V9: position, smooth speedj, filter 5 Hz, Kalman on, exponent 1.0
#
# Refuses while a replay is running or logs/NEW_RUN exists (a commit mid-run would mislabel it).

set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETTINGS="${PKG}/macro_micro_robot/teleop_settings.py"
MODE="${1:-}"

case "$MODE" in
  trajectory)          OUT=trajectory; SMOOTH=False; FILTER=5.0; KALMAN=True;  EXP=0.5 ;;
  position)            OUT=position;   SMOOTH=False; FILTER=5.0; KALMAN=True;  EXP=0.5 ;;
  velocity-unfiltered) OUT=velocity;   SMOOTH=False; FILTER=0.0; KALMAN=False; EXP=0.5 ;;
  velocity-optimised)  OUT=velocity;   SMOOTH=True;  FILTER=5.0; KALMAN=True;  EXP=0.5 ;;
  final)               OUT=position;   SMOOTH=True;  FILTER=5.0; KALMAN=True;  EXP=1.0 ;;
  *) sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac

if pgrep -f "stylus_replay.py|replay_batch.sh" >/dev/null; then
  echo "A replay is running - wait for it to finish." >&2; exit 1
fi
if [ -e "${PKG}/logs/NEW_RUN" ]; then
  echo "logs/NEW_RUN exists - move it into a version folder first." >&2; exit 1
fi
if [ -n "$(git -C "$PKG" status --porcelain --untracked-files=no | grep -v 'macro_micro_robot/teleop_settings.py' || true)" ]; then
  echo "Uncommitted changes besides teleop_settings.py - commit or stash them first." >&2; exit 1
fi

python3 - "$SETTINGS" "$OUT" "$SMOOTH" "$FILTER" "$KALMAN" "$EXP" <<'EOF'
import re, sys
path, out, smooth, filt, kalman, exp = sys.argv[1:]
text = open(path).read()
for name, value in (("SERVO_OUTPUT", f'"{out}"'), ("SERVO_SMOOTH_SPEEDJ", smooth), ("TOUCH_COMMAND_FILTER", filt),
                    ("TOUCH_KALMAN", kalman), ("TOUCH_SCALE_ROTATION_EXPONENT", exp)):
    text, n = re.subn(rf"^{name} = .*$", f"{name} = {value}", text, flags=re.M)
    if n != 1:
        sys.exit(f"{name}: expected one line in {path}, found {n}")
open(path, "w").write(text)
EOF

grep -E "^(SERVO_OUTPUT|SERVO_SMOOTH_SPEEDJ|TOUCH_COMMAND_FILTER|TOUCH_KALMAN|TOUCH_SCALE_ROTATION_EXPONENT) =" "$SETTINGS"
if git -C "$PKG" diff --quiet -- "$SETTINGS"; then
  echo "Already in mode '${MODE}' - nothing to commit."
else
  git -C "$PKG" commit -q -m "V9 mode: ${MODE}" -- "$SETTINGS"
  echo "Committed: $(git -C "$PKG" log --oneline -1)"
fi
echo ">> Now restart the stack (Ctrl+C in its terminal, check 'pgrep -fa ros2_control_node' is empty, relaunch)."
