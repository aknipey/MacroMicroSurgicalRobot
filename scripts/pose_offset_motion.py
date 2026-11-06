#!/usr/bin/env python3
"""Write the pose-offset test motion (RESULTS.md, E0) as a trajectory_player file.

Does the NDI-vs-calibrated-FK offset follow the pose, and does it change with the camera?
This moves in joint space between two poses of the 11-03 SlowChecking replay, with a still
hold at each, so run_v9b.sh can record it once per camera position:

    python3 scripts/pose_offset_motion.py        # prints the file it wrote
    ros2 run macro_micro_robot trajectory_player.py <file> --approach-time 15

- LATCH is the still period just after the RCM latch (the ~2.5 mm offset region), WORK the
  working region (offset ~0.3 mm). Both come from the same run with the same tool, tip
  >= 190 mm above the base plane.
- Minimum-jerk moves, peak speed ~8 deg/s on the fastest joint (wrist 3, 21 deg in 5 s).
  Holds are the numbers to compare: skip the first second of each, the arm is settling.
- Nothing moves here. trajectory_player.py asks before moving.
- Written to ~/ros2_ws/trajectories/pose_offset_<time>.yaml.
"""
import argparse
import os
import time

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = os.path.join(PKG, "logs/V9/PositionMode/SlowCheckingRCMSimpleHolder_Replay1/TRAJECTORY.yaml")
LATCH_TIME = 14.0   # s into SOURCE
WORK_TIME = 30.0
SAMPLE_RATE = 25.0  # Hz; = PLAYER_MIN_POINT_SPACING
OUT_DIR = os.path.expanduser("~/ros2_ws/trajectories")


def sample_at(data, t):
    return min(data["samples"], key=lambda s: abs(s["time"] - t))["positions"]


def build_samples(a, b, move, hold, cycles):
    samples = []
    t = 0.0
    dt = 1.0 / SAMPLE_RATE

    def add(positions):
        samples.append({"time": round(t, 4), "positions": [float(p) for p in positions],
                        "ee_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]})

    add(a)
    for _ in range(cycles):
        for start, end in ((a, b), (b, a)):
            t += hold
            add(start)
            for k in range(1, int(round(move * SAMPLE_RATE)) + 1):
                t += dt
                s = k * dt / move
                blend = 10 * s**3 - 15 * s**4 + 6 * s**5
                add([p + blend * (q - p) for p, q in zip(start, end)])
    t += hold
    add(a)
    return samples, t


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--move", type=float, default=5.0, help="s per move (default 5)")
    ap.add_argument("--hold", type=float, default=4.0, help="s still at each pose (default 4)")
    ap.add_argument("--cycles", type=int, default=6, help="there-and-back cycles (default 6)")
    args = ap.parse_args()

    with open(SOURCE) as f:
        data = yaml.safe_load(f)
    a, b = sample_at(data, LATCH_TIME), sample_at(data, WORK_TIME)
    samples, duration = build_samples(a, b, args.move, args.hold, args.cycles)

    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, time.strftime("pose_offset_%Y%m%d_%H%M%S.yaml"))
    with open(path, "w") as f:
        yaml.safe_dump({"joint_names": data["joint_names"], "duration": round(duration, 3),
                        "base_frame": "base_link",
                        "notes": f"scripts/pose_offset_motion.py: {SOURCE} t={LATCH_TIME:g} <-> t={WORK_TIME:g}, "
                                 f"move {args.move:g} s, hold {args.hold:g} s, {args.cycles} cycles",
                        "samples": samples}, f, default_flow_style=None, sort_keys=False)
    print(path)


if __name__ == "__main__":
    main()
