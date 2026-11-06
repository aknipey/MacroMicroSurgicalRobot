#!/usr/bin/env python3
"""Write a scripted stylus run that drives the arm into the wrist singularity and back (E6).

stylus_replay.py plays it like a logged run, so guard on and guard off get exactly the same
input, with no Touch and no hand in the loop:

    python3 scripts/singularity_input.py        # writes logs/V9b/SingularityInput
    ros2 run macro_micro_robot stylus_replay.py V9b/SingularityInput

- Start pose: wrist 2 at -20 deg, flange ~750 mm above the base plane, c ~12. A straight
  tool move of ~180 mm with the orientation held takes wrist 2 to 0 (c -> infinity). Only
  pan and wrist 2 move by more than 5 deg.
- The stylus goes in, holds, and comes back along the same line. Minimum-jerk strokes, peak tool
  speed ~35 mm/s. So the run shows the approach, the hold at the limit, and whether the arm
  can leave.
- The stylus commands STROKE mm, past the singularity on purpose: the guard should stop short of it.
"""
import csv
import math
import os

import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PKG, "logs/V9b/SingularityInput")
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
START_DEG = [-3.0, -70.0, 70.0, -180.0, -20.0, 0.0]
# UR16e D-H (nominal), base frame as in the arm's own controller
D = [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655]
A = [0.0, -0.4784, -0.36, 0.0, 0.0, 0.0]
ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
STROKE = 220.0      # mm
MOVE = 12.0         # s per stroke
HOLD = 3.0          # s at the far end
LEAD = 1.0          # s still, WHITE up, then WHITE down for this long before moving
RATE = 500.0        # Hz
QUATERNION = [0.342626, 0.155934, 0.801200, -0.465157]  # stylus pose from V7/SlowChecking, held still
STYLUS_START = [0.0, 0.0, 0.0]


def jacobian(q):
    T, frames = np.eye(4), [np.eye(4)]
    for i in range(6):
        c, s, ca, sa = math.cos(q[i]), math.sin(q[i]), math.cos(ALPHA[i]), math.sin(ALPHA[i])
        T = T @ np.array([[c, -s * ca, s * sa, A[i] * c], [s, c * ca, -c * sa, A[i] * s],
                          [0, sa, ca, D[i]], [0, 0, 0, 1]])
        frames.append(T)
    tip = frames[-1][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(frames[i][:3, 2], tip - frames[i][:3, 3])
        J[3:, i] = frames[i][:3, 2]
    return J, tip


def stylus_direction(q):
    """Unit stylus (right, towards the Touch body, up) that takes wrist 2 towards 0."""
    J, tip = jacobian(q)
    row = np.linalg.inv(J)[4, :3]
    tool = -math.copysign(1.0, q[4]) * row / np.linalg.norm(row)
    # ROS base_link is the controller's base turned 180 deg about z
    flip = np.diag([-1.0, -1.0, 1.0])
    tool, tip = flip @ tool, flip @ tip
    # As toward_base_matrix() in ur16e_touch_teleop.py
    forward = -np.array([tip[0], tip[1], 0.0])
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, up)
    return np.array([right, forward, up]) @ tool


def blend(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def main():
    q = np.radians(START_DEG)
    direction = stylus_direction(q)
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "TRAJECTORY.yaml"), "w") as f:
        yaml.safe_dump({"joint_names": JOINTS, "samples": [{"time": 0.0, "positions": [float(p) for p in q]}]}, f)

    total = 2 * LEAD + 2 * MOVE + HOLD
    with open(os.path.join(OUT, "STYLUS_RAW.csv"), "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["t", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "white", "grey"])
        for k in range(int(total * RATE) + 1):
            t = k / RATE
            travel = t - 2 * LEAD
            if travel < 0:
                s = 0.0
            elif travel < MOVE:
                s = blend(travel / MOVE)
            elif travel < MOVE + HOLD:
                s = 1.0
            else:
                s = blend(max(0.0, 1.0 - (travel - MOVE - HOLD) / MOVE))
            p = np.array(STYLUS_START) + STROKE * s * direction
            out.writerow([f"{t:.4f}", f"{t:.4f}", *(f"{v:.5f}" for v in p), *QUATERNION, 0.0, 0.0, 0.0,
                          int(t >= LEAD), 0])
    print(OUT)
    print(f"Stylus direction {np.round(direction, 3)}, {STROKE:g} mm, {total:.0f} s")


if __name__ == "__main__":
    main()
