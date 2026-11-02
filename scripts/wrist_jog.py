#!/usr/bin/env python3
"""Write the wrist-jog motion for the wand hand-eye calibration as a trajectory_player file.

wand_tool_calibration.py needs a `robot-tcp --wand-is-tool` recording with wrist 1, 2 and
3 each rotated on their own, and the whole sequence twice (each half must contain all
three rotations for its repeatability check). Jogging by hand on the pendant doesn't
repeat and needs External Control stopped. This writes the motion instead, from the
arm's current joint angles:

    python3 scripts/wrist_jog.py                      # +-30 deg, 14 s per wrist -> 90 s of motion
    ros2 run macro_micro_robot trajectory_player.py <printed file>

- Each wrist follows angle(t) = A sin(2 pi t / T): out to +A, back through the start to
  -A, back to the start. Smooth at both ends, peak speed A * 2 pi / T (+-30 deg, 14 s:
  13 deg/s). Only one wrist moves at a time, then a PAUSE at rest.
- The first sample is the current pose, so the player's approach move goes nowhere.
- Nothing moves here. trajectory_player.py asks before moving, pauses Servo, uses
  scaled_joint_trajectory_controller (so the pendant speed slider still scales it) and
  hands the arm back to the teleop afterwards. Ctrl+C there cancels the motion.
- Written to ~/ros2_ws/trajectories/wrist_jog_<time>.yaml. The name doesn't start with
  "trajectory_", so `trajectory_player.py latest` still means the newest teleop recording.

Before playing: check the wand clears everything over the full +-A on wrist 1 and 2
(the 37 cm wand tip sweeps ~20 cm at 30 deg) and stays in the Vega's view.
"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from sensor_msgs.msg import JointState

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
WRISTS = [3, 4, 5]  # indices into JOINTS
SAMPLE_RATE = 50.0  # Hz; the player keeps points >= PLAYER_MIN_POINT_SPACING (0.04 s) apart
PAUSE = 1.0         # s at rest after each wrist
MAX_AMPLITUDE_DEG = 45.0
MAX_PEAK_SPEED_DEG = 30.0
OUT_DIR = os.path.expanduser("~/ros2_ws/trajectories")


def current_joints(timeout=5.0):
    """Latest /joint_states as positions in JOINTS order."""
    rclpy.init()
    node = rclpy.create_node("wrist_jog")
    latest = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: latest.update(zip(m.name, m.position)), 10)
    end = time.time() + timeout
    while time.time() < end and not all(j in latest for j in JOINTS):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not all(j in latest for j in JOINTS):
        sys.exit("No /joint_states with all six UR joints: is the robot driver running "
                 "(same ROS_DOMAIN_ID)?")
    return [latest[j] for j in JOINTS]


def build_samples(start, amplitude, period, passes):
    samples = []
    t = 0.0
    dt = 1.0 / SAMPLE_RATE

    def add(positions):
        samples.append({"time": round(t, 4), "positions": [float(p) for p in positions],
                        "ee_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]})

    add(start)
    for _ in range(passes):
        for w in WRISTS:
            for k in range(1, int(round(period * SAMPLE_RATE)) + 1):
                t += dt
                q = list(start)
                q[w] += amplitude * math.sin(2 * math.pi * k * dt / period)
                add(q)
            t += PAUSE
            add(start)
    return samples, t


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--amplitude", type=float, default=30.0, help="deg each way (default 30)")
    ap.add_argument("--period", type=float, default=14.0, help="s per wrist, out and back both ways (default 14)")
    ap.add_argument("--passes", type=int, default=2, help="times through wrists 1-2-3 (default 2)")
    args = ap.parse_args()

    peak = args.amplitude * 2 * math.pi / args.period
    if not 0 < args.amplitude <= MAX_AMPLITUDE_DEG:
        sys.exit(f"--amplitude must be in (0, {MAX_AMPLITUDE_DEG:g}] deg")
    if peak > MAX_PEAK_SPEED_DEG:
        sys.exit(f"peak wrist speed {peak:.0f} deg/s > {MAX_PEAK_SPEED_DEG:g}: use a longer --period")
    if args.passes < 2:
        print("Warning: with one pass wand_tool_calibration.py can't check the two halves agree")

    start = current_joints()
    samples, duration = build_samples(start, math.radians(args.amplitude), args.period, args.passes)
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, time.strftime("wrist_jog_%Y%m%d_%H%M%S.yaml"))
    with open(path, "w") as f:
        yaml.safe_dump({"joint_names": JOINTS, "duration": round(duration, 3), "base_frame": "base_link",
                        "notes": f"scripts/wrist_jog.py: wrists 1-2-3 one at a time, +-{args.amplitude:g} deg, "
                                 f"{args.period:g} s each, {args.passes} passes, from the pose at "
                                 f"{time.strftime('%H:%M:%S')}",
                        "samples": samples}, f, default_flow_style=None, sort_keys=False)

    print("Start pose (deg): " + ", ".join(f"{math.degrees(q):.1f}" for q in start))
    print(f"Wrists 1-2-3 one at a time, +-{args.amplitude:g} deg, peak {peak:.0f} deg/s, "
          f"{args.passes} passes: {duration:.0f} s of motion (+5 s player approach).")
    print(f"Wrote {path}")
    # Margin for the 3 s robot-tcp lead-in, answering the player prompt and its 5 s approach
    record = math.ceil(duration + 30)
    print("\nNext (the recording first, it waits 3 s before it starts):")
    print(f"  cd ~/ros2_ws/src/ndi_tracking && python3 -m ndi_tracking robot-tcp --wand-is-tool -d {record}")
    print(f"  ros2 run macro_micro_robot trajectory_player.py {path}")
    print("Don't move the arm before playing: the file starts from where it is now.")


if __name__ == "__main__":
    main()
