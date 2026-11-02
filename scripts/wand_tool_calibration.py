#!/usr/bin/env python3
"""Turn a divot-less `robot-tcp --wand-is-tool` recording into a tool-library calibration.

    cd ~/ros2_ws/src/ndi_tracking && python3 -m ndi_tracking robot-tcp --wand-is-tool -d 90
    # jog wrist 1, 2, 3 one at a time (+-30-40 deg each), then all three again,
    # wand in view the whole time
    python3 scripts/wand_tool_calibration.py            # newest calibrations/raw/robot_tcp_*.npz

Without a divot the robot-pivot tip robot-tcp prints and saves is invalid; the tip is
the wand's own calibrated tip mapped into tool0 by the hand-eye fit, the shaft the
wand's axis. This refits the hand-eye from the raw recording (ndi_tracking's own fit):
1. fit on everything, then drop NDI frames whose implied camera pose is more than
   OUTLIER_MM off the median (mis-tracked wand: a blocked marker, a stray reflection),
2. refit on the rest: the calibration,
3. fit each half on its own: the two tips should agree (2026-10-26: 0.9 mm). They only
   can if each half has all three wrist rotations, hence the sequence twice.
It writes config/calibrations/wand_handeye_<date_time>.json (tip_tool0_mm /
direction_tool0, so tool_library.py add --from reads it), compares with the library's
tool, and prints the tool_library command. Nothing in the library is changed here.
"""
import argparse
import glob
import json
import os
import sys
from datetime import datetime

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from ls_thesis import teleop_settings as cfg
from ls_thesis import tools as tool_library

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NDI_DIR = os.path.normpath(os.path.join(PKG, "..", "ndi_tracking"))
sys.path.insert(0, NDI_DIR)
from ndi_tracking import handeye as he              # noqa: E402
from ndi_tracking.calib import ToolCalibration      # noqa: E402

OUT_DIR = os.path.join(PKG, "config", "calibrations")
OUTLIER_MM = 30.0
# Limits for a usable fit. 2026-10-26: mismatch 0.42 deg, spread 8.1 mm / 0.34 deg, halves 0.9 mm
MAX_ANGLE_MISMATCH_DEG = 1.0
MAX_SPREAD_DEG = 1.0
MAX_HALVES_MM = 2.0


def camera_pose_deviation(rt, rT, nt, nT, X, delay):
    """mm each NDI frame's implied camera pose (base -> camera) is from the median."""
    times = np.clip(nt - delay, rt[0], rt[-1])
    A = np.zeros((len(times), 4, 4))
    A[:, 3, 3] = 1.0
    A[:, :3, :3] = Slerp(rt, Rotation.from_matrix(rT[:, :3, :3]))(times).as_matrix()
    A[:, :3, 3] = np.stack([np.interp(times, rt, rT[:, k, 3]) for k in range(3)], axis=1)
    positions = np.array([(a @ X @ np.linalg.inv(b))[:3, 3] for a, b in zip(A, nT)])
    return np.linalg.norm(positions - np.median(positions, axis=0), axis=1)


def tool_from_fit(X, wand):
    """(tip mm, direction holder -> tip) in tool0 from tool0 -> wand."""
    return X[:3, :3] @ wand.tip + X[:3, 3], -(X[:3, :3] @ wand.axis)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("raw", nargs="?", help="robot_tcp_*.npz (default: the newest in ndi_tracking/calibrations/raw)")
    p.add_argument("--wand", default="8700340", help="wand calibration name in ndi_tracking/calibrations")
    p.add_argument("--tool", type=int, default=1, help="library number it replaces / is compared with")
    p.add_argument("--length", type=float, default=158.6, help="shaft length, mm (wand: tip back to the first marker)")
    args = p.parse_args()

    raw = args.raw or max(glob.glob(os.path.join(NDI_DIR, "calibrations", "raw", "robot_tcp_*.npz")))
    stamp = os.path.basename(raw)[len("robot_tcp_"):-len(".npz")]        # YYYYmmdd_HHMMSS
    recorded = datetime.strptime(stamp, "%Y%m%d_%H%M%S")
    age_h = (datetime.now() - recorded).total_seconds() / 3600
    data = np.load(raw)
    rt, rT, nt, nT = data["robot_t"], data["robot_T"], data["ndi_t"], data["ndi_T"]
    wand = ToolCalibration.load(args.wand)
    print(f"{raw}\n  recorded {recorded:%Y-%m-%d %H:%M:%S} ({age_h:.1f} h ago): "
          f"{len(rt)} robot poses, {len(nt)} NDI frames")

    X, info = he.solve_with_delay(rt, rT, nt, nT)
    deviation = camera_pose_deviation(rt, rT, nt, nT, X, info["ndi_delay_s"])
    good = deviation < OUTLIER_MM
    if not np.all(good):
        bad_t = nt[~good] - nt[0]
        print(f"  dropped {np.sum(~good)} mis-tracked NDI frames (camera pose > {OUTLIER_MM:g} mm off), "
              f"at {bad_t.min():.1f}-{bad_t.max():.1f} s")
    X, info = he.solve_with_delay(rt, rT, nt[good], nT[good])
    tip, direction = tool_from_fit(X, wand)
    tilt = np.degrees(np.arccos(np.clip(direction[2], -1.0, 1.0)))

    middle = rt[0] + (rt[-1] - rt[0]) / 2
    halves = []
    for lo, hi in ((rt[0], middle), (middle, rt[-1])):
        robot = (rt >= lo) & (rt <= hi)
        ndi = good & (nt >= lo - 0.1) & (nt <= hi + 0.1)
        halves.append(tool_from_fit(he.solve_with_delay(rt[robot], rT[robot], nt[ndi], nT[ndi])[0], wand)[0])
    halves_mm = float(np.linalg.norm(halves[0] - halves[1]))

    print(f"  hand-eye: rotation mismatch {info['angle_mismatch_deg_rms']:.2f} deg rms (10-26: 0.42), "
          f"camera-pose spread {info['camera_pose_spread_mm_rms']:.1f} mm / "
          f"{info['camera_pose_spread_deg_rms']:.2f} deg rms (10-26: 8.1 / 0.34), "
          f"NDI delay {1000 * info['ndi_delay_s']:.0f} ms")
    print(f"  halves fitted alone: tips {np.round(halves[0], 1).tolist()} / {np.round(halves[1], 1).tolist()} mm, "
          f"{halves_mm:.1f} mm apart (10-26: 0.9)")
    print(f"  tip in tool0 {tip.round(2).tolist()} mm, shaft {tilt:.2f} deg off tool0 z")
    bad = []
    if info["angle_mismatch_deg_rms"] > MAX_ANGLE_MISMATCH_DEG:
        bad.append(f"rotation mismatch > {MAX_ANGLE_MISMATCH_DEG:g} deg")
    if info["camera_pose_spread_deg_rms"] > MAX_SPREAD_DEG:
        bad.append(f"camera-pose spread > {MAX_SPREAD_DEG:g} deg")
    if halves_mm > MAX_HALVES_MM:
        bad.append(f"the two halves disagree by {halves_mm:.1f} mm (> {MAX_HALVES_MM:g}): jog every wrist in both "
                   "halves (the sequence twice) and keep all wand markers in view")
    if age_h > 2:
        bad.append("recording is more than 2 h old - is this the right one?")

    _, tools = tool_library.load_library(cfg.TOOL_LIBRARY)
    old = tools.get(args.tool)
    if old is not None:
        moved = tip - np.array(old["tip_tool0_mm"])
        turned = np.degrees(np.arccos(np.clip(np.dot(direction, old["direction_tool0"]), -1.0, 1.0)))
        print(f"  vs tool {args.tool} ({old['name']}, {old.get('date', '?')}): tip moved "
              f"{np.linalg.norm(moved):.2f} mm {moved.round(2).tolist()}, shaft turned {turned:.2f} deg")

    stamp_out = recorded.strftime("%Y-%m-%d_%H%M%S")
    record = {
        "label": f"TOOL0 -> WAND hand-eye (wand is the robot tool), {stamp_out}",
        "method": "handeye_wrist_jog",
        "note": "No divot. Tip = the wand's calibrated tip through the hand-eye fit (refitted from the raw "
                "recording without mis-tracked frames); robot-tcp's robot-pivot tip is invalid without a divot.",
        "raw": os.path.abspath(raw),
        "wand_calibration": args.wand,
        "date": recorded.isoformat(),
        "robot_poses": int(len(rt)),
        "ndi_frames_dropped": int(np.sum(~good)),
        "handeye": {"tool0_to_wand": X.tolist(), **{k: v for k, v in info.items() if np.isscalar(v)}},
        "halves_tip_tool0_mm": [h.tolist() for h in halves],
        "halves_apart_mm": halves_mm,
        "tip_tool0_mm": tip.tolist(),
        "direction_tool0": direction.tolist(),
        "direction_angle_from_tool0_z_deg": round(float(tilt), 3),
        "shaft_length_mm": args.length,
        "previous": None if old is None else old.get("calibration"),
        "quality_warnings": bad,
    }
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, f"wand_handeye_{stamp_out}.json")
    with open(out, "w") as f:
        json.dump(record, f, indent=2)
    print(f"saved {out}")

    if bad:
        print("NOT RECOMMENDED: " + "; ".join(bad) + ".")
        return
    name = old["name"] if old is not None else f"NDI wand {args.wand}"
    print("\nTo use it (replaces the library entry; the teleop re-reads it on the next tool switch or start):")
    print(f"  ros2 run ls_thesis tool_library.py add {args.tool} \"{name}\" --from {out} "
          f"--length {args.length} --force --notes \"wand in the holder, re-calibrated {stamp_out[:10]}\"")


if __name__ == "__main__":
    main()
