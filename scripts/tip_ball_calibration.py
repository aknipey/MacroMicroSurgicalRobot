#!/usr/bin/env python3
"""Calibrate a tool tip in tool0 from a single NDI marker ball on it - no wand needed.

Stick one passive marker ball on the tool tip (e.g. the micro module's end, with the
module at rest), start the robot driver, stop `ndi_tracking run`, then:

    python3 scripts/tip_ball_calibration.py -d 60 --name micro_tip

and freedrive the arm for the whole recording with as much rotation as you can (30-45
deg each way about all three axes), keeping the ball in view. The Vega reports the ball
as a stray marker; frames with exactly one stray are used. The point fixed in tool0 is
fitted together with the base -> camera pose from the robot kinematics alone
(ndi_tracking.robot_tcp.fit_robot_point), so nothing else has to be tracked.

The tip is the ball centre moved --ball-radius back towards the holder (0 = the centre is
the tip). The shaft direction is the line from the holder point (0, 0, --holder-z) on
tool0 z to the tip; neither affects the fitted ball position, and the raw recording is
saved, so re-fit with `--file <raw .npz>` and other values instead of recording again.
The result is written to config/calibrations/tip_ball_<date>.json
(tip_tool0_mm / direction_tool0, as robot-tcp writes them), ready for
    ros2 run macro_micro_robot tool_library.py add N "name" --from <that file> --length <mm>

Uses src/ndi_tracking (its Vega client and the fit) as a library; nothing there changes.
"""
import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np

NDI_DIR = os.path.expanduser("~/ros2_ws/src/ndi_tracking")
sys.path.insert(0, NDI_DIR)
from ndi_tracking import handeye as he                          # noqa: E402
from ndi_tracking.robot_tcp import fit_robot_point              # noqa: E402
from ndi_tracking.vega import DEFAULT_HOST, Vega, quat_to_rot   # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "calibrations")


def record(args):
    """(robot_t, robot_T mm, ball_t, ball camera positions mm) over args.duration s."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("tip_ball_calibration")
    buf = Buffer()
    TransformListener(buf, node)
    robot_t, robot_T = [], []
    last_stamp = [None]

    def poll():
        try:
            tf = buf.lookup_transform(args.base_frame, args.ee_frame, Time())
        except TransformException:
            return
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if stamp == last_stamp[0]:
            return
        last_stamp[0] = stamp
        q, p = tf.transform.rotation, tf.transform.translation
        robot_t.append(stamp)
        robot_T.append(he.T(quat_to_rot([q.w, q.x, q.y, q.z]), 1000 * np.array([p.x, p.y, p.z])))

    node.create_timer(1 / 250, poll)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            try:
                executor.spin_once(timeout_sec=0.05)
            except Exception:
                break
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    ball_t, ball = [], []
    counts = {0: 0, 1: 0, "more": 0}
    vega = Vega(args.host)
    try:
        vega.init()
        # The wand's ROM, so its markers (if it is lying in view) count as a tool, not strays
        vega.load_rom(args.wand_rom)
        vega.start(60)
        print("waiting for TF ...", end="", flush=True)
        t_wait = time.time()
        while not robot_t and time.time() - t_wait < 10:
            time.sleep(0.1)
        if not robot_t:
            raise SystemExit(f"\nno TF {args.base_frame} -> {args.ee_frame}: is the robot driver running "
                             "(and ROS_DOMAIN_ID the same as the robot)?")
        print(f" ok. Freedrive the arm; recording {args.duration:.0f} s in 3 s ...")
        time.sleep(3)
        robot_t0 = robot_t[-1]
        t0 = time.time()
        last_frame = None
        while time.time() - t0 < args.duration:
            f = vega.read(strays=True)
            frame = f.tools[0].frame if f.tools else None
            if frame != last_frame or frame is None:
                last_frame = frame
                strays = f.strays if f.strays is not None else []
                if len(strays) == 1:
                    counts[1] += 1
                    ball_t.append(f.t)
                    ball.append(np.asarray(strays[0], float))
                else:
                    counts[0 if len(strays) == 0 else "more"] += 1
            time.sleep(0.004)
            print(f"\r  {time.time() - t0:5.1f} s  robot {len(robot_t):6d}  ball {counts[1]:5d}  "
                  f"no ball {counts[0]:5d}  >1 stray {counts['more']:4d}", end="", flush=True)
        print()
    finally:
        stop.set()
        thread.join(timeout=2)
        executor.shutdown(timeout_sec=1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        vega.close()
    keep = [i for i, t in enumerate(robot_t) if t >= robot_t0]
    return (np.array([robot_t[i] for i in keep]), np.array([robot_T[i] for i in keep]),
            np.array(ball_t), np.array(ball).reshape(-1, 3), counts)


def fit(robot_t, robot_T, ball_t, ball):
    """Best fit over the NDI delay; drops outliers (> 3x the median residual) once."""
    best = None
    for delay in np.arange(-0.05, 0.1001, 0.01):
        A, C = he.match(robot_t, robot_T, ball_t, list(ball), delay)
        if len(A) < 100:
            continue
        A, C = np.array(A), np.array(C)
        for p0 in ([0.0, 0.0, 300.0], [0.0, 0.0, 150.0], [0.0, 0.0, 450.0]):
            p, Y, res = fit_robot_point(A, C, p0)
            keep = res < 3 * np.median(res)
            p, Y, res_k = fit_robot_point(A[keep], C[keep], p)
            rms = float(np.sqrt(np.mean(res_k ** 2)))
            if best is None or rms < best["rms"]:
                best = {"p": p, "Y": Y, "res": res_k, "rms": rms, "delay": float(delay),
                        "poses": int(keep.sum()), "outliers": int((~keep).sum()), "A": A[keep]}
    if best is None:
        raise SystemExit("fewer than 100 ball sightings paired with robot poses - keep the ball in view")
    A = best.pop("A")
    best["swing_deg"] = max(np.degrees(np.linalg.norm(he.rot_log(A[0][:3, :3].T @ a[:3, :3]))) for a in A[::10])
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-d", "--duration", type=float, default=60.0)
    ap.add_argument("--holder-z", type=float, default=0.0,
                    help="m, where the tool leaves its holder on tool0 z; only sets the shaft direction "
                         "(default 0: tool0 origin -> ball). The measured ball offset does not depend on it")
    ap.add_argument("--ball-radius", type=float, default=0.0,
                    help="mm; tip = ball centre moved this far back towards the holder (0 = centre is the tip)")
    ap.add_argument("--name", default="tip_ball", help="file name stem in config/calibrations/")
    ap.add_argument("--file", help="re-fit a saved config/calibrations/raw/*.npz instead of recording")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--wand-rom", default="8700340")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--ee-frame", default="tool0")
    args = ap.parse_args()

    if args.file:
        d = np.load(os.path.expanduser(args.file))
        robot_t, robot_T, ball_t, ball = d["robot_t"], d["robot_T"], d["ball_t"], d["ball"]
        counts = {0: int(d["no_ball"]), 1: len(ball_t), "more": int(d["more"])}
    else:
        robot_t, robot_T, ball_t, ball, counts = record(args)
        # Raw data, so --holder-z / --ball-radius can be changed later without recording again
        os.makedirs(os.path.join(OUT_DIR, "raw"), exist_ok=True)
        raw = os.path.join(OUT_DIR, "raw", f"{args.name}_{datetime.now():%Y-%m-%d_%H%M%S}.npz")
        np.savez(raw, robot_t=robot_t, robot_T=robot_T, ball_t=ball_t, ball=ball,
                 no_ball=counts[0], more=counts["more"])
        print(f"raw recording: {raw}  (re-fit with --file {raw})")
    print(f"ball in {counts[1]} frames, none in {counts[0]}, more than one stray in {counts['more']} (skipped)")
    r = fit(robot_t, robot_T, ball_t, ball)
    c = r["p"]
    holder = np.array([0.0, 0.0, 1000 * args.holder_z])
    direction = (c - holder) / np.linalg.norm(c - holder)
    tip = c - args.ball_radius * direction
    p95 = float(np.percentile(r["res"], 95))
    print(f"\nBALL OFFSET FROM {args.ee_frame.upper()}: x {c[0]:.2f}  y {c[1]:.2f}  z {c[2]:.2f} mm  "
          f"(distance {np.linalg.norm(c):.2f} mm)")
    print(f"ball centre in {args.ee_frame}: {c.round(2).tolist()} mm  ({r['poses']} poses, "
          f"{r['outliers']} outliers dropped, swing {r['swing_deg']:.0f} deg, NDI delay {1e3 * r['delay']:.0f} ms)")
    print(f"  residual rms {r['rms']:.2f} mm, p95 {p95:.2f} mm")
    print(f"TIP {tip.round(2).tolist()} mm in {args.ee_frame}, direction {direction.round(5).tolist()} "
          f"({np.degrees(np.arccos(np.clip(direction[2], -1, 1))):.2f} deg off tool0 z)")
    if r["swing_deg"] < 25:
        print("  WARNING: swing < 25 deg - the tip is poorly constrained: tilt the tool further")
    if r["rms"] > 1.0:
        print("  WARNING: rms > 1 mm - the ball moved on the tool, or TF is not this arm's kinematics")

    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"{args.name}_{datetime.now():%Y-%m-%d_%H%M%S}.json")
    with open(path, "w") as f:
        json.dump({"date": datetime.now().isoformat(timespec="seconds"), "method": "tip_ball_no_wand",
                   "base_frame": args.base_frame, "ee_frame": args.ee_frame,
                   "ball_tool0_mm": c.tolist(), "ball_radius_mm": args.ball_radius, "holder_z_m": args.holder_z,
                   "tip_tool0_mm": tip.tolist(), "direction_tool0": direction.tolist(),
                   "direction_source": f"holder point (0, 0, {args.holder_z} m) on tool0 z -> tip",
                   "robot_fit_rms_mm": r["rms"], "robot_fit_p95_mm": p95, "swing_deg": r["swing_deg"],
                   "robot_poses": r["poses"], "outliers": r["outliers"], "ndi_delay_s": r["delay"],
                   "base_to_camera": r["Y"].tolist(), "frames": {str(k): v for k, v in counts.items()}},
                  f, indent=2)
    print(f"\nsaved {path}\nadd it:  ros2 run macro_micro_robot tool_library.py add N \"name\" --from {path} --length <mm>")


if __name__ == "__main__":
    main()
