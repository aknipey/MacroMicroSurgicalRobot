#!/usr/bin/env python3
"""Micro module shaft and tip in tool0, from four NDI dots - no wand needed.

Dots: two at the base of the shaft on opposite sides, two at the end of the metal part on
opposite sides. The midpoint of each pair is on the shaft axis, so the dots' own distance from
the axis cancels. Opposite dots are never seen together, and no frame needs them to be: each dot
is a point fixed in tool0, fitted over the whole recording together with base -> camera.

    # robot stack running (calibrated kinematics), `ndi_tracking run` STOPPED
    python3 scripts/micro_tool_calibration.py -d 120 --extension 45

and move the arm by hand (freedrive) for the whole recording. Keep the tool in the Vega's view, turn it
about the shaft so each dot of each pair faces the camera for a good while, and tilt it 30+ deg each way.
Nothing is live: everything is worked out after the recording, and the checks say if the motion fell short.
Or script the motion: `python3 scripts/wrist_jog.py --wrist3-amplitude 100` prints a player line.

--extension: mm from the midpoint of the tip dots to the true tip of the continuum section, along
the shaft line. The raw recording is saved, so measure it later if you like and re-fit with
--file <raw .npz> --extension <mm>. The continuum section is assumed straight, at rest.

Writes config/calibrations/micro_tool_<date>.json (tip_tool0_mm / direction_tool0, as robot-tcp
writes them), with the line to add it to the tool library. direction_tool0 runs base -> tip.

    python3 scripts/micro_tool_calibration.py --self-test     # no hardware: known answers + failures
"""
import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

NDI_DIR = os.path.expanduser("~/ros2_ws/src/ndi_tracking")
sys.path.insert(0, NDI_DIR)
from ndi_tracking import handeye as he                                  # noqa: E402
from ndi_tracking.robot_tcp import fit_robot_point, kabsch              # noqa: E402
from ndi_tracking.vega import DEFAULT_HOST, Vega, quat_to_rot           # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "calibrations")
DOTS = 4
LINK_MM = 12.0          # a stray this close to one in the last frames is the same dot (10 mm a frame by hand)
MIN_TRACK = 60          # frames, to try a track as the camera pose seed
SEEDS = 8               # tracks tried
CLUSTER_MM = 6.0        # first pass: a rough Y blurs the dots, and the tip pair may merge
FINE_CLUSTER_MM = 1.5   # second pass, with Y refined: splits dots a few mm apart
GATES_MM = (8.0, 5.0, 3.0)          # a sighting belongs to a dot within this, tightening
FINE_GATES_MM = (2.5, 2.0)
DELAYS = np.arange(-0.05, 0.1001, 0.01)
MIN_SIGHTINGS = 150
MIN_TILT_DEG = 25.0     # the shaft axis must swing this far, or the position along it is weak
MIN_ROLL_DEG = 120.0    # turning about the shaft, to show opposite dots
MAX_RMS_MM = 1.0
MAX_HALVES_MM = 1.0
CHECK_GATE_MM = 4.0     # for the halves check, wider than the fit's gates
MIN_DOT_GAP_MM = 2.5    # closer than this is one dot fitted twice
MAX_PAIR_GAP_MM = 60.0  # further apart than this is not one section of shaft
MERGE_DEG = 0.2         # simulation only


def parse_strays(raw):
    """(N, 3) mm camera-frame strays from a TX 1001 reply. The count is every marker, but only those
    in the volume have data, so ndi_tracking's parser (which reads count x 3 numbers) gives up as soon
    as one is out of the volume. Here N comes from the reply's length. None if it doesn't fit."""
    try:
        rest = raw.split("\n", 1)[1].strip()
        count = int(rest[:2], 16)
        start = 2 + (count + 3) // 4
        body = rest[start:-4]               # the last 4 are the system status
        if len(body) % 21 or len(body) // 21 > count:
            return None
        return np.array([[float(body[k + a:k + a + 7]) / 100.0 for a in (0, 7, 14)]
                         for k in range(0, len(body), 21)]).reshape(-1, 3)
    except (IndexError, ValueError):
        return None


def record(args):
    """(robot_t, robot_T mm, sight_t, sight_xyz, frames) over args.duration s: every stray of every frame."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("micro_tool_calibration")
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

    sight_t, sight_xyz = [], []
    seen = {}       # strays in a frame -> frames
    unreadable = 0
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
        print(f" ok. Start the motion; recording {args.duration:.0f} s in 3 s ...")
        time.sleep(3)
        robot_t0 = robot_t[-1]
        t0 = time.time()
        last_frame = None
        while time.time() - t0 < args.duration:
            f = vega.read(strays=True)
            frame = f.tools[0].frame if f.tools else None
            if frame != last_frame or frame is None:
                last_frame = frame
                strays = parse_strays(f.raw)
                if strays is None:
                    unreadable += 1
                    strays = np.zeros((0, 3))
                seen[len(strays)] = seen.get(len(strays), 0) + 1
                for point in strays:
                    sight_t.append(f.t)
                    sight_xyz.append(np.asarray(point, float))
            time.sleep(0.004)
            print(f"\r  {time.time() - t0:5.1f} s  robot {len(robot_t):6d}  dots seen per frame "
                  + "  ".join(f"{k}:{seen[k]}" for k in sorted(seen))
                  + (f"  UNREADABLE {unreadable}" if unreadable else ""), end="", flush=True)
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
            np.array(sight_t), np.array(sight_xyz).reshape(-1, 3))


def pair_robot(robot_t, robot_T, sight_t, delay, max_gap=0.15):
    """Robot pose interpolated to each sighting's time - delay. -> (A, index of the sightings kept)
    Interpolated, as TF can arrive at only ~20 Hz: taking the nearest pose within 10 ms lost most sightings."""
    wanted = sight_t - delay
    k = np.clip(np.searchsorted(robot_t, wanted), 1, len(robot_t) - 1)
    span = robot_t[k] - robot_t[k - 1]
    keep = np.where((wanted >= robot_t[0]) & (wanted <= robot_t[-1]) & (span <= max_gap) & (span > 0))[0]
    k, u = k[keep], ((wanted[keep] - robot_t[k[keep] - 1]) / span[keep])
    before, after = robot_T[k - 1], robot_T[k]
    turn = Rotation.from_matrix(np.einsum("nji,njk->nik", before[:, :3, :3], after[:, :3, :3])).as_rotvec()
    A = before.copy()
    A[:, :3, :3] = before[:, :3, :3] @ Rotation.from_rotvec(turn * u[:, None]).as_matrix()
    A[:, :3, 3] = before[:, :3, 3] + (after[:, :3, 3] - before[:, :3, 3]) * u[:, None]
    return A, keep


def tracks(sight_t, sight_xyz):
    """Index lists of sightings linked frame to frame: one dot, while it stays in view."""
    open_tracks, finished = [], []
    for i in range(len(sight_t)):
        best, best_d = None, LINK_MM
        for track in open_tracks:
            j = track[-1]
            if sight_t[i] == sight_t[j] or sight_t[i] - sight_t[j] > 0.1:
                continue
            d = np.linalg.norm(sight_xyz[i] - sight_xyz[j])
            if d < best_d:
                best, best_d = track, d
        if best is None:
            open_tracks.append([i])
        else:
            best.append(i)
        finished += [t for t in open_tracks if sight_t[i] - sight_t[t[-1]] > 0.1]
        open_tracks = [t for t in open_tracks if sight_t[i] - sight_t[t[-1]] <= 0.1]
    return sorted(finished + open_tracks, key=len, reverse=True)


def to_tool0(A, Y, cam):
    """Camera points -> tool0, for poses A and base -> camera Y."""
    base = (cam - Y[:3, 3]) @ Y[:3, :3]
    return np.einsum("nji,nj->ni", A[:, :3, :3], base - A[:, :3, 3])


def clusters(points, k=DOTS + 2, radius=CLUSTER_MM):
    """Up to k densest blobs, densest first: [(centre, sightings)]."""
    left = np.arange(len(points))
    found = []
    minimum = max(20, len(points) // 100)
    while len(found) < k and len(left) > minimum:
        tree = cKDTree(points[left])
        counts = tree.query_ball_point(points[left][:: max(1, len(left) // 400)], radius, return_length=True)
        centre = points[left][:: max(1, len(left) // 400)][int(np.argmax(counts))]
        for _ in range(5):
            near = tree.query_ball_point(centre, radius)
            centre = points[left][near].mean(0)
        near = tree.query_ball_point(centre, 2 * radius)
        if len(near) < minimum:
            break
        found.append((centre, len(near)))
        left = np.delete(left, near)
    return found


def refine(A, cam, centres, Y, gates=GATES_MM):
    """Alternate which dot each sighting is, the dots, and Y. -> (centres, Y, label: -1 = none, residual mm)"""
    centres = np.array(centres, float)
    for gate in gates:
        for _ in range(12):
            q = to_tool0(A, Y, cam)
            d = np.linalg.norm(q[:, None] - centres[None], axis=2)
            label = np.where(d.min(1) < gate, d.argmin(1), -1)
            used = label >= 0
            if used.sum() < 20:
                return centres, Y, label, np.full(len(cam), np.inf)
            for k in range(len(centres)):
                if (label == k).any():
                    centres[k] = q[label == k].mean(0)
            points = np.einsum("nij,nj->ni", A[used, :3, :3], centres[label[used]]) + A[used, :3, 3]
            R, t = kabsch(points, cam[used])
            Y = he.T(R, t)
    q = to_tool0(A, Y, cam)
    res = np.linalg.norm(q[:, None] - centres[None], axis=2).min(1)     # to the nearest dot, used or not
    return centres, Y, label, res


def polish(A, cam, label, centres, Y, per_dot=700):
    """Least squares over Y and every dot together, sightings as labelled, robust to the odd bad one.
    The alternation in refine() is good for finding which dot is which but creeps to the answer: it
    stopped 2 mm short, and that bias then picked the wrong NDI delay."""
    rng = np.random.default_rng(0)
    pick = np.concatenate([rng.permutation(np.where(label == k)[0])[:per_dot] for k in range(len(centres))])
    AR, At, c, dot = A[pick, :3, :3], A[pick, :3, 3], cam[pick], label[pick]
    scale = 100.0       # mm per rad, so one step moves a far point and a near one alike

    def residual(x):
        R = Rotation.from_rotvec(x[:3] / scale).as_matrix() @ Y[:3, :3]
        points = x[6:].reshape(-1, 3)
        base = np.einsum("nij,nj->ni", AR, points[dot]) + At
        return (base @ R.T + Y[:3, 3] + x[3:6] - c).ravel()

    fit_ = least_squares(residual, np.r_[np.zeros(6), np.ravel(centres)], loss="huber", f_scale=1.0, max_nfev=40)
    x = fit_.x
    return x[6:].reshape(-1, 3), he.T(Rotation.from_rotvec(x[:3] / scale).as_matrix() @ Y[:3, :3], Y[:3, 3] + x[3:6])


def relabel(A, cam, centres, Y, gate):
    """-> (label: nearest dot within gate, else -1; distance to the nearest dot)"""
    d = np.linalg.norm(to_tool0(A, Y, cam)[:, None] - np.asarray(centres)[None], axis=2)
    return np.where(d.min(1) < gate, d.argmin(1), -1), d.min(1)


def start_poses(A, cam, sight_t, dots):
    """Candidate [(score, centres, Y)], best first: Y seeded from single tracks, scored on how tight
    the sightings' blobs are in tool0."""
    found = []
    for track in [t for t in tracks(sight_t, cam) if len(t) >= MIN_TRACK][:SEEDS]:
        pick = np.array(track[:: max(1, len(track) // 300)])
        try:
            p, Y, _ = fit_robot_point(A[pick], cam[pick], [0.0, 0.0, 300.0], iters=400)
        except (np.linalg.LinAlgError, ValueError):
            continue
        blobs = clusters(to_tool0(A, Y, cam))
        found.append((sum(n for _, n in blobs[:dots]), [c for c, _ in blobs[:dots]], Y))
    if not found:
        raise SystemExit(f"no dot stayed in view for {MIN_TRACK} frames: check the dots are seen at all")
    return sorted(found, key=lambda f: -f[0])


def capped_score(res):
    """Mean squared distance to the nearest dot over every sighting, capped. Rms over the kept ones
    alone rewards an answer that drops the hard ones."""
    return float(np.mean(np.minimum(res, FINE_GATES_MM[-1]) ** 2))


def settle(A, cam, centres, Y, dots):
    """One start -> (score, centres, Y). Different starts can settle on different answers."""
    centres, Y, _, _ = refine(A, cam, centres, Y)
    fine = clusters(to_tool0(A, Y, cam), dots + 2, FINE_CLUSTER_MM)
    if len(fine) < dots:
        return np.inf, centres, Y
    centres = np.array([c for c, _ in fine[:dots]])
    for gate in (3.0, 2.5, 2.0):
        label, _ = relabel(A, cam, centres, Y, gate)
        if len(set(label[label >= 0])) < dots:
            return np.inf, centres, Y
        centres, Y = polish(A, cam, label, centres, Y)
    return capped_score(relabel(A, cam, centres, Y, FINE_GATES_MM[-1])[1]), centres, Y


def fit(robot_t, robot_T, sight_t, sight_xyz, dots=DOTS):
    if len(sight_t) < 100:
        raise SystemExit(f"only {len(sight_t)} dot sightings in the recording: the Vega did not see the dots")
    A, keep = pair_robot(robot_t, robot_T, sight_t, 0.01)
    if len(keep) < 100:
        raise SystemExit("fewer than 100 dot sightings paired with robot poses")
    cam = sight_xyz[keep]
    settled = [settle(A, cam, np.array(c), Y, dots) for _, c, Y in start_poses(A, cam, sight_t[keep], dots)]
    settled = [s for s in settled if np.isfinite(s[0])]
    if not settled:
        # Fewer blobs than dots from every start. Say how many, for the message in analyse()
        blobs = clusters(to_tool0(A, start_poses(A, cam, sight_t[keep], dots)[0][2], cam), dots + 2, FINE_CLUSTER_MM)
        raise SystemExit(f"found {len(blobs)} dots, not {dots}: each dot must be seen from the side it faces")
    _, centres, Y = min(settled, key=lambda s: s[0])
    best = None
    for _ in range(2):      # the second scan starts from the first one's better Y
        for delay in DELAYS:
            A, keep = pair_robot(robot_t, robot_T, sight_t, delay)
            cam = sight_xyz[keep]
            label, _ = relabel(A, cam, centres, Y, FINE_GATES_MM[-1])
            c, y = polish(A, cam, label, centres, Y, per_dot=350)
            label, res = relabel(A, cam, c, y, FINE_GATES_MM[-1])
            score = capped_score(res)
            if best is None or score < best["score"]:
                best = {"score": score, "delay": float(delay), "centres": c, "Y": y}
        centres, Y = best["centres"], best["Y"]

    A, keep = pair_robot(robot_t, robot_T, sight_t, best["delay"])
    cam = sight_xyz[keep]
    for _ in range(3):
        label, _ = relabel(A, cam, centres, Y, FINE_GATES_MM[-1])
        centres, Y = polish(A, cam, label, centres, Y)
    label, res = relabel(A, cam, centres, Y, FINE_GATES_MM[-1])
    used = label >= 0
    return {"centres": centres, "Y": Y, "label": label, "res": res, "delay": best["delay"],
            "rms": float(np.sqrt(np.mean(res[used] ** 2))), "A": A, "cam": cam, "t": sight_t[keep], "seen": len(cam)}


def name_dots(centres):
    """(base pair, tip pair) by distance along tool0 z, which the shaft roughly follows."""
    order = np.argsort(centres[:, 2])
    return order[:2], order[2:]


def analyse(r, extension, measured_span=None):
    """Shaft and tip, plus every check. -> (result dict, [warnings])"""
    centres, label, res, A = r["centres"], r["label"], r["res"], r["A"]
    warnings = []
    k = len(centres)
    if k != DOTS:
        raise SystemExit(f"found {k} dots, not {DOTS}: each dot must be seen from the side it faces")
    counts = np.array([(label == i).sum() for i in range(k)])
    dot_rms = np.array([float(np.sqrt(np.mean(res[label == i] ** 2))) if counts[i] else np.inf for i in range(k)])
    gaps = np.linalg.norm(centres[:, None] - centres[None], axis=2)
    distinct = k - int((np.triu(gaps < MIN_DOT_GAP_MM, 1)).any(0).sum())
    if distinct != DOTS:
        raise SystemExit(f"found {distinct} dots, not {DOTS} (one was fitted twice): each dot must be seen from the side it faces")
    base, tip = name_dots(centres)
    if max(gaps[base[0], base[1]], gaps[tip[0], tip[1]]) > MAX_PAIR_GAP_MM:
        raise SystemExit(f"a dot pair is {max(gaps[base[0], base[1]], gaps[tip[0], tip[1]]):.0f} mm apart: "
                         "these are not two pairs across the shaft, so a dot is missing or a reflection was taken for one")
    base_mid, tip_mid = centres[base].mean(0), centres[tip].mean(0)
    span = np.linalg.norm(tip_mid - base_mid)
    if span < 30.0:
        raise SystemExit(f"the base and tip dots are only {span:.0f} mm apart: wrong dots or a bad fit")
    axis = (tip_mid - base_mid) / span
    true_tip = tip_mid + extension * axis

    # Halves in time, each dot from each half on its own. A wider gate than the fit: a dot that
    # shifted by more than the fit's gate would otherwise just drop out of it unnoticed
    q = to_tool0(A, r["Y"], r["cam"])
    d = np.linalg.norm(q[:, None] - centres[None], axis=2)
    wide = np.where(d.min(1) < CHECK_GATE_MM, d.argmin(1), -1)
    mid = np.median(r["t"][label >= 0])
    halves = []
    for part in (r["t"] < mid, r["t"] >= mid):
        c = [np.median(q[part & (wide == i)], axis=0) if (part & (wide == i)).sum() > 10 else np.full(3, np.nan)
             for i in range(k)]
        halves.append(np.array(c))
    half_gap = np.linalg.norm(halves[0] - halves[1], axis=1)
    half_tips = []
    for c in halves:
        b, t = c[base].mean(0), c[tip].mean(0)
        half_tips.append(t + extension * (t - b) / np.linalg.norm(t - b))
    half_tip_gap = float(np.linalg.norm(half_tips[0] - half_tips[1]))

    # How well the motion constrained it
    used = label >= 0
    R = A[used, :3, :3]
    shaft = np.einsum("nij,j->ni", R, axis)
    mean_shaft = shaft.mean(0) / np.linalg.norm(shaft.mean(0))
    tilt = float(2 * np.degrees(np.arccos(np.clip((shaft @ mean_shaft).min(), -1, 1))))
    across = np.cross(axis, [1.0, 0.0, 0.0] if abs(axis[0]) < 0.9 else [0.0, 1.0, 0.0])
    side = np.einsum("nij,j->ni", R, across)
    side -= np.sum(side * shaft, axis=1, keepdims=True) * shaft
    side /= np.linalg.norm(side, axis=1, keepdims=True)
    ref = side[0]
    roll = np.degrees(np.arctan2(np.sum(np.cross(ref, side) * shaft, axis=1), side @ ref))
    roll_range = float(np.ptp(np.unwrap(np.radians(roll))) * 180 / np.pi)

    # Opposite dots: the line between them should cross the shaft square on
    pair_angle = [float(abs(90 - np.degrees(np.arccos(np.clip(abs(
        (centres[p[1]] - centres[p[0]]) @ axis) / np.linalg.norm(centres[p[1]] - centres[p[0]]), 0, 1)))))
        for p in (base, tip)]
    separation = [float(np.linalg.norm(centres[p[1]] - centres[p[0]])) for p in (base, tip)]

    if counts.min() < MIN_SIGHTINGS:
        warnings.append(f"a dot has only {counts.min()} sightings (want {MIN_SIGHTINGS}): show every dot to the camera")
    if max(dot_rms) > MAX_RMS_MM:
        warnings.append(f"a dot's rms is {max(dot_rms):.2f} mm (want < {MAX_RMS_MM}): a dot moved, or TF is not this arm's kinematics")
    if np.nanmax(half_gap) > MAX_HALVES_MM or np.isnan(half_gap).any():
        warnings.append(f"a dot differs by {np.nanmax(half_gap):.2f} mm between the two halves (want < {MAX_HALVES_MM}), "
                        "or was missing from one: a dot or the tool may have moved during the recording")
    if tilt < MIN_TILT_DEG:
        warnings.append(f"the shaft only tilted {tilt:.0f} deg (want {MIN_TILT_DEG:.0f}+): the position along it is weak")
    if roll_range < MIN_ROLL_DEG:
        warnings.append(f"only {roll_range:.0f} deg of turn about the shaft (want {MIN_ROLL_DEG:.0f}+)")
    if max(pair_angle) > 15.0:
        warnings.append(f"a dot pair is {max(pair_angle):.0f} deg off square to the shaft: the midpoint may not be on the axis")
    if measured_span is not None and abs(span - measured_span) > 2.0:
        warnings.append(f"fitted dot span {span:.1f} mm vs {measured_span:.1f} mm measured")
    if extension == 0.0:
        warnings.append("--extension 0: the tip here is the end of the metal part, not the true tip")

    return {"shaft_base_tool0_mm": base_mid.tolist(), "metal_end_tool0_mm": tip_mid.tolist(),
            "tip_tool0_mm": true_tip.tolist(), "direction_tool0": axis.tolist(),
            "extension_mm": float(extension), "dot_span_mm": float(span),
            "shaft_length_mm": float(np.linalg.norm(true_tip - base_mid)),
            "angle_from_tool0_z_deg": float(np.degrees(np.arccos(np.clip(axis[2], -1, 1)))),
            "base_dots_tool0_mm": centres[base].tolist(), "tip_dots_tool0_mm": centres[tip].tolist(),
            "dot_sightings": counts[np.r_[base, tip]].tolist(), "dot_rms_mm": dot_rms[np.r_[base, tip]].tolist(),
            "dot_separation_mm": separation, "pair_off_square_deg": pair_angle,
            "halves_dot_gap_mm": half_gap[np.r_[base, tip]].tolist(), "halves_tip_gap_mm": half_tip_gap,
            "shaft_tilt_deg": tilt, "roll_deg": roll_range, "fit_rms_mm": r["rms"], "ndi_delay_s": r["delay"],
            "sightings_used": int(used.sum()), "sightings_total": int(r["seen"]),
            "base_to_camera": r["Y"].tolist()}, warnings


def report(result, warnings):
    v = lambda key: np.round(result[key], 2).tolist()
    print(f"\nSHAFT BASE    {v('shaft_base_tool0_mm')} mm in tool0")
    print(f"METAL END     {v('metal_end_tool0_mm')} mm")
    print(f"TRUE TIP      {v('tip_tool0_mm')} mm  ({result['extension_mm']:g} mm past the metal end)")
    print(f"DIRECTION     {np.round(result['direction_tool0'], 5).tolist()}  base -> tip, "
          f"{result['angle_from_tool0_z_deg']:.2f} deg off tool0 z")
    print(f"LENGTH        shaft base -> tip {result['shaft_length_mm']:.1f} mm, dot span {result['dot_span_mm']:.1f} mm")
    print(f"\nFit rms {result['fit_rms_mm']:.2f} mm, {result['sightings_used']} of {result['sightings_total']} sightings, "
          f"NDI delay {1e3 * result['ndi_delay_s']:.0f} ms")
    print(f"Dots (base, base, tip, tip): sightings {result['dot_sightings']}, rms {np.round(result['dot_rms_mm'], 2).tolist()} mm")
    print(f"Dot pairs: {np.round(result['dot_separation_mm'], 1).tolist()} mm apart (compare with calipers), "
          f"{np.round(result['pair_off_square_deg'], 1).tolist()} deg off square")
    print(f"Halves: dots agree to {np.round(result['halves_dot_gap_mm'], 2).tolist()} mm, tip to {result['halves_tip_gap_mm']:.2f} mm")
    print(f"Motion: shaft tilt {result['shaft_tilt_deg']:.0f} deg, turn about the shaft {result['roll_deg']:.0f} deg")
    for w in warnings:
        print(f"  WARNING: {w}")
    print("\nPASS: all checks met" if not warnings else f"\nCHECK: {len(warnings)} warning(s) above")


def simulate(seed=0, noise=0.12, dropped=None, bump=False, ghosts=True, roll_deg=100.0, tilt_deg=32.0, kinematics=0.15, delay=0.02,
             blocked=0.15):
    """Synthetic recording with a known answer. Flat dots, seen only within 70 deg of facing the camera; each
    dot also blocked for spells of 1-4 s (`blocked` of the time: the tool or arm in the way); and dots
    lined up in view within MERGE_DEG (~5 mm at 1.5 m) reported as one blurred marker, as the Vega can't split them."""
    from scipy.spatial.transform import Rotation as Rot
    rng = np.random.default_rng(seed)
    shaft = np.array([np.radians(1.2), np.radians(-2.1), 1.0])
    shaft /= np.linalg.norm(shaft)
    origin = np.array([-8.0, 15.0, 0.0])
    side = np.cross(shaft, [1.0, 0.0, 0.0])
    side /= np.linalg.norm(side)
    truth_base, truth_tip = origin + 115.0 * shaft, origin + 395.0 * shaft
    radius_b, radius_t = 14.0, 4.0
    dots = np.array([truth_base + radius_b * side, truth_base - radius_b * side,
                     truth_tip + radius_t * side, truth_tip - radius_t * side])
    normals = np.array([side, -side, side, -side])

    duration = 170.0
    robot_t = np.arange(0.0, duration, 0.004) + 1000.0
    u = (robot_t - robot_t[0]) / duration
    wrist = Rot.from_euler("xyz", np.c_[np.radians(tilt_deg) * np.sin(2 * np.pi * 3 * u),
                                        np.radians(tilt_deg) * np.sin(2 * np.pi * 5 * u + 1.0),
                                        np.radians(roll_deg) * np.sin(2 * np.pi * 2 * u)])
    mount = Rot.from_euler("y", 180, degrees=True)
    rotation = (mount * wrist).as_matrix()
    position = np.array([550.0, 150.0, 650.0]) + np.c_[30 * np.sin(4 * u), 30 * np.cos(3 * u), 20 * np.sin(5 * u)]
    robot_T = np.tile(np.eye(4), (len(u), 1, 1))
    robot_T[:, :3, :3], robot_T[:, :3, 3] = rotation, position

    Y = he.T(Rot.from_euler("xyz", [66.0, -59.0, -66.0], degrees=True).as_matrix(), np.array([40.0, -200.0, -1500.0]))
    camera_base = -Y[:3, :3].T @ Y[:3, 3]
    sight_t, sight_xyz = [], []
    frame_t = np.arange(robot_t[0] + 0.1, robot_t[-1] - 0.1, 1 / 60)
    frame_index = np.searchsorted(robot_t, frame_t - delay)
    ghost = camera_base * 0 + Y[:3, 3] + np.array([200.0, -150.0, 120.0])
    moved = dots + np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [2.5, 0, 0]], float)
    hidden = np.zeros((len(frame_t), DOTS), bool)
    for k in range(DOTS):
        for _ in range(int(blocked * duration / 2.5)):
            start = rng.integers(len(frame_t))
            hidden[start:start + int(60 * rng.uniform(1.0, 4.0)), k] = True
    for i, f in enumerate(frame_index):
        R, p = rotation[f], position[f]
        wobble = kinematics * np.sin(robot_t[f] * np.array([0.13, 0.19, 0.07]))
        now = moved if bump and i > len(frame_index) // 2 else dots
        seen = []
        for k in range(DOTS):
            if dropped == k or hidden[i, k] or rng.random() < 0.03:
                continue
            world = R @ now[k] + p
            view = camera_base - world
            view /= np.linalg.norm(view)
            if (R @ normals[k]) @ view < np.cos(np.radians(70.0)):
                continue
            seen.append((world, view))
        while seen:
            world, view = seen.pop()
            together = [w for w, v in seen if v @ view > np.cos(np.radians(MERGE_DEG))]
            seen = [(w, v) for w, v in seen if v @ view <= np.cos(np.radians(MERGE_DEG))]
            point = np.mean([world] + together, axis=0)
            sight_t.append(frame_t[i])
            sight_xyz.append(Y[:3, :3] @ (point + wobble) + Y[:3, 3] + rng.normal(0.0, noise, 3))
        if ghosts:
            if rng.random() < 0.5:     # a fixed reflection in the room
                sight_t.append(frame_t[i])
                sight_xyz.append(ghost + rng.normal(0.0, 0.1, 3))
            if rng.random() < 0.02:    # a one-off false marker
                sight_t.append(frame_t[i])
                sight_xyz.append(Y[:3, 3] + rng.normal(0.0, 300.0, 3))
    return robot_t, robot_T, np.array(sight_t), np.array(sight_xyz), truth_base, truth_tip, shaft


def self_test():
    extension = 45.0
    failures = 0

    def check(name, ok, detail):
        nonlocal failures
        failures += 0 if ok else 1
        print(f"  {'pass' if ok else 'FAIL'}  {name}: {detail}")

    print("Known answer, 6 seeds (noise 0.12 mm, 0.15 mm arm error, reflections, 20 ms delay, dots blocked and merging):")
    tips, axes = [], []
    for seed in range(6):
        robot_t, robot_T, st, sx, base, tip, shaft = simulate(seed)
        result, warnings = analyse(fit(robot_t, robot_T, st, sx), extension)
        tips.append(np.linalg.norm(np.array(result["tip_tool0_mm"]) - (tip + extension * shaft)))
        axes.append(np.degrees(np.arccos(np.clip(np.array(result["direction_tool0"]) @ shaft, -1, 1))))
    check("tip within 0.5 mm", max(tips) < 0.5, f"worst {max(tips):.3f} mm")
    check("axis within 0.1 deg", max(axes) < 0.1, f"worst {max(axes):.4f} deg")
    robot_t, robot_T, st, sx, base, tip, shaft = simulate(0)
    r = fit(robot_t, robot_T, st, sx)
    result, warnings = analyse(r, extension)
    check("no warnings on good data", not warnings, str(warnings) or "none")
    check("delay found", abs(r["delay"] - 0.02) < 0.011, f"{1e3 * r['delay']:.0f} ms (true 20)")
    check("shaft base", np.linalg.norm(np.array(result["shaft_base_tool0_mm"]) - base) < 0.5,
          f"{np.linalg.norm(np.array(result['shaft_base_tool0_mm']) - base):.3f} mm")
    no_extension, warnings = analyse(r, 0.0)
    check("extension 0 is flagged", any("extension" in w for w in warnings), "warned")

    print("Must be caught, not given as an answer:")
    for missing in range(DOTS):
        try:
            data = simulate(1, dropped=missing)
            analyse(fit(*data[:4]), extension)
            check(f"dot {missing} missing", False, "no error raised")
        except SystemExit as exc:
            check(f"dot {missing} missing", True, str(exc)[:70])
    data = simulate(2, bump=True)
    _, warnings = analyse(fit(*data[:4]), extension)
    check("dot moved halfway", any("halves" in w or "rms" in w for w in warnings), "; ".join(warnings)[:90] or "no warning")
    data = simulate(3, tilt_deg=6.0)
    _, warnings = analyse(fit(*data[:4]), extension)
    check("too little tilt", any("tilted" in w for w in warnings), "; ".join(warnings)[:90] or "no warning")
    print(f"\n{'ALL PASSED' if not failures else str(failures) + ' FAILED'}")
    return 1 if failures else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-d", "--duration", type=float, default=200.0)
    ap.add_argument("--extension", type=float, default=0.0,
                    help="mm from the midpoint of the tip dots to the true tip, along the shaft line")
    ap.add_argument("--measured-span", type=float, help="mm, base dot pair to tip dot pair by ruler: checks the fit")
    ap.add_argument("--name", default="micro_tool", help="file name stem in config/calibrations/")
    ap.add_argument("--file", help="re-fit a saved config/calibrations/raw/*.npz instead of recording")
    ap.add_argument("--self-test", action="store_true", help="synthetic data, no hardware")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--wand-rom", default="8700340")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--ee-frame", default="tool0")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if args.file:
        d = np.load(os.path.expanduser(args.file))
        robot_t, robot_T, sight_t, sight_xyz = d["robot_t"], d["robot_T"], d["sight_t"], d["sight_xyz"]
    else:
        robot_t, robot_T, sight_t, sight_xyz = record(args)
        os.makedirs(os.path.join(OUT_DIR, "raw"), exist_ok=True)
        raw = os.path.join(OUT_DIR, "raw", f"{args.name}_{datetime.now():%Y-%m-%d_%H%M%S}.npz")
        np.savez(raw, robot_t=robot_t, robot_T=robot_T, sight_t=sight_t, sight_xyz=sight_xyz)
        print(f"raw recording: {raw}  (re-fit with --file {raw})")
    result, warnings = analyse(fit(robot_t, robot_T, sight_t, sight_xyz), args.extension, args.measured_span)
    report(result, warnings)

    path = os.path.join(OUT_DIR, f"{args.name}_{datetime.now():%Y-%m-%d_%H%M%S}.json")
    with open(path, "w") as f:
        json.dump({"date": datetime.now().isoformat(timespec="seconds"), "method": "four_dots_no_wand",
                   "base_frame": args.base_frame, "ee_frame": args.ee_frame, "warnings": warnings, **result}, f, indent=2)
    print(f"\nsaved {path}\nadd it:  ros2 run macro_micro_robot tool_library.py add 2 \"Micro module\" --from {path} "
          f"--length {result['shaft_length_mm']:.1f} --force")


if __name__ == "__main__":
    main()
