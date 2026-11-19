#!/usr/bin/env python3
"""E0: predict the calibrated-kinematics RCM error from bags already recorded (no lab time).

    source /opt/ros/jazzy/setup.bash
    cd ~/ros2_ws/src/MacroMicroSurgicalRobot
    python3 thesis/earlier/E0_calibrated_kinematics/e0_offline.py            # 10-26 + 11-03 sets
    python3 thesis/earlier/E0_calibrated_kinematics/e0_offline.py logs/V7/SlowCheckingRCMSimpleHolder_Replay1

Read-only on logs/. Plan: docs/lab_plans/07_remaining_experiments.md §1.4.

Per run, the same joint angles (/joint_states, 500 Hz RTDE) go through two forward-kinematics models:
nominal (ur_description UR16e default) and calibrated (this arm's controller calibration, 11-03 file).
For each model the hand-eye (base->camera Y, tool0->wand X) is fitted to the NDI wand poses, pooled over
every run of a day (camera and wand seat fixed within a day), with one NDI delay. Then:

  fit residual   tip mismatch |A X tip - Y B tip| over all samples: how well the model explains the
                 measured wand motion (pose-dependent model error cannot be absorbed by constant X, Y)
  offset field   per sample in the RCM segment, the measured shaft line (NDI through Y) minus the model
                 shaft line (FK through X), perpendicular to the shaft, at the RCM point
  predicted      if a controller held the *model* lines exactly through its point, the NDI would see
                 |o_i - o_latch| (perpendicular to the shaft): the RCM error that model alone causes.
                 For the model the run actually used, measured ~ predicted (+ the controller's own error);
                 for the other model it is the counterfactual.
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
NDI_CAL = os.path.expanduser('~/ros2_ws/src/ndi_tracking/calibrations')
MODELS = {
    'nominal': '/opt/ros/jazzy/share/ur_description/config/ur16e/default_kinematics.yaml',
    'calibrated': os.path.join(PKG, 'config/calibrations/ur16e_kinematics_2026-11-03.yaml'),
}
DAYS = {
    '10-26': sorted(glob.glob(os.path.join(PKG, 'logs/V7/*SimpleHolder_Replay[0-9]'))),
    '11-03': sorted(glob.glob(os.path.join(PKG, 'logs/V8/CalibratedKinematics/*SimpleHolder_Replay[0-9]'))),
}
FIT_SAMPLES = 1500          # per run, for the camera fit
# wrist-jog hand-eye recordings (ndi_tracking robot-tcp --wand-is-tool): TF tool0 + NDI wand, ~20 Hz robot side
JOGS = {
    '10-26': dict(raw=f'{NDI_CAL}/raw/robot_tcp_20261026_162504.npz', model='nominal', delay=0.010),
    '11-03': dict(raw=f'{NDI_CAL}/raw/robot_tcp_20261103_094111.npz', model='calibrated', delay=0.010),
}
DELAYS = np.arange(-0.040, 0.0601, 0.005)   # NDI host-receive delay searched (s)
LINKS = ['shoulder', 'upper_arm', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3']
JOINTS = [f'{j}_joint' for j in ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')]


# ---------------------------------------------------------------- kinematics (as thesis/run_tools/kinematics_check.py)
def transform(rpy=(0.0, 0.0, 0.0), xyz=(0.0, 0.0, 0.0)):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler('ZYX', [rpy[2], rpy[1], rpy[0]]).as_matrix()
    m[:3, 3] = xyz
    return m


BASE_LINK_TO_INERTIA = transform((0, 0, np.pi))
WRIST3_TO_TOOL0 = transform((0, -np.pi / 2, -np.pi / 2)) @ transform((np.pi / 2, 0, np.pi / 2))


def fk(kinematics, q):
    """tool0 in base_link (the teleop's frame), mm, for joint angles q (N, 6)."""
    m = np.broadcast_to(BASE_LINK_TO_INERTIA, (len(q), 4, 4)).copy()
    for j, name in enumerate(LINKS):
        k = kinematics[name]
        m = m @ transform((k['roll'], k['pitch'], k['yaw']), (k['x'], k['y'], k['z']))
        rz = np.zeros((len(q), 4, 4))
        c, s = np.cos(q[:, j]), np.sin(q[:, j])
        rz[:, 0, 0], rz[:, 0, 1], rz[:, 1, 0], rz[:, 1, 1], rz[:, 2, 2], rz[:, 3, 3] = c, -s, s, c, 1, 1
        m = m @ rz
    m = m @ WRIST3_TO_TOOL0
    m[:, :3, 3] *= 1000
    return m


def pose(v):
    """6-vector (rotvec rad, xyz mm) -> 4x4."""
    m = np.eye(4)
    m[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    m[:3, 3] = v[3:]
    return m


def vec(m):
    return np.r_[Rotation.from_matrix(m[:3, :3]).as_rotvec(), m[:3, 3]]


# ---------------------------------------------------------------- data
def read_bag(path):
    import rosbag2_py
    from geometry_msgs.msg import PointStamped
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/joint_states'] + [
        f'/{node}/rcm_point' for node in ('ur16e_touch_teleop', 'ur5e_touch_teleop')]))
    stamps, q, rcm = [], [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/joint_states':
            msg = deserialize_message(data, JointState)
            positions = dict(zip(msg.name, msg.position))
            if all(j in positions for j in JOINTS):
                stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
                q.append([positions[j] for j in JOINTS])
        else:   # latched: the first can be the previous run's; keep the last
            p = deserialize_message(data, PointStamped).point
            rcm = 1000 * np.array([p.x, p.y, p.z])
    order = np.argsort(stamps)
    return np.array(stamps)[order], np.array(q)[order], rcm


def load_run(d):
    meta = yaml.safe_load(open(f'{d}/META.yaml'))
    ndi_meta = yaml.safe_load(open(f'{d}/NDI_META.yaml'))
    tool = (meta.get('tool') or {}).get('at_start')
    if tool and tool.get('tip_tool0_mm'):
        lib = yaml.safe_load(open(os.path.join(PKG, 'config/tools.yaml')))['tools'][tool['number']]
        tip, direction = np.array(lib['tip_tool0_mm']), np.array(lib['direction_tool0'])
    else:
        s = meta['settings']
        direction = np.array(s['TOUCH_RCM_SHAFT_DIRECTION'])
        direction /= np.linalg.norm(direction)
        tip = 1000 * (np.array(s['TOUCH_RCM_SHAFT_START']) + s['TOUCH_RCM_SHAFT_LENGTH'] * direction)
    # recorder time t <-> RTDE header stamp, from the controller TCP rows (same packet as /joint_states)
    ur = [r for r in csv.DictReader(open(f'{d}/UR_PATH.csv')) if r['source'] == 'rtde']
    ur_t = np.array([float(r['t']) for r in ur])
    ur_stamp = np.array([float(r['stamp']) for r in ur])
    stamps, q, tele_rcm = read_bag(f'{d}/rosbag')
    nd = [r for r in csv.DictReader(open(f'{d}/NDI_PATH.csv')) if r['visible'] == '1']
    f = lambda keys: np.array([[float(r[k]) for k in keys] for r in nd])
    B = np.array([np.eye(4)] * len(nd))
    B[:, :3, :3] = Rotation.from_quat(f(('qx', 'qy', 'qz', 'q0'))).as_matrix()
    B[:, :3, 3] = f('xyz')
    seg = max(ndi_meta['rcm_segments'], key=lambda s: s.get('samples', 0))
    nt = f(['t'])[:, 0]
    return dict(name=os.path.relpath(d, os.path.join(PKG, 'logs')), dir=d, model_used=(meta.get('kinematics') or {}).get('model', 'nominal'),
                tool_tip=tip, tool_dir=direction, tele_rcm=tele_rcm,
                ur_t=ur_t, ur_stamp=ur_stamp, stamps=stamps, q=q,
                nt=nt, B=B, ndi_tip=f(('tip_x', 'tip_y', 'tip_z')), ndi_axis=f(('axis_x', 'axis_y', 'axis_z')),
                ndi_rcm=np.array(seg['rcm_point_mm']), seg=(nt >= seg['t_start']) & (nt <= seg['t_end']),
                seg_meta=seg)


def joints_at(run, t):
    """Joint angles at recorder times t (NaN outside the recording)."""
    stamp = np.interp(t, run['ur_t'], run['ur_stamp'], left=np.nan, right=np.nan)
    return np.column_stack([np.interp(stamp, run['stamps'], run['q'][:, j], left=np.nan, right=np.nan)
                            for j in range(6)])


# ---------------------------------------------------------------- hand-eye
def wand_points():
    cal = json.load(open(f'{NDI_CAL}/8700340.json'))
    tip, axis = np.array(cal['tip_offset_mm']), np.array(cal['shaft_axis'])
    # tip, a point 100 mm up the shaft and the ROM origin (off-axis, fixes the twist)
    return tip, axis, np.array([tip, tip + 100 * axis, np.zeros(3)])


def point_residual(As, X, Y, Bs):
    ph = np.c_[wand_points()[2], np.ones(3)].T
    return ((As @ X @ ph) - (Y @ Bs @ ph))[:, :3, :]


def tip_errors(As, X, Y, Bs):
    tip_h = np.r_[wand_points()[0], 1]
    err = np.linalg.norm((As @ X @ tip_h)[:, :3] - (Y @ Bs @ tip_h)[:, :3], axis=1)
    rot = np.degrees(Rotation.from_matrix(np.swapaxes((As @ X)[:, :3, :3], 1, 2) @ (Y @ Bs)[:, :3, :3]).magnitude())
    return err, rot


def fit_xy(As, Bs, x0, y0):
    sol = least_squares(lambda p: point_residual(As, pose(p[:6]), pose(p[6:]), Bs).ravel(),
                        np.r_[vec(x0), vec(y0)], loss='soft_l1', f_scale=2.0)
    return pose(sol.x[:6]), pose(sol.x[6:])


def fit_y(As, Bs, X, y0):
    sol = least_squares(lambda p: point_residual(As, X, pose(p), Bs).ravel(), vec(y0), loss='soft_l1', f_scale=2.0)
    return pose(sol.x)


def camera_guess(As, Bs, X):
    C = As @ X @ np.linalg.inv(Bs)
    U, _, Vt = np.linalg.svd(C[:, :3, :3].sum(0))
    y0 = np.eye(4)
    y0[:3, :3], y0[:3, 3] = U @ Vt, np.median(C[:, :3, 3], 0)
    return y0


def ik(kinematics, targets, seed):
    """Joint angles reproducing tool0 poses (mm) through a model, each solve seeded by the previous one."""
    out, q = [], np.asarray(seed, float)
    for T in targets:
        def res(qq):
            m = fk(kinematics, qq[None])[0]
            return np.r_[m[:3, 3] - T[:3, 3], 1000 * (m[:3, :3] - T[:3, :3]).ravel()]
        sol = least_squares(res, q, xtol=1e-12, ftol=1e-12)
        q = sol.x
        out.append((q.copy(), np.abs(sol.fun[:3]).max()))
    return np.array([o[0] for o in out]), np.array([o[1] for o in out])


def jog_handeye(day, kin, seed):
    """X (tool0 -> wand) through each model, from that day's wrist-jog hand-eye recording.

    The recording holds TF tool0 (built from the model the session ran), not joints: invert that model,
    then push the joints through both models."""
    jog = JOGS[day]
    d = np.load(jog['raw'])
    used = jog['model']
    i = np.clip(np.searchsorted(d['robot_t'], d['ndi_t'] - jog['delay']), 1, len(d['robot_t']) - 1)
    i -= np.abs(d['robot_t'][i - 1] - (d['ndi_t'] - jog['delay'])) < np.abs(d['robot_t'][i] - (d['ndi_t'] - jog['delay']))
    keep = np.abs(d['robot_t'][i] - (d['ndi_t'] - jog['delay'])) <= 0.01
    Bs = d['ndi_T'][keep]
    robot_idx = np.unique(i[keep])
    q, ik_err = ik(kin[used], d['robot_T'][robot_idx], seed)
    if ik_err.max() > 1e-3:
        sys.exit(f'{jog["raw"]}: IK through {used} failed ({ik_err.max():.3g} mm)')
    q_at = q[np.searchsorted(robot_idx, i[keep])]
    x_seed = np.array(json.load(open(f'{NDI_CAL}/robot_tcp_handeye_2026-10-26.json'))['handeye']['tool0_to_wand'])
    out = {}
    for model in MODELS:
        As = fk(kin[model], q_at)
        X, Y = fit_xy(As, Bs, x_seed, camera_guess(As, Bs, x_seed))
        err, rot = tip_errors(As, X, Y, Bs)
        out[model] = dict(X=X, tip_rms=rms(err), tip_p95=float(np.percentile(err, 95)), rot_rms=rms(rot),
                          tip_tool0=(X @ np.r_[wand_points()[0], 1])[:3], samples=int(len(Bs)))
    return out


# ---------------------------------------------------------------- RCM geometry
def perp(v, d):
    return v - np.einsum('ij,ij->i', v, d)[:, None] * d


def pivot(points, dirs):
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for p, d in zip(points, dirs):
        P = np.eye(3) - np.outer(d, d)
        A += P
        b += P @ p
    return np.linalg.solve(A, b)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def rcm_analysis(run, A, X, Y):
    """Measured vs model shaft lines in base_link over the RCM segment (A = FK at the NDI samples)."""
    tip, axis, _ = wand_points()
    s = run['seg'] & run['ok']
    Bs, As = run['B'][s], A[s]
    # model line: tip and direction (tip -> handle) through FK and X
    k_tip = (As @ X @ np.r_[tip, 1])[:, :3]
    k_dir = (As[:, :3, :3] @ X[:3, :3] @ axis)
    # measured line, mapped to base through Y
    m_tip = (Y @ Bs @ np.r_[tip, 1])[:, :3]
    m_dir = (Y[:3, :3] @ Bs[:, :3, :3] @ axis)
    # the model's RCM: where its lines pivot; offsets are taken at that point
    P = pivot(k_tip, k_dir)
    k_pt = k_tip + np.einsum('ij,ij->i', P - k_tip, k_dir)[:, None] * k_dir
    m_pt = m_tip + np.einsum('ij,ij->i', P - m_tip, m_dir)[:, None] * m_dir
    o = perp(m_pt - k_pt, k_dir)
    latch = np.arange(len(o)) < max(1, int(0.1 * 60))     # NDI latches the tip over ~0.1 s
    pred_latch = np.linalg.norm(perp(o - o[latch].mean(0), k_dir), axis=1)
    # about the best constant offset (latch noise removed): least squares like a pivot
    c = pivot(-o, k_dir)
    pred_best = np.linalg.norm(perp(o + c, k_dir), axis=1)
    measured = np.linalg.norm(perp(run['ndi_rcm'] - run['ndi_tip'][s], run['ndi_axis'][s]), axis=1)
    kin_err = np.linalg.norm(perp(P - k_tip, k_dir), axis=1)
    m_piv = pivot(m_tip, m_dir)
    # how far the latched offset sits from the best constant: the part of the error set at the latch
    latch_shift = np.linalg.norm(perp((o[latch].mean(0) + c)[None], k_dir[:1]))
    return dict(measured=rms(measured), predicted=rms(pred_latch), predicted_best=rms(pred_best),
                latch_shift=float(latch_shift), model_pivot_rms=rms(kin_err),
                pivot_offset=float(np.linalg.norm(m_piv - P)), pivot=P.tolist(),
                offset_t=run['nt'][s], offset=o, k_dir=k_dir)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='*', help='run folders (default: the 10-26 and 11-03 SimpleHolder replays)')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results'))
    args = ap.parse_args()
    groups = {'cli': [os.path.abspath(r) for r in args.runs]} if args.runs else DAYS
    os.makedirs(args.out, exist_ok=True)
    kin = {m: yaml.safe_load(open(p))['kinematics'] for m, p in MODELS.items()}
    results = {}

    for day, dirs in groups.items():
        runs = []
        for d in dirs:
            print(f'loading {d}', file=sys.stderr)
            runs.append(load_run(d))
        jog_day = day if day in JOGS else ('11-03' if 'V8' in dirs[0] else '10-26')
        jog = jog_handeye(jog_day, kin, runs[0]['q'][0])
        rng = np.random.default_rng(0)
        for model in MODELS:
            X = jog[model]['X']
            j = jog[model]
            print(f'\n[{day}] {model:10s} wrist-jog hand-eye ({j["samples"]} samples): tip residual rms {j["tip_rms"]:.2f} mm '
                  f'(p95 {j["tip_p95"]:.2f}), rotation rms {j["rot_rms"]:.3f} deg, '
                  f'tip in tool0 [{j["tip_tool0"][0]:.1f} {j["tip_tool0"][1]:.1f} {j["tip_tool0"][2]:.1f}]')
            results.setdefault('_handeye', {})[f'{day}/{model}'] = dict(
                jog=JOGS[jog_day]['raw'], tip_residual_rms_mm=j['tip_rms'], tip_residual_p95_mm=j['tip_p95'],
                rotation_rms_deg=j['rot_rms'], tip_tool0_mm=j['tip_tool0'].tolist(), tool0_to_wand=X.tolist())
            for run in runs:
                # camera pose (and NDI delay) per run, X fixed
                best = None
                for delay in DELAYS:
                    q = joints_at(run, run['nt'] - delay)
                    sel = np.flatnonzero(~np.isnan(q).any(1))
                    sel = rng.choice(sel, min(FIT_SAMPLES, len(sel)), replace=False)
                    As, Bs = fk(kin[model], q[sel]), run['B'][sel]
                    Y = fit_y(As, Bs, X, camera_guess(As, Bs, X) if best is None else best[1])
                    err, _ = tip_errors(As, X, Y, Bs)
                    if best is None or rms(err) < best[2]:
                        best = (delay, Y, rms(err))
                delay, Y, fit = best
                q = joints_at(run, run['nt'] - delay)
                run['ok'] = ~np.isnan(q).any(1)
                A = np.full((len(q), 4, 4), np.nan)
                A[run['ok']] = fk(kin[model], q[run['ok']])
                r = rcm_analysis(run, A, X, Y)
                r.update(fit_rms=fit, delay_s=float(delay), base_to_camera=Y.tolist())
                results.setdefault(run['name'], {'model_used': run['model_used'],
                                                 'ndi_meta_rms': run['seg_meta']['error_rms_mm'],
                                                 'ndi_meta_pivot_offset': run['seg_meta'].get('fitted_pivot_offset_mm'),
                                                 'swing_deg': run['seg_meta']['shaft_swing_deg']})[model] = r
                print(f'  {run["name"]:52s} fit {fit:.2f} ({1e3 * delay:+.0f} ms)  measured {r["measured"]:.2f}  '
                      f'predicted {r["predicted"]:.2f} (best {r["predicted_best"]:.2f})  '
                      f'latch shift {r["latch_shift"]:.2f}  model-lines pivot rms {r["model_pivot_rms"]:.2f}  '
                      f'NDI-vs-model pivot {r["pivot_offset"]:.2f}')
            # camera fits compared where they matter, at the day's RCM points: a fixed camera and a model
            # that explains every pose would put them all in the same place
            names = [run['name'] for run in runs]
            P = np.mean([results[n][model]['pivot'] for n in names], 0)
            Ys = [np.array(results[n][model]['base_to_camera']) for n in names]
            c = np.linalg.inv(Ys[0]) @ np.r_[P, 1]
            pts = np.array([(Y @ c)[:3] for Y in Ys])
            spread = np.linalg.norm(pts - np.median(pts, 0), axis=1)
            results['_handeye'][f'{day}/{model}']['camera_at_rcm_spread_mm'] = dict(zip(names, spread.round(2).tolist()))
            print(f'  camera fits at the RCM points: max {spread.max():.2f} mm from the median '
                  f'({", ".join(f"{n.split("/")[-1].replace("SimpleHolder", "")} {d:.1f}" for n, d in zip(names, spread))})')

    summary = {k: ({m: {kk: vv for kk, vv in v.items() if not isinstance(vv, np.ndarray)} if isinstance(v, dict) else v
                    for m, v in r.items()} if not k.startswith('_') else r) for k, r in results.items()}
    with open(os.path.join(args.out, 'e0_results.yaml'), 'w') as fh:
        yaml.safe_dump(summary, fh, sort_keys=False, width=120)
    np.save(os.path.join(args.out, 'e0_offsets.npy'), {k: {m: {kk: r[m][kk] for kk in ('offset_t', 'offset', 'k_dir')}
                                                         for m in MODELS} for k, r in results.items()
                                                    if not k.startswith('_')}, allow_pickle=True)
    print(f'\nwrote {args.out}/e0_results.yaml')


if __name__ == '__main__':
    main()
