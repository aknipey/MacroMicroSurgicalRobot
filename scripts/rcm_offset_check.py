#!/usr/bin/env python3
"""Where did each side put the RCM, and where did the arm really pivot? All in base_link (mm).

    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    cd ~/ros2_ws/src/MacroMicroSurgicalRobot/logs/V7
    python3 ../../scripts/rcm_offset_check.py FastCheckingRCMSimpleHolder_Replay1 ...

Per run: the teleop's latched RCM point (bag /ur16e_touch_teleop/rcm_point, or
/ur5e_touch_teleop/rcm_point in bags recorded before the 2026-11-02 rename; the last one:
it is latched, so the first can be the previous run's), the NDI's latched point, the pivot
of the kinematic shaft lines (UR_PATH tool0 + the run's TOUCH_RCM_SHAFT_* settings) and of
the NDI-measured shaft lines. The camera pose in base_link comes from pairing UR_PATH tool0
with the NDI wand pose through the tool0 -> wand hand-eye of HANDEYE below. Written for the
2026-10-26 replays (logs/V7/REPLAYS_2026-10-26.md); for runs on another wand seat, point
HANDEYE at that day's calibration.
"""
import sys, csv, json, yaml, numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
sys.path.insert(0, '/home/aidan-knipe/ros2_ws/src/ndi_tracking')
from ndi_tracking.calib import ToolCalibration

HANDEYE = '/home/aidan-knipe/ros2_ws/src/ndi_tracking/calibrations/robot_tcp_handeye_2026-10-26.json'
X = np.array(json.load(open(HANDEYE))['handeye']['tool0_to_wand'])
wand = ToolCalibration.load('8700340')

def T(p, R):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = p; return M

def pivot(points, dirs):
    """Point closest (least squares) to all lines p + s d."""
    A = np.zeros((3, 3)); b = np.zeros(3)
    for p, d in zip(points, dirs):
        P = np.eye(3) - np.outer(d, d); A += P; b += P @ p
    c = np.linalg.solve(A, b)
    e = [np.linalg.norm((np.eye(3) - np.outer(d, d)) @ (c - p)) for p, d in zip(points, dirs)]
    return c, float(np.sqrt(np.mean(np.square(e))))

def line_dist(c, p, d):
    return np.linalg.norm((np.eye(3) - np.outer(d, d)) @ (c - p), axis=-1) if np.ndim(p) == 1 else \
        np.array([np.linalg.norm((np.eye(3) - np.outer(di, di)) @ (c - pi)) for pi, di in zip(p, d)])

def run(d):
    meta = yaml.safe_load(open(f'{d}/META.yaml')); cfg = meta['settings']
    S0 = 1000 * np.array(cfg['TOUCH_RCM_SHAFT_START']); D0 = np.array(cfg['TOUCH_RCM_SHAFT_DIRECTION']); D0 /= np.linalg.norm(D0)
    L = 1000 * cfg['TOUCH_RCM_SHAFT_LENGTH']
    # UR tool0 in base (mm)
    ur = [r for r in csv.DictReader(open(f'{d}/UR_PATH.csv')) if r['source'] in ('rtde', 'tf')]
    ut = np.array([float(r['t']) for r in ur])
    up = 1000 * np.array([[float(r[k]) for k in 'xyz'] for r in ur])
    uR = Rot.from_quat([[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in ur]).as_matrix()
    # NDI raw wand pose in camera (mm)
    nd = [r for r in csv.DictReader(open(f'{d}/NDI_PATH.csv')) if r['visible'] == '1']
    nt = np.array([float(r['t']) for r in nd])
    nT = np.array([T([float(r[k]) for k in 'xyz'], Rot.from_quat([float(r[k]) for k in ('qx', 'qy', 'qz', 'q0')]).as_matrix()) for r in nd])
    rcm_on = np.array([r['rcm_active'] == '1' for r in nd])
    ndi_rcm = np.array([float(nd[np.argmax(rcm_on)][k]) for k in ('rcm_x', 'rcm_y', 'rcm_z')])
    # camera pose in base from matched samples
    k = np.clip(np.searchsorted(ut, nt), 1, len(ut) - 1); k -= (np.abs(ut[k - 1] - nt) < np.abs(ut[k] - nt))
    good = np.abs(ut[k] - nt) < 0.005
    C = np.array([T(up[i], uR[i]) @ X @ np.linalg.inv(b) for i, b in zip(k[good], nT[good])])
    U, _, Vt = np.linalg.svd(C[:, :3, :3].sum(0)); Cb = T(C[:, :3, 3].mean(0), U @ Vt)
    spread = np.sqrt(np.mean(np.sum((C[:, :3, 3] - Cb[:3, 3]) ** 2, 1)))
    cam2base = lambda p: Cb[:3, :3] @ p + Cb[:3, 3]
    # teleop latched point + time (bag)
    rd = SequentialReader(); rd.open(StorageOptions(uri=f'{d}/rosbag', storage_id='mcap'), ConverterOptions('', ''))
    rd.set_filter(StorageFilter(topics=[f'/{node}/{t}' for node in ('ur16e_touch_teleop', 'ur5e_touch_teleop') for t in ('rcm_point', 'rcm_active')]))
    tele_rcm = None
    while rd.has_next():
        topic, data, _ = rd.read_next()
        if topic.endswith('rcm_point'):   # latched: the first one can be the previous run's; keep the last
            m = deserialize_message(data, PointStamped); tele_rcm = 1000 * np.array([m.point.x, m.point.y, m.point.z])
    seg = (nt >= nt[rcm_on][0]) & rcm_on
    # kinematic shaft lines during the segment (the calibrated model), at the NDI sample times
    ks = k[seg & good]
    kin_p = np.array([up[i] + uR[i] @ S0 for i in ks]); kin_d = np.array([uR[i] @ D0 for i in ks])
    kin_piv, kin_rms = pivot(kin_p, kin_d)
    # measured shaft lines (NDI wand tip + axis), mapped to base
    ndi_tip = np.array([cam2base((b @ np.append(wand.tip, 1))[:3]) for b in nT[seg & good]])
    ndi_d = np.array([Cb[:3, :3] @ (b[:3, :3] @ -wand.axis) for b in nT[seg & good]])
    ndi_piv, ndi_rms = pivot(ndi_tip, ndi_d)
    # tip at the latch: kinematic vs NDI
    i0 = ks[0]; kin_tip0 = up[i0] + uR[i0] @ (S0 + L * D0); ndi_tip0 = ndi_tip[0]
    ndi_rcm_b = cam2base(ndi_rcm)
    f = lambda v: '[' + ' '.join(f'{x:7.2f}' for x in v) + ']'
    print(f'\n== {d}   (camera-in-base spread {spread:.2f} mm, {good.sum()} pairs)')
    print(f'  teleop latched RCM          {f(tele_rcm)}')
    print(f'  NDI latched RCM             {f(ndi_rcm_b)}   {np.linalg.norm(ndi_rcm_b - tele_rcm):5.2f} mm from teleop')
    print(f'  kinematic pivot (model)     {f(kin_piv)}   {np.linalg.norm(kin_piv - tele_rcm):5.2f} mm from teleop  (lines rms {kin_rms:.2f})')
    print(f'  NDI-measured pivot          {f(ndi_piv)}   {np.linalg.norm(ndi_piv - tele_rcm):5.2f} mm from teleop  (lines rms {ndi_rms:.2f})')
    print(f'  NDI pivot - kinematic pivot {f(ndi_piv - kin_piv)}   {np.linalg.norm(ndi_piv - kin_piv):5.2f} mm')
    print(f'  tip at latch: kin vs NDI    {np.linalg.norm(kin_tip0 - ndi_tip0):5.2f} mm   NDI tip vs NDI latched {np.linalg.norm(ndi_tip0 - ndi_rcm_b):5.2f} mm')
    print(f'  NDI shaft line dist to: teleop pt rms {np.sqrt(np.mean(line_dist(tele_rcm, ndi_tip, ndi_d)**2)):.2f}, NDI pt rms {np.sqrt(np.mean(line_dist(ndi_rcm_b, ndi_tip, ndi_d)**2)):.2f}')

for d in sys.argv[1:]:
    run(d)
