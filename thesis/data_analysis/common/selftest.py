#!/usr/bin/env python3
"""Self-test of the shared code on real data; prints PASS/FAIL lines. Run: python3 -m common.selftest"""
import numpy as np
from scipy.spatial.transform import Rotation
from . import bags, kin, runs, track

run = runs.Run(runs.LOGS + '/V9b/NoTool/PositionMode/LongBenchmark_Replay1')
b = bags.load(run)
js = b['/joint_states']
dt = np.diff(js['th'])
print(f"joint_states: {len(dt)+1} msgs, dt median {np.median(dt)*1e3:.2f} ms, p99 {np.percentile(dt,99)*1e3:.2f} ms")
print('topics:', sorted(b))
print('mode', run.mode, 'traj', run.trajectory, 'scale', run.scale, 'kin', run.kinematics[0])

# 1. FK (calibrated) vs controller TCP from the same RTDE packets: constant offset expected
K = kin.load('calibrated')
tcp = b['/tcp_pose_broadcaster/pose']
idx = np.searchsorted(js['th'], tcp['th'])
idx = np.clip(idx, 0, len(js['th']) - 1)
ok = np.abs(js['th'][idx] - tcp['th']) < 5e-4
sel = np.where(ok)[0][::25]
d = []
for i in sel:
    T = kin.kc.tool0_in_base(K, js['q'][idx[i]])        # the TCP topic is in the controller's 'base' frame
    d.append(np.linalg.inv(T) @ np.r_[tcp['p'][i], 1.0])
d = np.array(d)[:, :3] * 1000
print(f"FK vs TCP: TCP in tool0 = {d.mean(0).round(2)} mm, spread (max-min) {np.ptp(d,0).round(3)} mm "
      f"-> {'PASS' if np.ptp(d,0).max() < 0.05 else 'FAIL'}; META tcp_offset {run.meta['tcp_offset']['xyz_mm']}")

# 2. clock: FK tool0 on the CSV clock vs UR_PATH rtde rows
off = bags.to_run_clock(run, b)
ur = run.csv('UR_PATH.csv')
m = ur['source'] == 'rtde'
tq = ur['t'][m][::50]
Pur = np.c_[ur['x'], ur['y'], ur['z']][m][::50] * 1000
tj = js['tr'] - off
in_bag = (tq > tj[0] + 0.01) & (tq < tj[-1] - 0.01)
print(f'bag starts {tj[0]:.2f} s after the CSV recording; ends {tj[-1]:.2f} s')
tq, Pur = tq[in_bag], Pur[in_bag]
Pfk = np.array([kin.tool0(K, np.array([np.interp(t, tj, js['q'][:, k]) for k in range(6)]))[:3, 3] for t in tq]) * 1000
e = np.linalg.norm(Pfk - Pur, axis=1)
print(f"clock map: FK(joint_states) vs UR_PATH on CSV clock: rms {np.sqrt(np.mean(e**2)):.3f} mm, max {e.max():.3f} "
      f"-> {'PASS' if e.max() < 0.5 else 'FAIL'}")

# 3. lag estimators on synthetic data: arm = target delayed by D + noise
(tt, tp, tqq), _ = track.load_paths(run)
for D in (0.030, 0.075, 0.180):
    ut = tt + D
    rng = np.random.default_rng(0)
    up = tp + rng.normal(0, 0.05, tp.shape)
    m = track.metrics_arrays(tt, tp, tqq, ut, up, tqq)
    print(f"synthetic delay {D*1e3:.0f} ms -> lag_pos {m['lag_ms']:.1f}, lag_vel {m['lag_vel_ms']:.1f}, "
          f"lag_rot {m['lag_rot_ms']:.1f} ms -> {'PASS' if max(abs(m['lag_ms']-D*1e3), abs(m['lag_vel_ms']-D*1e3), abs(m['lag_rot_ms']-D*1e3)) < 2 else 'FAIL'}")
