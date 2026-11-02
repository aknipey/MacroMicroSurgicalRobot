#!/usr/bin/env python3
"""Quick-change exchange repeatability from an NDI recording of repeated swaps (preliminary, R8.2).

    cd ~/ros2_ws/src/MacroMicroSurgicalRobot/logs/Exchange
    python3 ../../thesis/planning/analysis/exchange_repeatability.py 2026*_QC_*

A "still segment" is >= 2 s where the wand is visible (> 90 % of frames) and the tip moves < 0.3 mm over every
1 s window. Each segment is one seated state; the gap between segments is taken as one swap. Reports the tip and
shaft-axis offset of each seated state from the session median, the gap durations and the spread.
Assumes the arm and camera are static for the whole recording (camera-frame positions).
"""
import sys, csv
import numpy as np
for d in sys.argv[1:]:
    rows=list(csv.DictReader(open(d+'/NDI_PATH.csv')))
    t=np.array([float(r['t']) for r in rows]); vis=np.array([r['visible']=='1' for r in rows])
    P=np.array([[float(r['tip_x'] or 'nan'),float(r['tip_y'] or 'nan'),float(r['tip_z'] or 'nan')] for r in rows])
    A=np.array([[float(r['axis_x'] or 'nan'),float(r['axis_y'] or 'nan'),float(r['axis_z'] or 'nan')] for r in rows])
    # still = visible and moved < 0.3 mm over a 1 s window
    still=np.zeros(len(t),bool)
    for i in range(len(t)):
        if not vis[i]: continue
        j=np.searchsorted(t,t[i]+1.0)
        seg=P[i:j][vis[i:j]]
        if len(seg)>30 and np.ptp(seg,0).max()<0.3 and vis[i:j].mean()>0.9: still[i:j]=True
    # segments
    segs=[];i=0
    while i<len(t):
        if still[i]:
            j=i
            while j<len(t) and still[j]: j+=1
            if t[j-1]-t[i]>=2.0: segs.append((t[i],t[j-1],np.nanmedian(P[i:j][vis[i:j]],0),np.nanmedian(A[i:j][vis[i:j]],0)))
            i=j
        else: i+=1
    print(f'== {d}: {len(segs)} still segments, visible {vis.mean():.0%}')
    if not segs: continue
    C=np.array([s[2] for s in segs]); m=np.median(C,0)
    for k,(a,b,p,ax) in enumerate(segs):
        gap = a-segs[k-1][1] if k else float('nan')
        ang=np.degrees(np.arccos(np.clip(np.dot(ax/np.linalg.norm(ax),np.median([s[3] for s in segs],0)/np.linalg.norm(np.median([s[3] for s in segs],0))),-1,1)))
        print(f'  {a:6.1f}-{b:6.1f}s  gap {gap:5.1f}s  tip off median {np.linalg.norm(p-m):6.2f} mm  axis {ang:5.2f} deg')
    dev=np.linalg.norm(C-m,axis=1); print(f'  tip spread: rms {np.sqrt((dev**2).mean()):.2f} mm, max {dev.max():.2f} mm')
