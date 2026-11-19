#!/usr/bin/env python3
"""Regenerate every result (or the ones named), in dependency order, and record which recordings each one reads.

    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    cd thesis/results
    python3 run_all.py              # self-test, then every folder (about 30 min with a warm bag cache)
    python3 run_all.py R02 R03      # only these, in the order given

Each folder's RESULTS.md and figures/ are rewritten. Console output goes to <folder>/run.log (not in git). The
recordings each folder read are merged into data/used_by.json, which data/make_index.py turns into
data/RECORDINGS.md. Stops at the first failure.

Order constraints: R03 and R05 read R02's cache/runs.json; L06 reads L09's measured_filter_lag.json (committed) and
L09 reads L06's RESULTS.md, so after a change to either, run L06, L09, L06.
"""
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from common import runs  # noqa: E402

ORDER = ['M01', 'L08', 'L07', 'L03', 'R02', 'R03', 'R04', 'S02', 'R01', 'T01', 'L01', 'L11', 'L06', 'L02', 'L04',
         'L05', 'L10', 'S01', 'T02', 'M02', 'L09', 'R05']
USED_BY = os.path.join(runs.PKG, 'data', 'used_by.json')


def main():
    ids = sys.argv[1:] or ORDER
    if not sys.argv[1:]:
        print('self-test ...', flush=True)
        subprocess.run([sys.executable, '-m', 'common.selftest'], cwd=HERE, check=True)
    used = json.load(open(USED_BY)) if os.path.exists(USED_BY) else {}
    for fid in ids:
        d = runs.folder(fid)
        t0 = time.time()
        with tempfile.NamedTemporaryFile(suffix='.json') as tmp, open(os.path.join(d, 'run.log'), 'w') as log:
            r = subprocess.run([sys.executable, '-m', 'common.trace_inputs', os.path.join(d, 'analysis.py'), tmp.name],
                               cwd=HERE, stdout=log, stderr=subprocess.STDOUT)
            if r.returncode:
                sys.exit(f'{fid} failed (exit {r.returncode}); see {os.path.relpath(d, HERE)}/run.log')
            used[os.path.basename(d)] = json.load(open(tmp.name))
        print(f'{fid}  {time.time() - t0:5.0f} s  {len(used[os.path.basename(d)]["runs"])} recordings', flush=True)
    json.dump(dict(sorted(used.items())), open(USED_BY, 'w'), indent=1)
    print(f'wrote {os.path.relpath(USED_BY, runs.PKG)}')


if __name__ == '__main__':
    main()
