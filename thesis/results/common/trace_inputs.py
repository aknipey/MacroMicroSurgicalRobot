"""Run one analysis script and record which recordings it reads (used by run_all.py).

    python3 -m common.trace_inputs <path/to/analysis.py> <out.json>

Records every file the script opens and every bag it loads (bags.load, which may be served from the cache without
opening the bag). A run counts as read when any of its data files is opened; META.yaml, REPLAY.yaml, NDI_META.yaml and
rosbag/metadata.yaml are only run discovery and do not count, unless the analysis opened that run by name
(runs.Run(path) outside runs.find_runs), as R01 does to read the law settings from META. Writes {"runs": [...], "files": [...]} with paths
relative to data/recordings, and "config": files read from config/.
"""
import json
import os
import runpy
import sys

DISCOVERY = {'META.yaml', 'REPLAY.yaml', 'NDI_META.yaml', os.path.join('rosbag', 'metadata.yaml')}


def main():
    script, out = os.path.abspath(sys.argv[1]), sys.argv[2]
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from common import bags, runs
    rec, pkg = os.path.realpath(runs.LOGS), os.path.realpath(runs.PKG)
    seen = set()

    def hook(ev, args):
        if ev == 'open' and args and isinstance(args[0], (str, bytes, os.PathLike)):
            try:
                seen.add(os.path.realpath(os.fsdecode(args[0])))
            except (TypeError, ValueError):
                pass
    sys.addaudithook(hook)
    _load = bags.load

    def load(run, *a, **k):
        seen.add(os.path.join(os.path.realpath(run.path), 'rosbag', '(loaded)'))
        return _load(run, *a, **k)
    bags.load = load

    named, depth = set(), [0]
    _init, _find = runs.Run.__init__, runs.find_runs

    def init(self, path, *a, **k):
        if not depth[0]:
            named.add(os.path.realpath(path))
        return _init(self, path, *a, **k)

    def find_runs(*a, **k):
        depth[0] += 1
        try:
            return _find(*a, **k)
        finally:
            depth[0] -= 1
    runs.Run.__init__, runs.find_runs = init, find_runs

    sys.argv = [script]
    try:
        runpy.run_path(script, run_name='__main__')
    finally:
        files, config = set(), set()
        used = {os.path.relpath(p, rec) for p in named if p.startswith(rec + os.sep)}
        for p in seen:
            if p.startswith(rec + os.sep):
                rel = os.path.relpath(p, rec)
                d = os.path.dirname(rel)
                while d and not os.path.exists(os.path.join(rec, d, 'META.yaml')):
                    d = os.path.dirname(d)
                inner = os.path.relpath(rel, d) if d else rel
                if inner in DISCOVERY:
                    continue
                files.add(rel)
                used.add(d or os.path.dirname(rel))
            elif p.startswith(os.path.join(pkg, 'config') + os.sep):
                config.add(os.path.relpath(p, pkg))
        json.dump({'runs': sorted(used), 'files': sorted(files), 'config': sorted(config)}, open(out, 'w'), indent=0)


if __name__ == '__main__':
    main()
