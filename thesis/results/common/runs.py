"""Run discovery and classification from each run's own files (META.yaml, REPLAY.yaml, bag topics).

Never trusts folder names for anything but the trajectory label; mode, filter, kinematics and code version
come from META. CSVs are read as numpy arrays; nothing in the recordings is written.
"""
import csv
import glob
import hashlib
import json
import os
import re

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                                    # thesis/results
PKG = os.path.abspath(os.path.join(ROOT, '..', '..'))           # the package
LOGS = os.path.join(PKG, 'data', 'recordings')                  # the recordings the report uses (data/README.md)


def folder(fid):
    """Result folder by ID, e.g. folder('L07') -> thesis/results/4.1_latency/L07_vibration_reduction."""
    hits = glob.glob(os.path.join(ROOT, '*', fid + '_*'))
    if len(hits) != 1:
        raise FileNotFoundError(f'result folder {fid}: {hits}')
    return hits[0]
JOINTS = [f'{j}_joint' for j in ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')]


def read_csv(path, cols=None):
    """{column: float array}; empty strings -> nan; non-numeric columns kept as str arrays."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    out = {}
    for k in (cols or rows[0].keys()):
        vals = [r.get(k, '') for r in rows]
        try:
            out[k] = np.array([float(v) if v not in ('', None) else np.nan for v in vals])
        except ValueError:
            out[k] = np.array(vals)
    return out


class Run:
    def __init__(self, path):
        self.path = os.path.abspath(path)
        self.key = os.path.relpath(self.path, LOGS)
        mp = os.path.join(self.path, 'META.yaml')
        self.meta = yaml.safe_load(open(mp)) if os.path.exists(mp) else {}
        rp = os.path.join(self.path, 'REPLAY.yaml')
        self.replay = yaml.safe_load(open(rp)) if os.path.exists(rp) else {}
        self.settings = self.meta.get('settings', {}) or {}

    # ---- identity -------------------------------------------------------------------------------
    @property
    def name(self):
        return os.path.basename(self.path)

    @property
    def trajectory(self):
        """Input label: the original's folder name for a replay, else this folder's, without _ReplayN."""
        src = self.replay.get('replay_of')
        base = os.path.basename(src) if src else self.name
        return re.sub(r'_Replay\d+$', '', base)

    @property
    def replay_n(self):
        m = re.search(r'_Replay(\d+)$', self.name)
        return int(m.group(1)) if m else 0

    @property
    def code_version(self):
        return self.meta.get('code_version')

    @property
    def date(self):
        return str(self.meta.get('date', ''))[:10]

    @property
    def kinematics(self):
        k = self.meta.get('kinematics') or {}
        return k.get('model', 'nominal' if k == {} else None), k.get('file')

    @property
    def mode(self):
        """trajectory / position / velocity_optimised / velocity / velocity_unfiltered, from META (V1: bag)."""
        s = self.settings
        out = s.get('SERVO_OUTPUT')
        if out is None:
            topics = bag_topics(self)
            if any('forward_position_controller' in t for t in topics):
                return 'position_v1'
            if any('joint_trajectory' in t for t in topics):
                return 'trajectory'
            if any('forward_velocity_controller' in t for t in topics):
                return 'velocity'
            return 'unknown'
        if out == 'velocity':
            if not s.get('TOUCH_COMMAND_FILTER') and not s.get('TOUCH_KALMAN'):
                return 'velocity_unfiltered'
            return 'velocity_optimised' if s.get('SERVO_SMOOTH_SPEEDJ') else 'velocity'
        return out

    @property
    def scale(self):
        """Motion scale used: from the replay settings if present, else TOUCH_TRANSLATION_SCALE."""
        if self.replay.get('scale_step') is not None:
            return float(self.replay['scale_step'])
        return float(self.settings.get('TOUCH_TRANSLATION_SCALE', 1.0))

    def settings_hash(self, ignore=('REPLAY_LEAD_IN',)):
        s = {k: v for k, v in self.settings.items() if k not in ignore}
        return hashlib.md5(json.dumps(s, sort_keys=True, default=str).encode()).hexdigest()[:10]

    # ---- data -----------------------------------------------------------------------------------
    def csv(self, name, cols=None):
        p = os.path.join(self.path, name)
        return read_csv(p, cols) if os.path.exists(p) else None

    def has(self, name):
        return os.path.exists(os.path.join(self.path, name))

    def __repr__(self):
        return f'Run({self.key}, {self.mode})'


def bag_topics(run):
    m = os.path.join(run.path, 'rosbag', 'metadata.yaml')
    if not os.path.exists(m):
        return []
    return re.findall(r'^\s+name:\s+(/\S+)', open(m).read(), re.M)


def find_runs(pattern):
    """Runs (folders with META.yaml) under data/recordings/ matching a glob relative to data/recordings/."""
    return [Run(p) for p in sorted(glob.glob(os.path.join(LOGS, pattern))) if os.path.exists(os.path.join(p, 'META.yaml'))]
