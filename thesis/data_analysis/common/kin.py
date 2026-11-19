"""Forward kinematics, geometric Jacobian and condition number of the UR16e, from a kinematics yaml.

FK follows thesis/run_tools/kinematics_check.py (validated there against the controller's TCP to 0.000 mm with the
calibrated file). The Jacobian is the geometric one of the teleop node / MoveIt Servo (linear velocity of the
tool0 origin and angular velocity, both in base_link), and c = s_max / s_min (SingularityGuard).
Frames: kinematics_check returns tool0 in the controller's 'base' frame; base_link = Rz(pi) * base.
"""
import os
import sys

import numpy as np
import yaml

from .runs import PKG

sys.path.insert(0, os.path.join(PKG, 'thesis', 'run_tools'))
import kinematics_check as kc  # noqa: E402

NOMINAL = '/opt/ros/jazzy/share/ur_description/config/ur16e/default_kinematics.yaml'
CALIBRATED = os.path.join(PKG, 'config', 'calibrations', 'ur16e_kinematics_2026-11-03.yaml')
BASE_LINK_FROM_BASE = kc.BASE_LINK_TO_BASE


def load(model_or_file):
    f = {'nominal': NOMINAL, 'calibrated': CALIBRATED}.get(model_or_file, model_or_file)
    return yaml.safe_load(open(f))['kinematics']


def tool0(kin, q):
    """4x4 tool0 in base_link for one joint vector (runs.JOINTS order)."""
    return BASE_LINK_FROM_BASE @ kc.tool0_in_base(kin, q)


def tool0_many(kin, Q):
    return np.array([tool0(kin, q) for q in Q])


def jacobian(kin, q, eps=1e-6):
    """6x6 geometric Jacobian of tool0 in base_link (numerical, central differences on FK)."""
    J = np.zeros((6, 6))
    T0 = tool0(kin, q)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        Tp, Tm = tool0(kin, q + dq), tool0(kin, q - dq)
        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * eps)
        dR = (Tp[:3, :3] - Tm[:3, :3]) / (2 * eps) @ T0[:3, :3].T
        J[3:, i] = [dR[2, 1], dR[0, 2], dR[1, 0]]
    return J


def condition(kin, q):
    s = np.linalg.svd(jacobian(kin, q), compute_uv=False)
    return s[0] / s[-1] if s[-1] > 0 else np.inf
