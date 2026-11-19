"""Read a run's rosbag into numpy arrays once, cache as .npz (common/cache/), keyed on bag size + mtime.

Every series gets two clocks:  'th' = sender header stamp (s, epoch) where the message has a header,
'tr' = bag receive time (s, epoch). Joint arrays are reordered by NAME into runs.JOINTS order
(/joint_states is not published in URDF order). Teleop topics are normalised: '/ur5e_touch_teleop/x'
and '/ur16e_touch_teleop/x' both become 'teleop/x'.
"""
import hashlib
import os

import numpy as np

from .runs import JOINTS, HERE

CACHE = os.path.join(HERE, 'cache')
DEFAULT = ['/joint_states', '/servo_node/delta_twist_cmds', '/servo_node/status', '/phantom/state',
           '/phantom/button', '/tcp_pose_broadcaster/pose', '/speed_scaling_state_broadcaster/speed_scaling',
           'teleop/target_pose', 'teleop/condition_number', 'teleop/rcm_error', 'teleop/rcm_active',
           'teleop/insertion_depth', 'teleop/rcm_point', 'teleop/motion_scale',
           '/forward_velocity_controller/commands', '/joint_velocity_limiter/commands',
           '/forward_position_controller/commands', '/scaled_joint_trajectory_controller/joint_trajectory']


def norm_topic(t):
    for p in ('/ur5e_touch_teleop/', '/ur16e_touch_teleop/'):
        if t.startswith(p):
            return 'teleop/' + t[len(p):]
    return t


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _bag_id(bagdir):
    st = [(f, os.path.getsize(os.path.join(bagdir, f)), int(os.path.getmtime(os.path.join(bagdir, f))))
          for f in sorted(os.listdir(bagdir))]
    from .runs import LOGS   # keyed on the path inside the recordings folder, so the cache survives a move
    return hashlib.md5(repr((os.path.relpath(os.path.abspath(bagdir), LOGS), st)).encode()).hexdigest()[:16]


def load(run, topics=DEFAULT):
    """{topic: {field: array}} for the requested (normalised) topics present in the bag."""
    bagdir = os.path.join(run.path, 'rosbag')
    if not os.path.isdir(bagdir):
        return {}
    os.makedirs(CACHE, exist_ok=True)
    cpath = os.path.join(CACHE, _bag_id(bagdir) + '.npz')
    if os.path.exists(cpath):
        z = np.load(cpath, allow_pickle=True)
        data = z['data'].item()
        if all(t in data or t in z['absent'].item() for t in topics):
            return {t: data[t] for t in topics if t in data}
    data = _read(bagdir)
    absent = set(DEFAULT) - set(data)
    np.savez_compressed(cpath, data=np.array(data, dtype=object), absent=np.array(absent, dtype=object))
    return {t: data[t] for t in topics if t in data}


def _read(bagdir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bagdir), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    wanted = {t for t in types if norm_topic(t) in DEFAULT}
    r.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    cls = {t: get_message(types[t]) for t in wanted}
    acc = {norm_topic(t): [] for t in wanted}
    while r.has_next():
        topic, raw, tr = r.read_next()
        acc[norm_topic(topic)].append((tr * 1e-9, deserialize_message(raw, cls[topic])))
    out = {}
    for t, items in acc.items():
        if not items:
            continue
        tr = np.array([a for a, _ in items])
        msgs = [m for _, m in items]
        d = {'tr': tr}
        m0 = msgs[0]
        if t == '/joint_states':
            idx = [list(m0.name).index(j) for j in JOINTS]
            ok = [list(m.name) == list(m0.name) for m in msgs]
            msgs = [m for m, k in zip(msgs, ok) if k]
            d['tr'] = tr[np.array(ok)]
            d['th'] = np.array([_stamp(m.header) for m in msgs])
            d['q'] = np.array([[m.position[i] for i in idx] for m in msgs])
            d['qd'] = np.array([[m.velocity[i] for i in idx] for m in msgs])
            d['current'] = np.array([[m.effort[i] for i in idx] for m in msgs])
        elif t == '/servo_node/delta_twist_cmds':
            d['th'] = np.array([_stamp(m.header) for m in msgs])
            d['frame'] = m0.header.frame_id
            d['lin'] = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for m in msgs])
            d['ang'] = np.array([[m.twist.angular.x, m.twist.angular.y, m.twist.angular.z] for m in msgs])
        elif t == '/servo_node/status':
            d['code'] = np.array([m.code for m in msgs])
        elif t == '/phantom/state':
            d['th'] = np.array([_stamp(m.header) for m in msgs])
            d['p'] = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in msgs])
            d['quat'] = np.array([[m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                                   m.pose.orientation.w] for m in msgs])
        elif t == '/phantom/button':
            d['grey'] = np.array([m.grey_button for m in msgs])
            d['white'] = np.array([m.white_button for m in msgs])
        elif t in ('/tcp_pose_broadcaster/pose', 'teleop/target_pose'):
            d['th'] = np.array([_stamp(m.header) for m in msgs])
            d['p'] = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in msgs])
            d['quat'] = np.array([[m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                                   m.pose.orientation.w] for m in msgs])
        elif t == 'teleop/rcm_point':
            d['p'] = np.array([[m.point.x, m.point.y, m.point.z] for m in msgs])
        elif t == '/scaled_joint_trajectory_controller/joint_trajectory':
            idx = [list(m0.joint_names).index(j) for j in JOINTS]
            d['th'] = np.array([_stamp(m.header) for m in msgs])
            d['first_t'] = np.array([m.points[0].time_from_start.sec + m.points[0].time_from_start.nanosec * 1e-9
                                     if m.points else np.nan for m in msgs])
            d['q'] = np.array([[m.points[0].positions[i] for i in idx] if m.points else [np.nan] * 6
                               for m in msgs])
            d['qd'] = np.array([[m.points[0].velocities[i] for i in idx] if m.points and m.points[0].velocities
                                else [np.nan] * 6 for m in msgs])
        elif t.endswith('/commands'):
            d['data'] = np.array([list(m.data) for m in msgs], dtype=float)
        elif hasattr(m0, 'data'):
            d['data'] = np.array([m.data for m in msgs], dtype=float)
        out[t] = d
    return out


def to_run_clock(run, bag):
    """Offset that maps bag receive time (epoch s) onto the CSV 't' clock (s since recording start).

    UR_PATH rows with source == rtde carry the TCP header stamp ('stamp') and host time ('t'); the bag's
    /tcp_pose_broadcaster/pose has the same header stamps and its own receive time. So
    t_csv = tr - (median(tr - th)_bag + median(stamp - t)_csv) ... returned as one offset to subtract.
    """
    ur = run.csv('UR_PATH.csv', ['t', 'stamp'])
    tcp = bag.get('/tcp_pose_broadcaster/pose')
    if ur is None or tcp is None:
        return None
    off_csv = np.nanmedian(ur['stamp'] - ur['t'])          # stamp -> t
    off_bag = np.median(tcp['tr'] - tcp['th'])              # th -> tr
    return off_csv + off_bag                                # t = tr - this
