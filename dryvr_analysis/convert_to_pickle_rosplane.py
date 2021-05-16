#!/usr/bin/env python3
import pickle
from typing import Any, NamedTuple, Sequence, Tuple

import numpy as np

from rosplane_msgs.msg import State, Current_Path


from rosbag_to_dist_trace import DistTrace, process_bag_file

MODE_TOPIC = "current_path"
STATE_TOPIC = "state"
ROSplaneModeT = NamedTuple("ROSplaneModeT", [
    ('path_type', float),
    ('Va_d', float),
    ('r_x', float), ('r_y', float), ('r_z', float),
    ('q_x', float), ('q_y', float), ('q_z', float),
    ('c_x', float), ('c_y', float), ('c_z', float),
    ('rho', float),
    ('rotation', float)
])

ROSplaneStateT = NamedTuple("ROSplaneStateT", [
    ('x', float), ('y', float), ('z', float),
    ('Va', float),
    ('alpha', float), ('beta', float),
    ('theta', float), ('psi', float), ('chi', float),
    ('p', float), ('q', float), ('r', float),
    ('Vg', float)
])

DryVRModeT = np.ndarray
DryVRTraceT = np.ndarray


def current_path_to_mode(msg: Current_Path) -> ROSplaneModeT:
    return ROSplaneModeT(
        msg.path_type,
        msg.Va_d,
        msg.r[0], msg.r[1], msg.r[2],
        msg.q[0], msg.q[1], msg.q[2],
        msg.c[0], msg.c[1], msg.c[2],
        msg.rho,
        msg.lambda_
    )


def state_to_state(msg: State) -> ROSplaneStateT:
    pos = msg.position
    return ROSplaneStateT(
        pos[0], pos[1], pos[2],
        msg.Va,
        msg.alpha, msg.beta,
        msg.theta, msg.psi, msg.chi,
        msg.p, msg.q, msg.r,
        msg.Vg
    )


TOPIC_TO_CB = {
    MODE_TOPIC: current_path_to_mode,
    STATE_TOPIC: state_to_state
}


def dist_trace_to_dryvr_traces(dist_trace: DistTrace) \
        -> Sequence[Tuple[DryVRModeT, DryVRTraceT]]:
    """ Cut the original trace into segments based on the timestamps of waypoints.
    The segment before the first waypoint is removed because no waypoint was given.
    The segment after the last waypoint is excluded to remove disturbance due to termination.
    """
    stamped_waypoint_iter = iter(dist_trace[MODE_TOPIC])
    stamped_state_iter = iter(dist_trace[STATE_TOPIC])

    trace_segments = []
    prev_wp_stamp, prev_wp = -np.inf, ()
    try:
        stamp, state = next(stamped_state_iter)
        for wp_stamp, wp in stamped_waypoint_iter:
            dryvr_trace_list = []
            while stamp < wp_stamp:
                dryvr_trace_list.append((stamp,) + tuple(state))
                stamp, state = next(stamped_state_iter)
            # NOTE: Last waypoint is automatically excluded due to this line using previous waypoints
            dryvr_mode = (prev_wp_stamp,) + tuple(prev_wp)
            dryvr_trace = dryvr_trace_list
            trace_segments.append((dryvr_mode, dryvr_trace))
            prev_wp_stamp, prev_wp = wp_stamp, wp
    except StopIteration:  # Exhaust all states
        pass

    ret = []
    seg_iter = iter(trace_segments)
    prev_mode, prev_trace = next(seg_iter)
    for mode, trace in seg_iter:
        if mode[1:] == prev_mode[1:]:
            prev_trace.extend(trace)
        else:
            ret.append((np.array(prev_mode), np.array(prev_trace)))
            prev_mode, prev_trace = mode, trace
    ret.append((np.array(prev_mode), np.array(prev_trace)))
    return ret[1:]  # Ignore the first segment


def main(argv: Any) -> None:
    dist_trace_iter = (process_bag_file(bag_file_name, TOPIC_TO_CB)
                       for bag_file_name in argv.bag_file)

    for file_name, dist_trace in dist_trace_iter:
        mode_trace_seq = dist_trace_to_dryvr_traces(dist_trace)
        with open(file_name + ".pickle", 'wb') as f:
            pickle.dump(mode_trace_seq, f)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('bag_file', nargs='+', type=str)
    main(parser.parse_args())
