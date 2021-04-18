#!/usr/bin/env python3
import itertools
import os.path
import pickle
from typing import Any,  Generator, Hashable, Iterable, NamedTuple, Sequence, Tuple

import numpy as np

from rosplane_msgs.msg import State, Current_Path

from rosbag_to_traces import process_bag_file, dist_trace_to_mode_seg_tuples, aggregate_by_mode

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


def current_path_to_mode(msg: Current_Path) -> ROSplaneModeT:
    # Set unused fields to None so that the same modes will not be consider different modes.
    # This is because the unused fields may be filled with values of previous messages.
    if msg.path_type == 0:  # Orbit Path
        c = [None if np.isnan(v) else v for v in msg.c]  # type: Sequence[float]
        return ROSplaneModeT(
            msg.path_type,
            msg.Va_d,
            None, None, None,
            None, None, None,
            c[0], c[1], c[2],
            msg.rho,
            msg.lambda_
        )
    else:
        assert msg.path_type == 1  # Straight Line path
        return ROSplaneModeT(
            msg.path_type,
            msg.Va_d,
            msg.r[0], msg.r[1], msg.r[2],
            msg.q[0], msg.q[1], msg.q[2],
            None, None, None,
            None,
            None
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


def concatenate_primitive_traces(mode_seg_iseq: Iterable[Tuple[Hashable, Sequence[Tuple[float, ...]]]]) \
        -> Generator[Tuple[Hashable, Sequence[Tuple[float, ...]]], None, None]:
    # FIXME this is a hacky way to merge the primitive traces into a composite trace
    mode_seg_iter = iter(mode_seg_iseq)
    new_trace = []
    stamped_mode, trace = next(mode_seg_iter)
    new_trace.extend(trace)
    for i, (stamped_mode, trace) in enumerate(mode_seg_iter):
        if i % 50 not in [1, 9, 37, 41, 49]:
            pass  # This mode is skipped
        else:
            yield stamped_mode, new_trace
            new_trace = []
        new_trace.extend(trace)


def main(argv: Any) -> None:
    common_prefix = os.path.commonprefix([f.name for f in argv.bag_file])
    out_file_name = os.path.basename(common_prefix)

    dist_trace_iter = (process_bag_file(bag_file_name, TOPIC_TO_CB)
                       for bag_file_name in argv.bag_file)

    # Chain the mode segments from all files
    mode_seg_tuple_iter = itertools.chain(*(dist_trace_to_mode_seg_tuples(
        dist_trace=dist_trace,
        mode_topic=MODE_TOPIC,
        state_topic=STATE_TOPIC)
        for dist_trace in dist_trace_iter))

    mode_seg_tuple_iter = concatenate_primitive_traces(mode_seg_tuple_iter)
    mode_seglist_map = aggregate_by_mode(mode_seg_tuple_iter)
    print(len(mode_seglist_map))
    print(*(len(v) for v in mode_seglist_map.values()))
    print(*(min(len(tr) for tr in v) for v in mode_seglist_map.values()))
    print(*(np.mean(a=[len(tr) for tr in v]) for v in mode_seglist_map.values()))
    print(*(max(len(tr) for tr in v) for v in mode_seglist_map.values()))

    dryvr_input_obj = {}
    for i, seg_list in enumerate(mode_seglist_map.values()):
        if len(seg_list) < 2:
            continue  # Skip this mode because too few traces for reachability analysis
        seg_len = min(*(len(seg) for seg in seg_list))
        time_step = 0.01  # Arbitrarily chosen time step

        seg_arr_list = []
        for seg in seg_list:
            seg_arr = np.asarray(seg[-seg_len:])  # Trim the segments to same length by choosing the suffix
            seg_arr[:, 0] = time_step * np.arange(0, len(seg_arr))  # Reassign timestamp assuming perfect periodic
            seg_arr_list.append(seg_arr)

        dryvr_mode = ["takeoff", "interchange", "loiter", "descend", "rtb"][i]
        if dryvr_mode == "takeoff" or dryvr_mode == "interchange":
            seg_arr_list = seg_arr_list[1:]  # Ignore first trace because of the disturbance when starting Gazebo
        dryvr_traces = np.stack(seg_arr_list)
        dryvr_input_obj[dryvr_mode] = dryvr_traces

    with open(out_file_name + ".pickle", 'wb') as f:
        pickle.dump(dryvr_input_obj, f)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('bag_file', nargs='+', type=argparse.FileType('rb'))
    main(parser.parse_args())
