#!/usr/bin/env python3.7
import itertools
import os.path
import pickle
from typing import Any, NamedTuple

import numpy as np

from hector_uav_msgs.msg import PoseActionGoal
from nav_msgs.msg import Odometry

from rosbag_to_traces import process_bag_file, dist_trace_to_mode_seg_tuples, aggregate_by_mode

WAYPOINT_TOPIC = "action/pose/goal"
STATE_TOPIC = "ground_truth/state"
ROSPointT = NamedTuple("ROSPointT", [('x', float), ('y', float), ('z', float)])
ROSStateT = NamedTuple("ROSStateT", [
    ('pos_x', float), ('pos_y', float), ('pos_z', float),
    ('ori_x', float), ('ori_y', float), ('ori_z', float), ('ori_w', float),
    ('lin_vel_x', float), ('lin_vel_y', float), ('lin_vel_z', float),
    ('ang_vel_x', float), ('ang_vel_y', float), ('ang_vel_z', float),
])


def pose_goal_to_waypoint(msg: PoseActionGoal) -> ROSPointT:
    pos = msg.goal.target_pose.pose.position
    return ROSPointT(pos.x, pos.y, pos.z)


def odom_to_state(msg: Odometry) -> ROSStateT:
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    lin_vel = msg.twist.twist.linear
    ang_vel = msg.twist.twist.angular
    return ROSStateT(
        pos.x, pos.y, pos.z,
        ori.x, ori.y, ori.z, ori.w,
        lin_vel.x, lin_vel.y, lin_vel.z,
        ang_vel.x, ang_vel.y, ang_vel.z
    )


TOPIC_TO_CB = {
    WAYPOINT_TOPIC: pose_goal_to_waypoint,
    STATE_TOPIC: odom_to_state
}


def main(argv: Any) -> None:
    common_prefix = os.path.commonprefix([f.name for f in argv.bag_file])
    out_file_name = os.path.basename(common_prefix)

    dist_trace_iter = (process_bag_file(bag_file, TOPIC_TO_CB)
                       for bag_file in argv.bag_file)
    # Chain the mode segments from all files
    mode_seg_tuple_iter = itertools.chain(*(dist_trace_to_mode_seg_tuples(
            dist_trace=dist_trace,
            mode_topic=WAYPOINT_TOPIC,
            state_topic=STATE_TOPIC)
        for dist_trace in dist_trace_iter))

    mode_seglist_map = aggregate_by_mode(mode_seg_tuple_iter)
    dryvr_input_obj = {}
    for mode, seg_list in mode_seglist_map.items():
        if len(seg_list) < 2:
            continue  # Skip because too few traces for reachability analysis
        seg_len = min(*(len(seg) for seg in seg_list))
        time_step = 0.01  # Arbitrarily chosen time step

        seg_arr_list = []
        for seg in seg_list:
            seg_arr = np.asarray(seg[-seg_len:])  # Trim the segments to same length by choosing the suffix
            seg_arr[:, 0] = time_step * np.arange(0, len(seg_arr))  # Reassign timestamp assuming perfect periodic
            seg_arr_list.append(seg_arr)

        dryvr_mode = mode
        dryvr_traces = np.stack(seg_arr_list)
        dryvr_input_obj[dryvr_mode] = dryvr_traces

    with open(out_file_name + ".pickle", 'wb') as f:
        pickle.dump(dryvr_input_obj, f)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('bag_file', nargs='+', type=argparse.FileType('rb'))
    main(parser.parse_args())
