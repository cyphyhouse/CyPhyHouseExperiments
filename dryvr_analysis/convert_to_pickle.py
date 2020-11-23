#!/usr/bin/env python3
import pickle
from typing import Any, NamedTuple, Sequence, Tuple

import numpy as np

from hector_uav_msgs.msg import PoseActionGoal
from nav_msgs.msg import Odometry

from rosbag_to_dist_trace import DistTrace, process_bag_file

WAYPOINT_TOPIC = "action/pose/goal"
STATE_TOPIC = "ground_truth/state"
ROSPointT = NamedTuple("ROSPointT", [('x', float), ('y', float), ('z', float)])
ROSStateT = NamedTuple("ROSStateT", [
    ('pos_x', float), ('pos_y', float), ('pos_z', float),
    ('ori_x', float), ('ori_y', float), ('ori_z', float), ('ori_w', float),
    ('lin_vel_x', float), ('lin_vel_y', float), ('lin_vel_z', float),
    ('ang_vel_x', float), ('ang_vel_y', float), ('ang_vel_z', float),
])

DryVRModeT = np.ndarray
DryVRTraceT = np.ndarray


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


def dist_trace_to_dryvr_traces(dist_trace: DistTrace) \
        -> Sequence[Tuple[DryVRModeT, DryVRTraceT]]:
    """ Cut the original trace into segments based on the timestamps of waypoints.
    The segment before the first waypoint is removed because no waypoint was given.
    The segment after the last waypoint is excluded to remove disturbance due to termination.
    """
    stamped_waypoint_iter = iter(dist_trace[WAYPOINT_TOPIC])
    stamped_state_iter = iter(dist_trace[STATE_TOPIC])

    ret = []
    prev_wp_stamp, prev_wp = -np.inf, ()
    stamp, state = next(stamped_state_iter)
    for wp_stamp, wp in stamped_waypoint_iter:
        dryvr_trace_list = []
        while stamp < wp_stamp:
            dryvr_trace_list.append((stamp,) + tuple(state))
            stamp, state = next(stamped_state_iter)
        # NOTE: Last waypoint is automatically excluded due to this line using previous waypoints
        dryvr_mode = np.array((prev_wp_stamp,) + tuple(prev_wp))
        dryvr_trace = np.array(dryvr_trace_list)
        ret.append((dryvr_mode, dryvr_trace))
        prev_wp_stamp, prev_wp = wp_stamp, wp
    ret = ret[1:]  # Remove the first trace
    return ret


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
