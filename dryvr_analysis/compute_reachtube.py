#!/usr/bin/env python3
from typing import Any, NamedTuple, Sequence, Tuple

import numpy as np

from hector_uav_msgs.msg import PoseActionGoal
from nav_msgs.msg import Odometry

from rosbag_to_dist_trace import DistTrace, process_bag_file

WAYPOINT_TOPIC = "action/pose/goal"
STATE_TOPIC = "ground_truth/state"
ROSPointT = NamedTuple("PointT", [('x', float), ('y', float), ('z', float)])
ROSStateT = NamedTuple("StateT", [
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
    prev_wp = None
    stamp, state = next(stamped_state_iter)
    for wp_stamp, wp in stamped_waypoint_iter:
        dryvr_mode = prev_wp  # NOTE: Last waypoint will be excluded
        dryvr_trace = []
        while stamp <= wp_stamp:
            dryvr_trace.append(state)
            stamp, state = next(stamped_state_iter)
        dryvr_mode = np.array(dryvr_mode)
        dryvr_trace = np.array(dryvr_trace)
        ret.append((dryvr_mode, dryvr_trace))
        prev_wp = wp
    ret = ret[1:]  # Remove the first trace
    return ret


def main(argv: Any) -> None:
    dist_trace_seq = tuple(process_bag_file(bag_file_name, TOPIC_TO_CB)[1]
                           for bag_file_name in argv.bag_file)

    for dist_trace in dist_trace_seq:
        mode_trace_seq = dist_trace_to_dryvr_traces(dist_trace)
        print([len(trace) for mode, trace in mode_trace_seq])
        # TODO call DryVR to compute reach-tube


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('bag_file', nargs='+', type=str)
    main(parser.parse_args())
