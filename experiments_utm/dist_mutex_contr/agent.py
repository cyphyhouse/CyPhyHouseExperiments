from copy import deepcopy
from enum import Enum
from typing import Any, Callable, Hashable, List, NamedTuple, Optional, Tuple

import rospy  # FIXME avoid rospy

import numpy as np
from reachtube.drone3d_types import Contract
from scipy.spatial import Rectangle
from scipy.spatial.distance import euclidean
from scipy.spatial.transform import Rotation as Rot

from .motion import MotionHectorQuad
from .tioa_base import Action, AutomatonBase

#############
import random
#############



StampT = float
StampedPoint = NamedTuple('StampedPoint',
                          [('stamp', StampT),
                           ('position', Tuple[float, float, float])])
StampedRect = NamedTuple('StampedRect',
                         [('stamp', StampT),
                          ('rect', Rectangle),
                          ('reaching_wp', bool)])


class Agent(AutomatonBase):
    class Status(Enum):
        IDLE = 0
        REQUESTING = 1
        WAITING = 2
        MOVING = 4
        RELEASING = 5
        STOPPING = 6
        ############
        # FAILING = 7
        ############

    def __init__(self, uid: Hashable, motion: MotionHectorQuad, waypoints: List):
        super(Agent, self).__init__()

        self.__uid = uid  # Set it as private to avoid being modified
        self.__motion = motion
        self.__way_points = deepcopy(waypoints)

        self._status = Agent.Status.IDLE
        self._plan = []  # type: List[StampedRect]
        self._plan_contr = Contract()  # type: Contract
        self._curr_contr = Contract()  # type: Contract
        self._free_contr = Contract()  # type: Contract
        self._retry_time = self.clk

        self._failure_reported = False

        # Continuously updated variables
        # self.__motion.position may be modified by ROS subscribers concurrently
        # so we need self._position to stay the same during a transition
        self._position = self.__motion.position
        ###############################
        self._orientation = Rot.from_quat(self.__motion._orientation).as_euler('zyx')
        self._true_wp = None
        ###############################

    def __repr__(self) -> str:
        return self.__class__.__name__ + "_" + str(self.uid)

    @property
    def _target(self) -> Tuple[float, float, float]:
        raise RuntimeWarning("Reading actuator value is not allowed")

    @_target.setter
    def _target(self, p: Tuple[float, float, float]) -> None:
        self.__motion.send_target(p)

    @property
    def uid(self) -> Hashable:
        return self.__uid

    @property
    def motion(self) -> MotionHectorQuad:
        return self.__motion

    def is_internal(self, act: Action) -> bool:
        ####################################
        return act[0] == "plan" \
            or act[0] == "next_region" or act[0] == "succeed" # or act[0] == "fail"

        #    or act[0] == "next_region" or act[0] == "succeed"
        ###################################

    def is_output(self, act: Action) -> bool:
        ###################################
        # return (act[0] == "request" or act[0] == "release") and act[1]["uid"] == self.uid
        return (act[0] == "request" or act[0] == "release" or act[0] == "fail") and act[1]["uid"] == self.uid
        ###################################

    def is_input(self, act: Action) -> bool:
        return act[0] == "reply" and act[1]["uid"] == self.uid

    def transition(self, act: Action) -> None:
        if not isinstance(act, tuple) or len(act) != 2:
            raise ValueError("Action should be a pair.")
        if not self.in_signature(act):
            raise ValueError("Action \"%s\" is not in signature." % str(act))

        def build_trans(eff: Callable[..., None],
                        pre: Optional[Callable[..., bool]] = None) \
                -> Callable[..., None]:
            def trans(**kwargs: Any) -> None:
                if pre is None or pre(**kwargs):
                    eff(**kwargs)
                else:
                    raise RuntimeError("Precondition for \"%s\" is not satisfied." % str(act[0]))
            return trans

        eff_dict = {
            "plan": build_trans(pre=self._pre_plan, eff=self._eff_plan),
            "request": build_trans(pre=self._pre_request,
                                   eff=self._eff_request),
            "reply": build_trans(eff=self._eff_reply),
            "next_region": build_trans(pre=self._pre_next_region, eff=self._eff_next_region),
            "succeed": build_trans(pre=self._pre_succeed, eff=self._eff_succeed),
            "fail": build_trans(pre=self._pre_fail, eff=self._eff_fail),
            "release": build_trans(pre=self._pre_release, eff=self._eff_release)
        }

        try:
            eff_dict[act[0]](**act[1])
        except KeyError:
            raise KeyError("Unknown action \"%s\"" % str(act))

    def _pre_plan(self) -> bool:
        return self._status == Agent.Status.IDLE and self._retry_time <= self.clk

    def _eff_plan(self) -> None:
        if self.__way_points:
            self._plan = waypoints_to_plan(self.clk.to_sec(), self._position, self.__way_points)
            self._plan_contr = self.__plan_to_contr(self._plan)
            self._status = Agent.Status.REQUESTING
        else:
            self.__motion.landing()
            self._status = Agent.Status.STOPPING

    def _pre_request(self, uid: Hashable, target: Contract) -> bool:
        return self._status == Agent.Status.REQUESTING \
            and target == self._plan_contr

    def _eff_request(self, uid: Hashable, target: Contract) -> None:
        self.queries["Req"] += 1
        self.queries["R(Req)"] += target.num_rects()
        self._status = Agent.Status.WAITING

    def _eff_reply(self, uid: Hashable, acquired: Contract) -> None:
        if self._status == Agent.Status.WAITING:
            if not self._subset_query(self._curr_contr, acquired):
                raise RuntimeError("Current contract is not a subset of acquired contract.\n"
                                   "Current: %s\nAcquired: %s" % (repr(self._curr_contr), repr(acquired)))
            if self._subset_query(self._plan_contr, acquired):  # and self._curr_contr <= acquired
                # Acquired enough contract to execute plan
                self._curr_contr = acquired
                self._status = Agent.Status.MOVING

                tgt = self.__way_points.pop(0)
                rospy.logdebug("%s going to %s." % (self, str(tgt)))
                self._target = tgt
                #################
                self._true_wp = deepcopy(tgt)
                ################
            else:
                # Not enough contract for the plan. Keep only current contracts
                self._free_contr = acquired - self._curr_contr
                self._status = Agent.Status.RELEASING
        else:
            raise RuntimeWarning("Unexpected \"reply\" action.")

    def _pre_next_region(self) -> bool:
        return self._status == Agent.Status.MOVING and len(self._plan) >= 2 \
            and self.clk.to_sec() >= self._plan[1].stamp

    def _eff_next_region(self) -> None:
        self._failure_reported = False
        prev = self._plan.pop(0)
        self._plan_contr = self.__plan_to_contr(self._plan)

        if prev.reaching_wp and self.__way_points:
            tgt = self.__way_points.pop(0)
            self._true_wp = deepcopy(tgt)
            rospy.logdebug("%s going to %s." % (self, str(tgt)))
            ############################
            rsample = random.random()
            if abs(rsample) <= 0.9:
                rdisturbance_x = random.gauss(0,5)
                rdisturbance_y = random.gauss(0,5)
                rdisturbance_z = random.gauss(0,5)
                tgt = (tgt[0] + rdisturbance_x, tgt[1] + rdisturbance_y, tgt[2] + rdisturbance_z)
                rospy.logdebug("%s is actuall going to %s after disturbance." % (self, str(tgt)))
                self._target = tgt
            else:
                self._target = tgt
            ############################

    def _pre_succeed(self) -> bool:
        return self._status == Agent.Status.MOVING and len(self._plan) == 1 \
            and self._membership_query(StampedPoint(self.clk.to_sec(), self._position),
                                       self._plan_contr)

    def _eff_succeed(self) -> None:
        self._free_contr = self._curr_contr - self._plan_contr
        self._status = Agent.Status.RELEASING
        rospy.logdebug("Agent %s succeeded" % str(self.__uid))

    def _pre_fail(self, uid: Hashable, target: Contract) -> bool: #
        return self._status == Agent.Status.MOVING \
            and uid == self.uid \
            and not self._failure_reported \
            and not self._membership_query(StampedPoint(self.clk.to_sec(), self._position),
                                           self._plan_contr)

    def _eff_fail(self, uid: Hashable, target: Contract) -> None: #
        rospy.logdebug("Failed to follow the plan contract. (%.2f, %s) not in %s."
                       " Real position: %s" %
                     (self.clk.to_sec(), str(self._position), str(self._plan_contr), str(self.__motion.position)))
        self.queries["fail"] += 1
        self._failure_reported = True
        ###################
        #if self.__way_points:
        self._target = deepcopy(self._true_wp)
        self.__way_points.insert(0, deepcopy(self._true_wp))
        self._plan = waypoints_to_plan(self.clk.to_sec(), self._position, self.__way_points)
        self._plan_contr = self.__plan_to_contr(self._plan)
        self._curr_contr = self._plan_contr
        self.__way_points.pop(0)
        # else:
        #     self.__motion.landing()
        #     self._status = Agent.Status.STOPPING
        ###################

    def _pre_release(self, uid: Hashable, releasable: Contract) -> bool:
        return self._status == Agent.Status.RELEASING \
            and uid == self.uid \
            and releasable == self._free_contr

    def _eff_release(self, uid: Hashable, releasable: Contract) -> None:
        self._status = Agent.Status.IDLE
        self._curr_contr -= releasable

        self._retry_time = self.clk + rospy.Duration.from_sec(1.0)  # Wait for a while before next plan

    def reached_sink_state(self) -> bool:
        return self._status == Agent.Status.STOPPING

    def _enabled_actions(self) -> List[Action]:
        ret = []  # type: List[Action]
        if self._pre_plan():
            ret.append(("plan", {}))
        if self._status == Agent.Status.REQUESTING:
            ret.append(("request", {"uid": self.uid, "target": self._plan_contr}))
        if self._pre_next_region():
            ret.append(("next_region", {}))
        if self._pre_succeed():
            ret.append(("succeed", {}))
        if self._pre_fail(self.uid, self._curr_contr): #
            ###############################
            # ret.append(("fail", {"uid": self.uid}))
            ret.append(("fail", {"uid": self.uid, "target": self._curr_contr}))
            ###############################
        if self._status == Agent.Status.RELEASING:
            ret.append(("release", {"uid": self.uid, "releasable": self._free_contr}))

        return ret

    def _update_continuous_vars(self) -> None:
        super(Agent, self)._update_continuous_vars()
        self._position = self.__motion.position

    @staticmethod
    def __plan_to_contr(plan: List[StampedRect]) -> Contract:
        return Contract.from_stamped_rectangles(
                tuple((t, rect) for t, rect, _ in plan))


BLOAT_WIDTH = 0.5


def waypoints_to_plan(clk: float, pos, way_points, default=True) -> List[StampedRect]:
    if default:
        rect_list = _bloat_path(pos, way_points)
        deadline = clk
        ret = []
        for rect in rect_list:
            ret.append(StampedRect(deadline, rect, True))
            deadline = deadline + 0.5*float(euclidean(rect.maxes, rect.mins))
        return ret
    # else:
    flagged_waypoints = _fixed_resolution(pos, way_points, resolution=2.5)

    deadline_list = [clk]
    prev_p, prev_reach = pos, True
    for p, reach in flagged_waypoints:
        d = float(euclidean(prev_p, p))
        # if reach, the drone is slowing down. if prev_reach, the drone should have slowed down
        # Therefore, the deadline is more relaxed.
        deadline = deadline_list[-1] + d*(0.7 if prev_reach else 0.3 if reach else 0.2)
        deadline_list.append(deadline)
        prev_p, prev_reach = p, reach

    flagged_rect_list = _bloat_flagged_path(pos, flagged_waypoints)
    assert len(flagged_rect_list) == len(deadline_list)
    ret = [StampedRect(deadline, rect, reached)
           for deadline, (rect, reached) in zip(deadline_list, flagged_rect_list)]
    return ret


def _fixed_resolution(current_position, waypoints, resolution=1.0):
    intermediate_pt_list = []  # type: List[Tuple[float, ...]]
    move_to_next_waypt = []  # type: List[bool]
    prev_waypoint = current_position
    for waypoint in waypoints:
        dist = euclidean(prev_waypoint, waypoint)
        num_intermediate_pts = int(np.ceil(dist / resolution))
        lin_list = np.linspace(prev_waypoint, waypoint, num_intermediate_pts + 1)
        assert len(lin_list) >= 2
        tail = [tuple(float(x) for x in pt) for pt in lin_list[1:]]
        intermediate_pt_list.extend(tail)
        move_to_next_waypt.extend([False] * (len(tail) - 1) + [True])
        prev_waypoint = waypoint
    assert len(intermediate_pt_list) == len(move_to_next_waypt)
    assert move_to_next_waypt[-1]
    return list(zip(intermediate_pt_list, move_to_next_waypt))


def _bloat_point(p: Tuple[float, ...]) -> Rectangle:
    p_arr = np.array(p)
    return Rectangle(mins=p_arr - BLOAT_WIDTH,
                     maxes=p_arr + BLOAT_WIDTH)


def _bloat_path(cur_pos: Tuple[float, ...],
                way_points: List[Tuple[float, ...]]) -> List[Rectangle]:
    ret = []  # type: List[Rectangle]
    prev_rect = _bloat_point(cur_pos)
    for p in way_points:
        curr_rect = _bloat_point(p)
        ret.append(_bloat_segment(prev_rect, curr_rect))
        prev_rect = curr_rect
    ret.append(prev_rect)  # Stay in the last rect
    return ret


def _bloat_flagged_path(cur_pos: Tuple[float, ...],
                        flagged_waypoints: List[Tuple[Tuple[float, ...], bool]]) \
        -> List[Tuple[Rectangle, bool]]:
    assert flagged_waypoints[-1][1]
    ret = []  # type: List[Tuple[Rectangle, bool]]

    curr_rect = _bloat_point(cur_pos)
    prev_rect_list = [curr_rect]
    for p, flag in flagged_waypoints:
        curr_rect = _bloat_point(p)
        if not flag:
            prev_rect_list.append(curr_rect)
        else:  # At a flagged waypoint
            rect_iter = (_bloat_segment(prev_rect, curr_rect) for prev_rect in prev_rect_list)
            flag_list = [False]*(len(prev_rect_list) - 1) + [True]
            ret.extend(zip(rect_iter, flag_list))
            prev_rect_list = [curr_rect]
    ret.append((curr_rect, True))  # Stay at the last waypoint
    assert len(ret) == len(flagged_waypoints) + 1
    return ret


def _bloat_segment(bloat_a: Rectangle, bloat_b: Rectangle) -> Rectangle:
    new_maxes = np.maximum(bloat_a.maxes, bloat_b.maxes)
    new_mins = np.minimum(bloat_a.mins, bloat_b.mins)
    return Rectangle(maxes=new_maxes, mins=new_mins)
