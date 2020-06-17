from enum import Enum
from typing import Any, Callable, Hashable, List, NamedTuple, Optional, Tuple

import rospy  # FIXME avoid rospy

import numpy as np
from reachtube.drone3d_types import Contract
from scipy.spatial import Rectangle

from .motion import MotionHectorQuad
from .tioa_base import Action, AutomatonBase

StampT = float
StampedPoint = NamedTuple('StampedPoint',
                          [('stamp', StampT),
                           ('position', Tuple[float, float, float])])
StampedRect = NamedTuple('StampedRect',
                         [('stamp', StampT),
                          ('rect', Rectangle)])


class Agent(AutomatonBase):
    class Status(Enum):
        IDLE = 0
        PLANNED = 1
        REQUESTED = 2
        MOVING = 4
        FINISHED = 5
        STOPPED = 6

    def __init__(self, uid: Hashable, motion: MotionHectorQuad):
        super(Agent, self).__init__()

        self.__uid = uid  # Set it as private to avoid being modified
        self.__motion = motion

        if self.uid == "drone0":
            self.__way_points = [(0, 0, 1), (2, 0, 1), (4, 0, 1), (6, 0, 1)]
        elif self.uid == "drone1":
            self.__way_points = [(0, 0, 1), (0, 2, 1), (0, 4, 1), (0, 6, 1)]
        elif self.uid == "drone2":
            self.__way_points = [(0, 0, 1), (-2, 0, 1), (-4, 0, 1), (-6, 0, 1)]
        elif self.uid == "drone3":
            self.__way_points = [(0, 0, 1), (0, -2, 1), (0, -4, 1), (0, -6, 1)]
        else:
            self.__way_points = []  # type: List[Tuple[float, float, float]]

        self._status = Agent.Status.IDLE
        self._plan = []  # type: List[StampedRect]
        self._plan_contr = Contract()  # type: Contract
        self._curr_contr = Contract()  # type: Contract
        self._free_contr = Contract()  # type: Contract
        self._retry_time = self.clk

        # Continuously updated variables
        # self.__motion.position may be modified by ROS subscribers concurrently
        # so we need self._position to stay the same during a transition
        self._position = self.__motion.position

    def __repr__(self) -> str:
        return self.__class__.__name__ + "_" + str(self.uid)

    @property
    def uid(self) -> Hashable:
        return self.__uid

    def is_internal(self, act: Action) -> bool:
        return act[0] == "plan" \
            or act[0] == "next_region" or act[0] == "succeed" or act[0] == "fail"

    def is_output(self, act: Action) -> bool:
        return (act[0] == "request" or act[0] == "release") and act[1]["uid"] == self.uid

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
            "fail": build_trans(pre=self._pre_succeed, eff=self._eff_succeed),
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
            rect_list = _bloat_path(self._position,
                                    self.__way_points)
            self._plan = [StampedRect(self.clk.to_sec() + 5*i, rect)
                          for i, rect in enumerate(rect_list)]
            self._plan_contr = Contract.from_stamped_rectangles(
                self._plan)
            self._status = Agent.Status.PLANNED

    def _pre_request(self, uid: Hashable, target: Contract) -> bool:
        return self._status == Agent.Status.PLANNED \
            and target == self._plan_contr

    def _eff_request(self, uid: Hashable, target: Contract) -> None:
        self._status = Agent.Status.REQUESTED

    def _eff_reply(self, uid: Hashable, acquired: Contract) -> None:
        if self._status == Agent.Status.REQUESTED:
            if not (self._curr_contr <= acquired):
                # raise RuntimeError("Current contract is not a subset of acquired contract.\n"
                #                     "Current: %s\nAcquired: %s" % (repr(self._curr_contr), repr(acquired)))
                pass
            if self._plan_contr <= acquired:  # and self._curr_contr <= acquired
                # Acquired enough contract to execute plan
                self._curr_contr = acquired
                tgt = self.__way_points[len(self.__way_points) + 1 - len(self._plan)]
                print("%s going to %s." % (self, str(tgt)))
                self.__motion.send_target(tgt)

                self._status = Agent.Status.MOVING
            else:
                # Not enough contract for the plan. Keep only current contracts
                self._free_contr = acquired - self._curr_contr
                self._status = Agent.Status.FINISHED
        else:
            raise RuntimeWarning("Unexpected \"reply\" action.")

    def _pre_next_region(self) -> bool:
        return self._status == Agent.Status.MOVING and len(self._plan) >= 2 \
            and self.clk.to_sec() >= self._plan[1].stamp

    def _eff_next_region(self) -> None:
        self._plan.pop(0)
        self._plan_contr = Contract.from_stamped_rectangles(self._plan)

        if len(self._plan) >= 2:
            idx = len(self.__way_points) + 1 - len(self._plan)
            assert 0 <= idx < len(self.__way_points)
            tgt = self.__way_points[idx]
            print("%s going to %s." % (self, str(tgt)))
            self.__motion.send_target(tgt)

    def _pre_succeed(self) -> bool:
        return self._status == Agent.Status.MOVING and len(self._plan) == 1 \
            and True  # TODO StampedPoint(self.clk.to_sec(), self._position) in self._plan_contr

    def _eff_succeed(self) -> None:
        self._free_contr = self._curr_contr - self._plan_contr
        self._status = Agent.Status.FINISHED
        print("Agent %s succeeded" % str(self.__uid))

    def _pre_fail(self) -> bool:
        return self._status == Agent.Status.MOVING \
            and False  # TODO StampedPoint(self.clk.to_sec(), self._position) not in self._plan_contr

    def _eff_fail(self) -> None:
        raise RuntimeError("Failed to follow the plan contract.")

    def _pre_release(self, uid: Hashable, releasable: Contract) -> bool:
        return self._status == Agent.Status.FINISHED \
            and uid == self.uid \
            and releasable == self._free_contr

    def _eff_release(self, uid: Hashable, releasable: Contract) -> None:
        self._status = Agent.Status.IDLE
        self._curr_contr -= releasable

        self._retry_time = self.clk + rospy.Duration(secs=2)  # Wait for a while before next plan

    def reached_sink_state(self) -> bool:
        return self._status == Agent.Status.STOPPED

    def _enabled_actions(self) -> List[Action]:
        ret = []  # type: List[Action]
        if self._pre_plan():
            ret.append(("plan", {}))
        if self._status == Agent.Status.PLANNED:
            ret.append(("request", {"uid": self.uid, "target": self._plan_contr}))
        if self._pre_next_region():
            ret.append(("next_region", {}))
        if self._pre_succeed():
            ret.append(("succeed", {}))
        if self._pre_fail():
            ret.append(("fail", {}))
        if self._status == Agent.Status.FINISHED:
            ret.append(("release", {"uid": self.uid, "releasable": self._free_contr}))

        return ret

    def _update_continuous_vars(self) -> None:
        super(Agent, self)._update_continuous_vars()
        self._position = self.__motion.position


BLOAT_WIDTH = 0.2


def _bloat_point(p: Tuple[float, float, float]) -> Rectangle:
    p_arr = np.array(p)
    return Rectangle(mins=p_arr - BLOAT_WIDTH,
                     maxes=p_arr + BLOAT_WIDTH)


def _bloat_path(cur_pos: Tuple[float, float, float],
                way_points: List[Tuple[float, float, float]]) -> List[Rectangle]:
    ret = []  # type: List[Rectangle]
    prev_rect = _bloat_point(cur_pos)
    for p in way_points:
        curr_rect = _bloat_point(p)
        ret.append(_bloat_segment(prev_rect, curr_rect))
        prev_rect = curr_rect
    ret.append(prev_rect)  # Stay in the last rect
    return ret


def _bloat_segment(bloat_a: Rectangle, bloat_b: Rectangle) -> Rectangle:
    new_maxes = np.maximum(bloat_a.maxes, bloat_b.maxes)
    new_mins = np.minimum(bloat_a.mins, bloat_b.mins)
    return Rectangle(maxes=new_maxes, mins=new_mins)
