from enum import Enum
from typing import Any, Callable, Hashable, List, Optional, Set

import rospy  # FIXME avoid rospy

from .motion import MotionHectorQuad
from .tioa_base import Action, AutomatonBase

Contract = Set[Any]


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

        self._status = Agent.Status.IDLE
        self._plan_contr = set()  # type: Contract
        self._curr_contr = set()  # type: Contract
        self._free_contr = set()  # type: Contract
        self._deadline = None
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
                    raise RuntimeError("Precondition for \"%s\"is not satisfied." % str(act[0]))
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
        # TODO compute plan
        if True:
            if self.uid == 0:
                self._plan_contr = {(1, 1, 3), (2, 2, 3), (3, 3, 3)}
            elif self.uid == 1:
                self._plan_contr = {(1, 5, 3), (2, 4, 3), (3, 3, 3)}
            elif self.uid == 2:
                self._plan_contr = {(5, 5, 3), (4, 4, 3), (3, 3, 3)}

            self._status = Agent.Status.PLANNED

    def _pre_request(self, uid: Hashable, target: Contract) -> bool:
        return self._status == Agent.Status.PLANNED \
            and target == self._plan_contr

    def _eff_request(self, uid: Hashable, target: Contract) -> None:
        self._status = Agent.Status.REQUESTED

    def _eff_reply(self, uid: Hashable, acquired: Contract) -> None:
        if self._status == Agent.Status.REQUESTED:
            if self._curr_contr > acquired:
                raise RuntimeError("Acquired contract is smaller than current contract.")
            elif self._plan_contr <= acquired:  # and self._curr_contr <= acquired
                # Acquired enough contract to execute plan
                self._curr_contr = acquired
                tgt = next(iter(self._plan_contr))
                print("%s going to %s." % (self, str(tgt)))
                self.__motion.send_target(tgt)

                self._deadline = rospy.Time.now() + rospy.Duration(5)
                print("Current time: %s, Deadline: %s" % (self.clk.to_sec(), self._deadline.to_sec()))
                self._status = Agent.Status.MOVING
            else:
                # Not enough contract for the plan. Keep only current contracts
                self._free_contr = acquired - self._curr_contr
                self._status = Agent.Status.FINISHED
        else:
            raise RuntimeWarning("Unexpected \"reply\" action.")

    def _pre_next_region(self) -> bool:
        return self._status == Agent.Status.MOVING and len(self._plan_contr) >= 2 \
            and self.clk >= self._deadline  # TODO self._plan_contr[1].stamp

    def _eff_next_region(self) -> None:
        self._plan_contr.pop()
        tgt = next(iter(self._plan_contr))
        print("%s going to %s." % (self, str(tgt)))
        self.__motion.send_target(tgt)

        self._deadline = rospy.Time.now() + rospy.Duration(secs=5)
        print("Current time: %s, Deadline: %s" % (self.clk.to_sec(), self._deadline.to_sec()))

    def _pre_succeed(self) -> bool:
        return self._status == Agent.Status.MOVING and len(self._plan_contr) == 1 \
            and (self.__motion.position, self.clk)  # TODO in self._plan_contr[0]

    def _eff_succeed(self) -> None:
        self._free_contr = self._curr_contr - self._plan_contr
        self._status = Agent.Status.FINISHED
        print("Agent %d succeeded" % self.__uid)

    def _pre_fail(self) -> bool:
        return self._status == Agent.Status.MOVING and len(self._plan_contr) >= 1 \
            and not (self.__motion.position, self.clk)  # TODO in self._plan_contr[0]

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
