from enum import Enum
from typing import Any, Callable, Hashable, List, Optional, Set

from .tioa_base import Action, AutomatonBase

Contract = Set[Any]


class Agent(AutomatonBase):
    class Status(Enum):
        IDLE = 0
        PLANNED = 1
        REQUESTED = 2
        ACQUIRED = 3
        MOVING = 4
        FINISHED = 5
        STOPPED = 6

    def __init__(self, uid: Hashable):
        super(Agent, self).__init__()

        self.__uid = uid  # Set it as private to avoid being modified

        self._status = Agent.Status.IDLE
        self._plan_contr = {"contr" + str(uid)}
        self._curr_contr = {"contr"}
        self._free_contr = {"free_contr"}

    def __repr__(self) -> str:
        return self.__class__.__name__ + "_" + str(self.uid)

    @property
    def uid(self) -> Hashable:
        return self.__uid

    def is_internal(self, act: Action) -> bool:
        return act[0] == "plan" \
            or act[0] == "update_plan" or act[0] == "report"  # Hidden actions

    def is_output(self, act: Action) -> bool:
        return (act[0] == "request" or act[0] == "release") and act[1]["uid"] == self.uid

    def is_input(self, act: Action) -> bool:
        return act[0] == "reply" and act[1]["uid"] == self.uid

    def transition(self, act: Action) -> None:
        if not isinstance(act, tuple) or len(act) != 2:
            raise ValueError("Action should be a pair.")
        if not self.in_signature(act):
            raise ValueError("Action \"" + str(act) + "\" is not in signature.")

        def build_trans(eff: Callable[..., None],
                        pre: Optional[Callable[..., bool]] = None) \
                -> Callable[..., None]:
            def trans(**kwargs: Any) -> None:
                if pre is None or pre(**kwargs):
                    eff(**kwargs)
                else:
                    raise RuntimeError("Precondition for \"" + act[0] + "\"is not satisfied.")
            return trans

        eff_dict = {
            "plan": build_trans(pre=self._pre_plan, eff=self._eff_plan),
            "request": build_trans(pre=self._pre_request,
                                   eff=self._eff_request),
            "reply": build_trans(eff=self._eff_reply),
            "update_plan": build_trans(pre=self._pre_update_plan, eff=self._eff_update_plan),
            "report": build_trans(pre=self._pre_report, eff=self._eff_report),
            "release": build_trans(pre=self._pre_release, eff=self._eff_release)
        }

        try:
            eff_dict[act[0]](**act[1])
        except KeyError:
            raise KeyError("Unknown action \"" + str(act) + '"')

    def _pre_plan(self) -> bool:
        return self._status == Agent.Status.IDLE

    def _eff_plan(self) -> None:
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
                self._status = Agent.Status.ACQUIRED
            else:
                # Not enough contract for the plan. Keep only current contracts
                self._free_contr = acquired - self._curr_contr
                self._status = Agent.Status.FINISHED
        else:
            raise RuntimeWarning("Unexpected \"reply\" action.")

    def _pre_update_plan(self, path: Contract) -> bool:
        return self._status == Agent.Status.ACQUIRED

    def _eff_update_plan(self, path: Contract) -> None:
        self._status = Agent.Status.MOVING

    def _pre_report(self, success: bool) -> bool:
        return self._status == Agent.Status.MOVING

    def _eff_report(self, success: bool) -> None:
        if success:
            if self._status == Agent.Status.MOVING:
                self._free_contr = self._curr_contr - self._plan_contr
                self._status = Agent.Status.FINISHED
            else:
                raise RuntimeWarning("Unexpected \"report\" action")
        else:
            raise RuntimeError("Failure when executing plan")

    def _pre_release(self, uid: Hashable, releasable: Contract) -> bool:
        return self._status == Agent.Status.FINISHED \
            and uid == self.uid \
            and releasable == self._free_contr

    def _eff_release(self, uid: Hashable, releasable: Contract) -> None:
        self._status = Agent.Status.IDLE
        self._curr_contr -= releasable

    def reached_sink_state(self) -> bool:
        return self._status == Agent.Status.STOPPED

    def _enabled_actions(self) -> List[Action]:
        ret = []  # type: List[Action]
        if self._status == Agent.Status.IDLE:
            ret.append(("plan", {}))
        if self._status == Agent.Status.PLANNED:
            ret.append(("request", {"uid": self.uid, "target": self._plan_contr}))
        if self._status == Agent.Status.ACQUIRED:
            ret.append(("update_plan", {"path": []}))
        if self._status == Agent.Status.MOVING:
            ret.append(("report", {"success": True}))
        if self._status == Agent.Status.FINISHED:
            ret.append(("release", {"uid": self.uid, "releasable": self._free_contr}))

        return ret
