from collections import defaultdict
from itertools import combinations
from typing import Dict, Hashable, Iterator, List, MutableSet, Tuple

from reachtube.drone3d_types import Contract

from .tioa_base import Action, AutomatonBase


class AirspaceManager(AutomatonBase):
    def __init__(self) -> None:
        super(AirspaceManager, self).__init__()

        self._contr_dict = defaultdict(Contract)  # type: Dict[Hashable, Contract]
        self._reply_set = set()  # type: MutableSet[Hashable]

    def is_internal(self, act: Action) -> bool:
        return False

    def is_output(self, act: Action) -> bool:
        return act[0] == "reply"

    def is_input(self, act: Action) -> bool:
        return act[0] == "request" or act[0] == "release"

    def invariant(self) -> bool:
        pairwise_values = combinations(self._contr_dict.values(), 2)  # type: Iterator[Tuple[Contract, ...]]
        return all(a.isdisjoint(b) for a, b in pairwise_values)

    def transition(self, act: Action) -> None:
        if not isinstance(act, tuple) or not bool(act):
            raise ValueError("Action should be a pair but received %s." % act)

        if act[0] == "request":
            self._eff_request(**act[1])
            if not self.invariant():
                print("Invariant is violated")
        elif act[0] == "reply":
            self._eff_reply(**act[1])
        elif act[0] == "release":
            self._eff_release(**act[1])
        else:
            raise ValueError("Unknown action \"%s\"" % act)

    def _eff_request(self, uid: Hashable, target: Contract) -> None:
        self._reply_set.add(uid)
        if all(target.isdisjoint(v) for k, v in self._contr_dict.items() if k != uid):
            self._contr_dict[uid] |= target

    def _eff_reply(self, uid: Hashable, acquired: Contract) -> None:
        self._reply_set.remove(uid)

    def _eff_release(self, uid: Hashable, releasable: Contract) -> None:
        self._contr_dict[uid] -= releasable

    def reached_sink_state(self) -> bool:
        return False

    def _enabled_actions(self) -> List[Action]:
        ret = []  # type: List[Action]
        if bool(self._reply_set):
            ret.extend(("reply",
                        {"uid": uid,
                         "acquired": self._contr_dict[uid]})
                       for uid in self._reply_set)
        return ret
