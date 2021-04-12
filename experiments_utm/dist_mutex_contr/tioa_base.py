import abc
from collections import Counter
from multiprocessing import Event, Queue
from queue import Empty
from typing import Any, Dict, List, Optional, Tuple

# TODO avoid importing rospy if not using ROS
from geometry_msgs.msg import PoseStamped
from reachtube import Contract
from rosgraph_msgs.msg import Clock
import rospy
from rospy.timer import sleep

Action = Tuple[str, Dict[str, Any]]


class AutomatonBase(abc.ABC):
    def __init__(self):
        self.__clk = rospy.Time(0, 0)  # Not exposed to child classes
        self.queries = Counter()

    def __repr__(self) -> str:
        return self.__class__.__name__

    def in_signature(self, act: Action) -> bool:
        assert sum([self.is_internal(act), self.is_output(act), self.is_input(act)]) <= 1, str(act)
        return self.is_internal(act) or self.is_output(act) or self.is_input(act)

    def _update_continuous_vars(self) -> None:
        self.__clk = rospy.Time.now()

    @property
    def clk(self) -> rospy.Time:
        return self.__clk

    def get_enabled_actions(self) -> List[Action]:
        # Clock value is updated before transition
        # and should remain the same during each transition
        self._update_continuous_vars()
        return self._enabled_actions()

    @abc.abstractmethod
    def is_internal(self, act: Action) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def is_output(self, act: Action) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def is_input(self, act: Action) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def transition(self, act: Action) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def reached_sink_state(self) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def _enabled_actions(self) -> List[Action]:
        raise NotImplementedError

    def _membership_query(self, item, contr: Contract) -> bool:
        self.queries["Qm"] += 1
        self.queries["R(Qm)"] += contr.num_rects()
        return item in contr

    def _subset_query(self, c1: Contract, c2: Contract) -> bool:
        self.queries["Qe"] += 1
        c3 = c1 - c2
        self.queries["R(Qe)"] += c3.num_rects()
        return c3.isempty()

    def _disjoint_query(self, c1: Contract, c2: Contract) -> bool:
        self.queries['Qe'] += 1
        c3 = c1 & c2
        self.queries["R(Qe)"] += c3.num_rects()
        return c3.isempty()


def _select_act(aut: AutomatonBase, i_queue: Queue) -> Optional[Action]:
    # TODO fairness
    # Check for input actions first
    while not i_queue.empty():
        try:
            act = i_queue.get_nowait()  # type: Action
            if aut.is_input(act):
                return act
        except Empty:
            break
    # No input action received. Select output or internal actions
    act_list = aut.get_enabled_actions()  # type: List[Action]
    if not bool(act_list):
        return None
    return act_list.pop()


def run_as_process(aut: AutomatonBase, i_queue: Queue, o_queue: Queue,
                   stop_ev: Event, **kwargs: Any) -> None:
    """ NOTE: Must be executed in the main thread of a process """
    start_time = float("NaN")
    try:
        rospy.init_node(repr(aut), anonymous=True, disable_signals=False)
        # Initialize Publishers and Subscribers

        # FIXME TIOA should not depend on specific implementations
        from .agent import Agent
        if isinstance(aut, Agent):
            pose_topic_name = "/vrpn_client_node/%s/pose" % str(aut.uid)

            def update_pose(data: PoseStamped):
                aut.motion.position = data.pose.position
                aut.motion.orientation = data.pose.orientation

            # NOTE This creates a thread in this process
            rospy.Subscriber(pose_topic_name, PoseStamped, update_pose, queue_size=10)

            rospy.wait_for_message(pose_topic_name, PoseStamped, timeout=10.0)

        rospy.wait_for_message("/clock", Clock, timeout=5.0)  # Wait for first update of clock
        busy_waiting_start = rospy.Time.now()
        start_time = busy_waiting_start.to_sec()
        rospy.logdebug("Start %s at %.2f" % (aut, busy_waiting_start.to_sec()))
        while not stop_ev.is_set() and not aut.reached_sink_state():
            sleep(0.0)  # Yield to other threads
            # TODO avoid each iteration of while loop running indefinitely long

            # Select an enabled action
            act = _select_act(aut, i_queue)

            # No enabled action, stay at current discrete state
            # NOTE clock and continuous variables are still updated
            if act is not None:
                busy_waiting_start = rospy.Time.now()
            else:
                timeout = 300
                if busy_waiting_start + rospy.Duration(secs=timeout) < rospy.Time.now():
                    print("Busy waiting for over %d seconds without new actions." % timeout, end=' ')
                    break
                continue

            # Send output action to the environment
            if aut.is_output(act):
                o_queue.put_nowait(act)

            # Run transition of automaton
            aut.transition(act)

    except KeyboardInterrupt:
        print("KeyboardInterrupt.", end=' ')
    # except RuntimeError as e:
    #    print(repr(e), end=' ')
    finally:
        o_queue.close()
        i_queue.close()
        if isinstance(aut, Agent):
            if not aut.motion.landing():
                print("Landing failed.")
        end_time = rospy.Time.now().to_sec()
        rospy.logdebug("Ending %s at %.2f..." % (aut, end_time))
        print('-', {"name": repr(aut), "t_start": start_time, "t_end": end_time, **aut.queries})
        rospy.signal_shutdown("Shutting down ROS node for %s" % aut)
