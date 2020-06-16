import abc
from multiprocessing.synchronize import Event
from multiprocessing.connection import Connection
from typing import Any, Dict, List, Optional, Tuple

# TODO avoid importing rospy if not using ROS
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
import rospy
from rospy.timer import sleep

Action = Tuple[str, Dict[str, Any]]


class AutomatonBase(abc.ABC):
    def __init__(self):
        self.__clk = rospy.Time(0, 0)  # Not exposed to child classes

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


def _select_act(aut: AutomatonBase, conn: Connection) -> Optional[Action]:
    # TODO fairness
    # Check for input actions first
    while conn.poll():
        act = conn.recv()  # type: Action
        if aut.is_input(act):
            return act
    # No input action received. Select output or internal actions
    act_list = aut.get_enabled_actions()  # type: List[Action]
    if not bool(act_list):
        return None
    return act_list.pop()


def run_as_process(aut: AutomatonBase, conn: Connection,
                   stop_ev: Event, **kwargs: Any) -> None:
    """ NOTE: Must be executed in the main thread of a process """
    try:
        rospy.init_node(repr(aut), anonymous=True, disable_signals=False)
        # Initialize Publishers and Subscribers

        # FIXME TIOA should not depend on specific implementations
        if hasattr(aut, "motion"):
            from .agent import Agent
            assert isinstance(aut, Agent)
            pose_topic_name = "/vrpn_client_node/drone" + str(aut.uid) + "/pose"

            def update_position(data: PoseStamped):
                aut.motion.position = data.pose.position

            # NOTE This creates a thread in this process
            rospy.Subscriber(pose_topic_name, PoseStamped, update_position, queue_size=10)

            timeout = 10
            try:
                rospy.wait_for_message(pose_topic_name, PoseStamped, timeout=timeout)
            except rospy.ROSException:
                raise RuntimeError("Unable to initialize position after %d sec."
                                   " Shutdown ROS node for motion." % timeout)

        rospy.wait_for_message("/clock", Clock, timeout=5.0)  # Wait for first update of clock
        busy_waiting_start = rospy.Time.now()
        print("Start %s at %.2f" % (aut, busy_waiting_start.to_sec()))
        while not stop_ev.is_set() and not aut.reached_sink_state():
            sleep(0.0)  # Yield to other threads
            # TODO avoid each iteration of while loop running indefinitely long

            # Select an enabled action
            act = _select_act(aut, conn)

            # No enabled action, stay at current discrete state
            # NOTE clock and continuous variables are still updated
            if act is not None:
                busy_waiting_start = rospy.Time.now()
            else:
                timeout = 30
                if busy_waiting_start + rospy.Duration(secs=timeout) < rospy.Time.now():
                    print("Busy waiting for over %d seconds without new actions." % timeout, end=' ')
                    break
                continue

            # Send output action to the environment
            if aut.is_output(act):
                conn.send(act)

            # Run transition of automaton
            aut.transition(act)

    except KeyboardInterrupt:
        print("KeyboardInterrupt.", end=' ')
    # except RuntimeError as e:
    #    print(repr(e), end=' ')
    finally:
        print("Ending %s at %.2f..." % (aut, rospy.Time.now().to_sec()))
        rospy.signal_shutdown("Shutting down ROS node for %s" % aut)
        conn.close()
