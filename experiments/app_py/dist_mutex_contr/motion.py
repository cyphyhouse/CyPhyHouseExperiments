from copy import deepcopy
from enum import Enum
from threading import RLock
from typing import Tuple, Union

from actionlib import GoalStatus, SimpleActionClient, SimpleGoalState
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from hector_uav_msgs.msg import LandingAction, LandingGoal, \
    PoseAction, PoseGoal, TakeoffAction, TakeoffGoal
import rospy


class MotionHectorQuad:
    class Status(Enum):
        STAYING = 0
        MOVING = 1

    def __init__(self, topic_prefix: str):
        self._status = self.Status.STAYING

        self._var_lock = RLock()
        self._position = (0.0, 0.0, 0.0)
        self._orientation = (0.0, 0.0, 0.0, 1.0)

        takeoff_topic = rospy.resolve_name(topic_prefix + "/action/takeoff")
        self._takeoff_client = SimpleActionClient(takeoff_topic, TakeoffAction)
        landing_topic = rospy.resolve_name(topic_prefix + "/action/landing")
        self._landing_client = SimpleActionClient(landing_topic, LandingAction)
        pose_topic = rospy.resolve_name(topic_prefix + "/action/pose")
        self._pose_client = SimpleActionClient(pose_topic, PoseAction)

    @property
    def position(self) -> Tuple[float, float, float]:
        with self._var_lock:
            # Return copy to avoid multiple threads accessing the same reference
            return deepcopy(self._position)

    @position.setter
    def position(self, p: Union[Point, Tuple[float, float, float]]) -> None:
        if isinstance(p, Point):
            p = (p.x, p.y, p.z)

        # NOTE the lock may be redundant because assigning references should be atomic
        with self._var_lock:
            self._position = p

    @property
    def orientation(self) -> Tuple[float, float, float, float]:
        with self._var_lock:
            # Return copy to avoid multiple threads accessing the same reference
            return deepcopy(self._orientation)

    @orientation.setter
    def orientation(self, p: Union[Quaternion, Tuple[float, float, float, float]]) -> None:
        if isinstance(p, Quaternion):
            p = (p.x, p.y, p.z, p.w)

        # NOTE the lock may be redundant because assigning references should be atomic
        with self._var_lock:
            self._orientation = p

    def takeoff(self, timeout: rospy.Duration = rospy.Duration()) -> bool:
        return self._send_action_and_wait(self._takeoff_client,
                                          TakeoffGoal(), timeout)

    def landing(self, timeout: rospy.Duration = rospy.Duration()) -> bool:
        return self._send_action_and_wait(self._landing_client,
                                          LandingGoal(), timeout)

    @staticmethod
    def _to_pose_stamped(point: Tuple[float, float, float]) -> PoseStamped:
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position = Point(*point)
        return target_pose

    def send_target(self, point: Tuple[float, float, float]):
        self._pose_client.wait_for_server()
        pose_goal = PoseGoal(target_pose=self._to_pose_stamped(point))
        # NOTE Do not wait for result
        return self._pose_client.send_goal(pose_goal)

    @staticmethod
    def _send_action_and_wait(action_client: SimpleActionClient,
                              goal, timeout: rospy.Duration = rospy.Duration()) -> bool:
        if timeout == rospy.Duration():
            if not action_client.wait_for_server():
                # Action server is not available
                return False
            assert action_client.simple_state == SimpleGoalState.DONE
            status = action_client.send_goal_and_wait(goal=goal)
        else:
            deadline = rospy.Time.now() + timeout
            if not action_client.wait_for_server(timeout=deadline - rospy.Time.now()):
                # Action server is not available
                return False
            assert action_client.simple_state == SimpleGoalState.DONE
            status = action_client.send_goal_and_wait(
                goal=goal, execute_timeout=deadline - rospy.Time.now())

        return status == GoalStatus.SUCCEEDED
