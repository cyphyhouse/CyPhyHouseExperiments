import abc
from copy import deepcopy
from enum import Enum
try:
    import importlib.resources as importlib_resources
except ImportError:
    import importlib_resources as importlib_resources
import pickle
from threading import RLock
from typing import Mapping, NamedTuple, Tuple, Type, Union, List

import numpy
from actionlib import GoalStatus, SimpleActionClient, SimpleGoalState
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from hector_uav_msgs.msg import LandingAction, LandingGoal, \
    PoseAction, PoseGoal, TakeoffAction, TakeoffGoal
from rosplane_msgs.msg import Waypoint
import rospy
import numpy as np
from scipy.spatial import Rectangle
from scipy.spatial.distance import euclidean

from . import primitive_contracts

MotionInitInfo = NamedTuple(
    "MotionInitInfo",
    [("bot_name", str), ("bot_type", str), ("topic_prefix", str), ("position", tuple), ("yaw", float)]
)

StampT = float
StampedRect = NamedTuple('StampedRect',
                         [('stamp', StampT),
                          ('rect', Rectangle),
                          ('reaching_wp', bool)])


class MotionBase(abc.ABC):
    def __init__(self, device_init_info: MotionInitInfo):
        self._var_lock = RLock()
        self._device_init_info = device_init_info
        self._position = device_init_info.position  # type: Tuple[float, float, float]
        self._orientation = (0.0, 0.0, 0.0, 1.0)  # TODO compute Quaternion from initial yaw

    @abc.abstractmethod
    def register_ros_pub_sub(self) -> None:
        """
        ROS publisher and subscriber should be created in this method instead of __init__.
        FIXME This is a temporary solution to ensure rospy.init_node is called before creating publisher/subscriber
            under multiple processes. Because Motion object currently is created in the parent/root process, ROS node
            for each child process is not initialized yet.
            A better way may be reading only multiprocessing-safe config objects and create MotionBase objects in each
            child process.
        """
        raise NotImplementedError

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

    @abc.abstractmethod
    def landing(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def send_target(self, point: Tuple[float, float, float]) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def waypoints_to_plan(self, clk: float, way_points: List) -> List[StampedRect]:
        raise NotImplementedError


def _load_reachtube_from_pickle(filename):
    # FIXME this is a temporary solution to read reachtube from pickle files
    bin_text = importlib_resources.read_binary(primitive_contracts, filename)
    rtube_dict = pickle.loads(bin_text)
    return rtube_dict


class MotionROSplane(MotionBase):
    # FIXME a temporary solution to store primitive contracts
    REACHTUBE_FILE_NAME = "rosplane.rtube.pickle"
    CONTRACT_DICT = _load_reachtube_from_pickle(REACHTUBE_FILE_NAME)

    @staticmethod
    def _position_ned_to_xyz(ned: np.ndarray) -> np.ndarray:
        """ ROSplane aligns north with x-axis, hence, east is negative y, and down is negative z"""
        assert len(ned) == 3
        return np.array([ned[0], -ned[1], -ned[2]])

    @staticmethod
    def _position_xyz_to_ned(xyz: np.ndarray) -> np.ndarray:
        """ ROSplane aligns x-axis with north, hence, y is negative east (west), and z is negative down (up)"""
        assert len(xyz) == 3
        return np.array([xyz[0], -xyz[1], -xyz[2]])

    def __init__(self, device_init_info: MotionInitInfo):
        super(MotionROSplane, self).__init__(device_init_info)

        self._first_flag = 1
        self._pose_client = None

        init_xyz = np.array(self._device_init_info.position)
        init_xyz[2] = 0.0  # Set z to 0
        self._init_ned = self._position_xyz_to_ned(init_xyz)

    def register_ros_pub_sub(self) -> None:
        self._pose_client = rospy.Publisher(self._device_init_info.topic_prefix + "/waypoint_path",
                                            Waypoint, queue_size=10)

    def landing(self) -> None:
        rospy.logwarn("Landing for ROSplane is not supported yet.")

    def send_target(self, point: Tuple[float, float, float]):
      
        target_pose = Waypoint()
        shifted_ned = self._position_xyz_to_ned(np.array(point)) - self._init_ned
        target_pose.w = shifted_ned
        target_pose.chi_d = 0
        target_pose.chi_valid = False
        target_pose.Va_d = 12

        if self._first_flag == 1:
            target_pose.set_current = True 
            print("first")
            self._first_flag = 0
        else:
            target_pose.set_current = False
        # NOTE Do not wait for result
        print("sending waypoints %s" % str(point))
        return self._pose_client.publish(target_pose)

    @classmethod
    def _extend_contract_from_reachtube(cls, plan: List[StampedRect], key: str, t_start: float = 0.0) -> float:
        """
        Extend the given plan with rectangles from the reachtube under the given key.
        Parameters
        ----------
        plan
        key
        t_start

        Returns
        -------
        float
            the timestamp where the last rectangle should still hold.
        """
        SUBSAMPLE_STEP = 40
        t_ned_arr = cls.CONTRACT_DICT[key][:, :, 0:4]
        assert len(t_ned_arr) > 0
        for t_ned in t_ned_arr[::SUBSAMPLE_STEP]:
            t_min, t_max = float(t_ned[0][0]), float(t_ned[1][0])
            ned_min, ned_max = t_ned[0][1:4].astype(float), t_ned[1][1:4].astype(float)
            xyz_min, xyz_max = cls._position_ned_to_xyz(ned_min), cls._position_ned_to_xyz(ned_max)
            rect = Rectangle(mins=xyz_min, maxes=xyz_max)
            plan.append(StampedRect(stamp=t_start+t_min, rect=rect, reaching_wp=False))
        return t_start+t_max

    def waypoints_to_plan(self, clk: float, way_points: List) -> List[StampedRect]:
        ret = []  # type: List[StampedRect]
        next_t_start = clk 
        next_t_start = self._extend_contract_from_reachtube(ret, "takeoff", next_t_start)
        # Shift the loitering contract to be after takeoff contract
        next_t_start += 41.0
        next_t_start = self._extend_contract_from_reachtube(ret, "loiter", next_t_start)

        assert len(ret) > 0
        last_rect = ret[-1].rect
        ret.append(StampedRect(stamp=next_t_start, rect=last_rect, reaching_wp=True))
        return ret


class MotionHectorQuad(MotionBase):
    BLOAT_WIDTH = 0.5

    class Status(Enum):
        STAYING = 0
        MOVING = 1

    def __init__(self, device_init_info: MotionInitInfo):
        super(MotionHectorQuad, self).__init__(device_init_info)
        self._status = self.Status.STAYING
        self._takeoff_client = None
        self._landing_client = None
        self._pose_client = None

    def register_ros_pub_sub(self) -> None:
        topic_prefix = self._device_init_info.topic_prefix
        takeoff_topic = rospy.resolve_name(topic_prefix + "/action/takeoff")
        self._takeoff_client = SimpleActionClient(takeoff_topic, TakeoffAction)
        landing_topic = rospy.resolve_name(topic_prefix + "/action/landing")
        self._landing_client = SimpleActionClient(landing_topic, LandingAction)
        pose_topic = rospy.resolve_name(topic_prefix + "/action/pose")
        self._pose_client = SimpleActionClient(pose_topic, PoseAction)

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

    def waypoints_to_plan(self, clk: float, way_points, default=True) -> List[StampedRect]:
        pos = self.position  # NOTE: self.position returns a copy, so the value won't be changed by other threads.
        if default:
            rect_list = self._bloat_path(pos, way_points)
            deadline = clk
            ret = []
            for rect in rect_list:
                ret.append(StampedRect(deadline, rect, True))
                deadline = deadline + 0.5 * float(euclidean(rect.maxes, rect.mins))
            return ret
        # else:
        flagged_waypoints = self._fixed_resolution(pos, way_points, resolution=2.5)

        deadline_list = [clk]
        prev_p, prev_reach = pos, True
        for p, reach in flagged_waypoints:
            d = float(euclidean(prev_p, p))
            # if reach, the drone is slowing down. if prev_reach, the drone should have slowed down
            # Therefore, the deadline is more relaxed.
            deadline = deadline_list[-1] + d * (0.7 if prev_reach else 0.3 if reach else 0.2)
            deadline_list.append(deadline)
            prev_p, prev_reach = p, reach

        flagged_rect_list = self._bloat_flagged_path(pos, flagged_waypoints)
        assert len(flagged_rect_list) == len(deadline_list)
        ret = [StampedRect(deadline, rect, reached)
               for deadline, (rect, reached) in zip(deadline_list, flagged_rect_list)]
        return ret

    @staticmethod
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

    @classmethod
    def _bloat_flagged_path(cls, cur_pos: Tuple[float, ...],
                            flagged_waypoints: List[Tuple[Tuple[float, ...], bool]]) \
            -> List[Tuple[Rectangle, bool]]:
        assert flagged_waypoints[-1][1]
        ret = []  # type: List[Tuple[Rectangle, bool]]

        curr_rect = cls._bloat_point(cur_pos)
        prev_rect_list = [curr_rect]
        for p, flag in flagged_waypoints:
            curr_rect = cls._bloat_point(p)
            if not flag:
                prev_rect_list.append(curr_rect)
            else:  # At a flagged waypoint
                rect_iter = (cls._bloat_segment(prev_rect, curr_rect) for prev_rect in prev_rect_list)
                flag_list = [False] * (len(prev_rect_list) - 1) + [True]
                ret.extend(zip(rect_iter, flag_list))
                prev_rect_list = [curr_rect]
        ret.append((curr_rect, True))  # Stay at the last waypoint
        assert len(ret) == len(flagged_waypoints) + 1
        return ret

    @classmethod
    def _bloat_path(cls, cur_pos: Tuple[float, ...],
                    way_points: List[Tuple[float, ...]]) -> List[Rectangle]:
        ret = []  # type: List[Rectangle]
        prev_rect = cls._bloat_point(cur_pos)
        for p in way_points:
            curr_rect = cls._bloat_point(p)
            ret.append(cls._bloat_segment(prev_rect, curr_rect))
            prev_rect = curr_rect
        ret.append(prev_rect)  # Stay in the last rect
        return ret

    @staticmethod
    def _bloat_segment(bloat_a: Rectangle, bloat_b: Rectangle) -> Rectangle:
        new_maxes = np.maximum(bloat_a.maxes, bloat_b.maxes)
        new_mins = np.minimum(bloat_a.mins, bloat_b.mins)
        return Rectangle(maxes=new_maxes, mins=new_mins)

    @classmethod
    def _bloat_point(cls, p: Tuple[float, ...]) -> Rectangle:
        p_arr = np.array(p)
        return Rectangle(mins=p_arr - cls.BLOAT_WIDTH,
                         maxes=p_arr + cls.BLOAT_WIDTH)


MOTION_CLASS_MAP = {
    "QUAD": MotionHectorQuad,
    "PLANE": MotionROSplane
}  # type: Mapping[str, Type[MotionBase]]


def build_motion_controller(init_info: MotionInitInfo) -> MotionBase:
    try:
        motion_class = MOTION_CLASS_MAP[init_info.bot_type.upper()]
        return motion_class(init_info)
    except KeyError:
        raise ValueError("Unknown vehicle type '%s' for '%s'" % (init_info.bot_type, init_info.bot_name))
