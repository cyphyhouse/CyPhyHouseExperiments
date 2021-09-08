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

from actionlib import GoalStatus, SimpleActionClient, SimpleGoalState
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3, TwistStamped, Twist
from hector_uav_msgs.msg import LandingAction, LandingGoal, \
    PoseAction, PoseGoal, TakeoffAction, TakeoffGoal
# from hector_uav_msgs import HeadingCommand, VelocityXYCommand
from rosplane_msgs.msg import Waypoint
import rospy
import numpy as np
from scipy.spatial import Rectangle
from scipy.spatial.distance import euclidean
from scipy.spatial.transform import Rotation

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

import math as m

AXES_SEQ = "ZYX" 

from . import primitive_contracts

MotionInitInfo = NamedTuple(
    "MotionInitInfo",
    [("bot_name", str), ("bot_type", str), ("topic_prefix", str), ("position", tuple), ("yaw", float), ("init_pose", Pose), ("init_twist", Twist)]
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
        self._init_pose = device_init_info.init_pose
        self._init_twist = device_init_info.init_twist
        orientation = device_init_info.init_pose.orientation
        self._orientation = (orientation.x, orientation.y, orientation.z, orientation.w) # (0.0, 0.0, 0.0, 1.0)  # TODO compute Quaternion from initial yaw
        self._init_euler_angles = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self._init_phi =  self._init_euler_angles[0] # roll
        self._init_theta = self._init_euler_angles[1] # pitch
        self._init_psi = self._init_euler_angles[2] # yaw
        linear = device_init_info.init_twist.linear 
        angular = device_init_info.init_twist.angular 
        self._linear_velocity = (linear.x, linear.y, linear.z) # (0,0,0)
        self._init_linear_velocity = deepcopy(self._linear_velocity)
        self._angular_velocity = (angular.x, angular.y, angular.z)
        self._time_step = 0.01

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

    @property
    def linear_velocity(self) -> Tuple[float, float, float]:
        with self._var_lock:
            print("$$$$$$$$", self._linear_velocity)
            return deepcopy(self._linear_velocity)

    @linear_velocity.setter
    def linear_velocity(self, p: Union[Vector3, Tuple[float, float, float]]) -> None:
        if isinstance(p, Vector3):
            p = (p.x, p.y, p.z)

        with self._var_lock:
            self._linear_velocity = p

    @property
    def angular_velocity(self) -> Tuple[float, float, float]:
        with self._var_lock:
            return deepcopy(self._angular_velocity)

    @angular_velocity.setter
    def angular_velocity(self, p: Union[Vector3, Tuple[float, float, float]]) -> None:
        if isinstance(p, Vector3):
            p = (p.x, p.y, p.z)

        with self._var_lock:
            self._angular_velocity = p

    @abc.abstractmethod
    def landing(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def send_target(self, point: Tuple[float, float, float]) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def waypoints_to_plan(self, clk: float, way_points: List) -> List[StampedRect]:
        raise NotImplementedError
    
    def quaternion_to_euler(self, x, y, z, w):
        yaw, pitch, roll = Rotation.from_quat([x, y, z, w]).as_euler(AXES_SEQ)
        return [roll, pitch, yaw]


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

    # def waypoints_to_plan(self, clk: float, way_points: List) -> List[StampedRect]:
    #     ret = []  # type: List[StampedRect]
    #     next_t_start = clk + 5.0
    #     next_t_start = self._extend_contract_from_reachtube(ret, "takeoff", next_t_start)
    #     # Shift the loitering contract to be after takeoff contract
    #     next_t_start = self._extend_contract_from_reachtube(ret, "interchange", next_t_start)
    #     next_t_start = self._extend_contract_from_reachtube(ret, "loiter", next_t_start)
    #     next_t_start = self._extend_contract_from_reachtube(ret, "descend", next_t_start)

    #     assert len(ret) > 0
    #     last_rect = ret[-1].rect
    #     ret.append(StampedRect(stamp=next_t_start, rect=last_rect, reaching_wp=True))
    #     return ret
    
    def waypoints_to_plan(self, clk: float, way_points: List) -> List[StampedRect]:
        ret = []
        t_start = clk + 5.0
        ref_t = 0
        for wp in way_points:
            xyz_min, xyz_max = (wp[0]-0.1, wp[1]-0.1, wp[2]-0.1), (wp[0]+0.1, wp[1]+0.1, wp[2]+0.1)
            t_min, t_max = ref_t, ref_t + 0.01
            rect = Rectangle(maxes=xyz_max, mins=xyz_min)
            ret.append(StampedRect(stamp=t_start+t_min, rect = rect, reaching_wp = False))
            t_start += t_max
            ref_t += 1000

        assert len(ret) > 0
        last_rect = ret[-1].rect
        ret.append(StampedRect(stamp=t_start, rect=last_rect, reaching_wp=True))
        return ret
    
class MotionHectorQuad(MotionBase):
    BLOAT_WIDTH = 0.001

    class Status(Enum):
        STAYING = 0
        MOVING = 1

    def __init__(self, device_init_info: MotionInitInfo):
        super(MotionHectorQuad, self).__init__(device_init_info)
        self._status = self.Status.STAYING
        self._takeoff_client = None
        self._landing_client = None
        self._pose_client = None
        #self._velocityXY_client = None
        #self._heading_client = None

    def register_ros_pub_sub(self) -> None:
        topic_prefix = self._device_init_info.topic_prefix
        takeoff_topic = rospy.resolve_name(topic_prefix + "/action/takeoff")
        self._takeoff_client = SimpleActionClient(takeoff_topic, TakeoffAction)
        landing_topic = rospy.resolve_name(topic_prefix + "/action/landing")
        self._landing_client = SimpleActionClient(landing_topic, LandingAction)
        pose_topic = rospy.resolve_name(topic_prefix + "/action/pose")
        self._pose_client = SimpleActionClient(pose_topic, PoseAction)
        
        # pose_topic = rospy.resolve_name(topic_prefix + "/action/")
        self._twist_limit_publisher = rospy.Publisher(topic_prefix+"/command/twist_limit",Twist,queue_size=1)
        #velocityXY_topic = rospy.resolve_name(topic_prefix + "/action/velocityXY")
        #self._velocityXY_client = SimpleActionClient(velocityXY_topic, VelocityXYCommand)
        #heading_topic = rospy.resolve_name(topic_prefix + "/action/velocityXY")
        #self._headineg_client = SimpleActionClient(heading_topic, HeadingCommand)
        #rospy.init_node(topic_prefix + '_twist_control_node', anonymous=True)
        # self._twist_client = rospy.Publisher(topic_prefix + "/cmd_vel", Twist, queue_size=10)
        

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
    
    @staticmethod
    def _to_twist_stamped(point: Tuple[float, float, float]) -> Twist:
        res = Twist()
        res.linear.x = point[0]
        res.linear.y = point[1]
        res.linear.z = point[2]
        return res

    '''    
    def _control(self, desired, action):
        
        # Constants
        Kp, Kp_bar, Kd = 0.9, 0.1, 0.1

        # Variables
        # (x, y, z, vx, vy, vz, phi, theta, psi) = self._state[:9]
        x = self._position[0]
        y = self._position[1]
        z = self._position[2]
        vx = self._linear_velocity[0]
        vy = self._linear_velocity[1]
        vz = self._linear_velocity[2]
        euler_angles = self.quaternion_to_euler(self._orientation[0], self._orientation[1], self._orientation[2], self._orientation[3])
        phi =  euler_angles[0] # roll
        theta = euler_angles[1] # pitch
        psi = euler_angles[2] # yaw
        
        (x_d, y_d, z_d, vx_d, vy_d, vz_d, phi_d, theta_d, psi_d,
         ax_d, ay_d, az_d, dphi_d, dtheta_d, dpsi_d) = desired

        # Derivative of state
        (_, _, _, _, _, _, dphi, dtheta, dpsi) = self._dynamics(self._state[:9], 1.0, action)

        # Feedforward control
        f_ff = -self._mass * m.sqrt(ax_d ** 2 + ay_d ** 2 + (az_d - self.G) ** 2)
        w1_ff = dphi_d - m.sin(theta_d) * dpsi_d
        w2_ff = m.cos(phi_d) * dtheta_d + m.sin(phi_d) * m.cos(theta_d) * dpsi_d
        w3_ff = -m.sin(phi_d) * dtheta_d + m.cos(phi_d) * m.cos(theta_d) * dpsi_d

        # Feedback control
        f_fbx = ((m.cos(phi) * m.sin(theta) * m.cos(psi) + m.sin(phi) * m.sin(psi)) *
                 ((x_d - x) * Kp + (vx_d - vx) * Kd))
        f_fby = ((m.cos(phi) * m.sin(theta) * m.sin(psi) - m.sin(phi) * m.cos(psi)) *
                 ((y_d - y) * Kp + (vy_d - vy) * Kd))
        f_fbz = m.cos(phi) * m.cos(theta) * ((z_d - z) * Kp + (vz_d - vz) * Kd)
        f_fb = f_fbx + f_fby + f_fbz

        w1_fb = Kp * (phi_d - phi) + Kd * (dphi_d - dphi) + Kp_bar * (y_d - y)
        w2_fb = Kp * (theta_d - theta) + Kd * (dtheta_d - dtheta) + Kp_bar * (x_d - x)
        w3_fb = Kp * (psi_d - psi) + Kd * (dpsi_d - dpsi)

        return [f_ff + f_fb, w1_ff + w1_fb, w2_ff + w2_fb, w3_ff + w3_fb]
    
        
    

    @staticmethod
    def _compute_angle(psi, vec):
        unit = np.array([1.0, 0.0])
        c, s = m.cos(psi), m.sin(psi)
        R = np.array(((c, -s), (s, c)))

        heading = np.matmul(R, unit)
        diff = np.arctan2(np.linalg.det([heading, vec]), np.dot(heading, vec)) 
        return psi + diff


    def _compute_desired_state(self, goal):
    
        # State variables (x, y, z, vx, vy, vz, phi, theta, psi) = self._state[:9]
        
        x = self._position[0]
        y = self._position[1]
        z = self._position[2]
        vx = self._linear_velocity[0]
        vy = self._linear_velocity[1]
        vz = self._linear_velocity[2]
        euler_angles = self.quaternion_to_euler(self._orientation[0], self._orientation[1], self._orientation[2], self._orientation[3])
        phi =  euler_angles[0] # roll
        theta = euler_angles[1] # pitch
        psi = euler_angles[2] # yaw
        
        # Compute distance and angle to goal
        v2 = np.array([goal[0] - x, goal[1] - y])
        dist = np.linalg.norm(v2)
        angle = self._compute_angle(psi, v2)

        # Compute desired state
        if np.linalg.norm(v2) >= 1:
            con = v2 / dist
        else:
            con = v2
            
        delta = 0.2

        x_d = (delta * self._init_linear_velocity[0] + (1- delta) * con[0]) * self._time_step + x # con[0]
        y_d = (delta * self._init_linear_velocity[1] + (1- delta) * con[1]) * self._time_step + y # con[1] 
        z_d = (delta * self._init_linear_velocity[2] + (1- delta) * (goal[2] - z)) * self._time_step + z # (goal[2] - z)

        vx_d =  (delta * self._init_linear_velocity[0] + (1- delta) * con[0] / self._time_step) # 
        vy_d =  (delta * self._init_linear_velocity[1] + (1- delta) * con[1] / self._time_step) # 
        vz_d =  (delta * self._init_linear_velocity[2] + (1- delta) * (goal[2] - z) / self._time_step) #

        ax_d = (vx_d - vx)
        ay_d = (vy_d - vy)
        az_d = vz_d - vz

        psi_d = (delta * self._init_psi + (1- delta) * (m.pi / 2 - angle))
        beta_a = -ax_d * m.cos(psi_d) - ay_d * m.sin(psi_d)
        beta_b = -az_d + 9.81
        beta_c = -ax_d * m.sin(psi_d) + ay_d * m.cos(psi_d)
        theta_d = m.atan2(beta_a, beta_b) + self._init_theta
        phi_d = m.atan2(beta_c, m.sqrt(beta_a ** 2 + beta_b ** 2)) + self._init_phi

        dphi_d = (phi_d -phi) / self._time_step
        dtheta_d = (theta_d - theta) / self._time_step
        dpsi_d = (psi_d - psi) / self._time_step
        
        desired = [x_d, y_d, z_d, vx_d, vy_d, vz_d, phi_d, theta_d, psi_d, ax_d, ay_d, az_d, dphi_d, dtheta_d, dpsi_d]
        return desired
    ''' 


    def send_target(self, point: Tuple[float, float, float]):
        self._pose_client.wait_for_server()
        pose_goal = PoseGoal(target_pose=self._to_pose_stamped(point))
        # NOTE Do not wait for result
        return self._pose_client.send_goal(pose_goal)
        
    def send_target_twist_limit(self, twist: Tuple[float, float, float]):
        self._twist_limit_publisher.publish(self._to_twist_stamped(twist))
        # return point
        
        '''
        dist_vec = np.array([point[0] - self._position[0], point[1] - self._position[1], point[2] - self._position[2]])
        dist_norm = np.linalg.norm(dist_vec)
        vel_norm = np.linalg.norm([self._init_linear_velocity[0], self._init_linear_velocity[1], self._init_linear_velocity[2]])
        time_to_reach =  dist_norm / vel_norm
        steps_to_reach = time_to_reach / self._time_step
        rate = rospy.Rate(100)
        while abs(self._position[0] - point[0]) > 2 or abs(self._position[1] - point[1]) > 2: # for i in range(m.floor(steps_to_reach)):
            desired_state = self._compute_desired_state(point)
            lin = Vector3(x=desired_state[3], y=desired_state[4], z=desired_state[5])  # linear velocity
            ang = Vector3(x=desired_state[-1], y=0.0, z=0.0)  # angular velocity
            target_twist = Twist(linear=lin, angular=ang)
            self._twist_client.publish(target_twist)
            print("publishing twist, linear:" + str(target_twist.linear) + ", angular:" + str(target_twist.angular))
            rate.sleep()
        '''
        
        # print("publishing twist, linear:" + str(target_twist.linear) + ", angular:" + str(target_twist.angular))
        # return self._twist_client.publish(target_twist)

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
            for i,rect in enumerate(rect_list):
                ret.append(StampedRect(deadline, rect, True))
                if i == 0:
                    deadline = deadline + 0.2*float(euclidean(way_points[i], pos))
                elif i != len(way_points):                    
                    deadline = deadline + 0.2*float(euclidean(way_points[i], way_points[i-1]))
            return ret
        # else:
        flagged_waypoints = self._fixed_resolution(pos, way_points, resolution=2.5)

        deadline_list = [clk]
        prev_p, prev_reach = pos, True
        for p, reach in flagged_waypoints:
            d = float(euclidean(prev_p, p))
            # if reach, the drone is slowing down. if prev_reach, the drone should have slowed down
            # Therefore, the deadline is more relaxed.
            # deadline = deadline_list[-1] + d * (0.7 if prev_reach else 0.3 if reach else 0.2)
            deadline = deadline_list[-1] + d * 0.2
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
            # ret.append(cls._bloat_segment(prev_rect, curr_rect))
            ret.append(prev_rect)
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
