from copy import deepcopy
from enum import Enum
from typing import Any, Callable, Hashable, List, NamedTuple, Optional, Tuple

import rospy  # FIXME avoid rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped

from reachtube.drone3d_types import Contract

from .motion import MotionBase, StampT, StampedRect
from .tioa_base import Action, AutomatonBase

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

from scipy.spatial.transform import Rotation

import math as m

AXES_SEQ = "ZYX"  # Convention for Euler angles. Follow REP-103
GZ_SET_MODEL_STATE = "/gazebo/set_model_state"


import time
import numpy as np

import pickle

StampedPoint = NamedTuple('StampedPoint',
                          [('stamp', StampT),
                           ('position', Tuple[float, float, float])])


class Agent(AutomatonBase):
    class Status(Enum):
        IDLE = 0
        REQUESTING = 1
        WAITING = 2
        MOVING = 4
        RELEASING = 5
        STOPPING = 6
        ACAS = 7
        INIT = 8

    class RA(Enum):
        CLIMB = 4
        DESCEND = 5
        VERT_RATE_LIMIT = 6
        TURN_RIGHT = 2
        TURN_LEFT = 3

    def __init__(self, uid: Hashable, motion: MotionBase, waypoints: List):
        super(Agent, self).__init__()

        self.id = id
        self.__uid = uid  # Set it as private to avoid being modified
        self.__motion = motion
        self.__way_points = deepcopy(waypoints)
        self.__way_points_backup = deepcopy(waypoints)

        self._status = Agent.Status.INIT # Agent.Status.IDLE
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
        self._orientation = self.__motion.orientation
        self._linear_velocity = self.__motion._linear_velocity #(0,0,0)
        self._angular_velocity = self.__motion._angular_velocity # (0,0,0)
        self.__init_pose = deepcopy(motion._init_pose)
        self.__init_twist = deepcopy(motion._init_twist)

        self.record_linear_v = self.__motion._linear_velocity # (0,0,0)
        
        self._time_step = 0.01
        
        

        self.vra_type: str = ""
        self.hra_type: str = ""
        self.hra_val: float = None
        self.clock = time.time()

        self.ra_waypoint = None

        self.trajectory = []
        self.received_ra_list = []
        self.handled_ra_list = []
        
        fn = f'./trajectories/{self.uid}_agent_acas_received'
        f = open(fn, 'wb+')
        f.close()
        fn = f'./trajectories/{self.uid}_agent_acas_handled'
        f = open(fn, 'wb+')
        f.close()
        
    @staticmethod
    def set_model_pose(model_name, pose: Pose, twist: Twist) -> None:
        msg = ModelState()
        msg.model_name = model_name
        msg.reference_frame = "world"  # Default is also world.
        msg.pose = pose
        msg.twist = twist

        rospy.wait_for_service(GZ_SET_MODEL_STATE)
        try:
            set_model_state_srv = \
                    rospy.ServiceProxy(GZ_SET_MODEL_STATE, SetModelState)
            resp = set_model_state_srv(msg)

        # TODO Check response
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)


    def register_ra_subscriber(self):
        self._acas_ra_sub = rospy.Subscriber('/acas/ra', String, self.__acas_ra_handler)            

    def __acas_ra_handler(self, data):
        raw_ra_string = data.data
        # self.received_ra_list.append((time.time(), raw_ra_string))
        fn = f'./trajectories/{self.uid}_agent_acas_received'
        with open(fn, "ab+") as f:
            pickle.dump((time.time(), raw_ra_string), f)
        # print(raw_ra_string)
        raw_ra_string = raw_ra_string.split(',')
        # print(raw_ra_string)
        self.vra_type = ""
        self.hra_type = ""
        data_id = raw_ra_string[0]
        if data_id == self.uid:
            for i in range(1, len(raw_ra_string)):
                ra = raw_ra_string[i]
                if "Climb" in ra:
                    self.vra_type = Agent.RA.CLIMB
                elif "Descend" in ra:
                    self.vra_type = Agent.RA.DESCEND
                elif "Limit" in ra:
                    self.vra_type = Agent.RA.VERT_RATE_LIMIT
                
                if "Right" in ra:
                    self.hra_type = Agent.RA.TURN_RIGHT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    self.hra_val = float(val_string[1].strip())
                elif "Left" in ra:
                    self.hra_type = Agent.RA.TURN_LEFT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    self.hra_val = float(val_string[1].strip())

    def record_trajectory(self):
        tmp_str_v = ''
        if self.vra_type == self.RA.CLIMB:
            tmp_str_v= 'Climb'
        elif self.vra_type == self.RA.DESCEND:
            tmp_str_v= 'Descend'

        tmp_str_h = ''
        if self.hra_type == self.RA.TURN_LEFT:
            tmp_str_h = f"Turn Left {self.hra_val}"
        elif self.vra_type == self.RA.TURN_RIGHT:
            tmp_str_h = f"Turn Right {self.hra_val}"
            
        tmp_str = f"{self.uid}, {tmp_str_v}, {tmp_str_h}"
        curr_orientation = self._orientation
        if self._status == Agent.Status.ACAS:
            self.trajectory.append([time.time(), self._position[0], self._position[1], self._position[2], tmp_str, 1, self.quaternion_to_euler(curr_orientation[0], curr_orientation[1], curr_orientation[2], curr_orientation[3])[2], self._linear_velocity[0], self._linear_velocity[1], self._linear_velocity[2]])
        else:
            self.trajectory.append([time.time(), self._position[0], self._position[1], self._position[2], tmp_str, 0, self.quaternion_to_euler(curr_orientation[0], curr_orientation[1], curr_orientation[2], curr_orientation[3])[2], self._linear_velocity[0], self._linear_velocity[1], self._linear_velocity[2]])

    def dump_trajectory(self):
        fn = f'./trajectories/{self.uid}'
        with open(fn,'wb+') as f:
            pickle.dump(self.trajectory, f)
        # fn = f'./trajectories/{self.uid}_agent_acas_received'
        # with open(fn,'wb+') as f:
        #     pickle.dump(self.received_ra_list, f)
        # fn = f'./trajectories/{self.uid}_agent_acas_handled'
        # with open(fn,'wb+') as f:
        #     pickle.dump(self.handled_ra_list, f)
        

    def __repr__(self) -> str:
        return self.__class__.__name__ + "_" + str(self.uid)


    @property
    def _target(self) -> Tuple[float, float, float]:
        raise RuntimeWarning("Reading actuator value is not allowed")

    @_target.setter
    def _target(self, p: Tuple[float, float, float]) -> None:
        print(f'sending target for agent {self.__uid} type {self.__motion._device_init_info.bot_type}')
        self.__motion.send_target(p)
        #self.__motion.send_target_twist(p) # self.__init_twist


    @property
    def uid(self) -> Hashable:
        return self.__uid

    @property
    def motion(self) -> MotionBase:
        return self.__motion

    def is_internal(self, act: Action) -> bool:
        return act[0] == "plan" \
            or act[0] == "next_region" or act[0] == "succeed" or act[0] == "fail" or act[0] == "acas" or act[0] == "init" # or act[0] == "moving"

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
            "fail": build_trans(pre=self._pre_fail, eff=self._eff_fail),
            "release": build_trans(pre=self._pre_release, eff=self._eff_release),
            "acas": build_trans(pre=self._pre_acas, eff=self._eff_acas),
            "init": build_trans(pre=self._pre_init, eff=self._eff_init),
            #"moving": build_trans(pre=self._pre_moving, eff=self._eff_moving)
        }

        try:
            eff_dict[act[0]](**act[1])
        except KeyError:
            raise KeyError("Unknown action \"%s\"" % str(act))

    def _pre_acas(self) -> bool:
        # print(self.vra_type, self.hra_type, self.hra_val)
        # return False
        return self._status == Agent.Status.ACAS or \
            (self._status == Agent.Status.MOVING and \
            self.vra_type!='' or (self.hra_type!='' and self.hra_val is not None))

    def find_closest_waypoints(self, curr_pos, waypoint_list):
        
        curr_pos = np.array(deepcopy(curr_pos))
        curr_waypoints = deepcopy(waypoint_list)
        if len(curr_waypoints) >= 2:
            tmp = np.linalg.norm(np.array(curr_waypoints[0]) - np.array(curr_waypoints[1]))
            curr_waypoints.insert(0,(curr_waypoints[0][0], curr_waypoints[0][1]-tmp, curr_waypoints[0][2]))
            tmp = np.linalg.norm(np.array(curr_waypoints[-1]) - np.array(curr_waypoints[-2]))
            curr_waypoints.append((curr_waypoints[-1][0], curr_waypoints[-1][1]+tmp, curr_waypoints[-1][2]))
        else:
            curr_waypoints.insert(0,(curr_waypoints[0][0], curr_waypoints[0][1]-100, curr_waypoints[0][2]))
            curr_waypoints.append((curr_waypoints[-1][0], curr_waypoints[-1][1]+100, curr_waypoints[-1][2]))
        
        curr_waypoints = np.array(curr_waypoints)
        print(">>>>>>>>", curr_waypoints)
        print(">>>>>>>>", curr_pos)
        tmp = np.linalg.norm(curr_waypoints - curr_pos, axis = 1)
        print(">>>>>>>>", tmp)
        sorted_idx = np.argsort(tmp)
        print(">>>>>>>>", sorted_idx)
        return max(max(sorted_idx[0]-1, sorted_idx[1]-1),0)

    def quaternion_to_euler(self, x, y, z, w):
        yaw, pitch, roll = Rotation.from_quat([x, y, z, w]).as_euler(AXES_SEQ)
        return [roll, pitch, yaw]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [roll, pitch, yaw]

    def _eff_acas(self) -> None:
        if self.__motion._device_init_info.bot_type == 'PLANE':
            if self.vra_type == '' and self.hra_type == '':
                self.ra_waypoint = None
                self._status = Agent.Status.RELEASING
                waypoint_idx = self.find_closest_waypoints(self._position, self.__way_points_backup)
                print(f">>>> waypoint idx for return{waypoint_idx}")
                if waypoint_idx<len(self.__way_points_backup):
                    self.__way_points = self.__way_points_backup[waypoint_idx:]
                else:
                    self.__way_points = []
            else:
                print("handling RA for plane 1")
                print("^^^^^^^^^^^", self._linear_velocity)
                if self._status != Agent.Status.ACAS:
                    self.record_linear_v = deepcopy(self._linear_velocity)
                self._status = Agent.Status.ACAS
                curr_linear_v = deepcopy(self._linear_velocity)
                print("---------->", curr_linear_v, self._linear_velocity, self._position)
                curr_speed = np.sqrt(curr_linear_v[0]**2 + curr_linear_v[1]**2)
                tgt_waypoint_list = []
                tgt_offset_v = (0,0,0)
                tmp_str_v = ''
                if self.vra_type == self.RA.CLIMB:
                    curr_orientation = self._orientation
                    tmp_str_v = 'Climb'
                    # self.clock = time.time()
                    roll, pitch, yaw = self.quaternion_to_euler(curr_orientation[0], curr_orientation[1], curr_orientation[2], curr_orientation[3])
                    x_off = curr_speed*5 * np.cos(np.pi/2 - yaw)
                    y_off = curr_speed*5 * np.sin(np.pi/2 - yaw)
                    z_off = 15
                    tgt_offset_v = (curr_linear_v[0]*5, curr_linear_v[1]*5, z_off)
                    print("############", tgt_offset_v)
                if self.vra_type == self.RA.DESCEND:
                    curr_orientation = self._orientation
                    tmp_str_v = 'Descend'
                    # self.clock = time.time()
                    roll, pitch, yaw = self.quaternion_to_euler(curr_orientation[0], curr_orientation[1], curr_orientation[2], curr_orientation[3])
                    x_off = curr_speed*5 * np.cos(np.pi/2 - yaw)
                    y_off = curr_speed*5 * np.sin(np.pi/2 - yaw)
                    z_off = -15
                    tgt_offset_v = (curr_linear_v[0]*5, curr_linear_v[1]*5, z_off)
                    print("############", tgt_offset_v)
                
                tgt_offset_h = (0,0,0)
                tmp_str_h = ''
                if self.hra_type == self.RA.TURN_LEFT:
                    tgt_offset_v = (0,0,tgt_offset_v[2])
                    tmp_str_h = f"Turn Left {self.hra_val}"
                    target_angle = self.hra_val
                    target_angle_rad = np.radians(target_angle)
                    x_off = np.cos(target_angle_rad) * 30
                    y_off = np.sin(target_angle_rad) * 30
                    tgt_offset_h = (x_off, y_off, 0)
                elif self.vra_type == self.RA.TURN_RIGHT:
                    tgt_offset_v = (0,0,tgt_offset_v[2])
                    tmp_str_h = f"Turn Right {self.hra_val}"
                    target_angle = self.hra_val
                    target_angle_rad = np.radians(target_angle)
                    x_off = np.cos(target_angle_rad) * 200
                    y_off = np.sin(target_angle_rad) * 200
                    tgt_offset_h = (x_off, y_off, 0)
                
                curr_pose = self._position
                for i in range(1,5):
                    tgt = (
                        curr_pose[0] + i*tgt_offset_v[0] + i*tgt_offset_h[0],
                        curr_pose[1] + i*tgt_offset_v[1] + i*tgt_offset_h[1],
                        curr_pose[2] + i*tgt_offset_v[2] + i*tgt_offset_h[2]
                    )
                    tgt_waypoint_list.append(tgt)
                
                print("handling RA for plane 2")
                if time.time() - self.clock > 1:
                    tmp_str = f"{self.uid}, {tmp_str_v}, {tmp_str_h}"
                    print(tmp_str)
                    # self.handled_ra_list.append((time.time(), tmp_str))
                    fn = f'./trajectories/{self.uid}_agent_acas_handled'
                    with open(fn, 'ab+') as f:
                        pickle.dump((time.time(), tmp_str), f)
                    print("handling RA for plane 3")
                
                    rospy.logdebug("%s sending all waypoints %s." % (self, tgt_waypoint_list))
                    self.__motion._first_flag = 1
                    for tgt in tgt_waypoint_list:
                        rospy.sleep(0.5)
                        self._target = tgt
        else:
            if self.vra_type == '' and self.hra_type == '':
                self.ra_waypoint = None
                self._status = Agent.Status.RELEASING
            else:
                if self._status != Agent.Status.ACAS:
                    self.record_linear_v = deepcopy(self._linear_velocity)
                self._status = Agent.Status.ACAS
                curr_linear_v = self.record_linear_v
                tgt_vector_v = (0,0,0)
                tmp_str_v = ''
                if self.vra_type == self.RA.CLIMB:
                    tmp_str_v= 'Climb'
                    tgt_vector_v = (curr_linear_v[0],curr_linear_v[1],3)
                elif self.vra_type == self.RA.DESCEND:
                    # if time.time() - self.clock > 1:
                    tmp_str_v= 'Descend'
                    # self.clock = time.time()
                    tgt_vector_v = (curr_linear_v[0],curr_linear_v[1],-3)

                tgt_vector_h = (0,0,0)
                tmp_str_h = ''
                if self.hra_type == self.RA.TURN_LEFT:
                    # print(f"Turn Left {self.hra_val}")
                    # pass
                    tmp_str_h = f"Turn Left {self.hra_val}"
                    target_angle = self.hra_val
                    curr_speed = np.sqrt(curr_linear_v[0]**2+curr_linear_v[1]**2)+3
                    target_angle_rad = np.radians(target_angle)
                    x_off = np.sin(target_angle_rad) * curr_speed
                    y_off = np.cos(target_angle_rad) * curr_speed
                    tgt_vector_h = (x_off, y_off, 0)
                elif self.hra_type == self.RA.TURN_RIGHT:
                    tmp_str_h = f"Turn Right {self.hra_val}"
                    target_angle = self.hra_val
                    curr_speed = np.sqrt(curr_linear_v[0]**2+curr_linear_v[1]**2)+3
                    target_angle_rad = np.radians(target_angle)
                    x_off = np.sin(target_angle_rad) * curr_speed
                    y_off = np.cos(target_angle_rad) * curr_speed
                    tgt_vector_h = (x_off, y_off, 0)

                
                # if self.ra_waypoint is None:
                #     self.ra_waypoint = (self._position[0], self._position[1], self._position[2])
                
                tgt = (
                    self._position[0] + tgt_vector_v[0] + tgt_vector_h[0], 
                    self._position[1] + tgt_vector_v[1] + tgt_vector_h[1], 
                    self._position[2] + tgt_vector_v[2] + tgt_vector_h[2]
                )
                
                if time.time() - self.clock > 0.1:
                    print(tgt)
                    self._target = tgt
                    self.ra_waypoint = tgt
                    self.clock = time.time()

                # if tmp_str_h != '' or tmp_str_v != '':
                    tmp_str = f"{self.uid}, {tmp_str_v}, {tmp_str_h}"
                    print(tmp_str)
                    # self.handled_ra_list.append((time.time(), tmp_str))
                    fn = f'./trajectories/{self.uid}_agent_acas_handled'
                    with open(fn, 'ab+') as f:
                        pickle.dump((time.time(), tmp_str), f)
    
    def _pre_init(self) -> bool:
        return self._status == Agent.Status.INIT # and self._retry_time <= self.clk
    
    def _eff_init(self) -> None:
        print("initializing drone: ", self.__uid)
        # self.set_model_pose(self.__uid, self.__init_pose, self.__init_twist)
        self._status = Agent.Status.IDLE
    
    def quaternion_to_euler(self, x, y, z, w):
        yaw, pitch, roll = Rotation.from_quat([x, y, z, w]).as_euler(AXES_SEQ)
        return [roll, pitch, yaw]

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
        
        x = self.__motion._position[0]
        y = self.__motion._position[1]
        z = self.__motion._position[2]
        vx = self.__motion._linear_velocity[0]
        vy = self.__motion._linear_velocity[1]
        vz = self.__motion._linear_velocity[2]
        euler_angles = self.quaternion_to_euler(self.__motion._orientation[0], self.__motion._orientation[1], self.__motion._orientation[2], self.__motion._orientation[3])
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
            
        init_lin_xy_mag = np.linalg.norm(np.array([self.__motion._init_linear_velocity[0], self.__motion._init_linear_velocity[1]]))
            
        delta = 0.0
        x_d = (delta * self.__motion._init_linear_velocity[0] + (1- delta) * con[0]) * self._time_step + x # con[0]
        y_d = (delta * self.__motion._init_linear_velocity[1] + (1- delta) * con[1]) * self._time_step + y # con[1] 
        z_d = (delta * self.__motion._init_linear_velocity[2] + (1- delta) * (goal[2] - z)) * self._time_step + z # (goal[2] - z)

        vx_d = (delta * self.__motion._init_linear_velocity[0] + (1- delta) * con[0] * init_lin_xy_mag) # # 
        vy_d = (delta * self.__motion._init_linear_velocity[1] + (1- delta) * con[1] * init_lin_xy_mag) # 
        vz_d = (delta * self.__motion._init_linear_velocity[2] + (1- delta) * (goal[2] - z)) #

        ax_d = (vx_d - vx)
        ay_d = (vy_d - vy)
        az_d = vz_d - vz

        psi_d = (delta * self.__motion._init_psi + (1- delta) * (angle))
        beta_a = -ax_d * m.cos(psi_d) - ay_d * m.sin(psi_d)
        beta_b = -az_d + 9.8065
        beta_c = -ax_d * m.sin(psi_d) + ay_d * m.cos(psi_d)
        theta_d = m.atan2(beta_a, beta_b) + self.__motion._init_theta
        phi_d = m.atan2(beta_c, m.sqrt(beta_a ** 2 + beta_b ** 2)) + self.__motion._init_phi

        dphi_d = (phi_d -phi) # / self._time_step
        dtheta_d = (theta_d - theta) # / self._time_step
        dpsi_d = (psi_d - psi) #/ self._time_step
        
        desired = [x_d, y_d, z_d, vx_d, vy_d, vz_d, phi_d, theta_d, psi_d, ax_d, ay_d, az_d, dphi_d, dtheta_d, dpsi_d]
        return desired
    '''
    def _pre_moving(self) -> bool:
        return (self._status == Agent.Status.MOVING and len(self._plan) >= 2 and self.clk.to_sec() < self._plan[1].stamp) or  self._status == Agent.Status.ACAS
    
    def _eff_moving(self) -> None:
        print("moving drone: ", self.__uid)
        self.__motion.send_target(self._target) 
        # while abs(self._position[0] - point[0]) > 2 or abs(self._position[1] - point[1]) > 2: # 
        ''''''
        rate = rospy.Rate(100)
        for i in range(100):
            desired_state = self._compute_desired_state(self._target)
            lin = Vector3(x=desired_state[3], y=desired_state[4], z=desired_state[5])  # linear velocity
            ang = Vector3(x=desired_state[-1], y=desired_state[-2], z=desired_state[-3])  # angular velocity
            target_twist = Twist(linear=lin, angular=ang)
            self.__motion._twist_client.publish(target_twist)
            print("publishing twist, linear:" + str(target_twist.linear) + ", angular:" + str(target_twist.angular))
            rate.sleep()
    '''      


    def _pre_plan(self) -> bool:
        return self._status == Agent.Status.IDLE and self._retry_time <= self.clk

    def _eff_plan(self) -> None:
        if self.__way_points:
            self._plan = self.__motion.waypoints_to_plan(self.clk.to_sec(), self.__way_points)
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

                if(self.__motion._device_init_info.bot_type=='PLANE'):
                    rospy.logdebug("%s sending all waypoints %s." % (self, self.__way_points))
                    self.__motion._first_flag = 1
                    for tgt in self.__way_points:
                        rospy.sleep(0.5)
                        self._target = tgt
                    self.__way_points_backup = deepcopy(self.__way_points)
                    self.__way_points.clear()

                else:
                    tgt = self.__way_points[0] # self.__way_points.pop(0)
                    rospy.logdebug("%s going to %s." % (self, str(tgt)))
                    self._target = tgt
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

        if(self.__motion._device_init_info.bot_type=='PLANE'):
            if prev.reaching_wp:
                rospy.logdebug("%s going to next region %s." % (self, prev.rect))
                print(f"agent {self.uid} reaching waypoint {tgt}")
        else:
            if prev.reaching_wp and self.__way_points:
                self.__way_points.pop(0)
                if self.__way_points:
                    tgt = self.__way_points[0]
                    rospy.logdebug("%s going to %s." % (self, str(tgt)))
                    self._target = tgt                     

    def _pre_succeed(self) -> bool:
        return self._status == Agent.Status.MOVING and len(self._plan) == 1 \
            and self._membership_query(StampedPoint(self.clk.to_sec(), self._position),
                                       self._plan_contr)

    def _eff_succeed(self) -> None:
        self._free_contr = self._curr_contr - self._plan_contr
        self._status = Agent.Status.RELEASING
        rospy.logdebug("Agent %s succeeded" % str(self.__uid))

    def _pre_fail(self) -> bool:
        return self._status == Agent.Status.MOVING \
            and not self._failure_reported \
            and not self._membership_query(StampedPoint(self.clk.to_sec(), self._position),
                                           self._plan_contr)

    def _eff_fail(self) -> None:
        rospy.logdebug("Failed to follow the plan contract. (%.2f, %s) not in %s."
                       " Real position: %s" %
                     (self.clk.to_sec(), str(self._position), str(self._plan_contr), str(self.__motion.position)))
        self.queries["fail"] += 1
        self._failure_reported = True

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
        if self._pre_fail():
            ret.append(("fail", {}))
        if self._status == Agent.Status.RELEASING:
            ret.append(("release", {"uid": self.uid, "releasable": self._free_contr}))
        if self._pre_acas():
            ret.append(("acas", {}))
        if self._pre_init():
            ret.append(("init", {}))
        #if self._pre_moving():
        #    ret.append(("moving", {}))

        return ret

    def _update_continuous_vars(self) -> None:
        super(Agent, self)._update_continuous_vars()
        self._position = self.__motion.position

    @staticmethod
    def __plan_to_contr(plan: List[StampedRect]) -> Contract:
        return Contract.from_stamped_rectangles(
                tuple((t, rect) for t, rect, _ in plan))
