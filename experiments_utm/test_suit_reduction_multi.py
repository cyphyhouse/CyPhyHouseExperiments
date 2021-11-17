# from gazebo_runner import run_simulation
import copy
from typing import List
import pickle
import numpy as np
from enum import Enum
import matplotlib.pyplot as plt

class RA(Enum):
    CLIMB = 4
    DESCEND = 5
    VERT_RATE_LIMIT = 6
    TURN_RIGHT = 2
    TURN_LEFT = 3
    NA = 0

class Test:
    def __init__(self, init=None, wp=None, T=None, idx=None, agent_list = None):
        self.init = init 
        self.T = T
        self.num_agents = len(init)
        self.waypoints = wp
        self.idx = idx
        self.agent_list = agent_list

class TestSuiteCache:
    def __init__(self, resolution, fine_resolution):
        self.cache_dict = {}
        self.resolution = resolution
        self.fine_resolution = fine_resolution
        self.content_similarity = 0.01

    def in_cache_2(self, state, desired_velocity, rotate_3d = True, fine = False):
        key = self.generate_key_2(state, desired_velocity, rotate_3d, fine)
                    
        if key in self.cache_dict:
            return True 
        else:
            return False

    def in_cache(self, state, desired_velocity, rotate_3d = True):
        # body_state = self.body(state)
        # key_state = body_state + state[12:18]
        # state_round = []
        # for i, val in enumerate(key_state):
        #     state_round.append(self.__round(val, self.resolution[i]))

        # key = tuple([tuple(state_round), ra])
        key = self.generate_key(state, desired_velocity, rotate_3d)
                    
        if key in self.cache_dict:
            return True 
        else:
            return False

    def get_next_state_2(self, state, desired_velocity, rotate_3d = True):
        roll = state[3]
        pitch = state[4] 
        yaw = state[5]

        key = self.generate_key_2(state, desired_velocity, rotate_3d)
                    
        new_state_list = copy.deepcopy(self.cache_dict[key])

        ret = []
        for new_state in new_state_list:
            vx = state[6]
            vy = state[7]
            vz = state[8]
            if rotate_3d:
                v_body = self.convert_pos_to_body([vx, vy, vz], roll, pitch, yaw)
            else:
                v_body = self.convert_pos_to_body([vx, vy, vz], 0, 0, yaw)

            vdesire_x = state[18]
            vdesire_y = state[19]
            vdesire_z = state[20]
            if rotate_3d:
                vdesire_body = self.convert_pos_to_body([vdesire_x, vdesire_y, vdesire_z], roll, pitch, yaw)
            else:
                vdesire_body = self.convert_pos_to_body([vdesire_x, vdesire_y, vdesire_z], 0, 0, yaw)

            new_state[3] += state[3]
            new_state[4] += state[4]
            new_state[5] += state[5]

            next_vx_body = new_state[6] + v_body[0]
            next_vy_body = new_state[7] + v_body[1]
            next_vz_body = new_state[8] + v_body[2]

            next_desire_vx_body = new_state[18] + v_body[0]
            next_desire_vy_body = new_state[19] + v_body[1]
            next_desire_vz_body = new_state[20] + v_body[2]

            x_offset = new_state[0] + v_body[0]
            y_offset = new_state[1] + v_body[1]
            z_offset = new_state[2] + v_body[2]

            if rotate_3d:
                offset = self.convert_body_to_pos([x_offset, y_offset, z_offset], roll, pitch, yaw)
                next_v = self.convert_body_to_pos([next_vx_body, next_vy_body, next_vz_body], roll, pitch, yaw)
                next_vdesire = self.convert_body_to_pos([next_desire_vx_body, next_desire_vy_body, next_desire_vz_body], roll, pitch, yaw)
            else:
                offset = self.convert_body_to_pos([x_offset, y_offset, z_offset], 0, 0, yaw)
                next_v = self.convert_body_to_pos([next_vx_body, next_vy_body, next_vz_body], 0, 0, yaw)
                next_vdesire = self.convert_body_to_pos([next_desire_vx_body, next_desire_vy_body, next_desire_vz_body], 0, 0, yaw)

            new_state[0] = state[0] + offset[0]
            new_state[1] = state[1] + offset[1]
            new_state[2] = state[2] + offset[2]

            new_state[6] = next_v[0]
            new_state[7] = next_v[1]
            new_state[8] = next_v[2]

            new_state[18] = next_vdesire[0] 
            new_state[19] = next_vdesire[1] 
            new_state[20] = next_vdesire[2] 

            ret.append(new_state)
        return ret 

    def get_next_state(self, state, desired_velocity, rotate_3d = True):
        # body_state = self.body(state)
        # key_state = body_state + state[12:18]
        # state_round = []
        # for i, val in enumerate(key_state):
        #     state_round.append(self.__round(val, self.resolution[i]))

        # key = tuple([tuple(state_round),ra])

        roll = state[3]
        pitch = state[4] 
        yaw = state[5]

        key = self.generate_key(state, desired_velocity, rotate_3d)
                    
        new_state_list = copy.deepcopy(self.cache_dict[key])

        ret = []
        for new_state in new_state_list:
            vx = state[6]
            vy = state[7]
            vz = state[8]
            if rotate_3d:
                v_body = self.convert_pos_to_body([vx, vy, vz], roll, pitch, yaw)
            else:
                v_body = self.convert_pos_to_body([vx, vy, vz], 0, 0, yaw)
                
            vdesire_x = state[18]
            vdesire_y = state[19]
            vdesire_z = state[20]
            # vdesire_body = self.convert_pos_to_body([vdesire_x, vdesire_y, vdesire_z], roll, pitch, yaw)

            new_state[3] += state[3]
            new_state[4] += state[4]
            new_state[5] += state[5]

            next_vx_body = new_state[6]
            next_vy_body = new_state[7]
            next_vz_body = new_state[8]

            next_desire_vx_body = new_state[18]
            next_desire_vy_body = new_state[19]
            next_desire_vz_body = new_state[20]

            x_offset_body = new_state[0]
            y_offset_body = new_state[1]
            z_offset_body = new_state[2]

            if rotate_3d:
                offset = self.convert_body_to_pos([x_offset_body, y_offset_body, z_offset_body], roll, pitch, yaw)
                next_v = self.convert_body_to_pos([next_vx_body, next_vy_body, next_vz_body], roll, pitch, yaw)
                next_vdesire = self.convert_body_to_pos([next_desire_vx_body, next_desire_vy_body, next_desire_vz_body], roll, pitch, yaw)
            else:
                offset = self.convert_body_to_pos([x_offset_body, y_offset_body, z_offset_body], 0, 0, yaw)
                next_v = self.convert_body_to_pos([next_vx_body, next_vy_body, next_vz_body], 0, 0, yaw)
                next_vdesire = self.convert_body_to_pos([next_desire_vx_body, next_desire_vy_body, next_desire_vz_body], 0, 0, yaw)

            new_state[0] = state[0] + offset[0]
            new_state[1] = state[1] + offset[1]
            new_state[2] = state[2] + offset[2]

            new_state[6] = next_v[0]
            new_state[7] = next_v[1]
            new_state[8] = next_v[2]

            new_state[18] = next_vdesire[0] 
            new_state[19] = next_vdesire[1] 
            new_state[20] = next_vdesire[2] 

            # return new_state
            ret.append(new_state)
        return ret 

    # def get_next_state(self, state, ra):
    #     # body_state = self.body(state)
    #     # key_state = body_state + state[12:18]
    #     # state_round = []
    #     # for i, val in enumerate(key_state):
    #     #     state_round.append(self.__round(val, self.resolution[i]))

    #     # key = tuple([tuple(state_round),ra])

    #     key = self.generate_key(state, ra)
                    
    #     new_state = copy.deepcopy(self.cache_dict[key])

    #     # x_offset_body = new_state[0]
    #     # y_offset_body = new_state[1]
    #     # z_offset_body = new_state[2]

    #     # vx_body = new_state[6]
    #     # vy_body = new_state[7]
    #     # vz_body = new_state[8]

    #     new_state[3] += state[3]
    #     new_state[4] += state[4]
    #     new_state[5] += state[5]

    #     roll = state[3]
    #     pitch = state[4] 
    #     yaw = state[5]

    #     offset = self.convert_body_to_pos(new_state[:3], roll, pitch, yaw)
    #     v = self.convert_body_to_pos(new_state[6:9], roll, pitch, yaw)
    #     vdesire = self.convert_body_to_pos(new_state[18:21], roll, pitch, yaw)

    #     new_state[0] = state[0] + offset[0]
    #     new_state[1] = state[1] + offset[1]
    #     new_state[2] = state[2] + offset[2]

    #     # new_vx = state[15] - new_state[15]
    #     # new_vy = state[16] - new_state[16]
    #     # new_vz = state[17] - new_state[17]

    #     # new_state[0] = state[0] + offset[0] + new_vx
    #     # new_state[1] = state[1] + offset[1] + new_vy
    #     # new_state[2] = state[2] + offset[2] + new_vz

    #     # new_state[3] += state[3]
    #     # new_state[4] += state[4]
    #     # new_state[5] += state[5]

    #     # new_state[6] = new_vx
    #     # new_state[7] = new_vy
    #     # new_state[8] = new_vz

    #     new_state[6] = v[0]
    #     new_state[7] = v[1]
    #     new_state[8] = v[2]

    #     new_state[18] = vdesire[0] 
    #     new_state[19] = vdesire[1] 
    #     new_state[20] = vdesire[2] 

    #     return new_state

    # def check_fine_resolution_2(state, desired_velocity, rotate_3d):
    #     key = self.generate_key_2(state, desired_velocity, rotate_3d)
    #     next_state = 

    def add_test_run_2(self, trajectories, rotate_3d = True):
        for i, time_step in enumerate(trajectories):
            for k, point in enumerate(time_step):
                desired_velocity = point[18:21]
                state = point[:-1]
                ra = point[-1]
                key = self.generate_key_2(state, desired_velocity, rotate_3d)
                if i+1 < len(trajectories):
                    # if self.check_fine_resolution_2(state, desired_velocity, rotate_3d):
                    #     continue
                    if not self.in_cache_2(state, desired_velocity, rotate_3d):
                        self.cache_dict[key] = []
                    next_state = copy.deepcopy(trajectories[i+1][k])
                    x_offset = (next_state[0] - state[0] - state[6]) 
                    y_offset = (next_state[1] - state[1] - state[7]) 
                    z_offset = (next_state[2] - state[2] - state[8]) 

                    roll = state[3]
                    pitch = state[4]
                    yaw = state[5]

                    if rotate_3d:
                        offset_body = self.convert_pos_to_body([x_offset, y_offset, z_offset], roll, pitch, yaw)    
                    else:
                        offset_body = self.convert_pos_to_body([x_offset, y_offset, z_offset], 0, 0, yaw)

                    vx = state[6]
                    vy = state[7]
                    vz = state[8]
                    if rotate_3d:
                        v_body = self.convert_pos_to_body([vx, vy, vz], roll, pitch, yaw)
                    else:
                        v_body = self.convert_pos_to_body([vx, vy, vz], 0, 0, yaw)    

                    next_vx = next_state[6]
                    next_vy = next_state[7]
                    next_vz = next_state[8]
                    if rotate_3d:
                        nextv_body = self.convert_pos_to_body([next_vx, next_vy, next_vz], roll, pitch, yaw)
                    else:
                        nextv_body = self.convert_pos_to_body([next_vx, next_vy, next_vz], 0, 0, yaw)
                        
                    vdesirex = next_state[18]
                    vdesirey = next_state[19]
                    vdesirez = next_state[20]
                    if rotate_3d:
                        vdesire_body = self.convert_pos_to_body([vdesirex, vdesirey, vdesirez], roll, pitch, yaw)
                    else:
                        vdesire_body = self.convert_pos_to_body([vdesirex, vdesirey, vdesirez], 0, 0, yaw)

                    next_state[0] = offset_body[0]
                    next_state[1] = offset_body[1]
                    next_state[2] = offset_body[2]

                    next_state[3] -= roll 
                    next_state[4] -= pitch 
                    next_state[5] -= yaw

                    next_state[6] = nextv_body[0] - v_body[0]
                    next_state[7] = nextv_body[1] - v_body[1]
                    next_state[8] = nextv_body[2] - v_body[2]

                    next_state[18] = vdesire_body[0] - v_body[0]
                    next_state[19] = vdesire_body[1] - v_body[1]
                    next_state[20] = vdesire_body[2] - v_body[2]

                    # self.cache_dict[key] = next_state
                    if not self.check_content_similarity(key, next_state):
                        self.cache_dict[key].append(next_state)

    def check_content_similarity(self, key, new_state):
        similar = False
        new_state_list = self.cache_dict[key]
        for tmp in new_state_list:
            state_diff = np.linalg.norm(np.array(new_state[:21]) - np.array(tmp[:21]))
            if state_diff < self.content_similarity:
                return True
        return False

    def add_test_run(self, trajectories, rotate_3d = True):
        for i, time_step in enumerate(trajectories):
            for k, point in enumerate(time_step):
                desired_velocity = point[18:21]
                state = point[:-1]
                ra = point[-1]
                key = self.generate_key(state, desired_velocity, rotate_3d)
                if not self.in_cache(state, desired_velocity, rotate_3d):
                    self.cache_dict[key] = []
                if i+1 < len(trajectories):
                    next_state = copy.deepcopy(trajectories[i+1][k])
                    x_offset = (next_state[0] - state[0]) 
                    y_offset = (next_state[1] - state[1]) 
                    z_offset = (next_state[2] - state[2]) 

                    roll = state[3]
                    pitch = state[4]
                    yaw = state[5]

                    if rotate_3d:
                        offset_body = self.convert_pos_to_body([x_offset, y_offset, z_offset], roll, pitch, yaw)     
                    else:
                        offset_body = self.convert_pos_to_body([x_offset, y_offset, z_offset], 0, 0, yaw)     
                        
                    vx = state[6]
                    vy = state[7]
                    vz = state[8]
                    if rotate_3d:
                        v_body = self.convert_pos_to_body([vx, vy, vz], roll, pitch, yaw)
                    else:
                        v_body = self.convert_pos_to_body([vx, vy, vz], 0, 0, yaw)
                        
                    next_vx = next_state[6]
                    next_vy = next_state[7]
                    next_vz = next_state[8]
                    if rotate_3d:
                        nextv_body = self.convert_pos_to_body([next_vx, next_vy, next_vz], roll, pitch, yaw)
                    else:
                        nextv_body = self.convert_pos_to_body([next_vx, next_vy, next_vz], 0, 0, yaw)
                        
                    vdesirex = next_state[18]
                    vdesirey = next_state[19]
                    vdesirez = next_state[20]

                    if rotate_3d:
                        vdesire_body = self.convert_pos_to_body([vdesirex, vdesirey, vdesirez], roll, pitch, yaw)
                    else:
                        vdesire_body = self.convert_pos_to_body([vdesirex, vdesirey, vdesirez], 0, 0, yaw)

                    next_state[0] = offset_body[0]
                    next_state[1] = offset_body[1]
                    next_state[2] = offset_body[2]

                    next_state[3] -= roll 
                    next_state[4] -= pitch 
                    next_state[5] -= yaw

                    next_state[6] = nextv_body[0]
                    next_state[7] = nextv_body[1]
                    next_state[8] = nextv_body[2]

                    next_state[18] = vdesire_body[0]
                    next_state[19] = vdesire_body[1]
                    next_state[20] = vdesire_body[2]

                    self.cache_dict[key] = next_state

    # def add_test_run(self, trajectories):
    #     for i, time_step in enumerate(trajectories):
    #         for k, point in enumerate(time_step):
    #             state = point[:-1]
    #             ra = point[-1]
    #             if not self.in_cache(state, ra):
    #                 key = self.generate_key(state, ra)
    #                 if i+1 < len(trajectories):
    #                     next_state = copy.deepcopy(trajectories[i+1][k])
    #                     x_offset = (next_state[0] - state[0]) 
    #                     y_offset = (next_state[1] - state[1]) 
    #                     z_offset = (next_state[2] - state[2]) 

    #                     roll = state[3]
    #                     pitch = state[4]
    #                     yaw = state[5]

    #                     offset_body = self.convert_pos_to_body([x_offset, y_offset, z_offset], roll, pitch, yaw)     

    #                     vx = state[6]
    #                     vy = state[7]
    #                     vz = state[8]
    #                     v_body = self.convert_pos_to_body([vx, vy, vz], roll, pitch, yaw)

    #                     next_vx = next_state[6]
    #                     next_vy = next_state[7]
    #                     next_vz = next_state[8]
    #                     nextv_body = self.convert_pos_to_body([next_vx, next_vy, next_vz], roll, pitch, yaw)

    #                     vdesirex = next_state[18]
    #                     vdesirey = next_state[19]
    #                     vdesirez = next_state[20]

    #                     vdesire_body = self.convert_pos_to_body([vdesirex, vdesirey, vdesirez], roll, pitch, yaw)

    #                     next_state[0] = offset_body[0]
    #                     next_state[1] = offset_body[1]
    #                     next_state[2] = offset_body[2]

    #                     next_state[3] -= roll 
    #                     next_state[4] -= pitch 
    #                     next_state[5] -= yaw

    #                     next_state[6] = nextv_body[0] - v_body[0]
    #                     next_state[7] = nextv_body[1] - v_body[1]
    #                     next_state[8] = nextv_body[2] - v_body[2]

    #                     next_state[18] = vdesire_body[0] - v_body[0]
    #                     next_state[19] = vdesire_body[1] - v_body[1]
    #                     next_state[20] = vdesire_body[2] - v_body[2]

    #                     self.cache_dict[key] = next_state

    def __round(self, val, base):
        res = base * round(val/base)
        return res

    def convert_body_to_pos(self, pos_body, roll, pitch, yaw):
        pos_body = np.array(pos_body)
        Rroll = [[1,0,0],[0, np.cos(roll), np.sin(roll)],[0, -np.sin(roll), np.cos(roll)]]
        Rpitch = [[np.cos(pitch),0 , -np.sin(pitch)],[0,1,0],[np.sin(pitch),0 ,np.cos(pitch)]]
        Ryaw = [[np.cos(yaw), np.sin(yaw), 0],[-np.sin(yaw), np.cos(yaw), 0],[0,0,1]]
        
        pos = np.linalg.inv(np.array(Rroll)@np.array(Rpitch)@np.array(Ryaw))@np.array(pos_body)
        pos = list(pos)
        return pos

    def convert_pos_to_body(self, pos, roll, pitch, yaw):
        pos = np.array(pos)
        Rroll = [[1,0,0],[0, np.cos(roll), np.sin(roll)],[0, -np.sin(roll), np.cos(roll)]]
        Rpitch = [[np.cos(pitch),0 , -np.sin(pitch)],[0,1,0],[np.sin(pitch),0 ,np.cos(pitch)]]
        Ryaw = [[np.cos(yaw), np.sin(yaw), 0],[-np.sin(yaw), np.cos(yaw), 0],[0,0,1]]
        
        pos_body = np.array(pos)
        pos_body = np.array(Ryaw)@np.array(pos)
        pos_body = np.array(Rroll)@np.array(Rpitch)@np.array(Ryaw)@np.array(pos)
        
        pos_body = list(pos_body)
        return pos_body

    def generate_key_2(self, state, desired_velocity, rotate_3d = True, fine = False):
        roll = state[3]
        pitch = state[4]
        yaw = state[5]

        vx = state[6]
        vy = state[7]
        vz = state[8]
        if rotate_3d:
            vbody = self.convert_pos_to_body([vx, vy, vz], roll, pitch, yaw)
        else:
            vbody = self.convert_pos_to_body([vx, vy, vz], 0, 0, yaw)
            
        vdesired_x = desired_velocity[0]
        vdesired_y = desired_velocity[1]
        vdesired_z = desired_velocity[2]
        if rotate_3d:
            vdesired_body = self.convert_pos_to_body([vdesired_x, vdesired_y, vdesired_z], roll, pitch, yaw)
        else:
            vdesired_body = self.convert_pos_to_body([vdesired_x, vdesired_y, vdesired_z], 0, 0, yaw)

        vdesired_body_offset = [vdesired_body[0] - vbody[0], vdesired_body[1] - vbody[1], vdesired_body[2] - vbody[2]]
        # key_state = [roll, pitch] + vdesired_body_offset
        key_state = vdesired_body_offset
        state_round = []
        for j, val in enumerate(key_state):
            state_round.append(self.__round(val, self.resolution[j]))
        key = tuple(state_round)
        
        return key

    def generate_key(self, state, desired_velocity, rotate_3d = True):
        roll = state[3]
        pitch = state[4]
        yaw = state[5]

        vx = state[6]
        vy = state[7]
        vz = state[8]
        
        if rotate_3d:
            vbody = self.convert_pos_to_body([vx, vy, vz], roll, pitch, yaw)
        else:
            vbody = self.convert_pos_to_body([vx, vy, vz], 0, 0, yaw)

        res = vbody

        vdesired_x = desired_velocity[0]
        vdesired_y = desired_velocity[1]
        vdesired_z = desired_velocity[2]

        if rotate_3d:
            vdesired_body = self.convert_pos_to_body([vdesired_x, vdesired_y, vdesired_z], roll, pitch, yaw)
        else:
            vdesired_body = self.convert_pos_to_body([vdesired_x, vdesired_y, vdesired_z], 0, 0, yaw)
        
        if rotate_3d:
            key_state = res + vdesired_body
        else:
            key_state = [roll, pitch] + res + vdesired_body

        state_round = []
        for j, val in enumerate(key_state):
            state_round.append(self.__round(val, self.resolution[j]))
        key = tuple(state_round)
        
        return key

    # def generate_key(self, state, ra):
    #     roll = state[3]
    #     pitch = state[4]
    #     yaw = state[5]

    #     vx = state[6]
    #     vy = state[7]
    #     vz = state[8]

    #     vbody = self.convert_pos_to_body([vx, vy, vz], roll, pitch, yaw)

    #     res = vbody

    #     vdesired_x = state[18] - vx 
    #     vdesired_y = state[19] - vy 
    #     vdesired_z = state[20] - vz  

    #     vdesired_body = self.convert_pos_to_body([vdesired_x, vdesired_y, vdesired_z], roll, pitch, yaw)
        
    #     key_state = res + vdesired_body

    #     state_round = []
    #     for j, val in enumerate(key_state):
    #         state_round.append(self.__round(val, self.resolution[j]))
    #     key = (tuple(state_round), ra)
        
    #     return key
        
def load_parse_trajectory(test: Test):
    idx = test.idx
    parsed_trajectory_list = []
    for i in range(test.num_agents):
        fn0 = f'./trajectories/all_runs_10/drone{i}_{idx}'       
    
        with open(fn0, 'rb') as f:
            trajectory_0 = pickle.load(f)

        parsed_trajectory = []
        t_ref = trajectory_0[0][0]
        t0 = trajectory_0[0][0]
        for j in range(len(trajectory_0)):
            time = trajectory_0[j][0]
            x = trajectory_0[j][1]
            y = trajectory_0[j][2]
            z = trajectory_0[j][3]
            roll = trajectory_0[j][6]
            pitch = trajectory_0[j][7]
            yaw = trajectory_0[j][8]

            vx = trajectory_0[j][9]
            vy = trajectory_0[j][10]
            vz = trajectory_0[j][11]

            vangularx = trajectory_0[j][12]            
            vangulary = trajectory_0[j][13]            
            vangularz = trajectory_0[j][14]

            altitude_control_roll = trajectory_0[j][15]
            altitude_control_pitch = trajectory_0[j][16]
            altitude_control_yawrate = trajectory_0[j][17]

            velocity_control_x = trajectory_0[j][18]
            velocity_control_y = trajectory_0[j][19]
            velocity_control_z = trajectory_0[j][20]

            desired_velocity_x = trajectory_0[j][21]
            desired_velocity_y = trajectory_0[j][22]
            desired_velocity_z = trajectory_0[j][23]
            
            if np.isnan(x+y+z\
                +roll+pitch+yaw\
                +vx+vy+vz\
                +vangularx+vangulary+vangularz\
                +altitude_control_roll+altitude_control_pitch+altitude_control_yawrate\
                +velocity_control_x+velocity_control_y+velocity_control_z\
                +desired_velocity_x+desired_velocity_y+desired_velocity_z
            ):
                return None

            ra_valid = trajectory_0[j][5]
    
            ra_string = trajectory_0[j][4]
            ra_string_list = ra_string.split(',')
            vra = 0
            hra = 0
            hra_val = 0
            
            if ra_valid:
                if 'Climb' in ra_string_list[1]:
                    vra = RA.CLIMB
                elif 'Descend' in ra_string_list[1]:
                    vra = RA.DESCEND      

                if 'Turn Left' in ra_string[2]:
                    hra = RA.TURN_LEFT
                    hra_val = float(ra_string[2].split(' ')[2])
                elif 'Turn Right' in ra_string[2]:
                    hra = RA.TURN_RIGHT
                    hra_val = float(ra_string[2].split(' ')[2])

            state = [
                x, y, z,
                roll, pitch, yaw,
                vx, vy, vz,
                vangularx, vangulary, vangularz,
                altitude_control_roll, altitude_control_pitch, altitude_control_yawrate,
                velocity_control_x, velocity_control_y, velocity_control_z, 
                desired_velocity_x, desired_velocity_y, desired_velocity_z, time,
                (vra, hra, hra_val)
            ]
            if time > t_ref + 1:
                parsed_trajectory.append(state)
                t_ref = time

        parsed_trajectory_list.append(parsed_trajectory)

    # Align trajectories
    # Find the minimum trajectory length
    trajectory_length = float('inf')
    for trajectory in parsed_trajectory_list:
        trajectory_length = min(len(trajectory), trajectory_length)

    align_parsed_trajectory = []
    for i in range(trajectory_length):
        align_parsed_trajectory.append([parsed_trajectory_list[0][i], parsed_trajectory_list[1][i]])
    return align_parsed_trajectory

def generate_test_suite(
    theta_list = [-np.pi*3/4, -np.pi/2, -np.pi*3/8, -np.pi/4, 0, np.pi/4, np.pi/3, np.pi*3/4, np.pi], 
    vint_list = [60, 150, 300, 450, 600, 750, 900, 1050, 1145], 
    rho_list = [10000, 43736, 87472, 120000]
) -> List[Test]:
    # theta_list = [-np.pi*3/4, -np.pi/2, -np.pi*3/8, -np.pi/4, 0, np.pi/4, np.pi/3, np.pi*3/4, np.pi]
    # vint_list = [60, 150, 300, 450, 600, 750, 900, 1050, 1145]
    # rho_list = [10000, 43736, 87472, 120000]

    # theta_list = [-np.pi*3/4, -np.pi/2, -np.pi*3/8, -np.pi/4, 0, np.pi/4, np.pi/3, np.pi*3/4, np.pi]
    # theta_list = [-np.pi*3/4, -np.pi/2, -np.pi*3/8, -np.pi/4, 0, np.pi/4, ]
    
    # vint_list = [900]
    # vint_list = [600, 750, 900, 1050]

    # rho_list = [43736]
    # rho_list = [10000, 43736, 87472, ]

    scale = 0.3048 / 40
    T = 100
    T_wp = 120
    n = 1

    test_suite = []
    for i, theta in enumerate(theta_list):
        for j, vint in enumerate(vint_list):
            for k, rho in enumerate(rho_list):
                print(f"Run simulation for configuration {theta}, {vint}, {rho}")
                init_pos_0 = [0, 0, 40]
                init_yaw_0 = np.pi / 2

                if theta>0 and theta<np.pi:
                    phi = -np.arcsin(rho*np.sin(abs(theta))/(T*vint))
                    psi = np.pi-abs(theta)-abs(phi)
                    vown = vint*np.sin(psi)/np.sin(abs(theta))
                elif theta>-np.pi and theta<0:
                    phi = np.arcsin(rho*np.sin(abs(theta))/(T*vint))
                    psi = np.pi-abs(theta)-abs(phi)
                    vown = vint*np.sin(psi)/np.sin(abs(theta))
                elif theta==abs(np.pi):
                    phi = 0
                    psi = 0
                    vown = (T*vint-rho)/T
                else:
                    phi = 0
                    psi = 0
                    vown = (T*vint+rho)/T 
    
                if np.isnan(phi) or np.isnan(psi) or np.isnan(vown):
                    print(f'Configuration {theta}, {vint}, {rho} is not available')
                    continue
                xint = init_pos_0[0]+rho*np.cos(theta+np.pi/2)
                yint = init_pos_0[1]+rho*np.sin(theta+np.pi/2)

                init_lin_vel_0 = [0, vown * scale, 0]
                init_pos_1 = [xint * scale, yint * scale, 40]
                init_yaw_1 = np.pi / 2 + phi
                init_lin_vel_1 = [vint * np.cos(init_yaw_1) * scale, vint * np.sin(init_yaw_1) * scale, 0]
                init_0 = init_pos_0 + [0,0,init_yaw_0] + init_lin_vel_0 + [0,0,0]
                init_1 = init_pos_1 + [0,0,init_yaw_1] + init_lin_vel_1 + [0,0,0]
                init = [init_0, init_1]

                waypoints_dict = {}
                waypoints_dict['drone0'] =[]
                waypoints_dict['drone1'] =[]
                for l in range(n):
                    waypoints_dict['drone0'].append((init_pos_0[0] + init_lin_vel_0[0] * (l+1) * T_wp, init_pos_0[1] + init_lin_vel_0[1] * (l+1) * T_wp, 40))
                    waypoints_dict['drone1'].append((init_pos_1[0] + init_lin_vel_1[0] * (l+1) * T_wp, init_pos_1[1] + init_lin_vel_1[1] * (l+1) * T_wp, 40))
                waypoints_dict['drone1'].append((init_pos_1[0] + init_lin_vel_1[0] * (n+1) * T_wp, init_pos_1[1] + init_lin_vel_1[1] * (n+1) * T_wp, 40))
                waypoints_list = [waypoints_dict['drone0'], waypoints_dict['drone1']]
                test_case = Test(init=init, wp=waypoints_list, T=T_wp, idx=f"{0}_{1}_{0}", agent_list = ['drone0', 'drone1'])

                test_suite.append(test_case)
    return test_suite

# def check_cache_hit(TS: List[Test]):
#     cache = TestSuiteCache()
#     execs = []
#     cache_hit = 0
#     total_points = 0
#     processed_trajectories = 0
#     for k, test in enumerate(TS):
#         cache_size = len(cache.cache_dict)
#         not_in_cache = False 
#         parsed_trajectory = load_parse_trajectory(test)
#         if parsed_trajectory is None:
#             continue
#         processed_trajectories += 1
#         total_points += len(parsed_trajectory) * 2
#         alpha = parsed_trajectory[0]
#         print(f"processed trajectories {processed_trajectories}, test {test.idx}, total points {total_points}, cache size {cache_size}, hit {cache_hit}")
#         num_agent = test.num_agents
#         for i in range(len(parsed_trajectory)-1):
#             next_alpha = []
#             for j, agent_state in enumerate(alpha):
#                 ra = agent_state[-1]
#                 if cache.in_cache(agent_state, ra):
#                     next_state = parsed_trajectory[i+1][j]
#                     cache_hit += 1
#                 else:
#                     not_in_cache = True
#                     break 
#                 next_alpha.append(next_state)
            
#             if not_in_cache:
#                 break 
#             alpha = next_alpha

#         if not_in_cache:
#             trajectory = parsed_trajectory[i:]
#             cache.add_test_run(trajectory)
#     return cache_hit, total_points, cache

def plot_trajectory(cache_trajectory, simulate_trajectory, idx = '', test_case = ''):
    color = ['b.','g.']
    x_cache_0 = []
    y_cache_0 = []
    x_cache_1 = []
    y_cache_1 = []
    z_cache_0 = []
    z_cache_1 = []
    roll_cache_0 = []
    pitch_cache_0 = []
    yaw_cache_0 = []
    roll_cache_1 = []
    pitch_cache_1 = []
    yaw_cache_1 = []
    # vx_cache_0 = []
    # vy_cache_0 = []
    # vz_cache_0 = []
    # vx_desire_cache_0 = []
    
    not_in_cache = []
    for time_step in cache_trajectory:
        # for i, point in enumerate(time_step):
        tmp_x = []
        tmp_y = []
        tmp_z = []
        for i, test_point in enumerate(time_step[0]):
            tmp_x.append(test_point[0])
            tmp_y.append(test_point[1])
            tmp_z.append(test_point[2])
        x_cache_0.append(tmp_x)
        y_cache_0.append(tmp_y)
        z_cache_0.append(tmp_z)
        # x_cache_0.append(time_step[0][0])
        # y_cache_0.append(time_step[0][1])
        # z_cache_0.append(time_step[0][2])
        tmp_x = []
        tmp_y = []
        tmp_z = []
        for i, test_point in enumerate(time_step[1]):
            tmp_x.append(test_point[0])
            tmp_y.append(test_point[1])
            tmp_z.append(test_point[2])
        x_cache_1.append(tmp_x)
        y_cache_1.append(tmp_y)
        z_cache_1.append(tmp_z)
        # x_cache_1.append(time_step[1][0])
        # y_cache_1.append(time_step[1][1])
        # z_cache_1.append(time_step[1][2])
        
        # roll_cache_0.append(time_step[0][3])
        # pitch_cache_0.append(time_step[1][3])
        # yaw_cache_0.append(time_step[0][4])
        # roll_cache_1.append(time_step[1][4])
        # pitch_cache_1.append(time_step[0][5])
        # yaw_cache_1.append(time_step[1][5])

    
        not_in_cache.append(time_step[2])

    x_simulate_0 = []
    y_simulate_0 = []
    x_simulate_1 = []
    y_simulate_1 = []
    z_simulate_0 = []
    z_simulate_1 = []
    roll_simulate_0 = []
    pitch_simulate_0 = []
    yaw_simulate_0 = []
    roll_simulate_1 = []
    pitch_simulate_1 = []
    yaw_simulate_1 = []
    for time_step in simulate_trajectory:
        # for i, point in enumerate(time_step):
        x_simulate_0.append(time_step[0][0])
        y_simulate_0.append(time_step[0][1])
        x_simulate_1.append(time_step[1][0])
        y_simulate_1.append(time_step[1][1])
        z_simulate_0.append(time_step[0][2])
        z_simulate_1.append(time_step[1][2])
        roll_simulate_0.append(time_step[0][3])
        pitch_simulate_0.append(time_step[1][3])
        yaw_simulate_0.append(time_step[0][4])
        roll_simulate_1.append(time_step[1][4])
        pitch_simulate_1.append(time_step[0][5])
        yaw_simulate_1.append(time_step[1][5])

    plt.figure(0)
    # plt.plot(x_cache_0, y_cache_0, 'g')
    # plt.plot(x_cache_1, y_cache_1, 'g')
    cache_hit_0 = []
    cache_hit_1 = []
    cache_miss_0 = []
    cache_miss_1 = []
    for i in range(len(not_in_cache)):
        if not_in_cache[i] == 1:
            plt.plot(x_simulate_0[i], y_simulate_0[i], 'r.')
            plt.plot(x_simulate_1[i], y_simulate_1[i], 'r.')
            cache_miss_0.append(z_simulate_0[i])
            cache_miss_1.append(z_simulate_1[i])
            cache_hit_0.append(float('nan'))
            cache_hit_1.append(float('nan'))
        else:
            for j in range(len(x_cache_0[i])):
                plt.plot(x_cache_0[i][j], y_cache_0[i][j], 'g.')
            for j in range(len(x_cache_1[i])):
                plt.plot(x_cache_1[i][j], y_cache_1[i][j], 'g.')
            cache_hit_0.append(z_cache_0[i])
            cache_hit_1.append(z_cache_1[i])
            cache_miss_0.append(float('nan'))
            cache_miss_1.append(float('nan'))

    plt.plot(x_simulate_0, y_simulate_0, 'b')
    plt.plot(x_simulate_1, y_simulate_1, 'b')
    fn = f'./figures/{test_case}_{idx}.png'
    plt.savefig(fn)
    # plt.clf()
    plt.show()

    # plt.figure(1)
    # plt.plot(z_cache_0, 'g')
    # plt.plot(z_simulate_0, 'b')
    # plt.plot(z_cache_1, 'g')
    # plt.plot(z_simulate_1, 'b')

    # plt.plot(cache_hit_0, 'g.')
    # plt.plot(cache_hit_1, 'g.')
    # plt.plot(cache_miss_0, 'r.')
    # plt.plot(cache_miss_1, 'r.')
    
    # plt.figure(2)
    # plt.plot(roll_cache_0, 'g')
    # plt.plot(roll_simulate_0, 'b')
    
    # plt.figure(3)
    # plt.plot(pitch_cache_0, 'g')
    # plt.plot(pitch_simulate_0, 'b')
    
    # plt.figure(4)
    # plt.plot(yaw_cache_0, 'g')
    # plt.plot(yaw_simulate_0, 'b')
    
    # plt.figure(5)
    # plt.plot(z_cache_1, 'g')
    # plt.plot(z_simulate_1, 'b')
    
    # plt.figure(6)
    # plt.plot(roll_cache_1, 'g')
    # plt.plot(roll_simulate_1, 'b')
    
    # plt.figure(7)
    # plt.plot(pitch_cache_1, 'g')
    # plt.plot(pitch_simulate_1, 'b')
    
    # plt.figure(8)
    # plt.plot(yaw_cache_1, 'g')
    # plt.plot(yaw_simulate_1, 'b')
    
    # # plt.show()
    # fn = f'./plots/{test_case}_z_{idx}.png'
    # plt.savefig(fn)
    
    # # plt.show()
    # plt.clf()

p_error_last = 0
i_error = 0
def pid_computeCommand(error, dt, gains):
    global p_error_last
    global i_error

    p_gain = gains[0]
    i_gain = gains[1]
    d_gain = gains[2]

    if dt == 0 or np.isnan(error) or np.isinf(error):
        return 0
    
    error_dot = 0
    if dt>0:
        error_dot = (error-p_error_last)/dt
        p_error_last = error
    
    p_error = error 
    d_error = error_dot

    if dt == 0 or np.isnan(error) or np.isinf(error) or np.isnan(error_dot) or np.isinf(error_dot):
      return 0.0

    p_term = p_gain * p_error
    i_error += dt * p_error
    i_term = i_gain * i_error
    # i_term = max( gains.i_min_, std::min( i_term, gains.i_max_) );
    d_term = d_gain * d_error 

    cmd = p_term + i_term + d_term 

    return cmd

def agent_planner(state, ra, wp):
    period = 0.012

    x = state[0]
    y = state[1]
    z = state[2]
    vx = state[6]
    vy = state[7]
    vz = state[8]

    tgt = (wp[0], wp[1], wp[2])    
    if RA(ra[0]) == RA.NA and RA(ra[1]) == RA.NA:
        tgt = (wp[0], wp[1], wp[2])
    else:
        tgt_vector_v = (0,0,0)
        if RA(ra[0]) == RA.CLIMB:
            tgt_vector_v = (vx, vy, 3)
        elif RA(ra[0]) == RA.DESCEND:
            tgt_vector_v = (vx, vy, -3)

        tgt_vector_h = (0,0,0)
        if RA(ra[1]) == RA.TURN_LEFT:
            target_angle = ra[2]
            curr_speed = np.sqrt(vx**2+vy**2)+3
            target_angle_rad = np.radians(target_angle)
            x_off = np.sin(target_angle_rad) * curr_speed
            y_off = np.cos(target_angle_rad) * curr_speed
            tgt_vector_h = (x_off, y_off, 0)
        elif RA(ra[1]) == RA.TURN_RIGHT:
            target_angle = ra[2]
            curr_speed = np.sqrt(vx**2+vy**2)+3
            target_angle_rad = np.radians(target_angle)
            x_off = np.sin(target_angle_rad) * curr_speed
            y_off = np.cos(target_angle_rad) * curr_speed
            tgt_vector_h = (x_off, y_off, 0)

        tgt = (
            x + tgt_vector_v[0] + tgt_vector_h[0], 
            y + tgt_vector_v[1] + tgt_vector_h[1], 
            z + tgt_vector_v[2] + tgt_vector_h[2]
        )

    output_linear_x = pid_computeCommand(tgt[0] - x, period, [2,0,0])
    output_linear_y = pid_computeCommand(tgt[1] - y, period, [2,0,0])
    output_linear_z = pid_computeCommand(tgt[2] - z, period, [2,0,0])

    yaw_error = 0
    output_angular_z = pid_computeCommand(yaw_error, period, [2,0,0])

    # linear_xy = np.sqrt(output_linear_x*output_linear_x + output_linear_y*output_linear_y)
    # limit_linear_xy  = max(twist_limit_.linear.x, twist_limit_.linear.y)
    # if limit_linear_xy > 0.0 and linear_xy > limit_linear_xy:
    #     output_linear_x *= limit_linear_xy / linear_xy
    #     output_linear_y *= limit_linear_xy / linear_xy
    # if twist_limit_.linear.z > 0.0 and fabs(output.linear.z) > twist_limit_.linear.z: 
    #     output.linear.z *= twist_limit_.linear.z / fabs(output.linear.z);
    
    # double angular_xy = sqrt(output.angular.x*output.angular.x + output.angular.y*output.angular.y);
    # double limit_angular_xy  = std::max(twist_limit_.angular.x, twist_limit_.angular.y);
    
    # if limit_angular_xy > 0.0 and angular_xy > limit_angular_xy:
    #     output.angular.x *= limit_angular_xy / angular_xy;
    #     output.angular.y *= limit_angular_xy / angular_xy;
    
    # if twist_limit_.angular.z > 0.0 and fabs(output.angular.z) > twist_limit_.angular.z:
    #     output.angular.z *= twist_limit_.angular.z / fabs(output.angular.z);


    output_x = output_linear_x
    output_y = output_linear_y
    output_z = output_linear_z

    absolute_maximum_xy = 10
    absolute_value_xy = np.sqrt(output_x * output_x + output_y * output_y)
    if absolute_value_xy > absolute_maximum_xy:
        output_x *= absolute_maximum_xy / absolute_value_xy
        output_y *= absolute_maximum_xy / absolute_value_xy
        output_z *= absolute_maximum_xy / absolute_value_xy

    absolute_maximum = float('inf')
    absolute_value = np.sqrt(output_x * output_x + output_y * output_y + output_z * output_z)
    if absolute_value > absolute_maximum:
        output_x *= absolute_maximum / absolute_value
        output_y *= absolute_maximum / absolute_value
        output_z *= absolute_maximum / absolute_value
    
    return [output_x, output_y, output_z]

def test_suite_reduction(TS: List[Test], rotate_3d = True, resolution = [0.1,0.1,0.1,0.1,0.1], trajectory_dict = {}, test_case = 0):
    cache = TestSuiteCache(resolution = resolution)
    execs = []
    cache_hit = 0
    total_points = 0
    processed_trajectories = 0
    for k, test in enumerate(TS):
        cache_size = len(cache.cache_dict)
        not_in_cache = False 
        if test.idx in trajectory_dict:
            parsed_trajectory = trajectory_dict[test.idx]
        else:
            parsed_trajectory = load_parse_trajectory(test)
    
        if parsed_trajectory is None:
            continue
        processed_trajectories += 1
        total_points += len(parsed_trajectory) * 2
        
        trajectory_length = len(parsed_trajectory*2)
        trajectory_hit = 0
        current_trajectory = []
        
        alpha = parsed_trajectory[0]
        # print(f"processed trajectories {processed_trajectories}, test {test.idx}, total points {total_points}, cache size {cache_size}, hit {cache_hit}")
        num_agent = test.num_agents
        # if processed_trajectories == 7:
        #     print("stop here")
        for i in range(len(parsed_trajectory)-1):
            next_alpha = []
            not_in_cache = False
            # if i == 12:
            #     print('stop here')
            # print(parsed_trajectory[i+1][0])
            for j, agent_state in enumerate(alpha):
                ra = parsed_trajectory[i][j][-1]
                desired_velocity = agent_planner(agent_state, ra, test.waypoints[j][0])
                if cache.in_cache(agent_state, desired_velocity, rotate_3d = rotate_3d):
                    next_state = cache.get_next_state(agent_state, desired_velocity, rotate_3d = rotate_3d)
                    # cache_hit += 1
                    trajectory_hit += 1
                    next_alpha.append(next_state)
                else:
                    not_in_cache = True
                    # break
                    # if k >= 5:
                    #     print('Stop Here!')
                    next_alpha = parsed_trajectory[i+1]
                    cache.add_test_run([parsed_trajectory[i], parsed_trajectory[i+1]], rotate_3d = rotate_3d)
                    
                if not_in_cache:
                    break 

            if not not_in_cache:
                cache_hit += 2
            # if not_in_cache:
            #     break 
            # print(i, cache.get_next_state(alpha[1], parsed_trajectory[i][1][-1])[:3])
            # print(i, parsed_trajectory[i+1][1][:3])
            current_trajectory.append(alpha+[int(not_in_cache)])
            
            alpha = next_alpha

        # simulated_trajectory = []
        # if not_in_cache:
        #     simulated_trajectory = parsed_trajectory[i:]
        #     cache.add_test_run(simulated_trajectory)
        # current_trajectory += simulated_trajectory

        # if trajectory_hit / trajectory_length > 0.5:
        plot_trajectory(current_trajectory, parsed_trajectory, test.idx, test_case)
        
    return cache_hit, total_points, cache

def test_suite_reduction_2(TS: List[Test], rotate_3d = True, resolution = [0.1,0.1,0.1,0.1,0.1], trajectory_dict = {}, test_case = 0):
    cache = TestSuiteCache(resolution = resolution)
    execs = []
    cache_hit = 0
    total_points = 0
    processed_trajectories = 0
    for k, test in enumerate(TS):
        cache_size = len(cache.cache_dict)
        not_in_cache = False 
        if test.idx in trajectory_dict:
            parsed_trajectory = trajectory_dict[test.idx]
        else:
            parsed_trajectory = load_parse_trajectory(test)
    
        if parsed_trajectory is None:
            continue
        processed_trajectories += 1
        total_points += len(parsed_trajectory) * 2
        
        trajectory_length = len(parsed_trajectory*2)
        trajectory_hit = 0
        current_trajectory = []
        
        alpha = parsed_trajectory[0]
        # print(f"processed trajectories {processed_trajectories}, test {test.idx}, total points {total_points}, cache size {cache_size}, hit {cache_hit}")
        num_agent = test.num_agents
        for i in range(len(parsed_trajectory)-1):
            next_alpha = []
            not_in_cache = False
            # if processed_trajectories == 3:
            #     print('stop here')
            for j, agent_state in enumerate(alpha):
                ra = agent_state[-1]
                desired_velocity = agent_planner(agent_state, ra, test.waypoints[j][0])
                if cache.in_cache(agent_state, desired_velocity, rotate_3d = rotate_3d):
                    next_state = cache.get_next_state(agent_state, desired_velocity, rotate_3d = rotate_3d)
                    # cache_hit += 1
                    trajectory_hit += 1
                    next_alpha.append(next_state)
                else:
                    not_in_cache = True
                    break
                    # if k >= 5:
                    #     print('Stop Here!')
                    # next_alpha = parsed_trajectory[i+1]
                    # cache.add_test_run([parsed_trajectory[i], parsed_trajectory[i+1]])
                    
                if not_in_cache:
                    break 

            if not_in_cache:
                break 

            if not not_in_cache:
                cache_hit += 2
            # current_trajectory.append(alpha+[int(not_in_cache)])
            current_trajectory.append(alpha+[int(not_in_cache)])

            alpha = next_alpha

        simulated_trajectory = []
        if not_in_cache:
            simulated_trajectory = parsed_trajectory[i:]
            cache.add_test_run(simulated_trajectory, rotate_3d = rotate_3d)
        current_trajectory += [time_step + [1] for time_step in simulated_trajectory]

        # if trajectory_hit / trajectory_length > 0.5:
        plot_trajectory(current_trajectory, parsed_trajectory, test.idx, test_case)
        
    return cache_hit, total_points, cache

def test_suite_reduction_3(TS: List[Test], resolution = [0.1,0.1,0.1], fine_resolution = [0,0,0], trajectory_dict = {}, test_case = 0, rotate_3d = True):
    cache = TestSuiteCache(resolution = resolution, fine_resolution = fine_resolution)
    execs = []
    cache_hit = 0
    total_points = 0
    processed_trajectories = 0
    for k, test in enumerate(TS):
        cache_size = len(cache.cache_dict)
        not_in_cache = False 
        if test.idx in trajectory_dict:
            parsed_trajectory = trajectory_dict[test.idx]
        else:
            parsed_trajectory = load_parse_trajectory(test)
        if parsed_trajectory is None:
            continue
        processed_trajectories += 1
        total_points += len(parsed_trajectory) * 2
        
        trajectory_length = len(parsed_trajectory*2)
        trajectory_hit = 0
        current_trajectory = []
        # if k == 3:
        #     print("stop here")
        
        alpha = [[parsed_trajectory[0][0]],[parsed_trajectory[0][1]]]
        print(f"processed trajectories {processed_trajectories}, test {test.idx}, total points {total_points}, cache size {cache_size}, hit {cache_hit}")
        num_agent = test.num_agents
        for i in range(len(parsed_trajectory)-1):
            next_alpha = []
            not_in_cache = False
            if i == 3:
                print('stop here')
            for j, agent in enumerate(alpha):
                in_cache = False
                potential_next_state = []
                for agent_state in agent:
                    ra = parsed_trajectory[i][j][-1]
                    desired_velocity = agent_planner(agent_state, ra, test.waypoints[j][0])
                    if cache.in_cache_2(agent_state, desired_velocity, rotate_3d):
                        next_state = cache.get_next_state_2(agent_state, desired_velocity, rotate_3d)
                        trajectory_hit += 1
                        # next_alpha.append(next_state)
                        potential_next_state += next_state
                    else:
                        not_in_cache = True
                    
                    if not_in_cache:
                        break 
                    # Detect one potential next state in cache 
                    in_cache = True

                not_in_cache = False
                if not in_cache:
                    next_alpha = [[parsed_trajectory[i+1][0]], [parsed_trajectory[i+1][1]]]
                    cache.add_test_run_2([parsed_trajectory[i], parsed_trajectory[i+1]], rotate_3d)
                    not_in_cache = True
                    break
                next_alpha.append(potential_next_state)

            if not not_in_cache:
                cache_hit += 2
            current_trajectory.append(alpha+[int(not_in_cache)])
            
            alpha = next_alpha

        plot_trajectory(current_trajectory, parsed_trajectory, test.idx, test_case)
        
    return cache_hit, total_points, cache


def test_suite_reduction_4(TS: List[Test], resolution = [0.1,0.1,0.1], trajectory_dict = {}, test_case = 0, rotate_3d = True):
    cache = TestSuiteCache(resolution = resolution)
    execs = []
    cache_hit = 0
    total_points = 0
    processed_trajectories = 0
    for k, test in enumerate(TS):
        cache_size = len(cache.cache_dict)
        not_in_cache = False 
        if test.idx in trajectory_dict:
            parsed_trajectory = trajectory_dict[test.idx]
        else:
            parsed_trajectory = load_parse_trajectory(test)
        if parsed_trajectory is None:
            continue
        processed_trajectories += 1
        total_points += len(parsed_trajectory) * 2
        
        trajectory_length = len(parsed_trajectory*2)
        trajectory_hit = 0
        current_trajectory = []
        # if k == 3:
        #     print("stop here")
        
        alpha = parsed_trajectory[0]
        # print(f"processed trajectories {processed_trajectories}, test {test.idx}, total points {total_points}, cache size {cache_size}, hit {cache_hit}")
        num_agent = test.num_agents
        for i in range(len(parsed_trajectory)-1):
            next_alpha = []
            not_in_cache = False
            # if i > 50:
            #     print('stop here')
            for j, agent_state in enumerate(alpha):
                ra = parsed_trajectory[i][j][-1]
                desired_velocity = agent_planner(agent_state, ra, test.waypoints[j][0])
                if cache.in_cache_2(agent_state, desired_velocity, rotate_3d):
                    next_state = cache.get_next_state_2(agent_state, desired_velocity, rotate_3d)
                    trajectory_hit += 1
                    next_alpha.append(next_state)
                else:
                    not_in_cache = True
                    break
                    
            if not_in_cache:
                break 

            if not not_in_cache:
                cache_hit += 2
            # current_trajectory.append(alpha+[int(not_in_cache)])
            current_trajectory.append(alpha+[int(not_in_cache)])

            alpha = next_alpha

        simulated_trajectory = []
        if not_in_cache:
            simulated_trajectory = parsed_trajectory[i:]
            cache.add_test_run_2(simulated_trajectory, rotate_3d)
        current_trajectory += [time_step + [1] for time_step in simulated_trajectory]

        plot_trajectory(current_trajectory, parsed_trajectory, test.idx, test_case)
        
    return cache_hit, total_points, cache

def plot_cache_cell_coverage(cache: TestSuiteCache, test_case = 0):
    
    for key in cache.cache_dict:
        plt.plot(key[-3], key[-2], 'r.')
    
    fn = f'./plots/{test_case}_cell_coverage.png'
    plt.savefig(fn)
    # plt.show()
    plt.clf()
    
def plot_acas_input_coverage(trajectory_dict):
    for key in trajectory_dict:
        parsed_trajectory = trajectory_dict[key]
        for time_step in parsed_trajectory:
            x_offset = time_step[0][0] - time_step[1][0]
            y_offset = time_step[0][1] - time_step[1][1]
            plt.plot(x_offset, y_offset, 'r.')
    
    fn = f'./plots/acas_inpu_coverage.png'
    plt.savefig(fn)
    plt.show()
    # plt.clf()

def plot_cache_content(cache: TestSuiteCache, test_case = 0):
    for key in cache.cache_dict:
        content = cache.cache_dict[key]
        plt.plot([0, content[0]], [0, content[1]], 'r')

    fn = f'./plots/{test_case}_cache_content.png'
    plt.savefig(fn)
    plt.clf()
    
if __name__ == "__main__":

    # theta_list = [-np.pi*3/4, -np.pi/2, -np.pi*3/8, -np.pi/4, 0, np.pi/4, np.pi/3, np.pi*3/4, np.pi]
    # vint_list = [60, 150, 300, 450, 600, 750, 900, 1050, 1145]
    # rho_list = [10000, 43736, 87472, 120000]

    # theta_list = [-np.pi*3/4, -np.pi/2, -np.pi*3/8, -np.pi/4, 0, np.pi/4, np.pi/3, np.pi*3/4, np.pi]
    # # theta_list = [-np.pi*3/4, -np.pi/2, -np.pi*3/8, -np.pi/4, 0, np.pi/4, ]
    theta_list = [-np.pi*3/4]
    
    # # vint_list = [900]
    # vint_list = [600, 750, 900, 1050]
    vint_list = [750]

    # # rho_list = [43736]
    # rho_list = [10000, 43736, 87472, ]
    rho_list = [10000, 10000]
    
    # theta_list = [-np.pi*3/4]
    # vint_list = [600,900]
    # rho_list = [10000, 43736,]

    TS = generate_test_suite(theta_list, vint_list, rho_list)

    trajectory_dict = {}

    # for k,test in enumerate(TS):
    #     parsed_trajectory = load_parse_trajectory(test)
    #     if parsed_trajectory is not None:
    #         trajectory_dict[test.idx] = parsed_trajectory
    #     print(f"processed trajectories {k}, test {test.idx}")

    # plot_acas_input_coverage(trajectory_dict)

    # # # # Per point simulation with 2d rotation
    # cache_hit, total_points, cache = test_suite_reduction(TS, rotate_3d=False, resolution = [0.1,0.1,0.1,0.1,0.5,0.1,0.1,0.5], trajectory_dict = trajectory_dict, test_case = 0)
    # cache_size = len(cache.cache_dict)
    # resolution = cache.resolution
    # plot_cache_cell_coverage(cache, 0)
    # plot_cache_content(cache, 0)
    # print('Per point simulation with 2d rotation')
    # print(f'Number of total points visited {total_points}, Number of cache hit {cache_hit}, Size of cache {cache_size}, Resolution {resolution}')

    # # # # Long simulation with 2d rotation
    # cache_hit, total_points, cache = test_suite_reduction_2(TS, rotate_3d=False, resolution = [0.1,0.1,0.1,0.1,0.5,0.1,0.1,0.5], trajectory_dict = trajectory_dict, test_case = 1)
    # cache_size = len(cache.cache_dict)
    # resolution = cache.resolution
    # plot_cache_cell_coverage(cache, 1)
    # plot_cache_content(cache, 1)
    # print('Long simulation with 2d rotation')
    # print(f'Number of total points visited {total_points}, Number of cache hit {cache_hit}, Size of cache {cache_size}, Resolution {resolution}')

    # # # Per point with 2d rotation and velocity translation
    # cache_hit, total_points, cache = test_suite_reduction_3(TS, resolution = [0.1,0.1,0.05], trajectory_dict = trajectory_dict, test_case = 6, rotate_3d = False)
    # cache_size = len(cache.cache_dict)
    # resolution = cache.resolution
    # plot_cache_cell_coverage(cache, 6)
    # plot_cache_content(cache, 6)
    # print('Per point with 2d rotation and velocity translation')
    # print(f'Number of total points visited {total_points}, Number of cache hit {cache_hit}, Size of cache {cache_size}, Resolution {resolution}')

    # # # Long simulation with 2d rotation and velocity translation
    # cache_hit, total_points, cache = test_suite_reduction_4(TS, resolution = [0.1,0.1,0.5], trajectory_dict = trajectory_dict, test_case = 7, rotate_3d = False)
    # cache_size = len(cache.cache_dict)
    # resolution = cache.resolution
    # plot_cache_cell_coverage(cache, 7)
    # plot_cache_content(cache, 7)
    # print('Long simulation with 2d rotation and velocity translation')
    # print(f'Number of total points visited {total_points}, Number of cache hit {cache_hit}, Size of cache {cache_size}, Resolution {resolution}')

    # # # Per point simulation with 3d rotation
    # cache_hit, total_points, cache = test_suite_reduction(TS, rotate_3d=True, resolution = [0.1,0.1,0.5,0.1,0.1,0.5], trajectory_dict = trajectory_dict, test_case = 2)
    # cache_size = len(cache.cache_dict)
    # resolution = cache.resolution
    # plot_cache_cell_coverage(cache, 2)
    # plot_cache_content(cache, 2)
    # print('Per point simulation with 3d rotation')
    # print(f'Number of total points visited {total_points}, Number of cache hit {cache_hit}, Size of cache {cache_size}, Resolution {resolution}')

    # # # Long simulation with 3d rotation
    # cache_hit, total_points, cache = test_suite_reduction_2(TS, rotate_3d=True, resolution = [0.1,0.1,0.5,0.1,0.1,0.5], trajectory_dict = trajectory_dict, test_case = 3)
    # cache_size = len(cache.cache_dict)
    # resolution = cache.resolution
    # plot_cache_cell_coverage(cache, 3)
    # plot_cache_content(cache, 3)
    # print('Long simulation with 3d rotation')
    # print(f'Number of total points visited {total_points}, Number of cache hit {cache_hit}, Size of cache {cache_size}, Resolution {resolution}')

    # # Per point with 3d rotation and velocity translation
    cache_hit, total_points, cache = test_suite_reduction_3(TS, resolution = [0.1,0.1,0.1], fine_resolution = [1e-5, 1e-5, 5e-5], trajectory_dict = trajectory_dict, test_case = 4, rotate_3d = True)
    cache_size = len(cache.cache_dict)
    resolution = cache.resolution
    # plot_cache_cell_coverage(cache, 4)
    # plot_cache_content(cache, 4)
    print('Per point with 3d rotation and velocity translation')
    print(f'Number of total points visited {total_points}, Number of cache hit {cache_hit}, Size of cache {cache_size}, Resolution {resolution}')

    # # # Long simulation with 3d rotation and velocity translation
    # cache_hit, total_points, cache = test_suite_reduction_4(TS, resolution = [0.1,0.1,0.5], trajectory_dict = trajectory_dict, test_case = 5, rotate_3d = True)
    # cache_size = len(cache.cache_dict)
    # resolution = cache.resolution
    # plot_cache_cell_coverage(cache, 5)
    # plot_cache_content(cache, 5)
    # print('Long simulation with 3d rotation and velocity translation')
    # print(f'Number of total points visited {total_points}, Number of cache hit {cache_hit}, Size of cache {cache_size}, Resolution {resolution}')