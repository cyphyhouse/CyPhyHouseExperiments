from src.harness.agentThread import AgentThread

from reachtube.drone3d_nosymmetry_v1 import tc_simulate, bloat_to_tube
from reachtube.rect_set_impl._boxes_map import Map


import rospy
from geometry_msgs.msg import PoseStamped, Point, Pose, Vector3
from cym_marker.msg import Marker, Material, Script
from std_msgs.msg import ColorRGBA


import sys
import copy
import time
import random
import math as m
import numpy as np
import yaml
from typing import List, Tuple, Union

from scipy.integrate import odeint





class Follow_points(AgentThread):

    def __init__(self, config, motion_config):
        super(Follow_points, self).__init__(config, motion_config)

        self.startTime = time.time()
        with open('/home/hdt2/Desktop/obsPts.data.yaml', 'r') as filehandle:
            self.obsPts = yaml.safe_load(filehandle)
        with open('/home/hdt2/Desktop/walls.data.yaml', 'r') as filehandle:
            self.walls = yaml.safe_load(filehandle)

        self.pub_marker = rospy.Publisher("/cym_marker", Marker, queue_size=10)
        self.initialPoints = [(-3.0, -3.0, 0.3), (4.0, -1.0, 0.3), (-2.0,  0.0, 0.3)]


    def initialize_vars(self):
        self.locals = {}
        self.locals['tries'] = 1
        self.locals['p'] = 10 - 3 * self.pid()

        mat1 = Material()
        mat2 = Material()
        mat3 = Material()

        ambient = ColorRGBA()
        diffuse = ColorRGBA()
        specular = ColorRGBA()
        emissive = ColorRGBA()

        ambient.r = .5
        ambient.g = .75
        ambient.g = 0
        ambient.a = 1

        diffuse.r = .7
        diffuse.g = .9
        diffuse.g = 0
        diffuse.a = 1

        specular.r = .2
        specular.g = .2
        specular.g = 2
        specular.a = 64

        emissive.r = .1
        emissive.g = 0
        emissive.g = .1
        emissive.a = 1


        mat1.ambient = ambient
        mat1.diffuse = diffuse
        mat1.specular = specular
        mat1.emissive = emissive

        mat2.ambient = ambient
        mat2.diffuse = diffuse
        mat2.specular = specular
        mat2.emissive = emissive

        mat3.ambient = ambient
        mat3.diffuse = diffuse
        mat3.specular = specular
        mat3.emissive = emissive

        self.markerColors = [mat1, mat2, mat3]

        self.waypoints = [[(20,14,3), (23,0,3), (16,-16,3)],
                          [(14.5,-16,3), (26,0,3), (20,14,3)],
                          [(2,2,3), (-2,-2,3), (-2,2,3) ]]

        self.pub_marker.publish(self.factory_marker(i=((self.pid()+1)*10000)+2,pos_x=self.waypoints[self.pid()][0][0], pos_y=self.waypoints[self.pid()][0][1], pos_z=self.waypoints[self.pid()][0][2], type=2, color=0))
        self.pub_marker.publish(self.factory_marker(i=((self.pid()+1)*10000)+3,pos_x=self.waypoints[self.pid()][1][0], pos_y=self.waypoints[self.pid()][1][1], pos_z=self.waypoints[self.pid()][1][2], type=2, color=0))
        self.pub_marker.publish(self.factory_marker(i=((self.pid()+1)*10000)+4,pos_x=self.waypoints[self.pid()][2][0], pos_y=self.waypoints[self.pid()][2][1], pos_z=self.waypoints[self.pid()][2][2], type=2, color=0))

        # self.waypoints = [[(19,13,1), (23,0,1), (16,-16,1), (-4,-4,1)],
        #                   [(18,13.5,2), (26,0,2), (20,-17,2), (-4,-7,2)],
        #                   [(16,14.5,3), (29,0,3), (22,-17,3), (-7,-7,3)],]

        self.waypoint_num = 0

        self.locals[self.pid()] = self.waypoints[self.pid()]

    def loop_body(self):
        if self.locals['tries'] == 1:
            #self.timeToNextPt = time.time()
            x = self.locals[self.pid()][self.waypoint_num][0]
            y = self.locals[self.pid()][self.waypoint_num][1]
            z = self.locals[self.pid()][self.waypoint_num][2]
            initX = self.initialPoints[self.pid()][0]
            initY = self.initialPoints[self.pid()][1]
            initZ = self.initialPoints[self.pid()][2]

            rtube = self.getTube(x=x, y=y, z=z, initX=initX, initY=initY, initZ=initZ)
            env = Map(self.walls)

            self.val = (self.visualizeTube(rtube))


            self.texecStart = time.time()
            self.write_to_actuator('Motion.target', self.pos3d(x, y, z))
            self.locals['tries'] += 1
            self.waypoint_num += 1
            return
        if self.locals['tries'] <= len(self.waypoints[self.pid()]) and  self.read_from_sensor('Motion.reached'):
            print(self.pid(), "reached waypoint", self.waypoint_num, time.time()-self.texecStart)
            self.deleteTubes(self.val)
            self.pub_marker.publish(self.factory_marker(i=((self.pid()+1)*10000)+self.locals['tries'],pos_x=self.waypoints[self.pid()][self.waypoint_num-1][0], pos_y=self.waypoints[self.pid()][self.waypoint_num-1][1], pos_z=self.waypoints[self.pid()][self.waypoint_num-1][2], type=2, color=1))

            x = self.locals[self.pid()][self.waypoint_num][0]
            y = self.locals[self.pid()][self.waypoint_num][1]
            z = self.locals[self.pid()][self.waypoint_num][2]
            initX = self.locals[self.pid()][self.waypoint_num-1][0]
            initY = self.locals[self.pid()][self.waypoint_num-1][1]
            initZ = self.locals[self.pid()][self.waypoint_num-1][2]

            rtube = self.getTube(x=x, y=y, z=z, initX=initX, initY=initY, initZ=initZ)
            env = Map(self.walls)

            self.val = (self.visualizeTube(rtube))

            self.texecStart = time.time()
            self.write_to_actuator('Motion.target', self.pos3d(x, y, z))
            self.locals['tries'] += 1
            self.waypoint_num += 1
            return
        if self.locals['tries'] == len(self.waypoints[self.pid()])+1 and self.read_from_sensor('Motion.reached'):
            print(self.pid(), "reached waypoint", self.waypoint_num, time.time()-self.texecStart)
            self.pub_marker.publish(self.factory_marker(i=((self.pid()+1)*10000)+self.locals['tries'],pos_x=self.waypoints[self.pid()][self.waypoint_num-1][0], pos_y=self.waypoints[self.pid()][self.waypoint_num-1][1], pos_z=self.waypoints[self.pid()][self.waypoint_num-1][2], type=2, color=1))

            self.deleteTubes(self.val)
            self.trystop()
            print("Total Time", self.pid(), time.time()-self.startTime)
            return

    #function to convert a reachtube to a map type
    def convertTubeToMap(self, rtube):
        newTubeType = []
        for j in range(1, len(rtube)-1,2):
            newTubeType.append(( (rtube[j+1][1],rtube[j+1][2],rtube[j+1][3]) , (rtube[j][1],rtube[j][2],rtube[j][3]) ))
        return newTubeType


    def getTube(self, x, y, z, initX, initY, initZ):
        start = time.time()
        trace = tc_simulate([x, y, z], [initX, initY, initZ, 0,0,0,0,0,0], 50)
        goal = [15, 23, 2.5]
        dimensions = len(trace[0])
        k = [1] * (dimensions - 1)
        gamma = [0] * (dimensions - 1)
        init_delta_array = [0.5,0.5,0.5] + [0.1] * (dimensions - 4)
        rtube = bloat_to_tube(init_delta_array, trace, dimensions, goal)
        print("Reachtube/bloting for", self.pid(), time.time()-start)
        return rtube


    #Functino to find findPotentialObstacles
    def findPotentialObstacles(self, rtube):
        rtubePt1X = (rtube[0][0][0] + rtube[0][1][0])/2
        rtubePt1Y = (rtube[0][0][1] + rtube[0][1][1])/2
        rtubePt2X = (rtube[-1][0][0] + rtube[-1][1][0])/2
        rtubePt2Y = (rtube[-1][0][1] + rtube[-1][1][1])/2
        lenReachTube = np.sqrt((rtubePt2X-rtubePt1X)**2 + (rtubePt2Y-rtubePt1Y)**2)
        #print(lenReachTube)
        obsToCheck = []
        for i in range(0, len(self.obsPts)):
            obsX1 = self.obsPts[i][0][0]
            obsX2 = self.obsPts[i][1][0]
            obsY1 = self.obsPts[i][0][1]
            obsY2 = self.obsPts[i][1][1]
            obsMidptX = (obsX1 + obsX2)/2
            obsMidptY = (obsY1 + obsY2)/2


            distToObs = np.sqrt((rtubePt1X-obsMidptX)**2 + (rtubePt1Y-obsMidptY)**2)
            distToObs2 = np.sqrt((rtubePt2X-obsMidptX)**2 + (rtubePt2Y-obsMidptY)**2)

            if(lenReachTube >= distToObs or lenReachTube >= distToObs2):
                #plt.plot([obsX1, obsX2], [obsY1, obsY2], 'go-')
                obsToCheck.append(self.walls[i])
            else:
                pass
                #plt.plot([obsX1, obsX2], [obsY1, obsY2], 'ko-')
        return obsToCheck

    def factory_script(self, i):
        PREDEFINED_SCRIPT = [
           "Gazebo/RedTransparent",
           "Gazebo/GreenTransparent",
           "Gazebo/OrangeTransparent",
           "Gazebo/YellowTransparent",
           "Gazebo/GreyTransparent",
           "Gazebo/BlueTransparent",
           "Gazebo/DarkMagentaTransparent",
           "Gazebo/BlackTransparent",
        ]
        return Script(name=PREDEFINED_SCRIPT[i])


    def factory_pose(self, i, x, y, z,tx,ty,tz,tw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = tx
        pose.orientation.y = ty
        pose.orientation.z = tz
        pose.orientation.w = tw
        return pose


    def factory_scale(self, i,x,y,z):
        return Vector3(x=x, y=y, z=z)


    def factory_marker(self, i, pos_x=0, pos_y=0, pos_z=1,scale_x=1, scale_y=1, scale_z=1, tx=0,ty=0,tz=0,tw=1, type=0, color=4, action = Marker.ADD_MODIFY):
        SIMPLE_TYPE = [
            Marker.BOX,
            Marker.CYLINDER,
            Marker.SPHERE,
            Marker.TEXT
        ]

        mat = Material(script=self.factory_script(color))
        #mat = self.markerColors[self.pid()]

        marker = Marker()
        marker.header.frame_id = "world"
        marker.action = action
        marker.id = i
        if action != Marker.ADD_MODIFY:
            return marker

        marker.type = SIMPLE_TYPE[type]
        marker.pose = self.factory_pose(i, pos_x, pos_y, pos_z,tx,ty,tz,tw)
        marker.scale = self.factory_scale(i,scale_x, scale_y, scale_z)
        marker.material = mat
        # marker.lifetime.secs = 3.0

        if marker.type == Marker.TEXT:
            marker.text = "Hello world!"

        return marker


    ################################################################################

    def visualizeTube(self, rtube):
        tubleList = []
        id=0

        #obsToCheck = self.findPotentialObstacles(rtube)
        #collisions = self.checkCollisions(obsToCheck, rtube)
        tube = Map(self.convertTubeToMap(rtube))
        start = time.time()
        obsToCheck = self.findPotentialObstacles(tube.obstacles)
        potObs = Map(obsToCheck)
        collisions = tube.find_overlapping_rectangles(potObs)
        print("Determine Obstacles for", self.pid(), time.time()-start)

        for i in range(0,len(tube.obstacles)):
            rect1 = tube.obstacles[i][0]
            rect2 = tube.obstacles[i][1]
            #print(rect1, rect2)
            #angle = np.arccos(np.dot((rect[1],rect[2]),()))
            id = id+1
            x = (rect1[0] + rect2[0])/2
            y = (rect1[1] + rect2[1])/2
            z = (rect1[2] + rect2[2])/2
            scale_x = abs((rect1[0]) - (rect2[0]))
            scale_y = abs((rect1[1]) - (rect2[1]))
            scale_z = abs((rect1[2]) - (rect2[2]))
            #print(scale_x,scale_y,scale_z)
            if i in collisions:
                color = 0
            else:
                color = self.pid()+2
            #color = self.pid()+2
            tubleList.append((id,x,y,z,scale_x,scale_y,scale_z,color))

        prev_id = 0;
        for i in range(0, len(tubleList),30):

            idx = tubleList[i][0] + (self.pid()*3000)
            x = tubleList[i][1]
            y = tubleList[i][2]
            z = tubleList[i][3]
            scale_x = tubleList[i][4]
            scale_y = tubleList[i][5]
            scale_z = tubleList[i][6]
            color = tubleList[i][7]
            #pub_marker.publish(factory_marker(prev_id, Marker.DELETE_MARKER))
            prev_id = idx
            self.pub_marker.publish(self.factory_marker(i=idx,pos_x=x, pos_y=y, pos_z=z,scale_x=scale_x, scale_y=scale_y, scale_z=scale_z/2, type=1, color=color))
            #print(idx)
            rospy.sleep(.0001)
        return len(tubleList)


    def deleteTubes(self, numTubes):
        for i in range((self.pid()*3000)+1, (numTubes+1)+(self.pid()*3000)):
            #print(i)
            self.pub_marker.publish(self.factory_marker(i, Marker.DELETE_MARKER))
            rospy.sleep(.0001)



    ###############################################################################
