import rospy
from geometry_msgs.msg import PoseStamped, Point

from src.harness.agentThread import AgentThread


class Follow_points(AgentThread):

    def __init__(self, config, motion_config):
        super(Follow_points, self).__init__(config, motion_config)



        self.waypoints = [[(-70.0,-65.0,1.0), (-45.0,-66.0,1.0), (-39.0,-65.0,0.3)],
                          [(-70.0,-65.0,1.0), (-45.0,-66.0,1.0), (-41.0,-65.0,0.3)],
                          [(-70.0,-65.0,1.0), (-45.0,-66.0,1.0), (-43.0,-65.0,0.3)]]

        if self.pid() != 0:
            self.nextWaypt = False
            rospy.Subscriber('/vrpn_client_node/drone'+str(self.pid()-1)+'/pose', PoseStamped, self.leadDronePose)
        else:
            self.nextWaypt = True

    def leadDronePose(self, data):
        currPos = (data.pose.position.x,  data.pose.position.y,  data.pose.position.z)
        #print("lead pid: ", currPos)
        goalPos = (self.waypoints[self.pid()-1][-1][0],self.waypoints[self.pid()-1][-1][1], self.waypoints[self.pid()-1][-1][2])

        if (currPos[0]-.01 < goalPos[0] < currPos[0]+.01) and (currPos[1]-.01 < goalPos[1] < currPos[1]+.01) and self.nextWaypt == False:
            self.nextWaypt = True




    def initialize_vars(self):
        self.locals = {}
        self.locals['tries'] = 1
        self.locals['p'] = 10 - 3 * self.pid()


        # self.waypoints = [[(-70,-68,1), (-45,-68,1)],
        #                   [(-70,-68,1), (-45,-68,1)],
        #                   [(-70,-68,1), (-45,-68,1)]]

        self.waypoint_num = 0

        self.locals[self.pid()] = self.waypoints[self.pid()]

    def loop_body(self):
        if self.locals['tries'] == 1 and self.nextWaypt:
            x = self.locals[self.pid()][self.waypoint_num][0]
            y = self.locals[self.pid()][self.waypoint_num][1]
            z = self.locals[self.pid()][self.waypoint_num][2]
            self.write_to_actuator('Motion.target', self.pos3d(x, y, z))
            self.locals['tries'] += 1
            self.waypoint_num += 1
            return
        if self.locals['tries'] < 4 and self.read_from_sensor('Motion.reached') and self.nextWaypt:
            x = self.locals[self.pid()][self.waypoint_num][0]
            y = self.locals[self.pid()][self.waypoint_num][1]
            z = self.locals[self.pid()][self.waypoint_num][2]
            self.write_to_actuator('Motion.target', self.pos3d(x, y, z))
            self.locals['tries'] += 1
            self.waypoint_num += 1
            return
        if self.locals['tries'] == 4 and self.read_from_sensor('Motion.reached') and self.nextWaypt:
            self.trystop()
            return


        # if self.locals['tries'] == 1:
        #     self.write_to_actuator('Motion.target', self.pos3d( -self.locals['p'], self.locals['p'], 1.0))
        #     self.locals['tries'] = 2
        #     return
        # if self.locals['tries'] == 2 and self.read_from_sensor('Motion.reached'):
        #     self.write_to_actuator('Motion.target', self.pos3d(self.locals['p'], self.locals['p'], 1.0))
        #     self.locals['tries'] = 3
        #     return
        # if self.locals['tries'] == 3 and self.read_from_sensor('Motion.reached'):
        #     self.write_to_actuator('Motion.target', self.pos3d(self.locals['p'],  -self.locals['p'], 1.0))
        #     self.locals['tries'] = 4
        #     return
        # if self.locals['tries'] == 4 and self.read_from_sensor('Motion.reached'):
        #     self.write_to_actuator('Motion.target', self.pos3d( -self.locals['p'],  -self.locals['p'], 1.0))
        #     self.locals['tries'] = 5
        #     return
        # if self.locals['tries'] == 5 and self.read_from_sensor('Motion.reached'):
        #     self.trystop()
        #     return
