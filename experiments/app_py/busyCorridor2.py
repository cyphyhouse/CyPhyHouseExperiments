import rospy
from geometry_msgs.msg import PoseStamped, Point, Pose, Vector3
from cym_marker.msg import Marker, Material, Script

from src.harness.agentThread import AgentThread


class Follow_points(AgentThread):

    def __init__(self, config, motion_config):
        super(Follow_points, self).__init__(config, motion_config)

        self.pub_marker = rospy.Publisher("/cym_marker", Marker, queue_size=10)


        self.waypoints = [[(-60.0, -65.0, 1.0), (-55.0, -65.0, 1.0), (-50.0, -65.0, 1.0), (-45.0, -65.0, 1.0), (-40.0, -65.0, 1.0), (-35.0, -65.0, 0.3)],
                          [(-60.0, -65.0, 1.0), (-55.0, -65.0, 1.0), (-50.0, -65.0, 1.0), (-45.0, -65.0, 1.0), (-40.0, -65.0, 1.0), (-36.0, -66.0, 0.3)],
                          [(-60.0, -65.0, 1.0), (-55.0, -65.0, 1.0), (-50.0, -65.0, 1.0), (-45.0, -65.0, 1.0), (-40.0, -65.0, 1.0), (-37.0, -67.0, 0.3)],
                          [(-45.0, -67.0, 1.0), (-50.0, -67.0, 1.0), (-55.0, -67.0, 1.0), (-60.0, -67.0, 1.0), (-65.0, -67.0, 1.0), (-80.0, -67.0, 0.3)],
                          [(-45.0, -67.0, 1.0), (-50.0, -67.0, 1.0), (-55.0, -67.0, 1.0), (-60.0, -67.0, 1.0), (-65.0, -67.0, 1.0), (-80.0, -68.0, 0.3)],
                          [(-45.0, -67.0, 1.0), (-50.0, -67.0, 1.0), (-55.0, -67.0, 1.0), (-60.0, -67.0, 1.0), (-65.0, -67.0, 1.0), (-80.0, -69.0, 0.3)]]

        if self.pid() == 0 or self.pid() == 3 :
            self.nextWaypt = True
        else:
            self.nextWaypt = False
            rospy.Subscriber('/vrpn_client_node/drone'+str(self.pid()-1)+'/pose', PoseStamped, self.leadDronePose)

    def leadDronePose(self, data):
        currPos = (data.pose.position.x,  data.pose.position.y,  data.pose.position.z)
        #print("lead pid: ", currPos)
        goalPos = (self.waypoints[self.pid()-1][1][0],self.waypoints[self.pid()-1][1][1], self.waypoints[self.pid()-1][1][2])

        if (currPos[0]-.05 < goalPos[0] < currPos[0]+.05) and (currPos[1]-.05 < goalPos[1] < currPos[1]+.05) and self.nextWaypt == False:
            self.nextWaypt = True




    def initialize_vars(self):
        self.locals = {}
        self.locals['tries'] = 1
        self.locals['p'] = 10 - 3 * self.pid()

        self.waypoint_num = 0

        self.locals[self.pid()] = self.waypoints[self.pid()]

    def loop_body(self):
        if self.locals['tries'] == 1 and self.nextWaypt:
            x = self.locals[self.pid()][self.waypoint_num][0]
            y = self.locals[self.pid()][self.waypoint_num][1]
            z = self.locals[self.pid()][self.waypoint_num][2]
            # nx = self.locals[self.pid()][self.waypoint_num+1][0]
            # ny = self.locals[self.pid()][self.waypoint_num+1][1]
            # nz = self.locals[self.pid()][self.waypoint_num+1][2]


            self.write_to_actuator('Motion.target', self.pos3d(x, y, z))
            self.locals['tries'] += 1
            self.waypoint_num += 1
            #marker = self.factory_marker(i=self.pid(), x=x, y=y, z=z, nx=nx, ny=ny, nz=nz)
            #self.pub_marker.publish(marker)
            return
        if self.locals['tries'] <= len(self.waypoints[self.pid()]) and self.read_from_sensor('Motion.reached') and self.nextWaypt:
            x = self.locals[self.pid()][self.waypoint_num-1][0]
            y = self.locals[self.pid()][self.waypoint_num-1][1]
            z = self.locals[self.pid()][self.waypoint_num-1][2]


            nx = self.locals[self.pid()][self.waypoint_num][0]
            ny = self.locals[self.pid()][self.waypoint_num][1]
            nz = self.locals[self.pid()][self.waypoint_num][2]



            self.write_to_actuator('Motion.target', self.pos3d(nx, ny, nz))
            self.locals['tries'] += 1
            self.waypoint_num += 1
            marker = self.factory_marker(i=self.pid(), x=x, y=y, z=z, nx=nx, ny=ny, nz=nz)
            self.pub_marker.publish(marker)
            return
        if self.locals['tries'] == len(self.waypoints[self.pid()])+1 and self.read_from_sensor('Motion.reached') and self.nextWaypt:
            self.trystop()
            marker = self.factory_marker(i=self.pid(), x=0, y=0, z=0, nx=0, ny=0, nz=0)
            self.pub_marker.publish(marker)
            return



    def factory_script(self, i):
        PREDEFINED_SCRIPT = [
           "Gazebo/RedTransparent",
           "Gazebo/GreenTransparent",
           "Gazebo/BlueTransparent",
           "Gazebo/DarkMagentaTransparent",
           "Gazebo/GreyTransparent",
           "Gazebo/BlackTransparent",
           "Gazebo/YellowTransparent",
        ]
        return Script(name=PREDEFINED_SCRIPT[i % len(PREDEFINED_SCRIPT)])


    def factory_pose(self, x,y,z,nx,ny,nz):
        pose = Pose()
        pose.position.x = (x + nx)/2
        pose.position.y = (y + ny)/2
        pose.position.z = z
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose


    def factory_scale(self, x,y,z,nx,ny,nz):
        scaleX = abs((x-nx))/2
        scaleY = 1.0
        scaleZ = 1.0
        return Vector3(x=scaleX, y=scaleY, z=scaleZ)


    def factory_marker(self, i, x=0,y=0,z=0, nx=0,ny=0,nz=0, action = Marker.ADD_MODIFY):
        SIMPLE_TYPE = [
            Marker.BOX,
            Marker.CYLINDER,
            Marker.SPHERE,
            Marker.TEXT
        ]

        mat = Material(script=self.factory_script(i))

        marker = Marker()
        marker.header.frame_id = "world"
        marker.action = action
        marker.id = i
        if action != Marker.ADD_MODIFY:
            return marker

        marker.type = SIMPLE_TYPE[0]
        marker.pose = self.factory_pose(x,y,z, nx,ny,nz)
        marker.scale = self.factory_scale(x,y,z,nx,ny,nz)
        marker.material = mat

        if marker.type == Marker.TEXT:
            marker.text = "Hello world!"

        return marker
