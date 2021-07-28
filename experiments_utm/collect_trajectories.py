import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from enum import Enum
import time 

import pickle

class trajectoryCollector:

    class RA(Enum):
        CLIMB = 4
        DESCEND = 5
        VERT_RATE_LIMIT = 6
        TURN_RIGHT = 2
        TURN_LEFT = 3
        NONE = -1


    def __init__(self):
        self.acas_subscriber = rospy.Subscriber('/acas/ra',String,self._acas_handler,queue_size = 10)
        self.drone0_subscriber = rospy.Subscriber('/vrpn_client_node/drone0/pose', PoseStamped, self._drone0_handler, queue_size=1)
        self.drone1_subscriber = rospy.Subscriber('/vrpn_client_node/drone1/pose', PoseStamped, self._drone1_handler, queue_size=1)
        self.drone0_ra = 0
        self.drone0_ra_str = ""
        self.drone1_ra = 0
        self.drone1_ra_str = ""
        self.drone0_state_list = []
        self.drone1_state_list = []
        fn = './trajectories/drone0_pb'
        f = open(fn,'wb+')
        f.close()
        fn = './trajectories/drone1_pb'
        f = open(fn,'wb+')
        f.close()

    def _drone0_handler(self, data):
        fn = './trajectories/drone0_pb'
        with open(fn,'ab') as f:
            pickle.dump((time.time(), data.pose.position.x, data.pose.position.y, data.pose.position.z, self.drone0_ra_str, self.drone0_ra), f)

    def _drone1_handler(self, data):        
        fn = './trajectories/drone1_pb'
        with open(fn,'ab') as f:
            pickle.dump((time.time(), data.pose.position.x, data.pose.position.y, data.pose.position.z, self.drone1_ra_str, self.drone1_ra), f)
        # self.drone1_state_list.append((time.time(), data.pose.position.x, data.pose.position.y, data.pose.position.z, self.drone1_ra_str, self.drone1_ra))

    def _acas_handler(self, data):
        tmp = data.data  
        print(tmp)
        if 'Climb' in tmp or 'Descend' in tmp or 'Limit' in tmp or 'Right' in tmp or 'Left' in tmp:
            print("here")
        raw_ra_string = tmp.split(',')
        vra_type = trajectoryCollector.RA.NONE
        hra_type = trajectoryCollector.RA.NONE
      
        if '0' in raw_ra_string[0]:
            for i in range(1, len(raw_ra_string)):
                ra = raw_ra_string[i]
                if "Climb" in ra:
                    vra_type = trajectoryCollector.RA.CLIMB
                elif "Descend" in ra:
                    vra_type = trajectoryCollector.RA.DESCEND
                elif "Limit" in ra:
                    vra_type = trajectoryCollector.RA.VERT_RATE_LIMIT
                
                if "Right" in ra:
                    hra_type = trajectoryCollector.RA.TURN_RIGHT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    hra_val = float(val_string[1].strip())
                elif "Left" in ra:
                    hra_type = trajectoryCollector.RA.TURN_LEFT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    hra_val = float(val_string[1].strip())
            
            if vra_type != trajectoryCollector.RA.NONE or hra_type != trajectoryCollector.RA.NONE:
                self.drone0_ra = 1
                self.drone0_ra_str = tmp
                print(">>>>",tmp)
            else:
                self.drone0_ra = 0
                self.drone0_ra_str = ""
                
        else:
            for i in range(1, len(raw_ra_string)):
                ra = raw_ra_string[i]
                if "Climb" in ra:
                    vra_type = trajectoryCollector.RA.CLIMB
                elif "Descend" in ra:
                    vra_type = trajectoryCollector.RA.DESCEND
                elif "Limit" in ra:
                    vra_type = trajectoryCollector.RA.VERT_RATE_LIMIT
                
                if "Right" in ra:
                    hra_type = trajectoryCollector.RA.TURN_RIGHT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    hra_val = float(val_string[1].strip())
                elif "Left" in ra:
                    hra_type = trajectoryCollector.RA.TURN_LEFT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    hra_val = float(val_string[1].strip())
            
            if vra_type != trajectoryCollector.RA.NONE or hra_type != trajectoryCollector.RA.NONE:
                self.drone1_ra = 1
                self.drone1_ra_str = tmp            
                print(">>>>",tmp)
            else:
                self.drone1_ra = 0
                self.drone1_ra_str = ""

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('pb_collector')
    collecotr = trajectoryCollector()
    collecotr.spin()