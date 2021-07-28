import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from enum import Enum
import time 

import pickle

class AcasPlayback:

    class RA(Enum):
        CLIMB = 4
        DESCEND = 5
        VERT_RATE_LIMIT = 6
        TURN_RIGHT = 2
        TURN_LEFT = 3
        NONE = -1

    def __init__(self):
        self.drone0_publisher = rospy.Publisher('/vrpn_client_node/drone0/pose', PoseStamped, queue_size=1)
        self.drone1_publisher = rospy.Publisher('/vrpn_client_node/drone1/pose', PoseStamped, queue_size=1)
        self.acas_subscriber = rospy.Subscriber('/acas/ra', String, self._acas_handler, queue_size = 20)
        self.drone0_ra = 0
        self.drone0_ra_str = ""
        self.drone1_ra = 0
        self.drone1_ra_str = ""

    def _acas_handler(self, data):
        tmp = data.data  
        # print(tmp)
        raw_ra_string = tmp.split(',')
        vra_type = AcasPlayback.RA.NONE
        hra_type = AcasPlayback.RA.NONE

        # self.drone0_ra = 0
        # self.drone0_ra_str = ""
        # self.drone1_ra = 0
        # self.drone1_ra_str = ""        
        if '0' in raw_ra_string[0]:
            for i in range(1, len(raw_ra_string)):
                ra = raw_ra_string[i]
                if "Climb" in ra:
                    vra_type = AcasPlayback.RA.CLIMB
                elif "Descend" in ra:
                    vra_type = AcasPlayback.RA.DESCEND
                elif "Limit" in ra:
                    vra_type = AcasPlayback.RA.VERT_RATE_LIMIT
                
                if "Right" in ra:
                    hra_type = AcasPlayback.RA.TURN_RIGHT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    hra_val = float(val_string[1].strip())
                elif "Left" in ra:
                    hra_type = AcasPlayback.RA.TURN_LEFT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    hra_val = float(val_string[1].strip())
            
            if vra_type != AcasPlayback.RA.NONE or hra_type != AcasPlayback.RA.NONE:
                self.drone0_ra = 1
                self.drone0_ra_str = tmp
                print(tmp)
            else:
                self.drone0_ra = 0
                self.drone0_ra_str = ""
                
        else:
            for i in range(1, len(raw_ra_string)):
                ra = raw_ra_string[i]
                if "Climb" in ra:
                    vra_type = AcasPlayback.RA.CLIMB
                elif "Descend" in ra:
                    vra_type = AcasPlayback.RA.DESCEND
                elif "Limit" in ra:
                    vra_type = AcasPlayback.RA.VERT_RATE_LIMIT
                
                if "Right" in ra:
                    hra_type = AcasPlayback.RA.TURN_RIGHT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    hra_val = float(val_string[1].strip())
                elif "Left" in ra:
                    hra_type = AcasPlayback.RA.TURN_LEFT
                    val_string = raw_ra_string[i+1]
                    val_string = val_string.split(':')
                    hra_val = float(val_string[1].strip())
            
            if vra_type != AcasPlayback.RA.NONE or hra_type != AcasPlayback.RA.NONE:
                self.drone1_ra = 1
                self.drone1_ra_str = tmp            
                print(tmp)
            else:
                self.drone1_ra = 0
                self.drone1_ra_str = ""

    def replay_trajectories(self, fn0, fn1):

        drone0_trajectory = []
        with open(fn0, 'rb') as f:
            drone0_trajectory = pickle.load(f)

        drone1_trajectory = []
        with open(fn1, 'rb') as f:
            drone1_trajectory = pickle.load(f)

        playback_len = max(len(drone0_trajectory), len(drone1_trajectory))

        drone0_replay = []
        drone1_replay = []
        # acas_advice = 
        # rate = rospy.Rate(10000)
        for i in range(playback_len):
            if i%10000 == 0:
                print(i)
            if i < len(drone0_trajectory):
                point = drone0_trajectory[i]
                msg = PoseStamped()
                msg.pose.position.x = point[1]
                msg.pose.position.y = point[2]
                msg.pose.position.z = point[3]
                self.drone0_publisher.publish(msg)
                drone0_replay.append([time.time(), point[1], point[2], point[3], self.drone0_ra_str, self.drone0_ra])

            if i < len(drone1_trajectory):
                point = drone1_trajectory[i]
                msg = PoseStamped()
                msg.pose.position.x = point[1]
                msg.pose.position.y = point[2]
                msg.pose.position.z = point[3]
                self.drone1_publisher.publish(msg)
                drone1_replay.append([time.time(), point[1], point[2], point[3], self.drone1_ra_str, self.drone1_ra])

            # time.sleep(0)
            
        with open('./trajectories/drone0_pb', 'wb+') as f:
            pickle.dump(drone0_replay, f)

        with open('./trajectories/drone1_pb', 'wb+') as f:
            pickle.dump(drone1_replay, f)


if __name__ == "__main__":
    rospy.init_node('acas_playback')
    playBack = AcasPlayback()
    playBack.replay_trajectories(
        './trajectories/rotation_transform/drone0_9', 
        './trajectories/rotation_transform/drone1_9'
    )