
import json
import mmap
import contextlib
import time
import rospy
from redhead.msg import Move #Route



def log(s):
    print(s)
    rospy.loginfo(s)

class AprilTag():

    APRILTAG_SHARED = '../data/apriltag.shared'
    ROS_NODE = "apriltag"
    ROS_MOVE_CHANNEL = "motor_movement"
    ROS_MOVE_CHANNEL_QUEUE = 10
    MIN_RAGE_CM = 10
    MAX_RANGE_CM = 200

    def __init__(self):
        self.move_pub = rospy.Publisher(Teleop.ROS_MOVE_CHANNEL, Move, queue_size=Teleop.ROS_MOVE_CHANNEL_QUEUE)
        self.last_tag_id = '1'
        self.destination_tag = None

    def get_map(self):
        pass

    def calculate_route(self, current_tag, destination_tag):
        pass

    def roam(self):
        pass

    def destination_tag_callback():
        self.destination_tag = 5

    def read_tag(self):
        m.seek(0)
        # print('\nreading...')
        apriltag = None
        arpriltag_json = m.readline().decode("utf-8")
        print(apriltag_json)
        try:
            apriltag = json.loads(sensors_json)
            if apriltag['id'] == self.last_tag_id:
                apriltag = None
        except Exception as e:
            print("exception %s"% str(e))
        return apriltag

    def command_loop(self):
        with open(APRILTAG_SHARED, 'r') as f:
            with contextlib.closing(mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)) as m:
                while not rospy.is_shutdown():
                    apriltag = self.read_tag()
                    if apriltag is not None:
                        # find closest detection not too close
                        # calculate route
                        # set next tag
                        # goto next tag
                    else: # look around
                        self.roam()
                    time.sleep(.25)

if __name__=='__main__':
    
