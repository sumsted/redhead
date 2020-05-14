
import json
import mmap
import contextlib
import time
import rospy
from redhead.msg import Move #Route
from cost import Cost


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
        self.current_tag = None
        self.closest_tag = None
        self.cost = Cost(self.get_map())

    def get_map(self):
        pass

    def calculate_route(self, current_tag, destination_tag):
        route = self.cost.start(current_tag['id'], destination_tag['id'])
        return route

    def destination_tag_callback():
        self.destination_tag = 5

    def read_tag(self):
        apriltag = None
        with open(self.APRILTAG_SHARED, 'r') as f:
            with contextlib.closing(mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)) as m:
                m.seek(0)
                # print('\nreading...')
                apriltag_json = m.readline().decode("utf-8")
                print(apriltag_json)
                try:
                    apriltag = json.loads(apriltag_json)
                    if apriltag['uuid'] == self.last_tag_id:
                        apriltag = None
                except Exception as e:
                    print("exception %s"% str(e))
        return apriltag

    def set_closest_tag(self, apriltag):
        self.closest_tag = None
        for tag in apriltag['detections']:
            if self.MIN_RAGE_CM < tag['distance'] < self.MAX_RANGE_CM and tag['distance'] < closest_tag['distance']:
                self.closest_tag = tag
        return self.closest_tag

    def set_current_tag(self, apriltag):
        self.current_tag = None
        for tag in apriltag['detections']:
            if tag['distance'] < self.MIN_RANGE_CM :
                self.current_tag = tag
        return self.current_tag

    def set_target_tag(self, apriltag):
        if self.destination_tag is None:
            if self.current_tag is not None:
                self.target_tag = None
            elif self.closest_tag is not None:
                self.target_tag = self.closest_tag
        else:
            if self.current_tag is not None:
                route = self.calculate_route(start_tag, self.destination_tag)
                self.target_tag_id = route[0]['next_node']
            elif self.closest_tag is not None:
                route = self.calculate_route(start_tag, self.destination_tag)
                self.target_tag_id = route[0]['next_node']

            start_tag = self.current_tag if self.current_tag is not None else self.closest_tag
            if start_tag is not None:


        return self.target_tag

    def find_tag(self, apriltag, find_tag):
        for tag in apriltag['detections']:
            if tag['id'] == find_tag:
                return tag
        return None

    def navigate(self, target_tag):
        # little control loop
        pass

    def roam(self):
        # little control loop for manuveur and find
        pass

    def control_loop(self):
        with open(APRILTAG_SHARED, 'r') as f:
            with contextlib.closing(mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)) as m:
                while not rospy.is_shutdown():
                    apriltag = self.read_tag()
                    last_uuid = 0
                    if apriltag is not None and apriltag['uuid'] != last_uuid:
                        self.set_current_tag(apriltag)
                        self.set_closest_tag(apriltag)
                        self.set_target_tag(apriltag)
                        if self.target_tag is not None:
                            self.navigate(self.target_tag)
                        else:
                            self.roam()
                        # calculate route
                        next_tag = self.calculate_route(closest_tag, self.destination_tag)
                        # set next tag
                        
                        # center on tag
                        # go to tag

                        pass
                    else: # look around
                        self.roam()
                    time.sleep(.25)


if __name__=='__main__':
    a = AprilTag()
    a.control_loop()
