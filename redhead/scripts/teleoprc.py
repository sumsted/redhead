#!/usr/bin/env python
""" teleop - user controlled robot commands
Through messages sent udp pull left and right motor directions.
Range -100 to 100 for left and right.
"""

# todo: Throttle messages for slow python motor node callback. Arduino c motor node can handle it :S.

import socket
import rospy
from redhead.msg import MoveRC, Ultra, Mpu


def log(s):
    print(s)
    rospy.loginfo(s)


class Teleop():
    # MoveRC Values
    LEFT = 1
    RIGHT = 2
    MIDDLE = 0 
    FORWARD = 1
    REVERSE = 2
    NEUTRAL = 0
    STOP = 0
    

    UDP_ADDRESS = "0.0.0.0"
    UDP_PORT = 8484
    ROS_NODE = "teleop"
    ROS_MOVE_CHANNEL = "motor_movement"
    ROS_MOVE_CHANNEL_QUEUE = 10
    ROS_ULTRASONIC_CHANNEL = "ultrasonic"
    ROS_MPU_CHANNEL = "mpu"
    MINIMUM_SAFE_DISTANCE = 40

    def __init__(self):
        self.log("Initializing...")
        rospy.init_node(Teleop.ROS_NODE, anonymous=True)
        self.move_pub = rospy.Publisher(Teleop.ROS_MOVE_CHANNEL, MoveRC, queue_size=Teleop.ROS_MOVE_CHANNEL_QUEUE)
        # rospy.Subscriber(Teleop.ROS_ULTRASONIC_CHANNEL, Ultra, self.ultrasonic_callback)
        # rospy.Subscriber(Teleop.ROS_MPU_CHANNEL, Mpu, self.mpu_callback)
        self.rate = rospy.Rate(5)

        self.move_msg = MoveRC()
        self.move_msg.tiller = Teleop.MIDDLE
        self.move_msg.direction = Teleop.NEUTRAL
        self.move_msg.velocity = Teleop.STOP

        self.command = "CONTROL"
        self.left = 0
        self.right = 0
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((Teleop.UDP_ADDRESS, Teleop.UDP_PORT))
        log("Listening on %s:%d" % (Teleop.UDP_ADDRESS, Teleop.UDP_PORT))
        self.ultra_left = 999
        self.ultra_low = 999
        self.ultra_right = 999
        self.ultra_front = 999
        self.ultra_rear = 999
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

    def log(self,s):
        print(s)
        rospy.loginfo(s)

    def mpu_callback(self, mpu):
        self.yaw = mpu.yaw
        self.pitch = mpu.pitch
        self.roll = mpu.roll

    def ultrasonic_callback(self, ultra):
        self.ultra_left = ultra.left
        self.ultra_low = ultra.low
        self.ultra_right = ultra.right
        self.ultra_front = ultra.front
        self.ultra_rear = ultra.rear

    def parse_udp_data(self, data):
        self.command = "CONTROL"
        if len(data) == 8:
            self.left = int(data[:4])
            self.right = int(data[4:])
        else:
            self.left = 0
            self.right = 0

    def safe(self):
        forward = True if (self.left + self.right) > 0 else False
        if forward:
            if self.ultra_front > 20 and self.ultra_low > 5:
                return True
            else:
                return False
        else:
            if self.ultra_rear > 20:
                return True
            else:
                return False

    def command_loop(self):
        while not rospy.is_shutdown():
            data, address = self.udp_socket.recvfrom(1024)
            self.parse_udp_data(data)
            if self.command == "CONTROL":
                if self.safe():
                    self.move_msg.tiller = self.tiller
                    self.move_msg.direction = self.direction
                    self.move_msg.velocity = self.velocity
                    self.move_pub.publish(self.move_msg)
                    # self.rate.sleep()
                    log("CONTROL: %d, %d, %d" % (self.move_msg.tiller, self.move_msg.direction, self.move_msg.velocity))
                else:
                    log("CONTROL: HALT TOO CLOSE!")
            else:
                pass # unknown command

if __name__ == '__main__':
    try:
        teleop = Teleop()
        teleop.command_loop()
    except rospy.ROSInterruptException as e:
        log(e)
