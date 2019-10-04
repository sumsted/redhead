#!/usr/bin/env/python
""" motor - node to listen for direction commands
Taking in left right Move message -100 to 100.
Transform to steps for motor controller.
Call out to adafruit interface.
"""

import rospy
from robot import Robot
from redhead.msg import Move


class Motor():
    FORWARD = 1
    STOP = 0
    REVERSE = -1
    THROTTLE = .5
    STEPS = {
        100: {'speed': 250, 'direction':Motor.FORWARD},
        80: {'speed': 200, 'direction':Motor.FORWARD},
        60: {'speed': 150, 'direction':Motor.FORWARD},
        40: {'speed': 100, 'direction':Motor.FORWARD},
        20: {'speed': 50, 'direction':Motor.FORWARD},
        0: {'speed': 0, 'direction':Motor.STOP},
        -20: {'speed': 50, 'direction':Motor.REVERSE},
        -40: {'speed': 100, 'direction':Motor.REVERSE},
        -60: {'speed': 150, 'direction':Motor.REVERSE},
        -80: {'speed': 200, 'direction':Motor.REVERSE},
        -100: {'speed': 250, 'direction':Motor.REVERSE}
    }
    
    ROS_NODE = "teleop"
    ROS_MOVE_CHANNEL = "motor_movement"

    def log(self, s):
        print(s)
        rospy.loginfo(s)

    def __init__(self):
        self.log("Initializing...")
        rospy.init_node(Motor.ROS_NODE, anonymous=True)
        rospy.Subscriber(Motor.ROS_MOVE_CHANNEL, Move, self.move_callback)
        self.robot = Robot()

    def move_callback(self, move):
        # check range

        # transform

        # call robot


    def command_loop(self):
        while not rospy.is_shutdown():
            pass


if __name__=='__main__':
    motor = Motor()
    motor.command_loop()