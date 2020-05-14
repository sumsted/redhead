#!/usr/bin/env python
""" motor - node to listen for direction commands
Taking in left right Move message -100 to 100.
Transform to steps for motor controller.
Call out to adafruit interface.
"""
if False:
    print("remote debugging wait")
    import ptvsd
    ptvsd.enable_attach(address=('0.0.0.0', 3000))
    ptvsd.wait_for_attach()

import rospy
from robotrc import Robot
from redhead.msg import MoveRC


class Motor():
    FORWARD = 1
    STOP = 0
    REVERSE = -1
    THROTTLE = .5    
    ROS_NODE = "motor"
    ROS_MOVE_CHANNEL = "motor_movement"

    TILLER_RANGE = (0,1,2)
    DIRECTION_RANGE = (0,1,2)
    VELOCITY_RANGE = (0, 150)
    TILLER_VELOCITY = 150    

    def log(self, s):
        print(s)
        rospy.loginfo(s)

    def __init__(self):
        self.log("Initializing...")
        rospy.init_node(Motor.ROS_NODE, anonymous=True)
        rospy.Subscriber(Motor.ROS_MOVE_CHANNEL, MoveRC, self.move_callback)
        self.robot = Robot()

    def move_callback(self, move):
        # check range
        assert (Motor.VELOCITY_RANGE[0] <= move.velocity <= Motor.VELOCITY_RANGE[1]), "velocity out of range"
        assert (move.tiller in Motor.TILLER_RANGE), "tiller out of range"
        assert (move.direction in Motor.DIRECTION_RANGE), "direction out of range"

        self.robot.go(
            move.velocity,
            move.direction,
            Motor.TILLER_VELOCITY,
            move.tiller
        )

    def command_loop(self):
        while not rospy.is_shutdown():
            pass #listening and triggering callback


if __name__=='__main__':
    motor = Motor()
    motor.command_loop()