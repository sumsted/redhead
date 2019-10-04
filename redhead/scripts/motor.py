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
        assert (-100 <= move.left <= 100 and -100 <= move.right <= 100), "out of range"

        # transform
        left_direction = Robot.FORWARD if move.left > 100 else Robot.REVERSE if move.left < 100 else Robot.STOP
        left_speed = (abs(move.left) * Robot.MAX_SPEED) / 100 * Motor.THROTTLE
        right_direction = Robot.FORWARD if move.right > 100 else Robot.REVERSE if move.right < 100 else Robot.STOP
        right_speed = (abs(move.right) * Robot.MAX_SPEED) / 100 * Motor.THROTTLE

        # call robot
        self.log("ls: %d, rs: %d, ld: %d, rd: %d" % (left_speed, right_speed, left_direction, right_direction))
        self.robot.go(left_speed, right_speed, left_direction, right_direction)

    def command_loop(self):
        while not rospy.is_shutdown():
            pass #listening and triggering callback


if __name__=='__main__':
    motor = Motor()
    motor.command_loop()