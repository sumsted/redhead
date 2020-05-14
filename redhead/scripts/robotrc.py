""" robot - adafruit motorhat interface
Altered from Adafruit library. 
Original....
 Author: Tony DiCola
 License: MIT License https://opensource.org/licenses/MIT
"""

import time
import atexit

from Adafruit_MotorHAT import Adafruit_MotorHAT


class Robot(object):
    MAX_SPEED = 150
    FORWARD = Adafruit_MotorHAT.FORWARD
    STOP = Adafruit_MotorHAT.RELEASE
    BACKWARD = Adafruit_MotorHAT.BACKWARD
    SPEED_STEP = 50
    DELAY = .5

    def __init__(self, addr=0x60, tiller_id=1, drive_id=2, tiller_trim=0, drive_trim=0,
                 stop_at_exit=True):
        """Create an instance of the robot.  Can specify the following optional
        parameters:
         - addr: The I2C address of the motor HAT, default is 0x60.
         - tiller_id: The ID of the tiller motor, default is 1.
         - drive_id: The ID of the drive motor, default is 2.
         - tiller_trim: Amount to offset the speed of the tiller motor, can be positive
                      or negative and use useful for matching the speed of both
                      motors.  Default is 0.
         - drive_trim: Amount to offset the speed of the drive motor (see above).
         - stop_at_exit: Boolean to indicate if the motors should stop on program
                         exit.  Default is True (highly recommended to keep this
                         value to prevent damage to the bot on program crash!).
        """
        self._mh = Adafruit_MotorHAT(addr)
        self._tiller = self._mh.getMotor(tiller_id)
        self._drive = self._mh.getMotor(drive_id)
        self._tiller_trim = tiller_trim
        self._drive_trim = drive_trim
        # Start with motors turned off.
        self._tiller.run(Adafruit_MotorHAT.RELEASE)
        self._drive.run(Adafruit_MotorHAT.RELEASE)
        # Configure all motors to stop at program exit if desired.
        if stop_at_exit:
            atexit.register(self.stop)
        self._current_speed = 0
        self._current_direction = 0

    def go(self, drive_speed, drive_direction, tiller_speed, tiller_direction):
        tiller_speed = max(0,min(Robot.MAX_SPEED,tiller_speed))
        self._tiller.setSpeed(tiller_speed)
        self._tiller.run(tiller_direction)
        current_speed = self._current_speed if self._current_direction == Robot.FORWARD else -self._current_speed
        drive_speed = max(0,min(Robot.MAX_SPEED,drive_speed))
        drive_speed = drive_speed if drive_direction == Robot.FORWARD else -drive_speed
        
        delta = abs(current_speed - drive_speed)
        while delta > Robot.SPEED_STEP:
            if current_speed < drive_speed:
                current_speed += Robot.SPEED_STEP
            else:
                current_speed -= Robot.SPEED_STEP
            delta = abs(current_speed - drive_speed)
            self.drive.setSpeed(abs(current_speed))
            self.drive.run(Robot.FORWARD if current_speed > 0 else Robot.BACKWARD)
            time.sleep(Robot.DELAY)
        current_speed = drive_speed
        self._current_speed = abs(current_speed)
        self._current_direction = Robot.FORWARD if current_speed > 0 else Robot.BACKWARD
        self._drive.setSpeed(self._current_speed)
        self._drive.run(self._current_direction)

    def _tiller_speed(self, speed):
        """Set the speed of the tiller, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._tiller_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._tiller.setSpeed(speed)

    def _drive_speed(self, speed):
        """Set the speed of the drive motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._drive_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._drive.setSpeed(speed)

    def stop(self):
        """Stop all movement."""
        self._tiller.run(Adafruit_MotorHAT.RELEASE)
        self._drive.run(Adafruit_MotorHAT.RELEASE)

    def forward(self, speed, seconds=None):
        """Move forward at the specified speed (0-255).  Will start moving
        forward and return unless a seconds value is specified, in which
        case the robot will move forward for that amount of time and then stop.
        """
        # Set motor speed and move both forward.
        self._drive_speed(speed)
        self._drive.run(Adafruit_MotorHAT.FORWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def backward(self, speed, seconds=None):
        """Move backward at the specified speed (0-255).  Will start moving
        backward and return unless a seconds value is specified, in which
        case the robot will move backward for that amount of time and then stop.
        """
        # Set motor speed and move both backward.
        self._drive_speed(speed)
        self._drive.run(Adafruit_MotorHAT.BACKWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def right(self, speed, direction, seconds=None):
        """Turn the tiller to the right.
        """
        # Set motor speed and move both forward.
        self._tiller_speed(speed)
        self._tiller.run(direction)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def straight(self):
        """Turn the tiller to the middle.
        """
        self._tiller_speed(0)
        self._tiller.run(Adafruit_MotorHAT.RELEASE)
        
    def left(self, speed, direction, seconds=None):
        """Turn the tiller to the left.
        """
        # Set motor speed and move both forward.
        self._tiller_speed(speed)
        self._tiller.run(direction)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()
