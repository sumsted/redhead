

SPEED_STEP = 50
FORWARD = 1
BACKWARD = 2

def go(_current_speed, drive_speed, drive_direction):
    print('start')
    drive_speed = drive_speed if drive_direction == FORWARD else -drive_speed
    print(_current_speed,drive_speed, abs(_current_speed), 1 if _current_speed > 0 else 2, abs(drive_speed), 1 if drive_speed > 0 else 2)
    delta = abs(_current_speed - drive_speed)
    while delta > SPEED_STEP:
        if _current_speed < drive_speed:
            _current_speed += 50
        else:
            _current_speed -= 50
        delta = abs(_current_speed - drive_speed)
        print(_current_speed,drive_speed, abs(_current_speed), 1 if _current_speed > 0 else 2, abs(drive_speed), 1 if drive_speed > 0 else 2)
    _current_speed = drive_speed
    print(_current_speed,drive_speed, abs(_current_speed), 1 if _current_speed > 0 else 2, abs(drive_speed), 1 if drive_speed > 0 else 2)


if __name__=="__main__":
    go(-50, 25, 2)
    go(-50, 100, 2)
    go(-50, 0, 2)
    go(-50, 25, 1)
    go(-50, 100, 1)

    go(0, 25, 2)
    go(0, 100, 2)
    go(0, 0, 2)
    go(0, 25, 1)
    go(0, 100, 1)

    go(50, 25, 2)
    go(50, 100, 2)
    go(50, 0, 2)
    go(50, 25, 1)
    go(50, 100, 1)
