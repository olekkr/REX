import time
import constants
import detection

def STRAIGT(arlo):
    print("FOUND TARGET: MOVING STRAIGHT")
    arlo.go_diff(constants.MIN_SPEED, constants.MIN_SPEED, 1, 1)

def TURN_LEFT(arlo):
    print("DETECTING: TURNING LEFT")
    arlo.go_diff(constants.MIN_SPEED, constants.MIN_SPEED , 0, 1)
    time.sleep(constants.FINDING_SLEEP)
    arlo.stop()

def TURN_RIGHT(arlo):
    print("DETECTING: TURNING RIGHT")
    arlo.go_diff(constants.MIN_SPEED, constants.MIN_SPEED, 1, 0)
    time.sleep(constants.FINDING_SLEEP)
    arlo.stop()

def TURN_BACK(arlo):
    print("OBJECT DETECTED: TURNING BACK")
    arlo.go_diff(constants.MIN_SPEED, constants.MIN_SPEED, 0, 0)

def MOVE_LEFT(arlo):
    print("FOUND TARGET: MOVING LEFT")
    arlo.go_diff(constants.MIN_SPEED, constants.HALF_SPEED, 0, 1)

def MOVE_RIGHT(arlo):
    print("FOUND TARGET: MOVING RIGHT")
    arlo.go_diff(constants.MIN_SPEED, constants.HALF_SPEED, 1, 0)

def FIND_TARGET(arlo, last_seen):
    if last_seen is not None:
        if last_seen == "left":
            TURN_LEFT(arlo)
        else:
            TURN_RIGHT(arlo)
    else:
        TURN_RIGHT(arlo)

def AVOID_OBSTACLE(arlo):
    arlo.stop()
    time.sleep(0.5)
    print("OBJECT DETECTED: TURNING LEFT")
    arlo.go_diff(constants.MIN_SPEED, constants.MIN_SPEED, 0, 1)




def TEST_TOWARDS_TARGET(corners, arlo, last_seen):
    x1, x2 = detection.TOP_LEFT_CORNER(corners)[0], detection.TOP_RIGHT_CORNER(corners)[0]
    center = constants.SCREEN_RESOLUTION[0] / 2
    x_center = detection.CENTER(x1, x2)

    if x_center > center - constants.THRESHOLD and x_center < center + constants.THRESHOLD:
        STRAIGT(arlo)
    else:
        if x_center > center - constants.THRESHOLD:
            MOVE_RIGHT(arlo)
            last_seen = "left"
        else:
            MOVE_LEFT(arlo)
            last_seen = "right"

def EIGHT(arlo, n):
    for _ in range(n):
        for i in range(4):
            print("moving straight")
            time.straight_move()
            print("waiting to rotate")
            time.sleep(2)
            print("rotating")
            print(arlo.rotate_move(sleep_s=0.725,mdir=(0,1)))
            time.sleep(2)
        time.sleep(1)

# Test functions
def TEST_STRAIGHT():
    print("FOUND TARGET: MOVING STRAIGHT")

def TEST_TURN_LEFT():
    print("DETECTING: TURNING LEFT")

def TEST_TURN_RIGHT():
    print("DETECTING: TURNING RIGHT")

def TEST_TURN_BACK():
    print("OBJECT DETECTED: TURNING BACK")

def TEST_MOVE_LEFT():
    print("FOUND TARGET: MOVING LEFT")

def TEST_MOVE_RIGHT():
    print("FOUND TARGET: MOVING RIGHT")

def TEST_FIND_TARGET(last_seen):
    if last_seen is not None:
        if last_seen == "left":
            print("DETECTING: TURNING LEFT")
        else:
            print("DETECTING: TURNING RIGHT")
    else:
        print("DETECTING: TURNING RIGHT")

def TEST_TOWARDS_TARGET(corners, last_seen):
    x1, x2 = detection.TOP_LEFT_CORNER(corners)[0], detection.TOP_RIGHT_CORNER(corners)[0]
    center = constants.SCREEN_RESOLUTION[0] / 2
    x_center = detection.CENTER(x1, x2)

    if x_center > center - constants.THRESHOLD and x_center < center + constants.THRESHOLD:
        TEST_STRAIGHT()
    else:
        if x_center > center - constants.THRESHOLD:
            TEST_MOVE_RIGHT()
            last_seen = "left"
        else:
            TEST_MOVE_LEFT()
            last_seen = "right"
    
def TEST_AVOID_OBSTACLE():
    print("OBJECT DETECTED: TURNING LEFT")