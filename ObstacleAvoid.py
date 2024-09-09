from time import sleep
import constants
import calculations
import robot
import sys


arlo = robot.Robot()

speed = 50
tspeed = 40
can_drive = True

def move():
    print("running straight")
    print(arlo.go_diff(speed, speed, 1, 1))

def left():
    print("running left")
    print(arlo.go_diff(tspeed, tspeed , 0, 1))

def right():
    print("running right")
    print(arlo.go_diff(tspeed, tspeed, 1, 0))


while 1:
    front = arlo.read_sensor(0)
    lefts = arlo.read_sensor(2)
    rights = arlo.read_sensor(3)

    print("Front: ", front)
    if front > 300:
        can_drive = True
    else:
        print("Stopping")
        can_drive = False

    if can_drive:
        move()
    else:
        if lefts > rights:
            left()
        else:
            right()