from time import sleep
import constants
import calculations
import robot
import sys


arlo = robot.Robot()

speed = 50
can_drive = True

def move():
    print("running straight")
    print(arlo.go_diff(speed,speed,1,1))

def left():
    print("running left")
    print(arlo.go_diff(speed//2, speed//2, 0, 1))

def right():
    print("running right")
    print(arlo.go_diff(speed//2, speed//2, 1, 0))


while 1:
    front = arlo.read_sensor(0)
    left = arlo.read_sensor(2)
    right = arlo.read_sensor(3)

    if front < 300:
        can_drive = True
    else:
        can_drive = False

    if can_drive:
        move()