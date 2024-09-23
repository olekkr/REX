from time import sleep
import robot


arlo = robot.Robot()

speed = 40
tspeed = 35
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
    if front > 300 and lefts > 300 and rights > 300:
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