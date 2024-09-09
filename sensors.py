import robot
import time as t

arlo = robot.Robot()

print("Running")

while 1:
    print(arlo.read_sensor(0))
    t.sleep(0.2)