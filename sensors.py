import robot
import time as t

arlo = robot.Robot()

print("Running")

while 1:
    #print(arlo.read_sensor(0))
    print(arlo.read_front_ping_sensor())
    t.sleep(0.2)