import robot
import time as t

arlo = robot.Robot()

print("Running")

while 1:
    #print(arlo.read_sensor(0))
    print("R")
    print(arlo.read_right_ping_sensor())
    #print(arlo.read_left_ping_sensor())
    t.sleep(1)
    print("L")
    print(arlo.read_left_ping_sensor())
    t.sleep(1)


# Front
# 10 cm, 130
# 50 cm, 506
# 100 cm, 1007
# 150 cm, 1485
# 200 cm, 1985 - 2000
# 250 cm, 2472
# 300 cm, 2971

