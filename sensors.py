import robot
import time as t

arlo = robot.Robot()

print("Running")

while 1:
    #print(arlo.read_sensor(0))
    print("R")
    print(arlo.read_sensor(1))
    t.sleep(1)


# Front                 Right           Left
# 10 cm, 130            10 cm, 120      10 cm, 114
# 50 cm, 506            50 cm, 506      50 cm, 507
# 100 cm, 1007          100,1015        100, 1015      
# 150 cm, 1485          150,1494        150, 1493
# 200 cm, 1985 - 2000   200,1994        200, 1988
# 250 cm, 2472          250,2479        250, 2480
# 300 cm, 2971          300,2973        300, 2970

