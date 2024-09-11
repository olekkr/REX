import robot
import time as t

arlo = robot.Robot()

print("Running")

i = 1
while i:
    #print(arlo.read_sensor(0))
    s1 = arlo.read_sensor(1)
    t.sleep(1)
    s2 = arlo.read_sensor(1)
    t.sleep(1)
    s3 = arlo.read_sensor(1)
    t.sleep(1)
    s4 = arlo.read_sensor(1)
    t.sleep(1)
    s5 = arlo.read_sensor(1)
    t.sleep(1)
    print((s1 + s2 + s3 + s4 + s5) / 5)
    i = 0


#       Front                 Right           Left                Back
# 10    130            10 cm, 120      10 cm, 114          10, 130
# 50    506            50 cm, 506      50 cm, 507          50, 517-518
# 100   1007          100,1015        100, 1015           100, 1025-1028
# 150   1485          150,1494        150, 1493           150, 1487
# 200   1985   200,1994        200, 1988           200, 1993
# 250   2472          250,2479        250, 2480           250, 2476
# 300   2971          300,2973        300, 2970           300, 2975 or 2608

# Lav gennemsnit