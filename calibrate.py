from time import sleep

import robot

arlo = robot.Robot()

print("running")
print(arlo.godiff(64,64,1,1))
sleep(3)
print(arlo.stop())
