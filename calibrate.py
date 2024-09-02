from time import sleep

import robot

arlo = robot.Robot()

print("running")
print(arlo.go_diff(64,64,1,1))
sleep(2.5)
print(arlo.stop())
