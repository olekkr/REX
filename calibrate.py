import sys
from time import sleep

import calculations
import constants
import robot

argvs = sys.argv

time_input = float(argvs[1]) if len(argvs) == 2 else None

arlo = robot.Robot()
print(calculations.calc_travel_time(100, 64))


def straight_move(dist=1):
    print("running 1m straight")
    print(arlo.go_diff(64, 64, 1, 1))
    sleep(2.4 * dist)
    print(arlo.stop())


def rotate_move(frac=0.25, power=64, sleep_s=0.725, mdir=(1, 0)):
    print("90 deg rotation test")
    print(arlo.go_diff(power, power, *mdir))
    sleep(sleep_s)
    # sleep(3.5*frac)
    print(arlo.stop())


if __name__ == "__main__":
    for i in range(4):
        # linear_descent(64, 1, 1)
        # linear_descent(64, 1, 0)
        # straight_move()
        # sleep(0.2)
        rotate_move(sleep_s=0.725)
        sleep(2)
"""
straight_move()
sleep(0.2)

print("done")
"""
