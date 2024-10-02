import sys
from time import sleep

import calculations
import robot
from constants import Constants

argvs = sys.argv

time_input = float(argvs[1]) if len(argvs) == 2 else None

arlo = robot.Robot()
# print(calculations.calc_travel_time(100, 64))


def straight_move(dist=1):
    print(f"moving {dist}m")
    print(arlo.go_diff(64, 64, 1, 1))
    sleep(2.4 * abs(dist))
    print(arlo.stop())


def rotate_move(frac=0.25, power=64, sleep_s=0.725, mdir=(1, 0)):
    if frac == 0:
        return
    if frac < 0:
        mdir = mdir[1], mdir[0]
    if frac > 4 or frac <= -4:
        frac %= 4
    if frac > 2:
        frac -= 2
        frac = -frac
    elif frac < -2:
        frac += 2
        frac = -frac
    print(f"rotating {frac*90}")
    print(arlo.go_diff(power, power, *mdir))
    sleep(sleep_s * abs(frac))

    print(arlo.stop())


def linear_descent(start_v: int, dist=1, dir=1):
    for i in range(start_v, 30, -1):
        print(arlo.go_diff(64, 64, dir, dir))
        sleep(i * 1 / 64 * (time_input or 2.4) * dist)
        print(arlo.stop())


if __name__ == "__main__":
    fracs = [4, 2, 1, 0.5, 0.25, 0.125]
    power = int(input("power: "))
    for frac in fracs:
        angle = float(input(f"time: "))
        rotate_move(power=power, sleep_s=angle)
# """
# straight_move()
# sleep(0.2)

# print("done")
# """
