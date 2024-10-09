import sys
from time import sleep

import calculations
import robot
from constants import Constants

argvs = sys.argv

time_input = float(argvs[1]) if len(argvs) == 2 else None

arlo = robot.Robot()
# print(calculations.calc_travel_time(100, 64))


# power=64 = sleep_s = 3.2 = 1 m = dist = 1
def straight_move(dist: float=1, power=64, sleep_s=3.2):
    print(f"moving {dist}m")
    print(arlo.go_diff(power, power, 1, 1))
    sleep(sleep_s * abs(dist))
    print(arlo.stop())


# power=32 = sleep_s = 7.3 = 360 deg = frac = 4
# power=64 = sleep_s = 3.2 = 360 deg = frac = 4
def rotate_move(frac=0.25, power=32, sleep_360=7.3, mdir=(1, 0)):
    sleep_s = sleep_360 / 4
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
    mode = input("1. Rotate\n2. Move\n3. rotate_move def\n")
    power = int(input("power: "))
    if mode == "1":
        fracs = [4, 2, 1, 0.5, 0.25, 0.125]
        for frac in fracs:
            angle = float(input(f"time: "))
            rotate_move(frac=1, power=power, sleep_s=angle)
    elif mode == "2":
        for _ in range(10):
            time_sleep = float(input("time sleep: "))
            amnt = int(input("amnt: "))
            for _ in range(amnt):
                straight_move(1, power=power, sleep_s=time_sleep)
                sleep(1)
    else:
        fracs = [4, 2, 1, 0.5, 0.25, 0.125]
        for frac in fracs:
            rotate_move(frac=frac)
            input("continue: ")

# """
# straight_move()
# sleep(0.2)

# print("done")
# """
