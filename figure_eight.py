from time import sleep

import robot

arlo = robot.Robot()


def circle(lpow, rpow, sleep_s, mdir=(1, 1)):
    # 64,32 takes approx 11.25 s
    print(arlo.go_diff(lpow, rpow, *mdir))
    sleep(sleep_s)
    print(arlo.stop())


if __name__ == "__main__":
    for i in range(3):
        circle(64, 33, 11.75)
        circle(32, 65, 11.5)
