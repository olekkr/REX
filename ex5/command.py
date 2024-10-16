import os
import time

import numpy as np

if "PICAM" in os.environ:
    import os
    import sys

    sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
    import robot

    IS_ARLO = True
else:
    IS_ARLO = False


def get_straight_p64_cm_s_velocity():
    time_1m = 2.5
    return 100 / time_1m


def get_rotate_p32_rad_s_velocity():
    time_360deg = 7.3
    return np.deg2rad(360 / time_360deg)


ROTATIONAL_SPEED = get_rotate_p32_rad_s_velocity()
FORWARD_SPEED = get_straight_p64_cm_s_velocity()


class Command:
    def __init__(self, robot, distance, angle):
        self.robot = ControlWrapper(robot, IS_ARLO)
        self.distance = distance
        if angle > 2*np.pi:
            self.angle = 2*np.pi - angle 
        else:
            self.angle = angle

        self.startTime = None  # None means not started

        self.rotationTime = self.angle / ROTATIONAL_SPEED
        self.forwardTime = self.distance / FORWARD_SPEED
        self.graceTime = 0.5
        self.finished = False
        print(f"{np.rad2deg(self.angle)} degrees, {self.distance} cm")

    # checks and updates controls on robot based on timestep
    # returns true if command has finished execution ... false if it has not.
    def update_command_state(self):
        def rotation_command():
            # if self.rotationTime - (time.time() - self.startTime) < self.graceTime:
            #     # the case when rotation has not finished but within grace time 
            #     # self.rotationTime = 0
            #     self.rotationTime = time.time() - self.startTime
            #     # redo the update now with 
            #     return self.update_command_state()
            self.velocity = 0
            if self.angle > 0:
                self.robot.go_diff(32, 32, 1, 0)
                self.rotation_speed = ROTATIONAL_SPEED
            else:
                self.robot.go_diff(32, 32, 0, 1)
                self.rotation_speed = -ROTATIONAL_SPEED

        # has not started yet
        if self.startTime is None:
            self.startTime = time.time()
            rotation_command()
            return False

        # has not finished rotation
        elif time.time() - self.startTime < self.rotationTime:
            rotation_command()
            return False
        # has not finished forward
        elif time.time() - self.startTime < self.rotationTime + self.forwardTime:
            self.rotation_speed = 0
            self.velocity = FORWARD_SPEED
            self.robot.go_diff(64, 64, 1, 1)
            return False

        # has finished rotation and forward
        else:
            self.rotation_speed = 0
            self.velocity = 0
            self.robot.go_diff(0, 0, 1, 1)
            self.finished = True
            return True
        # True means finished


# wraps robot for the purpose of interchangability with debug/Arlo
class ControlWrapper:
    def __init__(self, robot, isArlo=False):
        self.robot = robot
        self.isArlo = isArlo

    def go_diff(self, l, r, L, R):
        if self.isArlo:
            self.robot.go_diff(l, r, L, R)
        else:
            print(f"executing command diff({l, r, L, R}).")


# testing code
if __name__ == "__main__":
    # arlo = None
    arlo = robot.Robot()
    c1 = Command(arlo, 100, 3.1415 / 2)
    while not c1.update_command_state():
        time.sleep(0.1)
        pass

    # c2 = command(arlo, 1, -3.1415 / 2)
    # while not c2.update_command_state():
    #     pass
    #
    # c3 = command(arlo, 1, 3.1415)
    # while not c3.update_command_state():
    #     pass
