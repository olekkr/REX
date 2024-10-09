import os
import time

import numpy as np

import robot

if "PICAM" in os.environ:
    PICAM = True
else:
    PICAM = False

IS_ARLO = True


def get_straight_p64_cm_s_velocity():
    time_1m = 3.2
    return 100 / time_1m


def get_rotate_p32_rad_s_velocity():
    time_360deg = 7.3
    return np.deg2rad(360 / time_360deg)


ROTATIONAL_SPEED = get_rotate_p32_rad_s_velocity()
FORWARD_SPEED = get_straight_p64_cm_s_velocity()


class command:
    def __init__(self, robot, distance, angle):
        self.robot = controlWrapper(robot, IS_ARLO)
        self.distance = distance
        self.angle = angle

        self.startTime = None  # None means not started

        self.rotationTime = self.angle / ROTATIONAL_SPEED
        self.forwardTime = self.distance / FORWARD_SPEED
        print("NOT implemented")

    # checks and updates controls on robot based on timestep
    # returns true if command has finished execution ... false if it has not.
    def update_command_state(self):
        def rotation_command():
            self.rotation_speed = ROTATIONAL_SPEED
            self.velocity = 0
            if self.angle > 0:
                self.robot.diff(1, 0, 32, 32)
            else:
                self.robot.diff(0, 1, 32, 32)
        
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
            self.velocity = ROTATIONAL_SPEED
            self.robot.diff(1, 1, 64, 64)
            return False

        # has finished rotation and forward
        else:
            self.rotation_speed = 0
            self.velocity = 0
            self.robot.diff(1, 1, 0, 0)

            return True
        # True means finished
    


# wraps robot for the purpose of interchangability with debug/Arlo
class controlWrapper:
    def __init__(self, robot, isArlo=False):
        self.robot = robot
        self.isArlo = isArlo

    ## TODO: do this for all the commands diff etc...
    def diff(self, l, r, L, R):
        if self.isArlo:
            self.robot.diff(l, r, L, R)
        else:
            print(f"executing command diff({l, r, L, R}).")
