import os
import sys
from enum import Enum
from time import sleep, time
from typing import Optional

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from robot import Robot


class RobotState(Enum):
    following_path = 0
    is_lost = 1
    is_checking = 2


class StateRobot:
    def __init__(self, state, arlo, move_particle_func, particles):
        self.state = state
        self.move_particle_func = move_particle_func
        self.particles = particles

        self.current_path = []
        self.current_instructions = []
        self.current_instruction: Optional[tuple[float, float]] = None
        self.current_path_idx = 0
        self.robot_angle = np.pi / 2
        self.robot_position = (0, 0)
        self.robot_velocity = 0
        self.robot_angular_velocity = 0
        self.arlo = arlo
        self.move_start_time: Optional[float] = None
        self.move_stop_time: Optional[float] = None
        self.amnt_instruction_run = 3

        self.instructions = []

    def setCurrentPath(self, path):
        self.current_path = path
        self.current_path_idx = 0
        self.current_instructions = self.gen_instructions()

    def setAngle(self, angle):
        self.robot_angle = angle

    def setPos(self, pos):
        self.robot_position = pos

    def moveParticles(self):
        for parti in self.particles:
            theta = parti.getTheta()
            # unit vector pointing in the direction of the particle
            heading = np.array([np.cos(theta), np.sin(theta)])
            # scale with velocity
            deltaXY = heading * self.robot_velocity
            # do the update
            self.move_particle_func(parti, deltaXY[0], deltaXY[1], self.robot_angular_velocity)

    def run_loop(self):
        self.moveParticles()

        if self.move_stop_time is None:
            self.current_instruction = self.instructions[self.current_path_idx]
        else:
            self.ro
        if self.move_stop_time and time() >= self.move_stop_time:
            self.stop()
            self.current_path_idx += 1
            if (
                self.current_path_idx >= len(self.instructions)
                or self.current_path_idx >= self.amnt_instruction_run
            ):
                print("End of path")
                return True
            self.current_instruction = self.instructions[self.current_path_idx]
        if not self.current_path:
            return True
        return False

    @staticmethod
    def get_straight_p64_cm_s_velocity():
        sleep_1 = 3.2
        return 100 / sleep_1

    def straight_p64_move(self, time_s: float):
        print(self.arlo.go_diff(64, 64, 1, 1))
        self.move_start_time = time()
        self.move_stop_time = self.move_start_time + time_s
        self.robot_velocity = self.get_straight_p64_cm_s_velocity()

    def stop(self):
        print(self.arlo.stop())
        self.move_start_time = None
        self.move_stop_time = None
        self.robot_velocity = 0
        self.robot_angular_velocity = 0

    @staticmethod
    def get_rotate_p32_rad_s_velocity():
        sleep_360 = 7.3
        return np.deg2rad(360 / sleep_360)

    def rotate_p32(self, time_s, reverse: bool = False, mdir=(1, 0)):
        power = 32
        if reverse:
            mdir = (0, 1)
        print(self.arlo.go_diff(power, power, *mdir))
        self.move_start_time = time()
        self.move_stop_time = self.move_start_time + time_s
        self.robot_angular_velocity = self.get_rotate_p32_rad_s_velocity()

    def gen_instructions(self):
        path_w_angles = [(*self.robot_position, self.robot_angle)]
        angles_and_dist = []
        for x, y in self.current_path[1:]:
            current_x, current_y, current_angle = path_w_angles[-1]
            # current_x, current_y = center_points_to_robot((current_x, current_y), current_angle)
            vec = np.array([x, y]) - np.array([current_x, current_y])
            target_angle = np.arctan2(vec[1], vec[0]) % (2 * np.pi)

            turn_angle = current_angle - target_angle
            path_w_angles.append((x, y, target_angle))
            angles_and_dist.append(
                (
                    turn_angle,
                    np.linalg.norm(np.array([current_x, current_y]) - np.array([x, y])),
                )
            )
        return angles_and_dist
