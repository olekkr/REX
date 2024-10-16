import os
import sys
from enum import Enum
from time import sleep, time
from typing import Optional

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from command import Command
from particle import Particle

LOW_VARIANCE = 0.01
SEARCH_DEGREE = 30

class RobotState(Enum):
    following_path = 0
    is_checking = 1

class VarianceState(Enum):
    low_variance = 0
    high_variance = 1
    no_variance = 2


class StateRobot:
    def __init__(self, arlo, particles):
        self.state = RobotState.is_checking
        self.particles: list[Particle] = particles
        self.variance_state = VarianceState.no_variance
        self.variance = np.inf
        self.current_command: Optional[Command] = None
        self.arlo = arlo

    def set_variance(self):
        
        self.variance = np.var([p.getWeight() for p in self.particles])
        if self.variance <= LOW_VARIANCE:
            self.variance_state = VarianceState.low_variance
        else:
            self.variance_state = VarianceState.high_variance

    def check_variance(self):
        self.set_variance()
        if self.variance_state == VarianceState.low_variance:
            self.state = RobotState.following_path
        else:
            self.state = RobotState.is_checking

    def compute_next_action(self, dist, angle):
        if self.state == RobotState.following_path:
            if self.current_command is not None and self.current_command.finished is False:
                self.current_command.update_command_state()
            else:
                self.current_command = Command(self.arlo, dist, angle)
        elif self.state == RobotState.is_checking:
            if self.current_command is not None and self.current_command.finished is False:
                self.current_command.update_command_state()
            elif self.current_command is not None and self.current_command.finished is True:
                self.state = RobotState.following_path
                self.current_command = Command(self.arlo, dist, angle)
            else:
                self.current_command = Command(self.arlo, 0, np.deg2rad(SEARCH_DEGREE))

    def update(self, particles, dist, angle):
        self.particles = particles
        self.check_variance()
        self.compute_next_action(dist, angle)

