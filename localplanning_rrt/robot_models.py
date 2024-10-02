"""
some simple first-order robot dynamics models
"""
import math
import numpy as np
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from constants import Constants

class RobotModel:

    def __init__(self, ctrl_range) -> None:
        #record the range of action that can be used to integrate the state
        self.ctrl_range = ctrl_range
        return

    def forward_dyn(self, x, u, T):
        path = [x]
        for i in range(T):
            v, w = u[i]

            x_dot = v * np.cos(w)
            y_dot = v * np.sin(w)

            x_new = path[-1] + np.array([x_dot, y_dot])
            path.append(x_new)

        return path[1:]

    def inverse_dyn(self, x, x_goal, T):
        u = []
        for i in range(T):
            dx = x_goal - x
            if np.linalg.norm(dx) > 1e-3:
                v = self.ctrl_range[1]
                w = 0
            else:
                v = self.ctrl_range[1]
                w = 2 * self.ctrl_range[1] / Constants.Robot.RADIUS

            u.append((v, w))
            x = self.forward_dyn(x, [(v, w)], 1)[-1]

        return self.forward_dyn(x, u, 1)

class PointMassModel(RobotModel):
    #Note Arlo is differential driven and may be simpler to avoid Dubins car model by rotating in-place to direct and executing piecewise straight path  
    def forward_dyn(self, x, u, T):
        path = [x]
        #note u must have T ctrl to apply
        for i in range(T):
            x_new = path[-1] + u[i] #u is velocity command here
            path.append(x_new)    

        return path[1:]

    def inverse_dyn(self, x, x_goal, T):
        #for point mass, the path is just a straight line by taking full ctrl_range at each step
        dir = (x_goal-x)/np.linalg.norm(x_goal-x)

        u = np.array([dir*self.ctrl_range[1] for _ in range(T)])

        return self.forward_dyn(x, u, T)
