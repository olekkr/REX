import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

from camera.webcam import Camera
from constants import Constants
from localplanning_rrt import grid_occ, robot_models
from localplanning_rrt.rrt import RRT
from refactor import local_planning

dimensions = Constants.PID.SCREEN_RESOLUTION

FocalLength = Constants.PID.FOCALLENGTH

markerHeight = 145.0  # mm
FPS = 5

picam2 = Camera()

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()


CamCx, CamCy = dimensions[0] / 2, dimensions[1] / 2
CameraMatrix = np.array(
    [
        [FocalLength, 0, CamCx / 2],
        [0, FocalLength, CamCy / 2],
        [0, 0, 1],
    ],
    dtype=float,
)
DistortionCoefficient = np.array([0, 0, 0, 0, 0])

print("using dimention:", dimensions)
print("using FocalLength:", FocalLength)


def init():
    # grid init
    return local_planning.empty_grid()


def  sense(grid):  # map,
    return sense_camera(grid)


def sense_camera(grid):
    # capture RGB:
    im = picam2.take_image()

    # capture AruCo Corners
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image=im, dictionary=arucoDict)

    if Constants.PID.ENABLE_PREVIEW:
        try:
            for corner in corners:
                picam2.preview(cv2.aruco.drawDetectedCornersCharuco(im, corner))
        except:
            pass

    # get Markers in camera coordinate system
    _rt, tv, _objs = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        markerHeight,
        CameraMatrix,
        DistortionCoefficient,
    )
    if tv is None:
        tv = []

    for t in tv:
        grid = local_planning.draw_landmarks(t, grid)

    return grid


if __name__ == "__main__":
    grid = init()
    path_res = 0.05

    map = grid_occ.GridOccupancyMap(low=(-1, 0), high=(1, 2))

    robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])  #
    start_time = time.time()
    while True:
        map.grid = sense(map.grid)

        rrt = RRT(
            start=np.array([0, 0]),
            goal=[0, 1.9],
            robot_model=robot,
            map=map,
            expand_dis=0.2,
            path_resolution=path_res,
        )

        if time.time() - start_time < 15:
            continue
        else:
            start_time = time.time()

        metadata = dict(title="RRT Test")
        fig = plt.figure()

        path = rrt.planning(animation=False, writer=None)

        if path is None:
            # print("Cannot find path")
            pass
        else:
            # print("found path!!")

            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], "-r")
            plt.grid(True)
            plt.ion()
            plt.pause(0.01)  # Need for Mac
            plt.show()
