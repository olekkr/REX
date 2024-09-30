import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

from camera.picam import Camera
from constants import Constants
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


def sense(grid):  # map,
    return sense_camera(grid)


def sense_camera(grid):
    # capture RGB:
    im = picam2.take_image(enable_preview=False)

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
    robo_pos = (0, 0)

    plt.ion()  # Makes changes to
    axes = plt.gca()
    axes.invert_yaxis()
    local_planning.show_grid(grid, robo_pos, axes)
    start = time.time()
    while True:
        if not plt.get_fignums():
            print("plot closed, exiting...")
            exit()
        grid = sense(grid)
        if time.time() - start > 0.1:
            plt.pause(0.0001)
            local_planning.show_grid(grid, robo_pos, axes)
            start = time.time()
