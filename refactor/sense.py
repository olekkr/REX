import numpy as np 
import constants
import local_planning
from picamera2 import Picamera2
import cv2
import time
import matplotlib.pyplot as plt
import camera_setup

dimensions = (1920, 1080)

FocalLength = 2540

markerHeight = 145.0  # mm
FPS = 5

picam2 = camera_setup.camera_setup()

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()


CamCx, CamCy = dimensions[0]/2, dimensions[1]/2
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

def sense(grid): # map, 
    return sense_camera(grid)





def sense_camera(grid): 
    # capture RGB:
    im = picam2.capture_array("main")



    # capture AruCo Corners 
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image=im, dictionary=arucoDict)
    
    
    # get Markers in camera coordinate system
    _rt, tv, _objs = cv2.aruco.estimatePoseSingleMarkers(
        corners, markerHeight, CameraMatrix, DistortionCoefficient,
        )
    if tv is None:
        tv = []

    print(tv)
    for t in tv:
        local_planning.draw_landmarks(t, grid)

    return grid
    


if __name__ == "__main__":
    grid = init()
    print(grid)
    plt.ion()
    plt.show()
    shown = False
    while True:
        time.sleep(1)
        grid = sense(grid)
        # local_planning.show_grid(grid, (0.,0.))

