import numpy as np 
import constants
import local_planning
from picamera2 import Picamera2
import cv2

dimensions = (1920, 1080)

FocalLength = 2540

markerHeight = 145.0  # mm
FPS = 5

picam2 = None


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
    global picam2
    # aruco init
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    arucoParams = cv2.aruco.DetectorParameters()


    # pi cam init
    frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds
    picam2 = Picamera2()
    picam2_config = picam2.create_video_configuration(
        {"size": dimensions, "format": "RGB888"},
        controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
        queue=False,
    )
    picam2.configure(picam2_config)  # Not really necessary
    picam2.start(show_preview=False)
    time.sleep(2)
    picam2.start()

    # grid init
    return local_planning.empty_grid()

def sense(grid): # map, 
    sense_camera(grid)





def sense_camera(grid): 
    # capture RGB:
    im = picam2.capture_array("main")

    # capture AruCo Corners 
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image=im, dictionary=arucoDict)
    
    # get Markers in camera coordinate system
    _rt, tv, _objs = cv2.aruco.estimatePoseSingleMarkers(
        corners, markerHeight, CameraMatrix, DistortionCoefficient,
        )
    
    for t in tv:
        draw_landmarks(t, grid)

    


import time
if __name__ == "__main__":
    grid = init()
    while True:
        time.sleep(1)
        sense(grid)
