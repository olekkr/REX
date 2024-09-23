import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from picamera2 import Picamera2

import robot
from calibrate_camera_constants import CameraMatrix, DistortionCoefficient, markerHeight

arlo = robot.Robot()

speed = 40
tspeed_slow = 32
tspeed = 32
aruco = False
image = "aruco.png"
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
preview_downscale = 2
imageSize = (1920 // 2**preview_downscale, 1080 // 2**preview_downscale)
center_image = (imageSize[0] // 2, imageSize[1] // 2)
FPS = 5
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds


picam2 = Picamera2()
picam2_config = picam2.create_video_configuration(
    {"size": imageSize, "format": "RGB888"},
    controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
    queue=False,
)
picam2.configure(picam2_config)  # Not really necessary
picam2.start(show_preview=False)
time.sleep(2)
picam2.start()
# cap = cv2.VideoCapture()

arucoParams = cv2.aruco.DetectorParameters()


def detect(image_inp):
    # print(image_inp.shape)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image=image_inp, dictionary=arucoDict)
    return corners, ids, rejected


def cam_on():
    while True:
        im = picam2.capture_array()

        return im


# markerDist = int(input("Distance from marker: "))
# markerHeight = 145  # mm
focalLength = 648


def get_marker_dim(markerDist: int, markerHeight: int = 145, calc_f: bool = False):
    image = cam_on()

    (corners, ids, rejected) = detect(image)
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for markerCorner, markerID in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))


        # draw the bounding box of the ArUCo detection
        cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the ArUco
        # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
        # draw the ArUco marker ID on the image
        cv2.putText(
            image,
            str(markerID),
            (topLeft[0], topLeft[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

        leftHeight = bottomLeft[1] - topLeft[1]
        rightHeight = bottomRight[1] - topRight[1]
        heightPixels = (leftHeight + rightHeight) // 2
        topWidth = topRight[0] - topLeft[0]
        bottomWidth = bottomRight[0] - bottomLeft[0]
        heightPixels = (leftHeight + rightHeight) // 2
        widthPixels = (topWidth + bottomWidth) // 2
        # print("Right height", rightHeight)
        # print("Left height", leftHeight)
        # print("avg height", pixels)
        y = heightPixels
        Y = markerHeight
        x = widthPixels
        X = markerHeight
        Z = markerDist
        print()
        if calc_f:
            fx = x * (Z / X)
            fy = y * (Z / Y)
            print(f"height params: {y=}, {Y=}, {Z=}, y * (Z / Y) = {fy=}")
            print(f"width params : {x=}, {X=}, {Z=}, x * (Z / X) = {fx=}")
        else:
            if markerDist:
                print(f"height params: {y=}, {Y=}, {Z=}")
                print(f"width params : {x=}, {X=}, {Z=}")
            else:
                print(f"height params: {y=}, {Y=}")
                print(f"width params : {x=}, {X=}")
        print()

    cv2.imshow("Image", image)
    cv2.waitKey(1)



while 1:
    time.sleep(0.1)
    get_marker_dim(0, calc_f=False)
    nam = input("Enter to proceed next frame, otherwise write name")
    if nam != "":
        markDist = int(input("Distance from marker (default 1000): ") or 1000)
        markHeight = 145
        get_marker_dim(markDist, markHeight, calc_f=True)
        picam2.capture_file(f"img_Z{markDist}_X{markHeight}_{nam}.jpg")
        print(f"Created img_Z{markDist}_X{markHeight}_{nam}.jpg")
