import time

import cv2
from picamera2 import Picamera2, Preview

import robot

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
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image=image_inp, dictionary=arucoDict, parameters=arucoParams
    )
    return corners, ids, rejected

def cam_on():
    while True:
        im = picam2.capture_array()

        grey = cv2.cvtColor(im, cv2.COLOR_RGB2RGBA)
        return grey


markerDist = int(input("Distance from marker: "))
markerHeight = 50  # mm

def get_marker_dim():
    image = cam_on()
    cv2.imshow("Image", image)
    cv2.waitKey(1)
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
        leftHeight = topLeft[1] - bottomLeft[1]
        rightHeight = topRight[1] - bottomRight[1]
        pixels = (leftHeight + rightHeight) // 2
        print("Right height", rightHeight)
        print("Left height", leftHeight)
        print("avg height", pixels)
        x = pixels
        X = markerHeight
        Z = markerDist
        print("focal length", x*(Z/X))

# picam2 = Picamera2()
# full = (1920, 1080)
# preview_downscale = 2
# raw = {'size': full}
# main = {'size': (full[0]//(2**preview_downscale),full[1]//(2**preview_downscale))}
# preview_controls = {'FrameRate': 15}
# preview_config = picam2.create_preview_configuration(main, raw=raw, controls=preview_controls)
# capture_controls = {'FrameRate': (2, 20)}
# capture_config = picam2.create_still_configuration(controls=capture_controls)
# picam2.configure(preview_config)
#camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, lores={"size": (240, 135)}, display="lores")

#camera_config = picam2.create_still_configuration()
#picam2.configure(camera_config)
# picam2.start_preview(Preview.QTGL)
# picam2.start()

while 1:
    time.sleep(0.1)
    inp = input("name (q for quit): ").replace(" ", "_")
    if inp.lower() == "q":
        exit()
    get_marker_dim()

    picam2.capture_file(f"img{i}_{inp}.jpg")

