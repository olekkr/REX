import time

import cv2
from picamera2 import Picamera2

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


def drive_straight(i: int = 1):
    print("Found target")
    arlo.go_diff(speed, speed, 1, 1)
    time.sleep(1)
    return i + 1


def turn_left(i):
    print("moving left")
    arlo.go_diff(tspeed, tspeed, 0, 1)
    time.sleep(0.7)
    # arlo.stop()
    # time.sleep(2)
    # if i < 20:
    #     arlo.go_diff(tspeed, tspeed, 0, 1)
    # elif i > 100:
    #     i = 0
    # else:
    #     arlo.stop()
    return i + 1


def turn_right():
    print("moving right")
    arlo.go_diff(tspeed_slow, tspeed_slow, 1, 0)
    time.sleep(0.5)


def cam_on():
    # if not cap.isOpened():
    #     print("Camera not opened")
    #     exit()
    # ret, frame = cap.read()
    # while not ret:
    #     print("Frame unavailable waiting...")
    #     time.sleep(5)
    #     ret, frame = cap.read()

    while True:
        im = picam2.capture_array()

        grey = cv2.cvtColor(im, cv2.COLOR_RGB2RGBA)
        return grey
        print(detect(grey))

        cv2.imshow("Camera", im)
        cv2.waitKey(1)


i = 0
stop_and_see = 5
j = 0
while 1 and __name__ == "__main__":
    arlo.stop()
    image = cam_on()
    (corners, ids, rejected) = detect(image)
    front = arlo.read_sensor(0)
    lefts = arlo.read_sensor(2)
    rights = arlo.read_sensor(3)

    cX = None
    cY = None
    topRight = None
    bottomRight = None
    bottomLeft = None
    topLeft = None

    # verify *at least* one ArUco marker was detected
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
            print("[INFO] ArUco marker ID: {}".format(markerID))
            # show the output image
    cv2.imshow("Image", image)
    cv2.waitKey(1)
    if cv2.getWindowProperty("Image", 0) == -1:
        arlo.stop()
        exit()

    if len(corners) == 0 and j < stop_and_see:
        j += 1
        continue
    elif len(corners) == 0 and j >= stop_and_see:
        j = 0

    # middle = (qr_leftdown + qr_rightdown) / 2
    if cX and cY and topRight and bottomRight and bottomLeft and topLeft:
        imgcX, imgcY = center_image
        thresholdX = int(imgcX * 0.5)
        thresholdY = int(imgcY * 0.5)
        close_x = range(cX - thresholdX, cX + thresholdX)
        close_y = range(cY - thresholdY, cY + thresholdY)
        print(close_x, close_y, cX, cY, imgcX, imgcY)

        if imgcX in close_x and imgcY in close_y:
            drive_straight(i)
        elif bottomLeft[0] > imgcX:
            turn_right()
        else:
            i = turn_left(i)
    else:
        i = turn_left(i)
