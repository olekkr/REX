import cv2

from camera.webcam import camera_setup, take_image
from constants import Constants

cam = camera_setup()

while 1:
    image = take_image(cam, False)
    if Constants.PID.ENABLE_PREVIEW:
        cv2.imshow("Image", cv2.resize(image, (800, 600), interpolation=cv2.INTER_AREA))
        if cv2.waitKey(1) == ord("q"):
            exit()
