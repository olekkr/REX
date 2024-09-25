import cv2
import camera_setup
import constants


picam2 = camera_setup.camera_setup()

while 1:
    image = picam2.capture_array("main")
    if constants.ENABLE_PREVIEW:
        cv2.waitKey(1)
        cv2.imshow("Image", image)
