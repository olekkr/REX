import cv2

from constants import Constants

imageSize = Constants.PID.SCREEN_RESOLUTION
center_image = (imageSize[0] // 2, imageSize[1] // 2)
FPS = Constants.PID.CAMERA_FPS
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds


def camera_setup():
    cam = cv2.VideoCapture(0)

    if not cam.isOpened():  # Error
        raise OSError("Could not open camera")

    return cam


def take_image(cam, enable_preview=Constants.PID.ENABLE_PREVIEW):
    _, image = cam.read()

    if enable_preview:
        cv2.imshow("Image", cv2.resize(image, (800, 600), interpolation=cv2.INTER_AREA))
        if cv2.waitKey(1) == ord("q"):
            exit()

    return image
