import cv2

from constants import Constants
from camera.camera_base import CameraBase
import custom_types

imageSize = Constants.PID.SCREEN_RESOLUTION
center_image = (imageSize[0] // 2, imageSize[1] // 2)
FPS = Constants.PID.CAMERA_FPS
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds


class Camera(CameraBase):


    def setup_camera(self):
        cam = cv2.VideoCapture(0)

        if not cam.isOpened():  # Error
            raise OSError("Could not open camera")

        return cam


    def take_image(self, enable_preview=Constants.PID.ENABLE_PREVIEW) -> custom_types.MatLikeT:
        _, image = self.cam.read()

        if enable_preview:
            cv2.imshow("Image", cv2.resize(image, (800, 600), interpolation=cv2.INTER_AREA))
            if cv2.waitKey(1) == ord("q"):
                exit()

        return image

    def capture_file(self, name: str):
        return cv2.imwrite(name, self.take_image(False))
