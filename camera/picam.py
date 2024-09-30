import time

import cv2
from picamera2 import Picamera2

from camera.camera_base import CameraBase
from constants import Constants


class Camera(CameraBase):
    def setup_camera(self):
        picam2 = Picamera2()
        picam2_config = picam2.create_video_configuration(
            **self.video_configuration
        )
        picam2.configure(picam2_config)
        if self.still_configuration:
            picam2_still_config = picam2.create_still_configuration(
                **self.still_configuration
            )
            picam2.configure(picam2_still_config)
        if self.preview_configuration:
            picam2_preview_config = picam2.create_preview_configuration(
                **self.preview_configuration
            )
            picam2.configure(picam2_preview_config)

        picam2.start(show_preview=self.picam_show_preview)

        time.sleep(2)
        picam2.start()
        return picam2

    def take_image(self, enable_preview=Constants.PID.ENABLE_PREVIEW) -> cv2.typing.MatLike:
        image = self.cam.capture_array("main")

        if enable_preview:
            self.preview(image)

        return image

    def capture_file(self, name: str):
        return self.cam.capture_file(name)
