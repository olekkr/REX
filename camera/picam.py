import time

from picamera2 import Picamera2

from camera.camera_base import CameraBase
from constants import Constants


class Camera(CameraBase):
    def setup_camera(self):
        FPS = Constants.PID.CAMERA_FPS
        frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds
        picam2 = Picamera2()
        picam2_config = picam2.create_video_configuration(
            {"size": (1640 // 2, 1232 // 2), "format": "RGB888"},
            controls={
                "ScalerCrop": (0, 0, 3280, 2464),
                "FrameDurationLimits": (frame_duration_limit, frame_duration_limit),
            },
            queue=False,
        )
        picam2.configure(picam2_config)
        picam2.start(show_preview=False)

        time.sleep(2)
        picam2.start()
        return picam2

    def take_image(self, enable_preview=Constants.PID.ENABLE_PREVIEW):
        image = self.cam.capture_array("main")

        if enable_preview:
            self.preview(image)
