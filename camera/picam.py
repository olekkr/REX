import time

import cv2
from picamera2 import Picamera2, Preview




class Camera:
    def setup_camera(self):
        FPS = properties.CAMERA_FPS
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
        picam2.configure(picam2_config)  # Not really necessary
        picam2.start(show_preview=False)

        time.sleep(2)
        picam2.start()
        self.cam = picam2

    def __init__(self):
        self.preview_name = "Image"
        self.resize_dimensions = (800, 600)
        self.interpolation = cv2.INTER_AREA

        self.setup_camera()

    def take_image(self):
        image = self.cam.capture_array("main")

        if properties.ENABLE_PREVIEW:
            cv2.waitKey(1)
            cv2.imshow(
                self.preview_name,
                cv2.resize(image, self.resize_dimensions, interpolation=self.interpolation),
            )
    
    def read()


def camera_setup():
    picam2 = Picamera2()
    picam2_config = picam2.create_video_configuration(
        {"size": (1640 // 2, 1232 // 2), "format": "RGB888"},
        controls={
            "ScalerCrop": (0, 0, 3280, 2464),
            "FrameDurationLimits": (frame_duration_limit, frame_duration_limit),
        },
        queue=False,
    )
    picam2.configure(picam2_config)  # Not really necessary
    picam2.start(show_preview=False)

    time.sleep(2)
    picam2.start()
    return picam2


def take_image(cam):
    image = cam.capture_array("main")

    if properties.ENABLE_PREVIEW:
        cv2.waitKey(1)
        cv2.imshow("Image", cv2.resize(image, (800, 600), interpolation=cv2.INTER_AREA))
