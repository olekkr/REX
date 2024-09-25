from picamera2 import Picamera2, Preview
import constants
import time

imageSize = (constants.SCREEN_RESOLUTION)
center_image = (imageSize[0] // 2, imageSize[1] // 2)
FPS = constants.CAMERA_FPS
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds

def camera_setup():
    picam2 = Picamera2()
    picam2_config = picam2.create_video_configuration(
        {"size": (1640,1232), "format": "RGB888"},
        controls={
            "ScalerCrop": (0, 0, 1640,1232),
            "FrameDurationLimits": (frame_duration_limit, frame_duration_limit)}, 
            queue=False,
        )
    picam2.configure(picam2_config)  # Not really necessary
    picam2.start(show_preview=False)

    time.sleep(2)
    picam2.start()
    return picam2