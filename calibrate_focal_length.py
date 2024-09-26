from picamera2 import Picamera2
from picamera2 import Preview
import constants
import time



picam2 = Picamera2()
main = {'size': (640, 360)}
raw = {'size': (1920, 1080)}
preview_controls = {'FrameRate': 15}
preview_config = picam2.create_preview_configuration(main, raw=raw, controls=preview_controls)
capture_controls = {'FrameRate': (2, 20)}
capture_config = picam2.create_still_configuration(controls=capture_controls)
picam2.configure(preview_config)
#camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, lores={"size": (240, 135)}, display="lores")

#camera_config = picam2.create_still_configuration()
#picam2.configure(camera_config)
picam2.start_preview(Preview.QTGL)
picam2.start()

for i in range(100):
    time.sleep(2)
    inp = input("name (q for quit): ").replace(" ", "_")
    if inp.lower() == "q":
        exit()
    picam2.capture_file(f"img{i}_{inp}.jpg")

