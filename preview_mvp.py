
from camera.webcam import Camera

cam = Camera()

while 1:
    image = cam.take_image(enable_preview=True)
