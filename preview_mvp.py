from camera.webcam import Camera

cam = Camera()

try:
    while 1:
        image = cam.take_image(enable_preview=True)
except:
    pass
finally:
    print(len(image), len(image[0]))
