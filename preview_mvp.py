from camera.picam import Camera

cam = Camera()

while 1:
    image = cam.take_image(enable_preview=True)
    print(len(image))
    print(len(image[0]))
