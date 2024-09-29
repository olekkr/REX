import time

from camera.picam import Camera

picam2 = Camera()


for i in range(100):
    time.sleep(2)
    inp = input("name (q for quit): ").replace(" ", "_")
    if inp.lower() == "q":
        exit()
    picam2.capture_file(f"img{i}_{inp}.jpg")
