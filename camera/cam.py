from constants import Constants

if Constants.PID.CAMERA_MODEL == "picam":
    from camera.picam import Camera
else:
    from camera.webcam import Camera

