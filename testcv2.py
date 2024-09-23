import cv2
import constants

cam = cv2.VideoCapture(0)

if not cam.isOpened():
    print("Cannot open the camera")
    exit()

while 1:
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame")
        break
    cv2.imshow('Macbook Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cam.release()
cv2.destroyAllWindows()