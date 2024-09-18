import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Camera not opened")
else:
    ret, frame = cap.read()
    if ret:
        # Process the frame using OpenCV operations
        cv2.imshow("Frame", frame)

    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()