import cv2

for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera found at /dev/video{i}")
        ret, frame = cap.read()
        if ret:
            print(f"Successfully read from /dev/video{i}")
        cap.release()