import cv2
import time

print('OpenCV version:', cv2.__version__)

for i in range(6):
    cam = cv2.VideoCapture(i)
    opened = cam.isOpened()
    print(f'Index {i}: opened={opened}')
    if opened:
        ret, frame = cam.read()
        print(f'  read={ret}, frame_ok={frame is not None}')
        cam.release()
    time.sleep(0.3)

print('Test complete. If no index opened, check camera drivers, privacy settings, or other apps using the camera.')
