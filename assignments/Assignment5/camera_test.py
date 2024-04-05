
import cv2
import time
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
time.sleep(1)
ret, frame = cap.read()
print(ret)

cv2.imwrite('test_frame.jpg', frame)
cap.release()