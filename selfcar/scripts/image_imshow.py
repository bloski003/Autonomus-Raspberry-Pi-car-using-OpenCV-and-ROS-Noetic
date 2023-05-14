#!/usr/bin/env python3

import cv2
cap = cv2.VideoCapture(0)
print(cap.isOpened())

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2150)
#cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
	print("Cant open camera")

while True:
	ret, frame = cap.read()
	print(ret)
	if not ret:
		break
	cv2.imshow('frame', frame)
	
	if cv2.waitKey(1) == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()
