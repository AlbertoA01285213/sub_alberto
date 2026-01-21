import numpy as np
import cv2

cv_file = cv2.FileStorage()
cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

cap_left = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap_right = cv2.VideoCapture(2, cv2.CAP_DSHOW)

while(cap_right.isOpened() and cap_left.isOpened()):

    success_left, frame_left = cap_left.read()
    success_right, frame_right = cap_right.read()

    frame_left = cv2.remap(frame_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    frame_right = cv2.remap(frame_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    cv2.imshow("frame left", frame_left)
    cv2.imshow("frame right", frame_right)

    if cv2.waitKey(1) & 0xFF ==ord('q'):
        break

cap_right.release()
cap_left.release()

cv2.destroyAllWindows()