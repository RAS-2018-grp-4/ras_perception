import numpy as np
import cv2

def rotateImage(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result

cap = cv2.VideoCapture('./vid1.mp4')
while(cap.isOpened()):
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = rotateImage(hsv, 270)
    frame = rotateImage(frame, 270)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.resize(hsv, (640,480))
    mask = cv2.inRange(hsv, np.array([6,120,50]), np.array([13,255,255]))
    frame = cv2.resize(frame, (640,480))
    cv2.imshow('frame',frame)
    #cv2.waitKey(0)
    cv2.imshow('threshold',mask)
    cv2.imshow('hsv',hsv)
    cv2.waitKey(0)
    if cv2.waitKey(0) == 27:
      #std::cout<<"esc key pressed"<<std::endl;
      break
	  

cap.release()
cv2.destroyAllWindows()
