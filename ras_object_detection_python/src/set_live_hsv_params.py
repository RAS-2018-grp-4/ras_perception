#!/usr/bin/env python
'''
Author:     Ajinkya 
Date:       2018/11/20
Description: This script reads the live camera feed and allows you to set HSV params.

Credits and source: https://botforge.wordpress.com/2016/07/02/basic-color-tracker-using-opencv-python/
'''
#import the necessary packages
import cv2
import numpy as np
# imports for ROS integration
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import copy

#'optional' argument is required for trackbar creation parameters
def nothing():
    pass
 
#Capture video from the stream
cap = cv2.VideoCapture(0)
cv2.namedWindow('Colorbars') #Create a window named 'Colorbars'
 
#assign strings for ease of coding
hh='Hue High'
hl='Hue Low'
sh='Saturation High'
sl='Saturation Low'
vh='Value High'
vl='Value Low'
wnd = 'Colorbars'
#Begin Creating trackbars for each
cv2.createTrackbar(hl, wnd,0,179,nothing)
cv2.createTrackbar(hh, wnd,0,179,nothing)
cv2.createTrackbar(sl, wnd,0,255,nothing)
cv2.createTrackbar(sh, wnd,0,255,nothing)
cv2.createTrackbar(vl, wnd,0,255,nothing)
cv2.createTrackbar(vh, wnd,0,255,nothing)


class Frame:
    def __init__(self):
        self.image = []
        self.flagImage = False
        
# create a global object of Frame
lastFrame = Frame()

#call back function to store image in class object 
def callback_storage_image(image_message):
    #print('trace 2')
    bridge = CvBridge()
    global lastFrame
    lastFrame.image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    #lastFrame.image = cv2.cvtColor(lastFrame.image, cv2.COLOR_BGR2RGB)
    lastFrame.flagImage = True

# MAIN FUNCTION
def main():
    rospy.init_node('set_hsv_param_node', anonymous=True)

    # Subscriber to RGB image
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, callback_storage_image)
 
    r = rospy.Rate(10) # Hz
    #begin our 'infinite' while loop
    while(not rospy.is_shutdown()):
        global lastFrame
        processFrame = copy.deepcopy(lastFrame)
        if processFrame.flagImage == True:
            #read the streamed frames (we previously named this cap)
            frame = cv2.cvtColor(processFrame.image, cv2.COLOR_RGB2BGR)
                
            #frame = cv2.imread('2200.jpg')
        
            #it is common to apply a blur to the frame
            frame=cv2.GaussianBlur(frame,(5,5),0)
            frame = cv2.resize(frame, (640,480))
            #convert from a BGR stream to an HSV stream
            hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            #read trackbar positions for each trackbar
            hul=cv2.getTrackbarPos(hl, wnd)
            huh=cv2.getTrackbarPos(hh, wnd)
            sal=cv2.getTrackbarPos(sl, wnd)
            sah=cv2.getTrackbarPos(sh, wnd)
            val=cv2.getTrackbarPos(vl, wnd)
            vah=cv2.getTrackbarPos(vh, wnd)
        
            #make array for final values
            HSVLOW=np.array([hul,sal,val])
            HSVHIGH=np.array([huh,sah,vah])
        
            #create a mask for that range
            mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
            res = cv2.bitwise_and(frame,frame, mask =mask)
        
            cv2.imshow(wnd, res)
            k = cv2.waitKey(5) and 0xFF
            if k == ord('q'):
                break

            r.sleep()

    cv2.destroyAllWindows()

    # while not rospy.is_shutdown():
    #     global lastFrame
    #     processFrame = copy.deepcopy(lastFrame)
    #     #print('trace inside WHILE LOOP')
    #     if processFrame.flagImage == True and processFrame.flagDepth == True:
    #         #print('REACHED THIS POINT')
    #         a = datetime.now()
    #         for i in range(N_COLORS):
    #             frame = cv2.cvtColor(processFrame.image, cv2.COLOR_RGB2BGR)
    #             detect_object(frame, i)

    #         b = datetime.now()
    #         c = b - a
    #         fps = 1.0/(c.total_seconds())

    #         cv2.imshow('result', frame)
    #         cv2.waitKey(1)
    #         print('## FPS: ' + str(fps))
    #         print('')
        
        # r.sleep()

if __name__ == '__main__':
    main()