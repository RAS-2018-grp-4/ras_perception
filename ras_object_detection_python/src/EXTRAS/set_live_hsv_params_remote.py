#!/usr/bin/env python
'''
Author:     Ajinkya 
Date:       2018/11/20
Description: This script reads the live camera feed and allows you to set HSV params.
NOTE: Also usable remotely, using keyboard

Credits and source: https://botforge.wordpress.com/2016/07/02/basic-color-tracker-using-opencv-python/

0:  hMIN
1:  hMAX
2:  sMIN
3:  sMAX
4:  vMIN
5:  vMAX
'''

#import the necessary packages
import cv2
import numpy as np
# imports for ROS integration
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import time
import copy
import readchar

#'optional' argument is required for trackbar creation parameters
def nothing():
    pass
 
class Frame:
    def __init__(self):
        self.image = []
        self.flagImage = False
        
# create a global object of Frame
lastFrame = Frame()

#call back function to store image in class object 
def callback_storage_image(image_message):
    '''Uncomment this if you use RAW images'''
    # #print('trace 2')
    # bridge = CvBridge()
    # global lastFrame
    # lastFrame.image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    # #lastFrame.image = cv2.cvtColor(lastFrame.image, cv2.COLOR_BGR2RGB)
    # lastFrame.flagImage = True

    '''Uncomment this for compressed images'''
    # Image to numpy array
    np_arr = np.fromstring(image_message.data, np.uint8)
    # Decode to cv2 image and store
    lastFrame.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    lastFrame.flagImage = True
    

def print_screen(str, cnt):
    if cnt != 0:
        print()
# MAIN FUNCTION
def main():
    # Local variables
    hul = 0
    huh = 179
    sal = 0
    sah = 255
    val = 0
    vah = 255

    rospy.init_node('set_hsv_param_node', anonymous=True)

    # Subscriber to RGB image
    rospy.Subscriber('/camera/rgb/image_rect_color/compressed', CompressedImage, callback_storage_image)
    
    # Publisher for calibrated image
    #calibratedImage = rospy.Publisher("/hsv_calibrated_image", CompressedImage, queue_size=10)
    calibratedImage = rospy.Publisher("/hsv_calibrated_image", Image, queue_size=10)

    r = rospy.Rate(10) # Hz

    print('Press 0-5')
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
            
            
            '''USE READCHAR'''####################################################################
            # col_key = cv2.waitKey(5)

            # if col_key == ord('0'): 
            #     print('Calibrating H_MIN')
            #     param_key = cv2.waitKey(5)
            #     if param_key == ord('w'):
            #         hul += 1
            #     elif param_key == ord('s'):
            #         hul -= 1
            # elif col_key == ord('1'): 
            #     print('Calibrating H_MAX')
            #     param_key = cv2.waitKey(5)
            #     if param_key == ord('w'):
            #         huh += 1
            #     elif param_key == ord('s'):
            #         huh -= 1
            # elif col_key == ord('2'): 
            #     print('Calibrating S_MIN')
            #     param_key = cv2.waitKey(5)
            #     if param_key == ord('w'):
            #         sal += 1
            #     elif param_key == ord('s'):
            #         sal -= 1
            # elif col_key == ord('3'): 
            #     print('Calibrating S_MAX')
            #     param_key = cv2.waitKey(5)
            #     if param_key == ord('w'):
            #         sah += 1
            #     elif param_key == ord('s'):
            #         sah -= 1
            # elif col_key == ord('4'): 
            #     print('Calibrating V_MIN')
            #     param_key = cv2.waitKey(5)
            #     if param_key == ord('w'):
            #         val += 1
            #     elif param_key == ord('s'):
            #         val -= 1
            # elif col_key == ord('5'): 
            #     print('Calibrating V_MAX')
            #     param_key = cv2.waitKey(5)
            #     if param_key == ord('w'):
            #         vah += 1
            #     elif param_key == ord('s'):
            #         vah -= 1

            col_key = readchar.readchar()
            print('trace1')
            if col_key == '0': 
                print('Calibrating H_MIN')
                param_key = readchar.readchar()
                if param_key == 'w':
                    hul += 1
                elif param_key == 's':
                    hul -= 1
            elif col_key == '1': 
                print('Calibrating H_MAX')
                param_key = readchar.readchar()
                if param_key == 'w':
                    huh += 1
                elif param_key == 's':
                    huh -= 1
            elif col_key == '2': 
                print('Calibrating S_MIN')
                param_key = readchar.readchar()
                if param_key == 'w':
                    sal += 1
                elif param_key == 's':
                    sal -= 1
            elif col_key == '3': 
                print('Calibrating S_MAX')
                param_key = readchar.readchar()
                if param_key == 'w':
                    sah += 1
                elif param_key == 's':
                    sah -= 1
            elif col_key == '4': 
                print('Calibrating V_MIN')
                param_key = readchar.readchar()
                if param_key == 'w':
                    val += 1
                elif param_key == 's':
                    val -= 1
            elif col_key == '5': 
                print('Calibrating V_MAX')
                param_key = readchar.readchar()
                if param_key == 'w':
                    vah += 1
                elif param_key == 's':
                    vah -= 1

      
            #make array for final values
            HSVLOW=np.array([hul,sal,val])
            HSVHIGH=np.array([huh,sah,vah])
        
            #create a mask for that range
            mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
            res = cv2.bitwise_and(frame,frame, mask =mask)
            

            # msg = CompressedImage()
            # msg.header.stamp = rospy.Time.now()
            # msg.format = "jpeg"
            # msg.data = np.array(cv2.imencode('.jpg', res)[1]).tostring()
            # calibratedImage.publish(msg)
            
            # Publish processed image
            calibratedImage.publish(CvBridge().cv2_to_imgmsg(res))
            
            # cv2.imshow(wnd, res)
            # k = cv2.waitKey(5) and 0xFF
            # if k == ord('q'):
            #     break

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