#!/usr/bin/env python

import os
import rospy
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg 
import geometry_msgs.msg
import math
import tf
import numpy as np
#import sensor_msgs.point_cloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt 
import cv2



#####################################################
#                 battery Detection Class              #
#####################################################
class battery_detection:
    #####################################################
    #              Initialize Object                    #
    #####################################################
    def __init__(self):
        self.distance_threshold = 0.3
        self.battery_position = geometry_msgs.msg.PointStamped()
        self.msg = std_msgs.msg.Bool()
        self.msg.data = False                          # detect battery or noy
        self.remaining_mapping_times = 5                  # mapping times
        self.remaining_skipping_times = 20
        self.detect_angle = 20
        self.map_angle = 90
        self.max_mapping_distance = 2 * self.distance_threshold
        self.rgb_img = []
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('battery_detection_node', anonymous=True)

        self.pub_battery_detection = rospy.Publisher('/battery_detection', std_msgs.msg.Bool, queue_size=1)
        self.pub_battery_position = rospy.Publisher('/battery_position', geometry_msgs.msg.PoseArray, queue_size=1)

        rospy.Subscriber('/camera/depth/image', Image, self.callback_depth)
        # Subscriber to RGB image
        rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.callback_storage_image)
        self.rate = rospy.Rate(10)
        self.listenser = tf.TransformListener()
        # self.depth_img = []

    def morphOpen(self,image):
        # define structuring element
        # take 5% of least dimension of image as kernel size
        kernel_size = min(5, int(min(image.shape[0],image.shape[1])*0.1))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
        #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        return opening


    #call back function to store image in class object 
    def callback_storage_image(self,image_message):
        #print('trace 2')
        bridge = CvBridge()
        self.rgb_img = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    
    #####################################################
    #                   depth_Callback                  #
    #####################################################
    def callback_depth(self,depth_msg):
        bridge = CvBridge()
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        sobely = cv2.Sobel(depth_img,cv2.CV_64F,0,1,ksize=5)

        sobely_nonan = sobely.copy()
        bad_I = np.argwhere(np.isnan(sobely))
        sobely_nonan[bad_I[:,0],bad_I[:,1]] = 0

        sobely_thresh = np.where(sobely_nonan>0, 255, 0)
        sobely_thresh=np.uint8(sobely_thresh)
        # cv2.imshow('thresh',np.uint8(sobely_thresh))
        # cv2.waitKey(20)
        #plt.imshow(sobely) 
        

        #Morph open 
        sobely_thresh=self.morphOpen(sobely_thresh)
        # find contours
        _, contours, _ = cv2.findContours(sobely_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours != []:
            contour_sizes = [(cv2.contourArea(contour)>1000) for contour in contours]
            ind_temp = np.argwhere(contour_sizes)
            for j in range(ind_temp.shape[0]):
                largest_contours = contours[ind_temp[j,0]]
            
                #for k in range(largest_contours.__len__()):
                box = cv2.boundingRect(largest_contours)
                #if detect_color(self.rgb_img[yy:yy+h,xx:xx+w]==False):
                if float(box[2])/box[3] >1.5 or float(box[2])/box[3] < 0.7:     # flat or upright battery?
                    cv2.rectangle(self.rgb_img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (0,0,255), 2)
            cv2.imshow('detected battery or wall',self.rgb_img)
            cv2.waitKey(2)

            cv2.imshow('sobel',sobely_thresh)
            cv2.waitKey(2)
            print('got here')

    #####################################################
    #                   Main_Loop                       #
    #####################################################
    def loop(self):
        rospy.loginfo('battery Detection Start')

        while not rospy.is_shutdown():

           # self.detect_battery()
            
            if self.msg.data:
                rospy.sleep(8)

                #reset
                self.remaining_mapping_times = 5                  
                self.remaining_skipping_times = 20
                self.msg.data = False
                rospy.loginfo('Reset')

            else:
                pass   
            self.rate.sleep()

    
#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        WD = battery_detection()
        WD.loop()
    except rospy.ROSInterruptException:
        pass