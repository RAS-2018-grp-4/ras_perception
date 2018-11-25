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

    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('battery_detection_node', anonymous=True)

        self.pub_battery_detection = rospy.Publisher('/battery_detection', std_msgs.msg.Bool, queue_size=1)
        self.pub_battery_position = rospy.Publisher('/battery_position', geometry_msgs.msg.PoseArray, queue_size=1)

        rospy.Subscriber('/camera/depth/image', Image, self.callback_depth)

        self.rate = rospy.Rate(10)
        self.listenser = tf.TransformListener()
        

    #####################################################
    #                   Laser_Callback                  #
    #####################################################
    def callback_depth(self,depth_msg):
        # point_cloud = np.zeros((pc_msg.height,pc_msg.width))
        # pt_x = []
        # pt_y = []
        # pt_z = []
        # for point in sensor_msgs.point_cloud2.read_points(pc_msg, skip_nans=False):
        #     pt_x.append(point[0]) 
        #     pt_y.append(point[1])
        #     pt_z.append(point[2])  
        #     #sensor_msgs.point_cloud2.read_points_list()
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
        
        # find contours
        _, contours, _ = cv2.findContours(sobely_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_contours = sorted(contours, key=cv2.contourArea)[-10:]
        print('got here')
    #####################################################
    #                   Main_Loop                       #
    #####################################################
    def loop(self):
        rospy.loginfo('battery Detection Start')

        while not rospy.is_shutdown():
            
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