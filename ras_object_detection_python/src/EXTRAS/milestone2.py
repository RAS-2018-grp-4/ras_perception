#!usr/bin/env python

import cv2
import numpy as np 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2

# DEBUG = 0

# class maze_object():
#     def __init__(self):
#         # image containers
#         self.rgb_input = []
#         self.depth_input = []
    
#     '''
#     Realsense camera intrinsic parameters. To obtain type 

#           rostopic echo /camera/rgb/camera_info
      
#     Source: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
#     '''
#     fx = 616.344
#     fy = 616.344
#     cx = 314.855
#     cy = 223.877

#     #Object Position
#     self.x_obj = 0
#     self.y_obj = 0
#     self.z_obj = 0

#   // Max and Min Depth in meters (0 and 255 respectively on depth image)
#   float max_depth = 2.0;  
#   float min_depth = 0.05

def callback_depth(img_msg):
    #print('HEY')
    bridge = CvBridge()
    depth_input = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    cv2.imshow('depth image', depth_input)
    cv2.waitKey(10)

def callback_pointcloud(pc_msg):
    point_cloud = PointCloud2()
    point_cloud = pc_msg
    print('Reached PC2 callback')

def main():
    #print('Reached main')
    rospy.init_node('expt_listener', anonymous=True)
    #/camera/depth_registered/sw_registered/image_rect
    rospy.Subscriber('/camera/depth/image_raw', Image, callback_depth)
    #rospy.Subscriber('/camera/depth/points', PointCloud2, callback_pointcloud)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    #print('progrtas starts')
    main()