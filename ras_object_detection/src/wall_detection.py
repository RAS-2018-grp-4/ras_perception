#!/usr/bin/env python

import os
import rospy
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg 
import geometry_msgs.msg
import math
import tf

from std_msgs.msg import Float32MultiArray

#####################################################
#                 Wall Detection Class              #
#####################################################
class Wall_Detection:
    #####################################################
    #              Initialize Object                    #
    #####################################################
    def __init__(self):
        self.DISTANCE_THRESHOLD = 0.4
        self.X = 0.0
        self.Y = 0.0
        self.THETA = 0.0
        self.WALL_POSITION = geometry_msgs.msg.PointStamped()

        self.ROBOT_POSITION = nav_msgs.msg.Odometry()
        self.MSG = std_msgs.msg.Bool()
        self.MSG.data = False                          # detect wall or noy
        self.REMAIN_MAPPING_TIMES = 0                  # mapping times
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('wall_detection_node', anonymous=True)
        self.pub_WALL_DETECTION = rospy.Publisher('/wall_detection', std_msgs.msg.Bool, queue_size=1)
        self.pub_WALL_POSITION = rospy.Publisher('/wall_position', geometry_msgs.msg.PoseArray, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/scan', sensor_msgs.msg.LaserScan, self.feedback_laser)
        rospy.Subscriber('/robot_filter', nav_msgs.msg.Odometry, self.feedback_odom)

        self.LISTENER = tf.TransformListener()
        

    #####################################################
    #                   Laser_Feedback                  #
    #####################################################
    def feedback_laser(self,scan):
        count = (int)(scan.scan_time / scan.time_increment)
        #self.MSG.data = False
        wall_position_array = geometry_msgs.msg.PoseArray()
        for i in range(0, count): 
            x = scan.angle_min + scan.angle_increment * i 
            degree = ((x)*180./3.14)
            # print(str(i) + ' ' + str(degree) + ' ' + str(scan.ranges[i]))
        
            if (i >= 170) and (i < 190) and not self.MSG.data:# -10 to 10
                if scan.ranges[i] < self.DISTANCE_THRESHOLD:
                    
                    # publish message
                    self.MSG.data = True
                    self.pub_WALL_DETECTION.publish(self.MSG)


                    self.REMAIN_MAPPING_TIMES = 5
                else:
                    pass
            else:
                pass
            
            if scan.ranges[i] != float("inf") and (i >= 150) and (i < 210) and self.REMAIN_MAPPING_TIMES > 0 and scan.ranges[i] < 0.47:

                #print(str(i) + ' ' + str(degree) + ' ' + str(scan.ranges[i]))
                #self.WALL_POSITION.header.stamp = rospy.Time.now()
                self.WALL_POSITION.header.frame_id = 'odom'
                self.WALL_POSITION.point.x = self.X + math.cos(self.THETA + x) * scan.ranges[i]
                self.WALL_POSITION.point.y = self.Y + math.sin(self.THETA + x) * scan.ranges[i]
                self.WALL_POSITION.point.z = 0
                self.LISTENER.waitForTransform("/odom", "/map", rospy.Time(0),rospy.Duration(4.0))
                self.WALL_POSITION = self.LISTENER.transformPoint("/map",self.WALL_POSITION)

                somePose = geometry_msgs.msg.Pose()
                somePose.position.x = self.WALL_POSITION.point.x
                somePose.position.y = self.WALL_POSITION.point.y
                somePose.position.z = 0

                somePose.orientation.x = 0.0
                somePose.orientation.y = 0.0
                somePose.orientation.z = 0.0
                somePose.orientation.w = 1.0

                wall_position_array.poses.append(somePose)

        if self.REMAIN_MAPPING_TIMES > 0:
            self.pub_WALL_POSITION.publish(wall_position_array)   
            self.REMAIN_MAPPING_TIMES  = self.REMAIN_MAPPING_TIMES -1      
        
        
    #####################################################
    #                   Odom_Feedback                   #
    #####################################################
    def feedback_odom(self,odom):
        self.X = odom.pose.pose.position.x
        self.Y = odom.pose.pose.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([
            odom.pose.pose.orientation.x, 
            odom.pose.pose.orientation.y, 
            odom.pose.pose.orientation.z, 
            odom.pose.pose.orientation.w])
        self.THETA = y
       
    #####################################################
    #                   Main_Loop                       #
    #####################################################
    def loop(self):
        while not rospy.is_shutdown():
            
            if self.MSG.data:
                rospy.sleep(5)

                #reset
                self.REMAIN_MAPPING_TIMES = 0 
                self.MSG.data = False

            else:
                pass   
            self.rate.sleep()

    
#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        WD = Wall_Detection()
        WD.loop()
    except rospy.ROSInterruptException:
        pass