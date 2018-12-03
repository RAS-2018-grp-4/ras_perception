#!/usr/bin/env python

import os
import rospy
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg 
import geometry_msgs.msg
import math
import tf


#####################################################
#                 Wall Detection Class              #
#####################################################
class Wall_Detection:
    #####################################################
    #              Initialize Object                    #
    #####################################################
    def __init__(self):
        self.distance_threshold = 0.2
        self.wall_position = geometry_msgs.msg.PointStamped()
        self.msg = std_msgs.msg.Bool()
        self.msg.data = False                          # detect wall or noy
        self.remaining_mapping_times = 5                  # mapping times
        self.remaining_skipping_times = 20
        self.detect_angle = 20
        self.map_angle = 90
        self.max_mapping_distance = 2 * self.distance_threshold

    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('wall_detection_node', anonymous=True)

        self.pub_wall_detection = rospy.Publisher('/wall_detection', std_msgs.msg.Bool, queue_size=1)
        self.pub_wall_position = rospy.Publisher('/wall_position', geometry_msgs.msg.PoseArray, queue_size=1)

        rospy.Subscriber('/scan', sensor_msgs.msg.LaserScan, self.callback_laser)

        self.rate = rospy.Rate(10)
        self.listenser = tf.TransformListener()
        

    #####################################################
    #                   Laser_Callback                  #
    #####################################################
    def callback_laser(self,scan):
        count = (int)(scan.scan_time / scan.time_increment)
        wall_position_array = geometry_msgs.msg.PoseArray()

        # check if a wall in front of robot
        if not self.msg.data:
            for i in range(180 - self.detect_angle/2, 180 + self.detect_angle/2):
                if scan.ranges[i] < self.distance_threshold:
                        rospy.loginfo('Detect Wall')

                        # publish message
                        self.msg.data = True
                        self.pub_wall_detection.publish(self.msg)
                        rospy.loginfo('Send Stop Message')

                        break
                else:
                    pass
        
        if self.remaining_mapping_times > 0 and self.remaining_skipping_times <= 0:
            if self.remaining_mapping_times == 5:
                rospy.loginfo('Mapping Start')
            else:
                pass
            

            for i in range(180 - self.map_angle /2, 180 + self.map_angle/2):
                if scan.ranges[i] !=float("inf") and scan.ranges[i] < self.max_mapping_distance:
                    x = scan.angle_min + scan.angle_increment * i 
                    degree = ((x)*180./3.14)
                    self.wall_position.header.frame_id = 'laser'
                    self.wall_position.point.x = math.cos(x) * scan.ranges[i]
                    self.wall_position.point.y = math.sin(x) * scan.ranges[i]
                    self.wall_position.point.z = 0
                    self.listenser.waitForTransform("/laser", "/map", rospy.Time(0),rospy.Duration(4.0))
                    self.wall_position = self.listenser.transformPoint("/map",self.wall_position)

                    somepose = geometry_msgs.msg.Pose()
                    somepose.position.x = self.wall_position.point.x 
                    somepose.position.y = self.wall_position.point.y 
                    somepose.position.z = 0
                    somepose.orientation.x = 0.0
                    somepose.orientation.y = 0.0
                    somepose.orientation.z = 0.0
                    somepose.orientation.w = 1.0
                    wall_position_array.poses.append(somepose)
                else:
                    pass

            self.pub_wall_position.publish(wall_position_array)
            self.remaining_mapping_times = self.remaining_mapping_times - 1 

            if self.remaining_mapping_times == 0:
                rospy.loginfo('Mapping Done')
            else:
                pass
        if self.msg.data:
            self.remaining_skipping_times = self.remaining_skipping_times - 1
    #####################################################
    #                   Main_Loop                       #
    #####################################################
    def loop(self):
        rospy.loginfo('Wall Detection Start')

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
        WD = Wall_Detection()
        WD.loop()
    except rospy.ROSInterruptException:
        pass