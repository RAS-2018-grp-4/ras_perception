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
        self.MSG.data = False
        
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('wall_detection_node', anonymous=True)
        self.pub_WALL_DETECTION = rospy.Publisher('/wall_detection', std_msgs.msg.Bool, queue_size=1)
        self.pub_WALL_POSITION = rospy.Publisher('/wall_position', geometry_msgs.msg.PoseArray, queue_size=1)
        self.pub_WALL_DISTANCE = rospy.Publisher('/wall_distance', std_msgs.msg.Float32MultiArray, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/scan', sensor_msgs.msg.LaserScan, self.feedback_laser)
        rospy.Subscriber('/robot_odom', nav_msgs.msg.Odometry, self.feedback_odom)

        self.LISTENER = tf.TransformListener()
        

    #####################################################
    #                   Laser_Feedback                  #
    #####################################################
    def feedback_laser(self,scan):
        '''
        # initalize distance sensor value
        d_left = []
        d_tangent = 999
        d_right = []

        count = (int)(scan.scan_time / scan.time_increment)
        self.MSG.data = False
        wall_position_array = geometry_msgs.msg.PoseArray()
        for i in range(0, count): 
            x = scan.angle_min + scan.angle_increment * i
            degree = ((x)*180./3.14)
            # print(str(i) + ' ' + str(degree) + ' ' + str(scan.ranges[i]))

            # publish the distance sensor                         
            if i >= 100 and i <= 160: # right +80deg
                d_right.append(scan.ranges[i])
            elif i == 180: # straight ahead
                d_tangent = [scan.ranges[i]]
            elif i < 260 and i >= 200: # left -80deg
                d_left.append(scan.ranges[i])
            elif i == 260:
                #print(round(d_left,2), round(d_tangent,2), round(d_right,2))   
                print [round(a,2) for a in d_left]

                distance_sensor = Float32MultiArray()
                distance_sensor.data = [d_left + d_tangent + d_right]
                self.pub_WALL_DISTANCE.publish(distance_sensor)

        '''
        
            if (i >= 160) and (i < 200):# -20 to 20
                if scan.ranges[i] < self.DISTANCE_THRESHOLD:
                    self.MSG.data = True
                else:
                    pass
            else:
                pass
            
            if scan.ranges[i] != float("inf"):

                #print(str(i) + ' ' + str(degree) + ' ' + str(scan.ranges[i]))
                #self.WALL_POSITION.header.stamp = rospy.Time.now()
                self.WALL_POSITION.header.frame_id = 'laser'
                self.WALL_POSITION.point.x = self.X + math.cos(self.THETA + x) * scan.ranges[i]
                self.WALL_POSITION.point.y = self.Y + math.sin(self.THETA + x) * scan.ranges[i]
                self.WALL_POSITION.point.z = 0
                self.LISTENER.waitForTransform("/laser", "/map", rospy.Time(0),rospy.Duration(4.0))
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

                #print(str(self.WALL_POSITION.point.x))

                # THIS IS TEMPORARILY DISABLED#########################################################
                #self.pub_WALL_POSITION.publish(wall_position_array)         
        
        
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
        #print(str(self.X)+ ' ' + str(self.Y) + ' ' + str(self.THETA))
    #####################################################
    #                   Main_Loop                       #
    #####################################################
    def loop(self):
        while not rospy.is_shutdown():
            self.pub_WALL_DETECTION.publish(self.MSG)


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