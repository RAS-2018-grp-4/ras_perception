#!/usr/bin/env python

import os
import rospy
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg 
import geometry_msgs.msg
import math
import tf
from object_detection_test.msg import objects_found
#####################################################
#                 Object_tf Class              #
#####################################################
class Object_tf:
    #####################################################
    #              Initialize Object                    #
    #####################################################
    def __init__(self):
        self.OBJ_POSITION = geometry_msgs.msg.PointStamped()
        self.OBJ_FOUND = objects_found()
        self.BATTERY_FOUND = geometry_msgs.msg.PoseArray()
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('object_tf_node', anonymous=True)
        self.pub_OBJ_POS = rospy.Publisher('/object_position_map', objects_found, queue_size=1)
        self.rate = rospy.Rate(5)
        rospy.Subscriber('/object_position_cam_link', objects_found, self.feedback_obj_pos)
        self.LISTENER = tf.TransformListener()
        self.LISTENER_BATTERY = tf.TransformListener()
        
        self.pub_BATTERY_POS = rospy.Publisher('/battery_position_map', geometry_msgs.msg.PoseArray, queue_size=1)
        rospy.Subscriber('/battery_position_cam_link', geometry_msgs.msg.PoseArray, self.feedback_battery_pos)
        
    #####################################################
    #                 obj_pos_feedback                  #
    #####################################################
    def feedback_obj_pos(self,pos):
        self.LISTENER.waitForTransform("/camera_link", "/map", rospy.Time(0),rospy.Duration(4.0))
        self.OBJ_FOUND = pos
        for i in range(0,pos.number_of_objects):
            self.OBJ_FOUND.array_objects_found[i] = self.LISTENER.transformPoint("/map",pos.array_objects_found[i])

        self.pub_OBJ_POS.publish(self.OBJ_FOUND)         
        
    #####################################################
    #                 battery_pos_feedback                  #
    #####################################################
    def feedback_battery_pos(self,pos):
        #self.LISTENER_BATTERY.waitForTransform("/camera_depth_frame", "/map", rospy.Time(0),rospy.Duration(4.0))
        self.LISTENER_BATTERY.waitForTransform("/camera_link", "/map", rospy.Time(0),rospy.Duration(4.0))
        self.BATTERY_FOUND = pos
        for i in range(0,len(pos.poses)):
            point = geometry_msgs.msg.PointStamped()
            point_map = geometry_msgs.msg.PointStamped()
            #point.header.frame_id = "/camera_depth_frame"
            point.header.frame_id = "/camera_link"
            point.point.x = pos.poses[i].position.x
            point.point.y = pos.poses[i].position.y
            point.point.z = pos.poses[i].position.z
            point_map = self.LISTENER_BATTERY.transformPoint("/map",point)
            self.BATTERY_FOUND.poses[i].position.x = point_map.point.x
            self.BATTERY_FOUND.poses[i].position.y = point_map.point.y
            self.BATTERY_FOUND.poses[i].position.z = point_map.point.z

        self.pub_BATTERY_POS.publish(self.BATTERY_FOUND)


    #####################################################
    #                   Main_Loop                       #
    #####################################################
    def loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    
#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        OT = Object_tf()
        OT.loop()
    except rospy.ROSInterruptException:
        pass