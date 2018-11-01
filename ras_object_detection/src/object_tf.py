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
#                 Object_tf Class              #
#####################################################
class Object_tf:
    #####################################################
    #              Initialize Object                    #
    #####################################################
    def __init__(self):
        self.OBJ_POSITION = geometry_msgs.msg.PointStamped()
        
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('object_tf_node', anonymous=True)
        self.pub_OBJ_POS = rospy.Publisher('/object_position_map', geometry_msgs.msg.PointStamped, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/object_position_cam_link', geometry_msgs.msg.PointStamped, self.feedback_obj_pos)
        self.LISTENER = tf.TransformListener()
        

    #####################################################
    #                 obj_pos_feedback                  #
    #####################################################
    def feedback_obj_pos(self,pos):
        self.LISTENER.waitForTransform("/camera_link", "/map", rospy.Time(0),rospy.Duration(4.0))
        self.OBJ_POSITION = self.LISTENER.transformPoint("/map",pos)
        self.pub_OBJ_POS.publish(self.OBJ_POSITION)         
        
        
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