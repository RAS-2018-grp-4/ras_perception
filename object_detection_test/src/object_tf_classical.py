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
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('object_tf_node_classical', anonymous=True)
        self.pub_OBJ_POS = rospy.Publisher('/object_position_map_classical', objects_found, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/object_position_cam_link_classical', objects_found, self.feedback_obj_pos)
        self.LISTENER = tf.TransformListener()
        

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