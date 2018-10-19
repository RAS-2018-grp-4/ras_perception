#!/usr/bin/env python

import os
import rospy
import std_msgs.msg
import sensor_msgs.msg



#####################################################
#                 Wall Detection Class              #
#####################################################
class Wall_Detection:
    #####################################################
    #          Initialize Object                        #
    #####################################################
    def __init__(self):
        self.DISTANCE_THRESHOLD = 0.4
        self.MSG = std_msgs.msg.Bool()
        self.MSG.data = False
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('wall_detection_node', anonymous=True)
        self.pub_WALL_DETECTION = rospy.Publisher('/wall_detection', std_msgs.msg.Bool, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/scan', sensor_msgs.msg.LaserScan, self.feedback_laser)

    #####################################################
    #                   Laser_Feedback                  #
    #####################################################
    def feedback_laser(self,scan):
        count = (int)(scan.scan_time / scan.time_increment)
        self.MSG.data = False
        for i in range(160, 200): # -20 to 20
            x = scan.angle_min + scan.angle_increment * i
            degree = ((x)*180./3.14)
            print(str(i) + ' ' + str(degree) + ' ' + str(scan.ranges[i]))
            if scan.ranges[i] < self.DISTANCE_THRESHOLD:
                self.MSG.data = True
            else:
                pass
            #
        


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