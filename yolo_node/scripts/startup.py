#!/usr/bin/env python

import rospy
import utils.mainserver as ms

if __name__ == '__main__':
    rospy.init_node("yolopy", anonymous=True)        
    rospy.loginfo("WELCOME TO ROS YOLOv8")
    
    lala = ms.mainserver()
    
    rospy.spin()