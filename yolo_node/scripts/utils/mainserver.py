#! /usr/bin/env python3

import rospy
from ultralytics import YOLO
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import message_filters 
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class mainserver():
    
    def __init__(self) -> None:
        
        self._cv_bridge = CvBridge()
        self.img_pub = rospy.Publisher("/imagelala", Image, queue_size=10)
        
        self.input_type = rospy.get_param("~input_type")
        
        weight_path = rospy.get_param("~weight_path")
        self.model = YOLO(weight_path)
        
        if self.input_type == 0:
            self.rgb_raw_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_raw_callback)
            
        elif self.input_type == 1:
            print("HAHA")
            self.rgb_comp_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.rgb_comp_callback)
            
        elif self.input_type == 2:
            self.rgb_raw_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
            self.dep_raw_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
            
            ts = message_filters.ApproximateTimeSynchronizer([self.rgb_raw_sub, self.dep_raw_sub], queue_size=10, slop=0.1)
            ts.registerCallback(self.rgb_raw_dep_callback)
            
        elif self.input_type == 3:
            self.rgb_comp_sub = message_filters.Subscriber('/camera/color/image_raw/compressed', CompressedImage)
            self.dep_raw_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
            
            ts = message_filters.ApproximateTimeSynchronizer([self.rgb_comp_sub, self.dep_raw_sub], queue_size=10, slop=0.1)
            ts.registerCallback(self.rgb_comp_dep_callback)
            pass
        else:
            rospy.logerr("PLS CHECK LAUNCH FILE INPUT_TYPE")
        
        
        
    def rgb_raw_callback(self, data):
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(data, "bgr8")
        except ValueError:
            rospy.logerr("Unable to reshape image data")
            return
        
        # cv2.imshow("hii", cv_image)
        # cv2.waitKey(10)
        
        results = self.model(self.cv_image)[0]  # predict on an image
        # print(len(results))
    
    def rgb_comp_callback(self, data):
        try:
            cv_image = self._cv_bridge.compressed_imgmsg_to_cv2(data)
        except ValueError:
            rospy.logerr("Unable to reshape image data")
            return
        
    def rgb_raw_dep_callback(self, data_rgb, data_dep):
        try:
            rgb_image = self._cv_bridge.imgmsg_to_cv2(data_rgb, "bgr8")
        except ValueError:
            rospy.logerr("Unable to reshape image data")
            return

        try:
            dep_image = self._cv_bridge.imgmsg_to_cv2(data_dep, "16UC1")
        except ValueError:
            rospy.logerr("Unable to reshape image data")
            return        
    
    def rgb_comp_dep_callback(self, data_rgb, data_dep):
        try:
            rgb_image = self._cv_bridge.compressed_imgmsg_to_cv2(data_rgb)
        except ValueError:
            rospy.logerr("Unable to reshape image data")
            return

        try:
            dep_image = self._cv_bridge.imgmsg_to_cv2(data_dep, "16UC1")
        except ValueError:
            rospy.logerr("Unable to reshape image data")
            return
        
        print("lala")
        
        
        # ros_image = self._cv_bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        # ros_image = self._cv_bridge.cv2_to_imgmsg(dep_image, encoding="16UC1")
        # self.img_pub.publish(ros_image)

    
    