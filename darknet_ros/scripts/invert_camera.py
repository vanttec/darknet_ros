#!/usr/bin/env python

from std_msgs.msg import String
from imutils.video import VideoStream
from imutils.video import FPS

#from srv import DistanceCal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import imutils
import argparse
import numpy as np
import time
import rospy
import cv2
import math

import os

class Color():
    BLUE  = '\033[94m'
    GREEN = '\033[92m'
    RED  = '\033[91m'
    DONE  = '\033[0m'


class Detection_Node:
    def __init__(self):

        self.bridge = CvBridge()
        self.image = np.zeros((560,1000,3),np.uint8)
        self.depth = np.zeros((560,1000,3),np.uint8)
        self.points_list = [[0,0,0]]


        #rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.callback_zed_img)
        #rospy.Subscriber("/zed/zed_node/point_cloud/cloud_registered", PointCloud2, self.callback_zed_cp)
        rospy.Subscriber("/frontr200/camera/color/image_raw", Image, self.callback_zed_img)

        self.invertimage=rospy.Publisher("/invert_camera",Image,queue_size=10)
        # rospy.Subscriber("/frontr200/camera/depth_registered/points", PointCloud2, self.callback_zed_cp)

        #self.detector_pub = rospy.Publisher('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, queue_size=10)


    def callback_zed_img(self,img):
        """ ZED rect_image callback"""
        self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        self.image = cv2.flip(self.image,-1)
    def detect(self):
        """ Invert camera """

        (H, W) = (None, None)

        while not rospy.is_shutdown():
            # Grab next frame

            
            zed_cam_size = self.image.shape[1]
            frame = self.image

            #frame = add_brightness(frame)
            #frame = add_darkness(frame)

            #frame = imutils.resize(frame, width=1000)

            (H, W) = frame.shape[:2]

         
                
            #print((H, W))


            # Show current frame
            #cv2.imshow("Frame", frame)
            #print(self.depth)

            #cv2.waitKey(1)
            newImage=Image()
            newImage = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.invertimage.publish(newImage)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('yolo_zed')

        rate = rospy.Rate(20) # 20Hz
        D = Detection_Node()
        D.detect()
    except rospy.ROSInterruptException:
        pass
