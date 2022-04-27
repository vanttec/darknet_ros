#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
  rospy.init_node('image_converter', anonymous=True)
  image_pub = rospy.Publisher("/camera/rgb/image_raw",Image, queue_size=1)
  vid = cv2.VideoCapture(0)
  bridge = CvBridge()
  
  while not rospy.is_shutdown():
      __, frame = vid.read()
      # cv2.imshow('frame', frame)
      image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break
  # rospy.spin()
  
  vid.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()