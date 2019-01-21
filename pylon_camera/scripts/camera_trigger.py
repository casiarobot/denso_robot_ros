#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
from math import pi
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import yaml
import time

class camera_shooter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = "/pylon_camera_node/image_raw"

    def trigger(self, imgName=None):
      '''
      The image will save to $ROS_HOME directory, or 
      you can modify it by node "cwd" attribute
      '''
      data = rospy.wait_for_message(self.image_topic, Image)
      try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)
      else:
          if imgName is not None:
            print("Save an image!")
            print("filename: {}".format(imgName))
            cv2.imwrite(imgName, cv_image)
          return cv_image

    def image_stream(self):
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.callback)

    def callback(self, data):
        print("Received an image!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            print("Recieve image")
            cv2.imshow('img', cv_image)
            cv2.waitKey(3)
  
def main():
    rospy.init_node('image_shooter', anonymous=True)
    camera = camera_shooter()
    try:
        image = camera.trigger('img/basler.bmp')
        
        # Image streaming
        # camera.image_stream()
        # rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()
  

