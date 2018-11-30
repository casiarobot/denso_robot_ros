#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

class camera_shooter:
    def __init__(self):
        self.bridge = CvBridge()
        image_topic = "/arm_sensor/camera/image_raw"
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)

    def callback(self, data):
        print("Received an image!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            print("Save an image!")
            cv2.imwrite('camera_image.jpeg', cv_image)

def main():
    shooter = camera_shooter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        # rospy.spin()
        print('done')
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main()