#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.callback) # change image topic here

  def callback(self,data):
    try:
      # Converting an image message pointer to an OpenCV message only requires a call to the function
      # Takes in image message and encoding of destination Opencv image
      cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1") # bgr8 for color/image, 16UC1 for depth/image types 
    except CvBridgeError as e:
      # Catch conversion errors
      print(e)

    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)

    # Display in HighGui window with name "Image window"
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "16UC1")) # Change this encoding too
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)