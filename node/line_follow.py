#!/usr/bin/env python

'''
Original line follow code for robot in gazebo, written by Miti. 
'''
from __future__ import print_function

import roslib
roslib.load_manifest('smartcarts')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.namespace = rospy.get_namespace()

    # fetch topic_name from the ~private namespace
    self.camera_param = rospy.get_param('/camera_type')
    if self.camera_param == 'depth':
      self.image_sub = rospy.Subscriber("{}camera1/color/image_raw".format(self.namespace), Image, self.callback)
    else:
      self.image_sub = rospy.Subscriber("{}camera1/image_raw".format(self.namespace), Image, self.callback)
    self.cmd_vel_pub = rospy.Publisher('{}cmd_vel'.format(self.namespace), Twist, queue_size=1)

  def callback(self,data):
    # print("Processing image.")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    rows, cols, channels = cv_image.shape

    CM_x = self.process_image(cv_image)

    # print("CM: {}, img_center: {}".format(CM_x, cols/2))

    move_cmd = Twist()
    move_cmd.linear.x = 0.5
    move_cmd.angular.z = 0

    deadBand = 10
    if CM_x > cols / 2 + deadBand:
      move_cmd.linear.x = 0
      move_cmd.angular.z = -0.5

    if CM_x < cols / 2 - deadBand:
      move_cmd.linear.x = 0
      move_cmd.angular.z = 0.5

    self.cmd_vel_pub.publish(move_cmd)

  def process_image(self, image):
    (rows, cols, channels) = image.shape

    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray_img[rows-20:, :], 120, 255, cv2.THRESH_BINARY_INV)

    M = cv2.moments(thresh)
    if M["m00"] != 0:
      CM_x = int(M["m10"]/ M["m00"])
      CM_y = int(M["m01"] / M["m00"])
    else:
      CM_x, CM_y = 0, 0

    cv2.circle(image, (CM_x, rows-10), 5, (0, 0, 255), -1)
    # cv2.imshow("Image window", image)
    # cv2.waitKey(3)

    return CM_x

def main(args):
  print("Started line follow.")
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
