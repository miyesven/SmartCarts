#!/usr/bin/env python

## BALL RECOGNITION CODE TAKEN FROM https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
## Written by Adrian Rosebrock, 2015

##### ROS #####
import rospy
from std_msgs.msg import Int32, Int32MultiArray, UInt8MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

##### BALL RECOGNITION #####
# import the necessary packages
from collections import deque
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import sys
import matplotlib.pyplot as plt
import time

#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

##### GLOBAL VARIABLES #####
redLower = (130, 130, 0)
redUpper = (255, 255, 255)
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
x_list = []
y_list = []
radius_list = []
distance_list = []
timestamp_list = []
# RGBD data
# depth_frame_data # uint8
# color_frame_data # uint8
# args = {}
# pts = deque()

# Init cv_bridge
bridge = CvBridge()


## Takes in an img given by cv2.imread and applies the following filters
## Returns the filtered image
def contour_filter(img):
    blurred = cv2.bilateralFilter(img,12,125,125)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    return mask


## Takes in [mask] a binary image (1/0 for each pixel) and [img] the 2D color image
##  Finds the contours and appends the x,y,radius,timestamp lists from results of each image
##  MODIFIES [img] by drawing circle and centroid on image
## Returns [circle] a list for information about enclosing circle (x,y,radius)
def parse_color_image(mask, img):
    ## Finding Contours
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Find max contour area
    i = 0
    maxContour = 0
    maxContourArea = 0
    for contour in contours:
        contourArea = cv2.contourArea(contour)
        if contourArea > maxContourArea:
            maxContourArea = contourArea
            maxContour = i
        i += 1
    # Find coordinates + radius of min enclosing circle of blob in mask
    ((x, y), radius) = cv2.minEnclosingCircle(contours[maxContour])
    # Drawing on image the circle & corresponding centroid calculated above if circle is detected
    # Also populating x,y,radius lists
    if radius > 10:
        x_list.append(x)
        y_list.append(y)
        radius_list.append(radius)
        # Define list to return
        circle = [x,y,radius]
        return circle
    else:
        return None


# ## Callback functions for realsense frames using cv_bridge
# # color
# def camera_color_callback(image):
#     global color_frame_data
#     color_frame_data = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
#
#
# # depth
# def camera_depth_callback(image):
#     global depth_frame_data
#     depth_frame_data = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
#
#
# ## Callback functions for bag frames
# # color
# def bag_color_callback(image):
# 	color_frame_data = image.data
#
#
# # depth
# def bag_depth_callback(image):
# 	depth_frame_data = image.data


# ## Main ball_tracking node.
# ##  Opens stream/video and executes the logic to parse each frame
# ## Returns [circle] tuple of (x,y,radius,distance(from depth))
# def ball_tracker():
# 	### ROS INTERFACE ###
# 	# Subscribe to the following topics:
# 	# Video Stream:
# 		##<------TODO------>##
# 	# 	1. Realsense Camera Node Package > Color Frame Topic
# 	# 	2. Realsense Camera Node Package > Depth Frame Topic
# 	# Playing from .bag file:
# 	#	1. rosbag > Depth image Topic
# 	#	2. rosbag > Color image Topic
# 	# Publish the following topics:
# 	# 	1. Tuple (x in meters, y in meters, depth in meters)
# 	# 	2. Time of video frame
#
#     # Init ball_tracker ROS node with annonymous node name
#     rospy.init_node('ball_tracker', anonymous= True)
#     rospy.myargv(argv=sys.argv)
#
#  	# Subscribe to different topics based on parameters passed in.
#  	# Default is realsense camera node (input_type = 0)
#     #if (sys.argv[0] == 0):
#     rospy.Subscriber('/camera/depth/image_rect_raw', Image, camera_depth_callback) # THIS IS FROM GLYNN'S CODE AND NOT TESTED
#     rospy.sleep(1)
#     rospy.Subscriber('/camera/color/image_raw', Image, camera_color_callback)
#     rospy.sleep(1)
#     # else:
#     #     rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data", Image, bag_depth_callback)
#     #     rospy.Subscriber("/device_0/sensor_1/Color_0/image/data", Image, bag_color_callback)
#  	# Publish the parsed x,y,radius,distance (uint8[]) array
#     # circle_pub = rospy.Publisher('target_distance', UInt8MultiArray, queue_size = 30)
#     # r = rospy.Rate(30)
#     rospy.spin()
#     frame_num = 0
#
#     while not rospy.is_shutdown():
#
#         if (depth_frame_data is not ([] or None) and color_frame_data is not ([] or None)):
#             ##### BELOW IS ALL COMMENTED OUT BECAUSE IM NOT SURE HOW TO PARSE THIS NEW "IMAGE" MESSAGE TYPE #####
#             depth_image = np.asanyarray(depth_frame_data)
#             print(depth_frame_data)
#             # color_image = cv2.cvtColor(np.asanyarray(color_frame_data), cv2.COLOR_BGR2RGB)
#             #
#             # # parsing color image
#             # mask = contour_filter(color_image)
#             # # gets the x,y,radius reading from image
#             # circle_list = parse_color_image(mask, color_image)
#             #
#             # if circle_list is None:
#             #     break
#             #
#             # # get the depth reading from the depth frame at circle centroid (units: meters) and save into tuple
#             # distance = depth_frame_data.get_distance(int(circle_list[0]), int(circle_list[1]))
#             # distance_list.append(distance)
#             # circle_list.append(distance)
#             #
#             # # get the timestamp of the current color/depth frame
#             # timestamp_list.append(color_frame.get_timestamp())
#             #
#             # #circle_list = [frame_num, frame_num] #pseudo code, just stepping through frames
#             # #frame_num += 1
#
#             ### Publish Results on ROS Node ###
#             # circle_pub.publish(data=circle_list)
#             pass
#
#
#     # r.sleep()

class ball_tracker:

    def __init__(self):
        self.depth_image_raw = None
        self.color_image_raw = None
        self.bridge = CvBridge()

        self.position_pub = rospy.Publisher("target_distance", Float32MultiArray ,queue_size = 10)

        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback, queue_size = 2)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size = 2)


    def color_callback(self, image):
        try:
            self.color_image_raw = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            # cv2.imshow("RGB", self.color_image_raw)
            # key = cv2.waitKey(1) & 0xFF
            self.compute_target_pos()
        except CvBridgeError as e:
            print(e)


    def depth_callback(self, image):
        try:
            self.depth_image_raw = self.bridge.imgmsg_to_cv2(image, 'passthrough')
            # print(self.depth_image_raw)
            # self.compute_target_pos()
        except CvBridgeError as e:
            print(e)


    def compute_target_pos(self):
        if self.color_image_raw is not None and self.depth_image_raw is not None:
            depth_image = np.asanyarray(self.depth_image_raw)
            #color_image = cv2.cvtColor(np.asanyarray(self.color_image_raw), cv2.COLOR_BGR2HSV)
            color_image = np.asanyarray(self.color_image_raw)

            # insert image processing method or choice here
            mask = contour_filter(color_image)
            circle_list = parse_color_image(mask, color_image)

            if circle_list is None:
                return #TODO should tracker node raise a flag if target cannot be identified
            else:
                color_image = cv2.circle(color_image, (int(circle_list[0]), int(circle_list[1])), int(circle_list[2]), (0, 255, 255), 2)
                color_image = cv2.circle(color_image, (int(circle_list[0]), int(circle_list[1])), 5, (0, 0, 255), -1)
                distance = depth_image[int(circle_list[0]), int(circle_list[1])]
                distance_list.append(distance)
                circle_list.append(distance)

            cv2.imshow("RGB", self.color_image_raw)
            key = cv2.waitKey(1) & 0xFF

            self.position_pub.publish(data=[circle_list[len(circle_list)-1]])



def main(args):
    bt = ball_tracker()

    rospy.init_node('target_tracker', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("target_tracker shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
