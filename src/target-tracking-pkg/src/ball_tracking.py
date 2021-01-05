#!/usr/bin/env python

## BALL RECOGNITION CODE TAKEN FROM https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
## Written by Adrian Rosebrock, 2015

##### ROS #####
import rospy
from std_msgs.msg import Int32, Int32MultiArray

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
x_list = []
y_list = []
radius_list = []
distance_list = []
timestamp_list = []
# args = {}
# pts = deque()

## Takes in an img given by cv2.imread and applies the following filters
## Returns the filtered image
def contour_filter(img):
    blurred = cv2.bilateralFilter(img,12,125,125)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, redLower, redUpper)
    return mask

## Takes in [mask] a binary image (1/0 for each pixel) and [img] the 2D color image
##  Finds the contours and appends the x,y,radius,timestamp lists from results of each image
##  MODIFIES [img] by drawing circle and centroid on image
## Returns [circle] a list for information about enclosing circle (x,y,radius)
def parse_color_image(mask, img):
    ## Finding Contours
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    if radius > 0:
        img = cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        img = cv2.circle(img, (int(x), int(y)), 5, (0, 0, 255), -1)
        x_list.append(x)
        y_list.append(y)
        radius_list.append(radius)
        # Define list to return
        circle = [x,y,radius]
        return circle
    else:
        return None

## Main ball_tracking node.
##  Opens stream/video and executes the logic to parse each frame
## Returns [circle] tuple of (x,y,radius,distance(from depth))
def ball_tracker():
	### ROS INTERFACE ###
	##<------TODO------>##
	# Subscribe to the following topics:
	# 	1. Realsense Camera Node Package > Color Frame Topic  
	# Publish the following topics:
	# 	1. Tuple
	# 	2. Time 
	target_dist_pub = rospy.Publisher('target_distance', Int32, queue_size = 10)
	target_angle_pub = rospy.Publisher('target_relative_angle', Int32MultiArray, queue_size = 10)
	rospy.init_node('ball_tracker', anonymous=True)
	r = rospy.Rate(10)


	while not rospy.is_shutdown():

		# grab the current frame
		frame = vs.read()
		try:
        config = rs.config()
        config.enable_device_from_file(file)
        pipeline = rs.pipeline()
        # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        pipeline.start(config)
        # create colorizer object
        colorizer = rs.colorizer()
        time.sleep(2.0)

        while True:
            # get frameset of depth
            frames = pipeline.wait_for_frames()
            # get depth frame
            depth_frame = frames.get_depth_frame()
            # Colorize depth frame to jet colormap
            depth_color_frame = colorizer.colorize(depth_frame)
            # Convert depth_frame to numpy array to render image in opencv
            depth_color_image = np.asanyarray(depth_color_frame.get_data())
            # get color frame and convert it to a numpy array
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            if  color_frame is None or depth_frame is None:
                break

            # parsing color image
            mask = contour_filter(color_image)
            # gets the x,y,radius reading from image
            circle_list = parse_color_image(mask, color_image)

            if circle_list is None:
                break

            # get the timestamp of the current color/depth frame
            timestamp_list.append(color_frame.get_timestamp())
            # get the depth reading from the depth frame at circle centroid (units: meters) and save into tuple
            distance = depth_frame.get_distance(int(circle_list[0]), int(circle_list[1]))
            distance_list.append(distance)
            circle_list.append(distance)
            circle = tuple(circle_list)
            # Render image in opencv window
            cv2.imshow("Depth Stream", depth_color_image)
            cv2.imshow("Color Stream", color_image)

            key = cv2.waitKey(1) & 0XFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                break

        ### Publish Results on ROS Node ###
		target_distance = target_distance_calc(radius)
		target_relative_angle = target_rel_angle_calc(target_distance, center)
 
		target_dist_pub.publish(target_distance)
		target_angle_pub.publish(data=target_relative_angle)

		r.sleep()

    finally:
        pass
    
    return circle

if __name__ == '__main__':
	try:
		ball_tracker()
	except rospy.ROSInterruptException:
		pass

