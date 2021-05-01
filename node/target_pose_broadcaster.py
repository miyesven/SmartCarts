#!/usr/bin/env python

'''
Parses information from depth_camera_ball_tracking & broadcasts information over tf2
Published:
	/calc_pub
	tf broadcast for 'target' in 'follower' (parent) frame
Subscribed:
	/target_pose
Written By: Svena Yu 21 Feb 2021
'''
##### ROS #####
import rospy
from std_msgs.msg import Int32, Int32MultiArray, UInt8MultiArray, Float32MultiArray, Float32, String, Header
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image, CameraInfo
from smartcarts.msg import Float32Stamped, Float32MultiArrayStamped, BallPoseStamped
import geometry_msgs
from geometry_msgs.msg import Pose, Twist, Quaternion, PoseStamped, Point
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber

#### TF ####
import tf_conversions
import tf2_ros

##### BALL RECOGNITION #####
# import the necessary packages
from collections import deque
import pyrealsense2 as rs
import numpy as np
import cv2
import os, sys
import math
import matplotlib.pyplot as plt
import time

class TargetBroadcaster:

	def __init__(self):
		time.sleep(2.0)
		# self.target_name = rospy.get_namespace().replace('/','') # should be 'follower'
		# Initialise Subscribers and Publishers
		self.target_pose_sub = rospy.Subscriber('/target_pose', PoseStamped, self.target_pose_broadcast_callback, queue_size=1)
		# Debugging
		self.calc_pub = rospy.Publisher('/calc_debug', String, queue_size=10)
		print("starting Target Broadcasting")

	'''Callback function for /target_pose Subscriber. Broadcasts the Pose passed into the function '''
	def handle_target_broadcast(self, target_pose, parent_frame):
	    br = tf2_ros.TransformBroadcaster()
	    t = geometry_msgs.msg.TransformStamped()
	    t.header.stamp = rospy.Time.now()
	    t.header.frame_id = parent_frame
	    t.child_frame_id = "target"
	    pt = target_pose.pose.position
	    q = target_pose.pose.orientation
	    t.transform.translation.x = pt.x
	    t.transform.translation.y = pt.y
	    t.transform.translation.z = pt.z
	    t.transform.rotation.x = q.x
	    t.transform.rotation.y = q.y
	    t.transform.rotation.z = q.z
	    t.transform.rotation.w = q.w
	    br.sendTransform(t)

   	def target_pose_broadcast_callback(self, target_pose):
		if target_pose.pose.position.x == -1:
			self.handle_target_broadcast(PoseStamped(header = Header(), \
													pose = Pose(position = Point(), orientation = Quaternion(w=1))), 'odom')
		else:
			self.handle_target_broadcast(target_pose, 'camera_link')
			# time_taken = rospy.Timer(rospy.Duration(0.05), self.handle_target_broadcast(target_pose))
			# print("Time taken to send pose = {}".format(time_taken))
		return

def main(args):
	rospy.init_node('target_pose_broadcaster', anonymous=True)
	wt = TargetBroadcaster()
	try:
		rospy.spin()
	except KeyboardInterrupt:
	    print("target_pose_broadcaster shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
