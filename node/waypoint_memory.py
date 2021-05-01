#!/usr/bin/env python

"""
ROS Node for storing waypoints obtained by transforming output from ball tracking node to robot's frame
- Includes some simple filtering so not all waypoints are stored
- Publishes the intended waypoint for the robot to follow and detects if robot has reached within threshold
Waypoints are stored in a deque for quick and easy appending and pops. Appends on right. Pops on left.
Earlier points (left) -----> Recent points (right)

Subscribed:
	/gazebo/model_states
Published:
	/target_waypoint (PoseStamped)
Written By: Svena Yu, Mar 2021
"""

import csv
import math
import time
import numpy as np
import rospy
import random
from collections import deque
from std_msgs.msg import Header
# from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, TransformStamped, Vector3, Quaternion
import tf2_ros
import tf_conversions
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
# Custom imports
from smartcarts.msg import Float32Stamped, Float32MultiArrayStamped, BallPoseStamped
from interfaces import invalid_PoseStamped, check_invalid_PoseStamped

# To store a waypoint, transform it from target rel to follower, to target rel to world
class waypoint_queue:
	def __init__(self):
		time.sleep(2.0)
		# Publishers and Subscribers
		self.states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.states_sub_callback)
		self.target_waypoint_pub = rospy.Publisher('/target_waypoint', PoseStamped, queue_size=10)
		# General Variables
		self.follower_start = PoseStamped() # Empty pose to store starting point
		self.parent_name = "follower" ; self.child_name = "leader" ; self.target_name = "target"
		self.parent_pose = PoseStamped() # Parent pose is the current pose of parent robot
		self.current_waypoint = invalid_PoseStamped(); # Current waypoint being published on /target_waypoint
		self.waypoints = deque([]) # Double ended queue of PoseStamped type
		# Threshold Variables
		self.min_follow_distance = rospy.get_param('min_follow_distance')
		self.max_target_dist = 3.0 # Min and max distance to target
		self.threshold_to_store = 0.20 # Threshold to store the newly found waypoint
		self.max_fluctuation_between_frames = 1.00 # Do not store if the distance fluctuation is more than this value
		self.following_deadband = 0.15 # Threshold to stop following the current waypoint


	# Callback function for ModelStates Subcriber, stores the current state of the follower robot in class variable
	def states_sub_callback(self, states):
		if (len(states.name) != 3):
			return
		# if self.follower_start.header.stamp.nsecs == 0:
		# 	self.follower_start = PoseStamped(header = Header(stamp=rospy.Time.now()), pose = msg.pose[msg.name.index("follower")])
		parent_idx = states.name.index(self.parent_name)
		self.parent_pose = PoseStamped(header = Header(), pose = states.pose[parent_idx])

	# Takes two waypoints (wp1 - Pose, wp2 - Pose) and calculates the absolute distance between their points
	# Returns the distance calculated (float)
	def calc_3d_euler_distance(self, wp1, wp2):
		return math.sqrt((wp1.position.x - wp2.position.x)**2 + \
						(wp1.position.y - wp2.position.y)**2 + \
						(wp1.position.z - wp2.position.z)**2)


	# Takes a waypoint (wp - Pose) and tests if it should be stored in the based on several conditions
	# Returns boolean True if it should be stored
	def waypoints_threshold(self, wp):
		if wp.position.x == 0 and wp.position.y == 0: # if absolute coord of target = 0,0 --> negative result
			return False
		if len(self.waypoints) == 0:
			return True
		else:
			distance_wp = self.calc_3d_euler_distance(wp, self.waypoints[-1].pose)
			if distance_wp >= self.threshold_to_store : # compare current and last waypoint
				return True
		return False


	# TransformStamped to PoseStamped conversion
	def transformStamped_to_poseStamped(self, transformStamped):
		transform = transformStamped.transform
		return PoseStamped(header = transformStamped.header, pose = Pose(position = Point(x = transform.translation.x, \
																	y = transform.translation.y,  \
																	z = transform.translation.z), \
																orientation = transform.rotation))


	# For debugging messages, returns string of tuple (x,y,z) of pose
	def str_xyz(self, pose):
		return "({},{},{})".format(str(pose.position.x),str(pose.position.y),str(pose.position.z))


	# For debugging messages, returns string of list of PoseStamped
	def str_poseStampedList(self, poseStampedList):
		return ",".join([self.str_xyz(poseStamped.pose) for poseStamped in poseStampedList])


	# Print debug message
	def debug(self):
		i = 0 ; head = "" ; tail = ""
		if len(self.waypoints) <= 4:
			while i < len(self.waypoints):
				head += self.str_xyz(self.waypoints[i].pose)
				head += ", "
				i += 1
		else:
			while i < 4 :
				head += self.str_xyz(self.waypoints[i].pose)
				head += ", "
				i += 1
		if len(self.waypoints) > 1:
			tail = self.str_xyz(self.waypoints[-1].pose)
		# print("waypoint head = {}, waypoint tail = {}".format(head, tail))
		return

	def main(self):
		rate = rospy.Rate(10)
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		while not rospy.is_shutdown():
			try:
				# If integrated odom, this "world" frame will not be gazebo world. Will be the relative starting pt of follower
				# target_abs_tf = tfBuffer.lookup_transform('follower_start', 'target', rospy.Time.now())
				target_abs_tf = tfBuffer.lookup_transform('odom', 'target', rospy.Time()) #TODO: fix the time issue
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rate.sleep()
				continue
			target_abs_ps = self.transformStamped_to_poseStamped(target_abs_tf)
			# Storing and Publishing Waypoints
			if self.waypoints_threshold(target_abs_ps.pose):
				self.waypoints.append(target_abs_ps)
				self.current_waypoint = target_abs_ps
				self.target_waypoint_pub.publish(self.current_waypoint)
			else:
				self.target_waypoint_pub.publish(self.current_waypoint) 
			# Get rid of waypoints that have been reached and keep track of current ones, not sure if this is needed
			if not check_invalid_PoseStamped(self.current_waypoint):
				if (self.calc_3d_euler_distance(self.current_waypoint.pose, self.parent_pose.pose) < self.following_deadband):
					self.waypoints.popleft () # POPLEFT
					if (len(self.waypoints) >= 1):
						self.current_waypoint = self.waypoints[0]
					else:
						self.current_waypoint = invalid_PoseStamped()
			self.debug()

if __name__ == "__main__":
	rospy.init_node("waypoint_memory", anonymous=True)
	mem = waypoint_queue()
	mem.main()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("waypoint_memory shutting down")
