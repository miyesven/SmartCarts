#!/usr/bin/env python

"""
Auxillary code to deal with interfaces between different nodes.
Currently this code contains:
- Negative interfaces 
	-- Generates invalid messages for common ROS message types and checks for if a message is invalid

Written By: Svena Yu April 2021
"""
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, TransformStamped, Vector3, Quaternion

'''Returns invalid Pose Stamped message'''
def invalid_PoseStamped():
    return PoseStamped( header = Header(stamp = rospy.Time.now(), frame_id = "world"), \
        pose = Pose(position = Point(x = -1, y = -1, z = -1), orientation = Quaternion(w=1)))

'''
Checks for invalid Pose Stamped message
Returns TRUE if invalid and FALSE if valid
'''
def check_invalid_PoseStamped(ps):
	return (ps.pose.position.x == -1 or ps.pose.position.y == -1 or ps.pose.position.z == -1 or ps == PoseStamped())

