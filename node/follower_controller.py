#!/usr/bin/env python

'''
Modified from "Simple node to run dead-reckoning target tracking (proportional speed / angular)" by Simon Z.
Currently, this code tries to follow the waypoints that the follower sees. Adapted from leader_controller.py
Subscribed:
    /gazebo/model_states
    /target_waypoint
Published:
    /<robot_name>/cmd_vel
Written by Svena, Mar 2021
'''
import csv
import math
import numpy as np
import rospy


# Some scaling constants
speed_scale = 0.7
min_distance = 0.7
angular_scale = 0.007
deadband = 5
image_width = 640.0 # depth image width in pixels
image_mid = image_width/2

def heading_cb(msg):
    position_x = msg.data[0]
    if(np.isnan(position_x) or position_x == -1.0):
        move_cmd.angular.z = 0
    else:
        if(position_x > (image_mid + deadband)):
            # print("RIGHT")
            move_cmd.linear.x = move_cmd.linear.x
            move_cmd.angular.z = angular_scale * (image_mid - position_x)
            angular_z_last = move_cmd.angular.z
        if(position_x < (image_mid - deadband)):
            # print("LEFT")
            move_cmd.linear.x = move_cmd.linear.x
            move_cmd.angular.z = angular_scale * (image_mid - position_x)
            angular_z_last = move_cmd.angular.z

def speed_cb(msg):
    #print(msg.data)
    if(np.isnan(msg.data) or msg.data == -1.0):
        move_cmd.linear.x = 0
    elif(msg.data > min_distance):
        move_cmd.linear.x = msg.data*speed_scale
    else:
        move_cmd.linear.x = 0

import random
from std_msgs.msg import Int32MultiArray, Float32, Int32
from geometry_msgs.msg import Twist, PoseStamped, Pose, PoseArray, Vector3, Point
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
# Custom imports
from smartcarts.msg import Float32Stamped, Float32MultiArrayStamped, BallPoseStamped
from interfaces import invalid_PoseStamped, check_invalid_PoseStamped
from states import State, F_STATE_0, F_STATE_1, F_STATE_2, F_STATE_3, F_STATE_4

class Waypoint_Follower():
    def __init__(self):
        rospy.sleep(1.0)
        self.robot_name = rospy.get_namespace().replace('/','') # should be 'follower'
        self.min_follow_distance = rospy.get_param('/min_follow_distance')
        self.min_camera_distance = rospy.get_param('/min_camera_distance')
        # Publishers and Subscribers
        # rviz interface
        self.waypoint_pub = rospy.Publisher('/{}/waypoints_viz'.format(self.robot_name), MarkerArray, queue_size = 1)
        # state output
        self.cmd_vel_pub = rospy.Publisher('/{}/cmd_vel'.format(self.robot_name), Twist, queue_size=10)
        self.cmd_vel = Twist()
        self.leader_help_pub = rospy.Publisher('/{}/leader_help'.format(self.robot_name), Int32, queue_size=1)
        # state input
        self.waypoint_sub = rospy.Subscriber('/target_waypoint', PoseStamped, self.waypoint_goal_callback, queue_size=10)
        self.target_distance_sub = rospy.Subscriber('/ball_distance', Float32, self.distance_callback, queue_size=10)
        self.goal_pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_feedback_callback, queue_size=10)
        self.robot_pose = None; self.current_pose_goal = None; # Pose
        self.pose_goal_buffer = [] # List of Pose
        self.distance = 0.0
        # state variables
        self.L_HELP_STATE_1 = 1 # Help me, leader should stop
        self.L_HELP_STATE_0 = 0 # Everything is nominal
        self.F_STATE = F_STATE_0() # initialize state machine
        # thresholds
        self.heading_deadband = 0.20; self.position_deadband = 0.15; self.max_angular_vel = math.pi/4; self.max_linear_vel = 1
        ## Some variables for visualization
        self.marker_array = MarkerArray()
        self.prev_wp = None # Check previous target_waypoint pose value

    '''
    Check if two poses are the same. Only comparing position values
    Returns TRUE if same, FALSE if different
    '''
    def check_same_pose(self, wp1, wp2):
        return (wp1.position.x == wp2.position.x and \
                wp1.position.y == wp2.position.y and \
                wp1.position.z == wp2.position.z)


    # Renumber marker array after adding or removing a marker
    def renumber_markers(self):
        id = 0
        for m in self.marker_array.markers:
            m.id = id
            id += 1

    # Adds marker to marker array, define style here
    def add_marker(self, pose):
        if (len(self.marker_array.markers) >= 1):
            prev_marker_pose = self.marker_array.markers[-1].pose
            if self.check_same_pose(prev_marker_pose, pose):
                return
        print("Marker array len = {}".format(len(self.marker_array.markers)))
        marker = Marker();
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.03; marker.scale.y = 0.03; marker.scale.z = 0.05
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
        marker.pose = pose;
        marker.pose.orientation.w = 1.0 #pointing straight up
        self.marker_array.markers.append(marker)
        # renumber marker IDs to cap MAX
        self.renumber_markers()

    '''
    Callback function for /target_waypoint. Populates self.current_pose_goal
    '''
    def waypoint_goal_callback(self, wp_stamped):
        # print("Check invalid pose = {}".format(check_invalid_PoseStamped(wp_stamped)))
        if not check_invalid_PoseStamped(wp_stamped) and \
	        	not isinstance(self.F_STATE, F_STATE_3) and \
	        	not isinstance(self.F_STATE, F_STATE_2) and \
	        	self.cmd_vel.angular.z == 0:
            # Initialise
            if self.prev_wp is None:
                self.prev_wp = wp_stamped.pose
            if self.current_pose_goal is None:
                self.pose_goal_buffer.append(wp_stamped.pose)
                self.current_pose_goal = self.pose_goal_buffer[0]
                print("Buffer len = {}".format(len(self.pose_goal_buffer)))
            elif not self.check_same_pose(self.prev_wp, wp_stamped.pose):
                self.pose_goal_buffer.append(wp_stamped.pose)
                self.prev_wp = wp_stamped.pose
                print("Buffer len = {}".format(len(self.pose_goal_buffer)))
            self.add_marker(wp_stamped.pose)
            self.waypoint_pub.publish(self.marker_array)

    '''
    Call back function for /target_distance. Populates self.distance
    '''
    def distance_callback(self, distance):
        self.distance = distance.data
        if np.isnan(self.distance):
            self.distance = -1.0

    '''
    Callback function for /gazebo/model_states : Follow the current waypoint based on perfect odometry from model_states
    '''
    def pose_feedback_callback(self, msg):
        cmd_vel = Twist()
        if isinstance(self.F_STATE, F_STATE_1) or isinstance(self.F_STATE, F_STATE_2):
            index = msg.name.index(self.robot_name)
            self.robot_pose = msg.pose[index]
            if self.robot_pose is not None and self.current_pose_goal is not None:
                current_rpy = euler_from_quaternion((self.robot_pose.orientation.x,      \
                                                        self.robot_pose.orientation.y,   \
                                                        self.robot_pose.orientation.z,   \
                                                        self.robot_pose.orientation.w))
                dest_rpy = euler_from_quaternion((self.current_pose_goal.orientation.x,  \
                                                        self.current_pose_goal.orientation.y,    \
                                                        self.current_pose_goal.orientation.z,    \
                                                        self.current_pose_goal.orientation.w))

                dx = self.current_pose_goal.position.x - self.robot_pose.position.x
                dy = self.current_pose_goal.position.y - self.robot_pose.position.y
                heading = math.atan2(dy, dx)
                angle = current_rpy[2]-heading
                while angle >= math.pi:
                    angle = angle - 2*math.pi
                while angle <= -math.pi:
                    angle = angle + 2*math.pi
                dist = math.hypot(dy,dx) # distance between current position and next goal

                # Rotate to the direct goal heading
                if abs(angle) > self.heading_deadband and dist > self.position_deadband:
                    if angle > self.heading_deadband:
                        cmd_vel.angular.z = -(math.pi/8 + (angle - self.heading_deadband) * self.max_angular_vel)
                    elif angle < -self.heading_deadband:
                        cmd_vel.angular.z = math.pi/8 + (self.heading_deadband - angle) * self.max_angular_vel
                # Drive forwards to goal position
                elif dist > self.position_deadband:
                    if self.distance == -1.0 and isinstance(self.F_STATE, F_STATE_2): # ignore camera distance reading if nan
                        cmd_vel.linear.x = 0.7
                    elif (self.distance - self.min_camera_distance) > 1:
                        cmd_vel.linear.x = self.max_linear_vel
                    elif (self.distance - self.min_camera_distance) > 0:
                        cmd_vel.linear.x = 0.3 + 0.7 * (self.distance - self.min_camera_distance)
                    else:
                        pass
                else:
                    if(len(self.pose_goal_buffer) >= 1):
                        ## Set the new goal
                        self.pose_goal_buffer.pop(0)
                        if len(self.pose_goal_buffer) == 0:
                        	self.current_pose_goal = None
                        else:
                        	self.current_pose_goal = self.pose_goal_buffer[0]

        self.cmd_vel = cmd_vel

    def main(self):
        rospy.sleep(rospy.Duration(0.5))  # Wait for current state to stabilize
        while not rospy.is_shutdown():
            self.waypoint_pub.publish(self.marker_array)

            # IMPLEMENTATION OF FOLLOWER STATE MACHINE
            if isinstance(self.F_STATE, F_STATE_3) and self.distance != -1.0:
                self.cmd_vel_pub.publish(Twist())  # Let target tracking node find target
                print('Time START sleep in {} = {}'.format(self.F_STATE, rospy.Time.now()))
                rospy.sleep(rospy.Duration(0.5))
                print('Time STOP sleep in {} = {}'.format(self.F_STATE, rospy.Time.now()))
            current_F_STATE = self.F_STATE.on_event(self.pose_goal_buffer, self.distance)
            if self.F_STATE != current_F_STATE:
                print("F_STATE = {}, distance = {}, buffer_len = {}".format(current_F_STATE, self.distance, len(self.pose_goal_buffer)))

            self.F_STATE = current_F_STATE

            if isinstance(self.F_STATE, F_STATE_0):
                self.leader_help_pub.publish(self.L_HELP_STATE_0)
                self.cmd_vel_pub.publish(Twist())

            if isinstance(self.F_STATE, F_STATE_1):
                self.leader_help_pub.publish(self.L_HELP_STATE_0)
                self.cmd_vel_pub.publish(self.cmd_vel)

            if isinstance(self.F_STATE, F_STATE_2):
                self.leader_help_pub.publish(self.L_HELP_STATE_0)
                self.cmd_vel_pub.publish(self.cmd_vel)

            if isinstance(self.F_STATE, F_STATE_3):
                self.leader_help_pub.publish(self.L_HELP_STATE_1)
                self.cmd_vel_pub.publish(Twist(angular=Vector3(z = self.max_angular_vel)))

            if isinstance(self.F_STATE, F_STATE_4):
                self.leader_help_pub.publish(self.L_HELP_STATE_1)
                self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    rospy.init_node('follower_controller', anonymous=True)
    cw = Waypoint_Follower()
    cw.main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
