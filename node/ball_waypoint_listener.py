#!/usr/bin/env python

'''
TF Listener Node:
Subscribes to selected rostopics and listens to transforms
Outputs a /data/log.txt (csv format) file for post processing
Subscribed:
    /gazebo/model_states
    /target_pose
    /target_waypoint
    tf transform of 'target' in 'follower' (parent) frame
    tf transform of 'leader' in 'follower' (parent) frame
Written By: Svena Yu, March 2021
'''

## ROS
import rospy
from std_msgs.msg import Int32, Int32MultiArray, UInt8MultiArray, Float32MultiArray, Float32, String
from smartcarts.msg import Float32Stamped, Float32MultiArrayStamped, BallPoseStamped
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist, Quaternion, PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
## tf
import tf2_ros
import tf_conversions
from tf.transformations import euler_from_quaternion
## Others
import os, sys, time

## Contains callback functions for all subscribed topics that we are interested in tracking
class waypoint_listener:
    def __init__(self):
        # Publishers and Subscribers
        self.states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.states_sub_callback)
        self.ball_radius_sub = rospy.Subscriber('/ball_radius', Float32, self.ball_radius_callback)
        self.ball_distance_sub = rospy.Subscriber('/ball_distance', Float32, self.ball_distance_callback)
        self.target_pose_sub = rospy.Subscriber('/target_pose', PoseStamped, self.target_pose_callback)
        self.target_waypoint_sub = rospy.Subscriber('/target_waypoint', PoseStamped, self.target_waypoint_callback)
        # General Variables
        self.parent_name = "follower"
        self.child_name = "leader"
        self.target_name = "target"
        # Data Arrays (Note: time recorded in nsecs)
        self.time_now = 0; self.stamp = 0; self.x = 1; self.y = 2; self.euler_y = 3; # idx for arrays
        self.parent_data = [0,0,0,0] # idx: 0->Stamp, 1->Point.x, 2->Point.y, 3->Euler.y
        self.child_data = [0,0,0,0] # idx: 0->Stamp, 1->Point.x, 2->Point.y, 3->Euler.y
        self.ball_data = [0,0,0] # idx: 0->Stamp, 1->Ball Radius, 2->Ball Distance
        self.target_pose_data = [0,0,0] # idx: 0->Stamp, 1->x, 2->y
        self.target_waypoint_data = [0,0,0] # idx: 0->Stamp, 1->x, 2->y
        time.sleep(2.0)

    # Model State Subscriber Callback Function, Parses and saves into parent_data & child_data array
    def states_sub_callback(self, states):
        if (len(states.name) != 3):
            return
        parent_idx = states.name.index(self.parent_name)
        child_idx = states.name.index(self.child_name)
        stamp = rospy.Time.now()
        self.parent_data[self.stamp] = stamp.nsecs
        stamp = rospy.Time.now()
        self.child_data[self.stamp] = stamp.nsecs
        pt = states.pose[parent_idx].position
        self.parent_data[self.x] = pt.x
        self.parent_data[self.y] = pt.y
        pt = states.pose[child_idx].position
        self.child_data[self.x] = pt.x
        self.child_data[self.y] = pt.y
        parent_q = states.pose[parent_idx].orientation
        r,p,y = euler_from_quaternion((parent_q.x, parent_q.y, parent_q.z, parent_q.w))
        self.parent_data[self.euler_y] = y
        child_q = states.pose[child_idx].orientation
        r,p,y = euler_from_quaternion((child_q.x, child_q.y, child_q.z, child_q.w))
        self.child_data[self.euler_y] = y

    # Ball Radius Callback, Saves into ball_data array
    def ball_radius_callback(self, radius):
        self.ball_data[0] = rospy.Time.now().nsecs
        self.ball_data[1] = radius.data

    # Ball Distance Callback, Saves into ball_data array
    def ball_distance_callback(self, distance):
        self.ball_data[0] = rospy.Time.now().nsecs
        self.ball_data[2] = distance.data

    def target_pose_callback(self, poseStamped):
        self.target_pose_data[self.stamp] = poseStamped.header.stamp.nsecs
        self.target_pose_data[self.x] = poseStamped.pose.position.x
        self.target_pose_data[self.y] = poseStamped.pose.position.y

    def target_waypoint_callback(self, wp):
        self.target_waypoint_data[self.stamp] =  wp.header.stamp.nsecs
        self.target_waypoint_data[self.x] = wp.pose.position.x
        self.target_waypoint_data[self.y] = wp.pose.position.y

# Takes in file to write to (f), waypoint_listener class (wl)
def main(f, wl):
    rate = rospy.Rate(10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rel_data = [0,0,0,0] # idx: 0->Stamp, 1->Point.x, 2->Point.y, 3->Euler.y
    target_data = [0,0,0,0] # idx: 0->Stamp, 1->Point.x, 2->Point.y, 3->Euler.y
    # rel_tf_-- refers to child rel to parent transform
    # target_tf -- refers to target rel to parent transform
    col_names = "time_now, ball_time, ball_radius, ball_distance, " + \
                "target_pose_time, target_pose_x, target_pose_y, " + \
                "target_waypoint_time, target_waypoint_x, target_waypoint_y, " + \
                "parent_time, parent_pt_x, parent_pt_y, parent_rpy_y, " + \
                "child_time, child_pt_x, child_pt_y, child_rpy_y, " + \
                "rel_tf_time, rel_tf_pt_x, rel_tf_pt_y, rel_tf_rpy_y, " + \
                "target_tf_time, target_tf_pt_x, target_tf_pt_y, target_tf_rpy_y" + "\n"
    f.write(col_names)

    while not rospy.is_shutdown():
        try:
            target_tf = tfBuffer.lookup_transform('follower', 'target', rospy.Time(), rospy.Duration(1)) # Based on Ball Tracking
            rel_tf = tfBuffer.lookup_transform('follower', 'leader', rospy.Time()) # Based on Robot tf
            # Parent frame, Child frame, returns TransformStamped message
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        time_now = rospy.Time.now()
        target_data[0] = target_tf.header.stamp.nsecs
        target_data[1] = target_tf.transform.translation.x
        target_data[2] = target_tf.transform.translation.y
        tf_q = target_tf.transform.rotation
        target_data[3] = euler_from_quaternion((tf_q.x, tf_q.y, tf_q.z, tf_q.w))[2]
        rel_data[0] = rel_tf.header.stamp.nsecs
        rel_data[1] = rel_tf.transform.translation.x
        rel_data[2] = rel_tf.transform.translation.y
        tf_q = rel_tf.transform.rotation
        rel_data[3] = euler_from_quaternion((tf_q.x, tf_q.y, tf_q.z, tf_q.w))[2]
        sep = ", "
        string = str(time_now) + sep + sep.join([str(el) for el in wl.ball_data]) + \
                    sep + sep.join([str(el) for el in wl.target_pose_data]) + \
                    sep + sep.join([str(el) for el in wl.target_waypoint_data]) + \
                    sep + sep.join([str(el) for el in wl.parent_data]) + \
                    sep + sep.join([str(el) for el in wl.child_data]) + \
                    sep + sep.join([str(el) for el in rel_data]) + \
                    sep + sep.join([str(el) for el in target_data]) + "\n"
        f.write(string)

if __name__ == '__main__':
    rospy.init_node('waypoint_listener', anonymous=True)
    wl = waypoint_listener()

    dir_path = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(dir_path,"data/log.txt")
    f = open(file_path, "w")
    main(f, wl)

    f.close()
    print("file closed")
