#!/usr/bin/env python

'''
Currently unused in simulation but this is an attempt by Simon Zheng to try and setup an odometry node
'''

#Gazebo interface for ground truth position 
import rospy
from nav_msgs.msg import Odometry
from gazebo.srv import GetModelState, GetModelStateRequest

rospy.init_node ('smart_cart_odom_publisher')

odom_pub = rospy.Publisher('/smartcart_odom', Odometry)

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
header = Header()
header.frame_id = '/odom'

model = GetModelStateRequest()
model.model_name = 'robot' #name of robot

r = rospy.Rate(60)

while not rospy.is_shutdown():

   result = get_model_srv(model)
	
   odom.pose.pose = result.pose
   odom.twist.twist = result.twist

   header.stamp = rospy.Time.now()
   odom.header = header

   odom_pub.publish(odom)

   r.sleep()	
