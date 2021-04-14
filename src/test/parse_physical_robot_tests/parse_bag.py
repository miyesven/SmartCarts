import rosbag
from nav_msgs.msg import Odometry
import numpy as np

bag = rosbag.Bag('carpet.bag')
f = open("carpet.csv", 'w')
for topic, msg, t in bag.read_messages(topics=['/odom']):
	t = t
	nsecs = msg.header.stamp.nsecs
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	qw = msg.pose.pose.orientation.w
	f.write('{},{},{},{},{}\n'.format(t, nsecs, x, y, qw))
bag.close()