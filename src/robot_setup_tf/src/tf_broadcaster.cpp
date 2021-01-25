#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;
	ros::Rate r(100);
	tf::TransformBroadcaster broadcaster;

	while(n.ok()){
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0,1), //pitch roll yaw = 0 = no rotation
							  tf::Vector3(0.1, 0.0, 0.2)), //x, y, z offset (m) - translation
							  ros::Time::now(), //timestamp of transform
							  "base_link", //parent node
							  "base_laser")); //child node
		r.sleep();
	}
}