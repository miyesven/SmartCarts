#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>

//wheel_rad is the wheel radius in meters, wheel_sep is separation distance between the wheels in meters
//I measured wheel_rad=4.25"=0.10795m~0.108m (wheel diameter is approx. 8.5"), wheel_sep=24.75"=0.62865m~0.629m
#define WHEEL_RAD 0.108
#define WHEEL_SEP 0.629

#define ENC_CPR 480 //with quadrature and the gear ratio, there are 480 cpr (counts per revolution)


double distancePerCount = (3.14159 * 2 * WHEEL_RAD)/ENC_CPR;
int leftCount_current = 0;
int rightCount_current = 0;
int leftCount_last = 0;
int rightCount_last = 0;
int deltaLeft;
int deltaRight;
double omega_left;
double omega_right;
double vLeft;
double vRight;
double vx;
double vy;
double vth;


void process_lCount( const std_msgs::Int16::ConstPtr& lCount){
  leftCount_current = lCount->data;
}

void process_rCount( const std_msgs::Int16::ConstPtr& rCount){
  rightCount_current = rCount->data;
}deltaLeft = leftCount_current - leftCount_last;


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_enc_publisher");

  ros::NodeHandle n;  //create node variable n
  //create a publisher called "odom_enc_pub" that publishes a message of type "nav_msgs::Odometry" to the topic "odom_enc" 
  //with a queue size of 50 (50 outgoing messages in the queue)
  ros::Publisher odom_enc_pub = n.advertise<nav_msgs::Odometry>("odom_enc", 50);  
  //create a transform broadcaster called "odom_enc_broadcaster"
  tf::TransformBroadcaster odom_enc_broadcaster;

  //create subscriber "lCount_sub" that subscribes to the "lCount" topic, has a queue size of 50 messages, 
  //and calls the function "process_lCount" whenever a new message arrives
  ros::Subscriber lCount_sub = n.subscribe("lCount", 50, process_lCount);
  
  //create subscriber "rCount_sub" that subscribes to the "rCount" topic, has a queue size of 50 messages, 
  //and calls the function "process_rCount" whenever a new message arrives
  ros::Subscriber rCount_sub = n.subscribe("rCount", 50, process_rCount);


  ros::Time current_time, last_time;
  current_time = ros::Time::now();  //needed here?
  last_time = ros::Time::now();

  ros::Rate r(5.0); //publish odometry info at 5.0Hz

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();


    //an Odometry message consists of a "pose" message and a "twist" message. Will now get the left and right wheel encoder counts to 
    //compute x,y,th values for the "pose" message, and vx,vy,vth values for the "twist" message
    deltaLeft = leftCount_current - leftCount_last;
    deltaRight = rightCount_current - rightCount_last;

    omega_left = (deltaLeft * distancePerCount) / (current_time - last_time);
    omega_right = (deltaRight * distancePerCount) / (current_time - last_time);

    vLeft = omega_left * WHEEL_RAD;
    vRight = omega_right * WHEEL_RAD;

    vx =  (vRight - vLeft) / 2;
    vy = 0;
    vth = (vRight - vLeft) / WHEEL_SEP;


    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_enc_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_enc_trans;
    odom_enc_trans.header.stamp = current_time;
    odom_enc_trans.header.frame_id = "odom_enc";
    odom_enc_trans.child_frame_id = "base_link";

    odom_enc_trans.transform.translation.x = x;
    odom_enc_trans.transform.translation.y = y;
    odom_enc_trans.transform.translation.z = 0.0;
    odom_enc_trans.transform.rotation = odom_enc_quat;

    //send the transform
    odom_enc_broadcaster.sendTransform(odom_enc_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom_enc;
    odom_enc.header.stamp = current_time;
    odom_enc.header.frame_id = "odom_enc";

    //set the position
    odom_enc.pose.pose.position.x = x;
    odom_enc.pose.pose.position.y = y;
    odom_enc.pose.pose.position.z = 0.0;
    odom_enc.pose.pose.orientation = odom_enc_quat;

    //set the velocity
    odom_enc.child_frame_id = "base_link";
    odom_enc.twist.twist.linear.x = vx;
    odom_enc.twist.twist.linear.y = vy;
    odom_enc.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom_enc);

    last_time = current_time;
    leftCount_last = leftCount_current;
    rightCount_last = rightCount_current;

    r.sleep();
  }
}