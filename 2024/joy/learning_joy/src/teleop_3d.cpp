
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <unistd.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

 
void tfCallback(const geometry_msgs::Twist::ConstPtr& twist)
{

  //declare variables
  static tf::TransformBroadcaster br;
 tf::Transform turtle_tf(tf::Transform::getIdentity());
  static ros::Time present = ros::Time::now();
  static ros::Time before;
  tf::Transform turtle_tf_delta; //small change for frame
  Eigen::Vector3d V(3); //linear velocity
  Eigen::Matrix3d R(3,3); //rotation matrix
  Eigen::Matrix3d J(3,3); //jacobian
  
  //get a time period
  before = present;
  ros::Duration dt;
  present = ros::Time::now();
  dt = present -before;
  //update linear velocity
  V(0) = twist->linear.x * dt.toSec();
  V(1) = twist->linear.y * dt.toSec();
  V(2) = twist->linear.z * dt.toSec();
  turtle_tf_delta.setOrigin(tf::Vector3( V(0),V(1),V(2)));
  //compute and update roation matrix
  J(0,0) = 0.;
  J(0,1) = -twist->angular.z;
  J(0,2) =  twist->angular.y;
  J(1,0) =  twist->angular.z;
  J(1,1) = 0.;
  J(1,2) = -twist->angular.x;
  J(2,0) = -twist->angular.y;
  J(2,1) =  twist->angular.x;
  J(2,2) = 0.;
  J = J * dt.toSec();
  R = J.exp();
  turtle_tf_delta.setBasis(tf::Matrix3x3 (R(0,0),R(0,1),R(0,2),R(1,0),R(1,1),R(1,2), R(2,0),R(2,1),R(2,2) ));
  turtle_tf = turtle_tf * turtle_tf_delta;
  //broadcast transform
  br.sendTransform(tf::StampedTransform(turtle_tf, ros::Time::now(),"world","turtle_3d"));

}


int main(int argc, char** argv)
{

  // init ros
  ros::init(argc, argv, "teleop_3d");

  // create node handle
  ros::NodeHandle node;

  // subcscribe to twist topic
  ros::Subscriber sub = node.subscribe("turtle1/cmd_vel", 10, &tfCallback);

  // lets publish at 20 Hz
  ros::Rate rate(20);

  ros::spin();

  return 0;

};
