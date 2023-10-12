#include "ros/ros.h"
#include "custom_msgs/velocities.h"
 
void messageCallback(const custom_msgs::velocities::ConstPtr& msg) {
  ROS_INFO("I have received: [%f] [%f]", msg->leftvel, msg->rightvel);
}
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_subscriber_node_custom_msgs");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("velocity", 1000, messageCallback);
  ros::spin();
  return 0;
}