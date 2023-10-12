#include "ros/ros.h"
#include "custom_msgs/velocities.h"
#include <sstream>
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_publisher_node_custom_msgs");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<custom_msgs::velocities>("velocity", 100);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    custom_msgs::velocities msg;
    msg.leftvel = 10.3;
    msg.rightvel = 12.7;
     
    pub.publish(msg);
    ros::spinOnce();
    
    loop_rate.sleep();
  }
  return 0;
}