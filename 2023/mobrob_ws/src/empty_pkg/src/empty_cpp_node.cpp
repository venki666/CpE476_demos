#include "ros/ros.h"
#include <ros/package.h>

int main(int argc, char* argv[])
{
    // Initialise the node
    ros::init(argc, argv, "empty_cpp_node");
    // Start the node by initialising a node handle
    ros::NodeHandle nh("~");
    // Display the namespace of the node handle
    ROS_INFO_STREAM("EMPTY CPP NODE] namespace of nh = " << nh.getNamespace());
    // Spin as a single-threaded node
    ros::spin();
    // Main has ended, return 0
    return 0;
}
