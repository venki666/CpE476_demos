#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>

int main( int argc, char **argv){

    ros::init(argc, argv, "auto_turtle");
    ros::NodeHandle nh;
    ros::Publisher pub_handle_cpp = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    geometry_msgs::Twist output;

    // linear_data
    output.linear.x = 1;
    output.linear.y = 0;
    output.linear.z = 0;

    // angular_data
    output.angular.x = 0;
    output.angular.y = 0;
    // output.angular.z = 0.5;

    ros::Rate loop_hz(10);

    while(ros::ok()){
        
        output.angular.z = (double)rand() / (double)RAND_MAX;
        pub_handle_cpp.publish(output);
        loop_hz.sleep();
    }
    
    return  0;
}