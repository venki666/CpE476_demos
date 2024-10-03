#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
#include "turtlesim/Pose.h"

bool flag;

void callback(turtlesim::Pose msg){
      
    flag = (msg.x < 2) || (msg.x > 8) || (msg.y < 2) || (msg.y > 8);

}

int main( int argc, char **argv){

    ros::init(argc, argv, "auto_turtle");
    ros::NodeHandle nh;
    ros::Publisher pub_handle_cpp = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Subscriber sub_handle_cpp = nh.subscribe("/turtle1/pose", 10, callback);

    geometry_msgs::Twist output;

    // linear_data
    output.linear.x = 1;
    output.linear.y = 0;
    output.linear.z = 0;

    // angular_data
    output.angular.x = 0;
    output.angular.y = 0;
    output.angular.z = 0;

    ros::Rate loop_hz(10);

    while(ros::ok()){

        if (flag){
            output.angular.z = rand()/RAND_MAX-1;
        }
        else {
            output.angular.z = 0;
        }

        pub_handle_cpp.publish(output);
        loop_hz.sleep();
        
        ros::spinOnce();
    }
    
    return 0;
}