#include <ros/ros.h>
#include "geometry_msgs/Twist.h" 
#include "geometry_msgs/Pose2D.h"
double v, w ;

void callback(const geometry_msgs::Twist& message) {
  ROS_INFO("received velocity command value is: %f , %f", message.linear.x, message.angular.z);
  v = message.linear.x;
  w = message.angular.z;
}
// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master as a "node" called "hello_world_node"
  ros::init(argc, argv, "node1");
  // Start the node resource managers (communication, time, etc)
  ros::NodeHandle n;

  ros::Subscriber recvelocity = n.subscribe("/cmd_vel", 1, callback);

  // Publish velocity commands to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  ros::Publisher currentPose = n.advertise<geometry_msgs::Pose2D>("/Pose2D", 1);

  geometry_msgs::Pose2D pubPose; 
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  // Specify a frequency that want the while loop below to loop at
  // In this case, we want to loop 10 cycles per second
  ros::Rate naptime(10); 
  // Broadcast a simple log message
  ROS_INFO_STREAM("Task1");
  pubPose.x = 0.0;
  pubPose.y = 0.0;
  pubPose.theta = 0.0;
  currentPose.publish(pubPose);
  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  while (ros::ok()) 
  {
    ros::spinOnce(); //allow data update from callback;
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = v * cos(th) * dt;
    double delta_y = v * sin(th) * dt;
    double delta_th = w * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    pubPose.x = x;
    pubPose.y = y;
    pubPose.theta = th*(180.0/3.141592653589793238463);
    ROS_INFO_STREAM("Pulishing");
    currentPose.publish(pubPose); 
    last_time = current_time;
    naptime.sleep(); // wait for remainder of specified period; this loop rate is faster than 
        // the update rate of the 10Hz controller that specifies force_cmd 
        // however, simulator must advance each 10ms regardless 
  }
    return 0; // should never get here, unless roscore dies 
}