//This program spawns a new turtlesim turtle by calling
// the appropriate service.
#include <ros/ros.h>
//The srv class for the service.
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <turtle_ros_service/Changespeed.h>  // For turtle_ros_service::Changespeed

ros::Publisher *pubPtr;

void commandVelocityMessageReceived(
    const geometry_msgs::Twist& msgIn)
{
    geometry_msgs::Twist msgOut;
    msgOut.linear.x = msgIn.linear.x;
    msgOut.angular.z = msgIn.angular.z;
    pubPtr->publish(msgOut);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "spawn_turtle_plus");
    ros::NodeHandle nh;

  //Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel", 1000,
    &commandVelocityMessageReceived);

  //publisher
  pubPtr = new ros::Publisher(
      nh.advertise<geometry_msgs::Twist>(
          "MyTurtle/cmd_vel",
          1000));

  //Create a client object for the spawn service. This
  //needs to know the data type of the service and its name.
    ros::ServiceClient spawnClient
		    = nh.serviceClient<turtlesim::Spawn>("spawn");

  //Create a client object for the change_speed service. This
  //needs to know the data type of the service and its name.
  ros::ServiceClient changeSpeedClient
		    = nh.serviceClient<turtle_ros_service::Changespeed>("change_speed");


  //Create a client object for the toggle_forward service. This
  //needs to know the data type of the service and its name.
  ros::ServiceClient toggleForwardClient
    		= nh.serviceClient<std_srvs::Empty>("toggle_forward");

  //Create the request and response objects for the spawn service
  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Response resp;
  req.x = 2;
  req.y = 3;
  req.theta = M_PI/2;
  req.name = "MyTurtle";

  //call the spawn service
  ros::service::waitForService("spawn", ros::Duration(5));
  bool success = spawnClient.call(req,resp);

  if(success){
	    ROS_INFO_STREAM("Spawned a turtle named "<< resp.name);
  }else{
	    ROS_ERROR_STREAM("Failed to spawn.");
  }


  //Create the request and response objects for the toggle_forward service
  std_srvs::Empty::Request req2;
  std_srvs::Empty::Response resp2;

  //call the toggle_forward service
  ros::service::waitForService("toggle_forward", ros::Duration(5));
  bool success2 = toggleForwardClient.call(req2, resp2);

  if(success2){
  	    ROS_INFO_STREAM("toggle_forward called with success");
  }else{
  	    ROS_ERROR_STREAM("Failed to toggle_forward.");
  }

  //Create the request and response objects for the change_speed service
  turtle_ros_service::Changespeed::Request req3;
  turtle_ros_service::Changespeed::Response resp3;
  req3.newspeed = 15;

  //call the change_speed service
  ros::service::waitForService("change_speed", ros::Duration(5));
  bool success3 = changeSpeedClient.call(req3,resp3);
  if(success3){
    ROS_INFO_STREAM("Changed speed to "<< req3.newspeed);
  }else{
    ROS_ERROR_STREAM("Failed Changed speed");
  }



  ros::spin();


}
