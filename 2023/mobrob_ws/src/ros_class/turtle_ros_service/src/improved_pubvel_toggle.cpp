//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <turtle_ros_service/Changerate.h>
#include <turtle_ros_service/Changespeed.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;
bool start = true;
double speed=1.0;

bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
  forward = !forward;
  ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
	return true;
}

bool startStop(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp){
        start = !start;
        ROS_INFO_STREAM("I'm "<<(start?
                "starting":"stopping"));
        return true;
}

bool changeSpeed(
        turtle_ros_service::Changespeed::Request &req,
        turtle_ros_service::Changespeed::Response &resp){

        ROS_INFO_STREAM("Changing velocity to "<<req.newspeed);
        speed = req.newspeed;

        return true;
}

bool changeRate(
        turtle_ros_service::Changerate::Request &req,
        turtle_ros_service::Changerate::Response &resp){

        ROS_INFO_STREAM("Changing rate to "<<req.newrate);

        newfrequency = req.newrate;
        ratechanged = true;
        return true;
}


int main(int argc, char **argv){
  double vel;
  ros::init(argc,argv,"improved_pubvel_toggle");
	ros::NodeHandle nh;

	ros::ServiceServer server =
		nh.advertiseService("toggle_forward",&toggleForward);

  ros::ServiceServer server0 =
    nh.advertiseService("change_rate",&changeRate);

  ros::ServiceServer server2 =
    nh.advertiseService("startstop",&startStop);

  ros::ServiceServer server3 =
    nh.advertiseService("change_speed",&changeSpeed);


  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);


  ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
    vel = start?speed:0.0;
    msg.linear.x = forward?vel:0.0;
    msg.angular.z=forward?0.0:vel;
		pub.publish(msg);
		ros::spinOnce();
    if(ratechanged) {
      rate = ros::Rate(newfrequency);
      ratechanged = false;
    }
		rate.sleep();
	}
}
