//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <turtle_ros_service/Changerate.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;

bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        forward = !forward;
        ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
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
        ros::init(argc,argv,"pubvel_toggle_rate");
	ros::NodeHandle nh;
        
	ros::ServiceServer server = 
		nh.advertiseService("toggle_forward",&toggleForward);
                
        ros::ServiceServer server0 =
                nh.advertiseService("change_rate",&changeRate);
                
        ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
        ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
                msg.linear.x = forward?1.0:0.0;
                msg.angular.z=forward?0.0:1.0;
		pub.publish(msg);
		ros::spinOnce();
                if(ratechanged) {
                    rate = ros::Rate(newfrequency);
                    ratechanged = false;
                }
		rate.sleep();
	}
}
