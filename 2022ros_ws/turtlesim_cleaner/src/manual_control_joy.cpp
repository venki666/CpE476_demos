#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

double ang = 0.0, lin = 0.0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // control via joystick 
	ang = joy->axes[0];
	lin = joy->axes[1];

    // if button is pushed, robot is stopped
    bool stop = joy->buttons[0] || joy->buttons[1] || joy->buttons[2] || joy->buttons[3];
    if(stop)
    {
        ang = 0;
        lin = 0;
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "manual_control_joy");

	ros::NodeHandle nh_;
	ros::Subscriber information_joy = nh_.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
	ros::Publisher vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	geometry_msgs::Twist drive;

	ros::Rate loop_rate(10);

	while (nh_.ok())
	{
		drive.angular.z = ang;
		drive.linear.x = lin;
		ROS_INFO_STREAM("Linear: " << lin << " Angular: " << ang);
		vel_pub_.publish(drive);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
