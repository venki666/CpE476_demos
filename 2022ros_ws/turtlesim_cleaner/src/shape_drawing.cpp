/*
 * Lab01: shape_drawing.cpp
 * Created on: Sep 29, 2015
 * Author: Anis Koubaa
 * Affiliation: Prince Sultan University
 * Website:http://cs460.coins-lab.org/index.php?title=Shape_Drawing
 *
 * TODO
 * Modified by: student_name
 * Modified on:date
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace turtlesim;

//declare publishers and subscribers
Publisher velocity_publisher;
Subscriber pose_subscriber;

//global variable for storing the pose of the turtlesim
//it is updated in the callback function [poseCallback] (see below)
Pose turtlesim_pose;

/**TODO****************************************
 * declare a publisher 'object_drawn_publisher'
 * for the '/object_drawn' topic.
 * Make only the declaration, the creation must
 * be done in the main() function.
 *********************************************/

//total distance traveled by the turtlesim
int distanceTraveled = 0;

//boundaries of the turtlesim board
const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 10.0;
const double y_max = 10.0;

const double PI = 3.14159265359;

/**** functions declarations *****/
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setInitialOrientation (double desired_angle_radians);
double getDistance(double x1, double y1, double x2, double y2);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void drawC();
void drawS();
void draw4();
void drawSquare(int side); //side is expressed in unit. one unit is distance traveled by the turtlesim with a speed 1.0 and for 1.0 second
void drawTriangle(int side); //side is expressed in unit. All sides of triangle are equal
void drawCircle(int radius); //radius is expressed in unit.
void menu();

/**main function**/
int main(int argc, char **argv)
{

	ros::init(argc, argv, "shape_drawing_node");
	ros::NodeHandle n;
	int choice=0;
	int side=0;
	int radius=0;


	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	/********* TODO *******************************
	 * create a subscriber for the /turtle1/pose topic
	 * which represents the pose of the turtlesim
	 * buffer size = 10;
	 * callback function name: poseCallback(already defined)
	 *********************************************/


	/********* TODO *******************************
	 * create the publisher 'object_drawn_publisher'
	 * for the '/object_drawn' topic that you
	 * declared above
	 *********************************************/

	ros::Rate loop_rate(10);
	ros::spinOnce();

	while(ros::ok()){

		menu();
		cin>>choice;
		ros::spinOnce();
		if (choice ==1){
			drawC();
		}else if (choice ==2){
			drawS();
		}else if (choice ==3){
			draw4();
		}else if (choice ==4){
			cout<<"side: ";
			cin>>side;
			drawSquare(side);
		}else if (choice ==5){
			cout<<"side: ";
			cin>>side;
			drawTriangle(side);
		}else if (choice ==6){
			cout<<"radius: ";
			cin>>radius;
			drawCircle(radius);
		}else {
			return 0;
		}
		ros::spinOnce();

	}

	//ros::spin();

	return 0;
}

/*************************************
 * COMPLETE THE FOLLOWING FUNCTIONS  *
 *************************************/

/** update the global variable turtlesim_pose with the pose of the robot **/
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	/********* TODO *******************************
	 * 1- update the turtlesim_pose global variable with
	 * the pose received in the message pose_message
	 * of the the callback function
	 *********************************************/

	turtlesim_pose.x=0; //change the value to make it correct
	turtlesim_pose.y=0;//change the value to make it correct
	turtlesim_pose.theta=0;//change the value to make it correct
}

/** draw the letter C. Use move and rotate functions defined below **/
void drawC(){
	ROS_INFO("\nDraw the letter 'C'\n");
	/********* TODO *******************************
	 * 1- make the turtlesim draw the letter C
	 * 2- publish the message on the topic /object_drawn
	 * that contains the letter C
	 * 3- display the time spent in drawing the letter C
	 * 4- display the pose of the tutlesim
	 *********************************************/
}

/** draw the letter S. Use move and rotate functions defined below **/
void drawS(){
	ROS_INFO("\nDraw the letter 'S'\n");
	/********* TODO *******************************
	 * 1- make the turtlesim draw the letter S
	 * 2- publish the message on the topic /object_drawn
	 * that contains the letter S
	 * 3- display the time spent in drawing the letter S
	 * 4- display the pose of the tutlesim
	 *********************************************/
}

/** draw the number 4. Use move and rotate functions defined below **/
void draw4(){
	ROS_INFO("\nDraw the number '4'\n");
	/********* TODO *******************************
	 * 1- make the turtlesim draw the letter 4
	 * 2- publish the message on the topic /object_drawn
	 * that contains the letter 4
	 * 3- display the time spent in drawing the letter 4
	 * 4- display the pose of the tutlesim
	 *********************************************/
}

/** draw the shape square. Use move and rotate functions defined below **/
void drawSquare(int side){
	ROS_INFO("\nDraw the shape 'Square'\n");
	/********* TODO *******************************
	 * 1- make the turtlesim draw the shaqe square
	 * 2- publish the message on the topic /object_drawn
	 * that contains the shape square
	 * 3- display the time spent in drawing the shape square
	 * 4- display the pose of the tutlesim
	 *********************************************/
}

/** draw the shape triangle. Use move and rotate functions defined below **/
void drawTriangle(int side){
	ROS_INFO("\nDraw the shape 'Triangle'\n");
	/********* TODO *******************************
	 * 1- make the turtlesim draw the shaqe triangle
	 * 2- publish the message on the topic /object_drawn
	 * that contains the shape triangle
	 * 3- display the time spent in drawing the shape triangle
	 * 4- display the pose of the tutlesim
	 *********************************************/
}

/** draw the shape circle. Use move and rotate functions defined below **/
void drawCircle(int radius){
	ROS_INFO("\nDraw the shape 'Circle'\n");
	/********* TODO *******************************
	 * 1- make the turtlesim draw the shaqe circle
	 * 2- publish the message on the topic /object_drawn
	 * that contains the shape circle
	 * 3- display the time spent in drawing the shape circle
	 * 4- display the pose of the tutlesim
	 *********************************************/

}


/*************************************
 * PROVIDED HELPER FUNCTIONS         *
 *************************************/

/** display the menu of the application **/
void menu(){

	cout<<"*********************************************"<<endl;
	cout<<"SLECTION MENU\n"<<endl;
	cout<<"Select the drawing operation to perform\n"<<endl;
	cout<<"Press 0 to QUIT \n"<<endl;
	cout<<"Press 1 to draw 'C' \n"<<endl;
	cout<<"Press 2 to draw 'S' \n"<<endl;
	cout<<"Press 3 to draw '4' \n"<<endl;
	cout<<"Press 4 to draw a 'square' \n"<<endl;
	cout<<"Press 5 to draw a 'triangle' \n"<<endl;
	cout<<"Press 6 to draw a 'circle' \n"<<endl;
	cout<<"*********************************************"<<endl;

}

/** move the robot with a certain speed for a certain distance, forward or backward **/
void move(double speed, double distance, bool isForward){
	distanceTraveled += distance;
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

}

/** rotate the robot with a certain angular speed for a certain relative angle **/
void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(1000);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	}while(current_angle<relative_angle);

	//force the robot to stop when it reaches the desired angle
	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);
}

/** converts an angle from degrees to radians **/
double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/** rotate the robot to point to the desired angle **/
void setInitialOrientation (double desired_angle_radians){

	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
	rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);

}

/** returns the distance between point (x1, y1) and point (x2, y2) **/
double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}







