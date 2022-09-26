
#include <stdlib.h>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>


float maxValue = 4.0f;
ros::Publisher drive_pub_tur;
bool avoid;
float theta;

void poseCallback(const turtlesim::Pose::ConstPtr& pose)
{
    if(/*TODO insert condition saying that the turtle is too close to a wall*/)
    {
        // TODO set flag that a wall needs to be avoided
        // avoid =
        // TODO save current turtle rotation
        // theta = 
    }
}

void calculateNextMove(geometry_msgs::Twist &drive)
{
    // calculate random drive values
    // TODO calculate random float number in the range from 0 to maxValue
    //drive.linear.x =
    // TODO calculate random float number in the range from -maxValue to maxValue
    //drive.angular.z = 
}

void avoidCollision()
{
	geometry_msgs::Twist drive;
    avoid = false;

    // stop
    // TODO stop the turtle

    // TODO find a strategy to avoid walls; you can directly publish drive messages here
    // TODO note that you might need to wait a little between messages if you are publishing several of them
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_drive_square");
    ros::NodeHandle n;
 
    // subscribe to topics
    ros::Subscriber sub_tur = n.subscribe<turtlesim::Pose>("/turtle1/pose", 1, poseCallback);

    // advertise topics
    drive_pub_tur = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    // init random number generator
    srand(time(NULL));

    avoid = false;
    int counter = 0;
	geometry_msgs::Twist drive;

    // call callback at 10 Hz
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        // publish message at 1 Hz
        if(/*TODO to make the turtle drive smoothly only enter this if at 1 Hz*/)
        {
            // calculate next move
            // TODO call calculate next move
            // TODO publish drive message
        }

        // avoid collision if too close to border
        // TODO check at 10 Hz if avoid flag set and if so, enter avoidCollision method

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}


