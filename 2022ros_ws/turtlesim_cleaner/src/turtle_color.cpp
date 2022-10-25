
#include <string>
#include "ros/ros.h"
// TODO include header for turtlesim::Pose
// TODO include header needed to call the "clear"-service of turtlesim

// service to be called later
std_srvs::Empty clear_srv;

// parameter values
std::string x_mapping;
std::string y_mapping;

// max turtle coordinate at x- and y-axis
const float max_coord_val = 11 + (1/9.0f);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
  // calculate colors
  // TODO calculate color values for axes in the range of 0 to 255 (beginning to end of axes)
  // TODO use the current position and the maximum position defined in max_coord_val
  // int x_color = 
  // int y_color = 

  // TODO set calculated color values to parameter server depending on which mapping for the axes was selected (variables x_mapping and y_mapping)

  // TODO call service "clear" to renew background

}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_color");
  // check parameters
  if (argc != 4)
  {
    ROS_INFO("Usage: turtle_color [color channel] [color channel] [value]");
    ROS_INFO("You must provide two different color channels (r, g or b), the value must be an integer from 0 to 255.");
    return 1;
  }
  std::string arg_var1 = std::string(argv[1]);
  std::string arg_var2 = std::string(argv[2]);
  if ((arg_var1 != "r" && arg_var1 != "g" && arg_var1 != "b") || (arg_var2 != "r" && arg_var2 != "g" && arg_var2 != "b"))
  {
    ROS_INFO("The color channels have to be: r, g or b.");
    return 1;
  }
  if (atol(argv[3]) < 0 || atol(argv[3]) > 255)
  {
    ROS_INFO("The color value must be in the range from 0 to 255.");
    return 1;
  }
  if (argv[1] == argv[2])
  {
    ROS_INFO("You must provide two different color channels (r, g or b).");
    return 1;
  }

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/turtle1/pose", 1, poseCallback);

  // TODO set color to axes mapping from user's input on command line
  // x_mapping = 
  // y_mapping =

  // TODO set fixed value from user's input on command line to parameter server

  // TODO enter callback loop

  return 0;
}
