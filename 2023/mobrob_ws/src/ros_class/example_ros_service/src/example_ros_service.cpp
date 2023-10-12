//example ROS service:
// run this as: rosrun example_ROS_service example_ROS_service
// in another window, tickle it manually with (e.g.): 
//    rosservice call lookup_by_name 'Ted'


#include <ros/ros.h>
#include <example_ros_service/example_server_msg.h>
#include <iostream>
#include <string>
using namespace std;

bool callback(example_ros_service::example_server_msgRequest& request, example_ros_service::example_server_msgResponse& response)
{
    ROS_INFO("callback activated");
    string in_name(request.name); //let's convert this to a C++-class string, so can use member funcs
    //cout<<"in_name:"<<in_name<<endl;
    response.on_the_list=false;
    
    // here is a dumb way to access a stupid database...
    // hey: this example is about services, not databases!
    if (in_name.compare("CPE476")==0)
 {
        ROS_INFO("asked about CPE476");
        response.numstuds = 26;
        response.good_class=true;
        response.on_the_list=true;
        response.nickname="CPE476TheSwag";
    } 
     if (in_name.compare("CPE403")==0)
 {
        ROS_INFO("asked about CPE403");
        response.numstuds = 21;
        response.good_class=true;
        response.on_the_list=true;
        response.nickname="CPE403Rocks";
    }    
    
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_ros_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("lookup_by_name", callback);
  ROS_INFO("Ready to look up names.");
  ros::spin();

  return 0;
}
