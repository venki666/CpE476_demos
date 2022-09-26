
#include <string>
#include "ros/ros.h"
#include "tf/LinearMath/Quaternion.h"
#include "turtlesim/Pose.h"
#include "std_srvs/Empty.h"
#include <Eigen/Core>
// TODO include header for the marker message

ros::Publisher marker_pub;
ros::Publisher marker_path_pub;

static int counter = 0;

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

// TODO extend this method to also take a parameter of type ros::Duration to set the marker lifetime
void createMarker(visualization_msgs::Marker& marker, const int type, const int id, const Eigen::Vector3f color, const Eigen::Vector3f size,
                  std::string marker_namespace)
{
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = marker_namespace;
    marker.id = id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = type;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = size[0];
    marker.scale.y = size[1];
    marker.scale.z = size[2];

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    // create a quaternion with the current orientation for later use
    tf::Quaternion orientation(/*TODO fill in values*/);

    // create arrow marker for current position, always use same ID
    visualization_msgs::Marker marker_arrow;
    // TODO call method to create marker of type ARROW

    // TODO fill in the position and orientation (replace zeros)
    marker_arrow.pose.position.x = 0;
    marker_arrow.pose.position.y = 0;
    marker_arrow.pose.orientation.x = 0;
    marker_arrow.pose.orientation.y = 0;
    marker_arrow.pose.orientation.z = 0;
    marker_arrow.pose.orientation.w = 0;

    // TODO publish marker with ARROW


    // TODO use the global variable "counter" to only send 4 SHERE markers of the turtles path per second
    if(/*TODO fill in*/)
    {
        // a new marker with the current position, use unique ID
        visualization_msgs::Marker waypoint_marker;
        // TODO call method to create marker of type SPHERE, use unique ID, marker should disappear after 10 seconds

        // TODO fill in the position and orientation

        // TODO publish marker with SPHERE
    }

}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_markrers");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(/* TODO fill in the topic of the turtle position */, 1, poseCallback);

  // advertise topics
  marker_pub = n.advertise<visualization_msgs::Marker>("/turtle_position", 1);
  marker_path_pub = n.advertise<visualization_msgs::Marker>("/turtle_path", 1);

  // TODO something is missing here ... what do we need to make callback calls happen?

  return 0;
}
