#include <ros/ros.h>
// TODO include header for the laserscan message from package sensor_msgs
// TODO include header for the marker message from package visualization_msgs
#include <Eigen/Core>

ros::Publisher marker_pub;
visualization_msgs::Marker marker_circles;

void createMarker(visualization_msgs::Marker& marker, const int type, const int id, const Eigen::Vector3f color, const Eigen::Vector3f size,
                  std::string marker_namespace)
{
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/laser";
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


void transform_coords(const sensor_msgs::LaserScan& scan, std::vector<Eigen::Vector3f> &coords)
{
    // TODO transform every point from "scan" from spherical to cartesian coordinates, store results in "coords"
    // hint: take a look at the documentation of the sensor_msgs::LaserScan message at
    //       http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
}

void find_segments(const float minDist, const std::vector<Eigen::Vector3f> &point_sequence, std::vector<int> &segment_indices)
{
    // TODO find segments in laser scan using "point_sequence", store results in "segment_indices"
    // hint: "segment_indices" should contain an even number of indices, indices at even positions mark the beginning of
    //       a segment, indices at odd positions the end of a segment (i.e. the segment that begins at point with
    //       index at position 0 ends at the point with index at position 1)
    // hint: a new segment starts if the distance between two adjacent points is greater than "minDist"
}


void find_leg_segments(const float max_leg_size, const std::vector<Eigen::Vector3f> &point_sequence,
                       const std::vector<int> &segment_indices, std::vector<int> &leg_segment_indices)
{
    // TODO from all found segments in "segment_indices" find segments that could be legs (i.e. distance from beginning
    //      to the end of the segment ist not greater than "max_leg_size", store results in "leg_segment_indices"
    // hint: use "point_sequece" to determine the distances
}


void check_leg_segments(const std::vector<Eigen::Vector3f> &point_sequence, const std::vector<int> &leg_segment_indices,
                        std::vector<Eigen::Vector3f> &circle_centers)
{
    // TODO iterate over all possible leg segments in "leg_segment_indices" and test whether they are formed like half-circles
    //      the center of each found half-circle has to be stored in "circle_centers"
    // hint: take a look at the task description for a half-circle test algorithm
}


void add_legs_to_marker(const std::vector<Eigen::Vector3f> &circle_centers, visualization_msgs::Marker &marker_circles)
{
    // TODO add all circle centers from "circle_centers" to the marker "marker_circles"
    // hint: take a look at the description of the visualization_msgs::Marker message at
    //       http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
}


void laserDataCallback(const sensor_msgs::LaserScan& scan)
{
    // vector to store all laser points in carthesian coordinates
    std::vector<Eigen::Vector3f> point_sequence;
    // convert each point in scan to carthesian coordinates
    transform_coords(scan, point_sequence);


    // store indices where segments begin and end
    std::vector<int> segment_indices;
    // separation threshold between two segments
    float minDist = 0.25f; /*TODO use this or find better threshold*/
    find_segments(minDist, point_sequence, segment_indices);


    // max length of segments that could be legs
    std::vector<int> leg_segment_indices;
    float max_leg_size = 0.2f; /*TODO use this or find better threshold*/
    find_leg_segments(max_leg_size, point_sequence, segment_indices, leg_segment_indices);


    // vector to store the leg centers
    std::vector<Eigen::Vector3f> circle_centers;
    check_leg_segments(point_sequence, leg_segment_indices, circle_centers);


    // marker message object
    visualization_msgs::Marker marker_circles;
    createMarker(marker_circles, visualization_msgs::Marker::SPHERE_LIST, 3, Eigen::Vector3f(1,1,0), Eigen::Vector3f(0.2, 0.2, 0), "object_marker");
    add_legs_to_marker(circle_centers, marker_circles);

    // publish marker
    marker_pub.publish(marker_circles);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_leg_detection");
    ros::NodeHandle n;

    // subscribe to topics
    ros::Subscriber sub = n.subscribe("scan", 10, laserDataCallback);

    // advertise topics
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::spin();

    return 0;
}
