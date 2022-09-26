#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



// TODO include header for the laser scan message
// TODO include header for the point cloud message


ros::Publisher laser_pub;

void initScanMessage(sensor_msgs::LaserScan &scan)
{
    scan.header.stamp = ros::Time(0);
    scan.header.frame_id = "/laser";
    scan.range_min = 0.6; // kinect is blind closer than 0.6 meters to the sensor
    scan.range_max = 5;   // up to 5 meters you still get more or less good measurements
    scan.angle_min = -0.5061; // horizontal opening angle of the kinect is 58 degrees = 1.0122 in rad
    scan.angle_max = 0.5061;
    scan.angle_increment = 0.00158; // a kinect line has 640 points increment is opening angle / 640
}

void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
    // convert from ROS message to PCL
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*input_cloud, pcl_cloud);
    // convert to templated point cloud
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_typed;
    pcl::fromPCLPointCloud2(pcl_cloud, pcl_cloud_typed);

    // create const pointer from input cloud
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(pcl_cloud_typed));


    // TODO find the index of the central horizontal line of the point cloud (replace 0)
    int line_of_interest = 0;

    sensor_msgs::LaserScan scan;
    initScanMessage(scan);

    // copy central line to LaserScan message
    pcl::PointXYZRGB current_point;
    for(unsigned int col = 0; col < point_cloud->width; col++)
    {
        current_point = point_cloud->at(col, line_of_interest);

        // undefined depth value
        if(isnan(current_point.x))
        {
            // we have invalid depth values
            // TODO put some fake value into the scan (e.g. scan.range_max)
        }
        // point with a valid depth value
        else
        {
            // we have a valid depth value
            // TODO insert the distance to the measured point into scan, use the euclidean distance of "current_point" to origin
        }
    }

    // TODO publish the laser scan message here
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_to_laser_data");
    ros::NodeHandle n;
 
    // subscribe to topics
    // TODO subscribe here to the topic "/camera/depth/points" on which you will receive kinect data
 
    // advertise topics
    // TODO advertise here the topic to publish laser scan data 

    ros::spin();

    return 0;
}


