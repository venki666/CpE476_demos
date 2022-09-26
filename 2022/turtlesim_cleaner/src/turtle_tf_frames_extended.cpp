#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
// TODO include header for the joint state message

std::string turtle_name;
ros::Time last_time(0);

float turn_angle_left_result = 0;
float turn_angle_right_result = 0;

// TODO create a global publisher object to publish the joint state


void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
  // on first call just init variables
  if(last_time == ros::Time(0))
  {
    last_time = ros::Time::now();
    return;
  }

  ros::Time now = ros::Time::now();
  float t = (now - last_time).toSec();
  last_time = now;  

  // compute delta
  float linear_delta = msg->linear_velocity * t;
  float angular_delta_left = msg->angular_velocity * t;
  float angular_delta_right = msg->angular_velocity * t;

  // add delta: note the different directions for both wheels
  // left wheel: positive linear, negative angular
  turn_angle_left_result += linear_delta;
  turn_angle_left_result -= angular_delta_left;
  // right wheel: positive linear, positive angular
  turn_angle_right_result += linear_delta;
  turn_angle_right_result += angular_delta_right;


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  transform.setRotation( tf::Quaternion(tf::Vector3(0,0,1), tfScalar(msg->theta)) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), /*TODO fixed frame*/, /*TODO base frame of your turtle*/));

  // send current leg angles as joint message
  // TODO create a joint state message and set the values
  // 1. insert the current time stamp
  // 2. resize the name and the position vector to the number of joints you want to move
  // 3. assign each joint name with the corresponding name from the URDF file
  // 4. assign each joint position the angle you want to set for the joint
  // 5. publish the joint state message

}

int main(int argc, char** argv){
  ros::init(argc, argv, "turtle_tf_frames_extended");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  // TODO create your global publisher object to publish the joint state

  ros::spin();
  return 0;
};

