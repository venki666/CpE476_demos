#include <ros/ros.h>
// TODO include header for the transform broadcaster
// TODO include header for turtlesim::Pose

std::string turtle_name;

ros::Time last_time(0);

float turn_angle_left_result = 0;
float turn_angle_right_result = 0;


void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
  // on first call just init variables
  if(last_time == ros::Time(0))
  {
    // TODO save current time in last_time
    // last_time =
    return;
  }

  // TODO get current time
  // ros::Time now =
  // TODO calculate difference between last_time and now in seconds
  // float t = 
  // TODO update last time for next pose callback
  // last_time =

  // compute delta
  // TODO calculate linear movement delta using time difference
  //float linear_delta = 
  // TODO calculate angular movement delta using time difference (note: same for both wheels)
  // float angular_delta_left = 
  // float angular_delta_right = 

  // TODO add linear and angular deltas to both wheels
  // TODO don't forget that wheels move in different directions when turning
  // turn_angle_left_result = 
  // turn_angle_right_result = 
 


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    transform.setRotation( tf::Quaternion(tf::Vector3(0,0,1), tfScalar(msg->theta)) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));

  // add transforms for left "turtle wheel"
  transform.setOrigin( /*TODO set origion of left wheel*/ );
  transform.setRotation( /*TODO set rotation of left wheel*/ );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), /*TODO parent should be the turtle frame*/, /*TODO child should be left wheel of turtle*/));

  // add transforms for right "turtle wheel"
  transform.setOrigin( /*TODO set origion of right wheel*/ );
  transform.setRotation( /*TODO set rotation of right wheel*/ );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), /*TODO parent should be the turtle frame*/, /*TODO child should be right wheel of turtle*/));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "turtle_tf_frames");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

