Static transformation

This tutorial will show you how to add a static transformation between two frames. Suppose we have a robot that is fixed at (1,-1,0) relative to the map and rotated to the left. For this purpose, we can use the static_transform_publisher node from the tf package. Running the function is simple and just execute the command below:

rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms

or the same command but using quaternions:

rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms

Now all you have to do is complete the above fields in the appropriate way.
husarion@husarion:~$

rosrun tf static_transform_publisher 1 -1 0 1.6 0 0 map robot 100

rostopic list 

TF_Echo
A simple tool that allows you to quickly check the transformation between any two frames.

rosrun tf tf_echo map robot

TF_Tree

The tool shows all the relationships between the individual forms in a diagram. This tool allows you to check if all frames are properly connected to each other.

rosrun rqt_tf_tree rqt_tf_tree 

RViz

The RVIZ program known from chapter 4 will be used for visualization. To run RViz just type in new terminal.

rviz

Now click Add. In the new window, select TF from the list.

Try adding another reverse_camera frame which is 20 cm above the robot and facing the back of the robot. Use static_transform_publisher and check the result with all the tools presented above.

In new terminal type:

rosrun tf static_transform_publisher 0 0 0.2 3.14 0 0 robot reverse_camera 100