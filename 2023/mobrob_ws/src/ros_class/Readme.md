 rosrun example_ros_msg rc_example_ros_message_publisher   (with roscore running)
 rostopic echo example_topic

 rosmsg show  example_ros_msg/ExampleMessage

rosrun example_ros_service example_ros_service
rosservice list
rosservice info /lookup_by_name
rosservice call lookup_by_name 'CPE476'
rosrun example_ros_service example_ros_client 


classes: see example_ros_class (pass in node handle; odd syntax for callbacks)

rosrun example_ros_class example_ros_class
rosservice call example_minimal_service 
rostopic pub -r 2 example_class_input_topic std_msgs/Float32 2.0

//main using a library:
rosrun creating_a_ros_library example_ros_class_test_main


rosrun example_action_server example_action_server
rosrun example_action_server example_action_client

rosrun example_action_server example_countdown_server
rosrun example_action_server timer_client

Parameter server:
rosparam
rosparam set /gains "p: 1.0
i : 2.0
d : 3.0"
rosparam list
rosparam get /gains

rosparam load jnt1_gains.yaml
rosparam list
rosparam get jnt1_gains
rosparam delete jnt1_gains
rosparam list
(launch file option--would need to copy over package/directory)
example read_param_from_node: uses: if ( nh.getParam ("/joint1_gains/p", P_gain))


Running the Static Broadcaster
$ roscore

You can now run your static_turtle_tf2_broadcaster by running
$ rosrun learning_ros_tf2 static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0

Checking the results
$ rostopic echo /tf_static

