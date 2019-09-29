The objective of this lab is to practice on the motion concepts in ROS and localization in 2D space by developing a ROS node that a turtlesim behaves like a roomba cleaning robot. 

    Task 1: Introducing the Cleaning Application
    Task 2: Moving in a Straight Line
    Task 3: Rotating Left and Right


The code is available in CpE476_demos package in your virtual machine in the folder src/turtlesim_cleaner_cpp/src/turtlesim_cleaner_cpp_node.cpp. It also available on GitHub on this link.

Create a catkin package:
~/catkin_ws$ catkin_create_pkg turtlesim_cleaner_cpp geometry_msgs roscpp

Then open CMakeLists.txt to edit as follows:

gedit CMakeLists.txt

    Then, add these two lines at the very end of the file CMakeLists.txt

add_executable(turtlesim_cleaner_cpp src/turtlesim_cleaner_cpp/src/turtlesim_cleaner_cpp_node.cpp)
target_link_libraries(turtlesim_cleaner_cpp ${catkin_LIBRARIES})

Pay attention not to make any random modification of that file, otherwise, it will not compile.

    Then, close the CMakeLists.txt file, and close the terminal.
    Open another terminal and write:

cd catkin_ws

    Then, compile using the command:

catkin_make


Testing

Now, after compilation you can run as follow: In a first terminal, start roscore:

$ roscore

In a second terminal, start turtlesim simulator as follows:

$ rosrun turtlesim turtlesim_node

In a third terminal, start the cleaning application already written in c++ and also compiled as follows:

 rosrun turtlesim_cleaner_cpp turtlesim_cleaner_cpp_node

You will see the Turtlesim moving as in the video. Now, this testing step is DONE. 
