cmake_minimum_required(VERSION 2.8.3)
project(ros_pubsub_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
add_dependencies(talker ros_pubsub_tutorial_generate_messages_cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
add_dependencies(listener ros_pubsub_tutorial_generate_messages_cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
