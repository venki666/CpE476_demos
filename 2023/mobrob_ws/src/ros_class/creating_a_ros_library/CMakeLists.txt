cmake_minimum_required(VERSION 2.8.3)
project(creating_a_ros_library)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs std_srvs roscpp)

#add_service_files(FILES SrvTutorial.srv)
#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES example_ros_library
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(example_ros_library src/example_ros_class.cpp)  


add_executable(ros_library_test_main src/example_ros_class_test_main.cpp)
add_dependencies(ros_library_test_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_library_test_main example_ros_library ${catkin_LIBRARIES})
