#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("auto_turtle")
ros_handle_py = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

output = Twist()
output.linear.x = 1
output.angular.z = 0.5

loop_hz = rospy.Rate(10) 

while not rospy.is_shutdown():
    ros_handle_py.publish(output)
    loop_hz.sleep()
    