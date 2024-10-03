#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

flag = None

def callback (msg):
    global flag
    flag = (msg.x > 8) or (msg.x < 2) or (msg.y > 8) or (msg.y < 2)

rospy.init_node("auto_turtle")
pub_handle_py = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
sub_handle_py = rospy.Subscriber("/turtle1/pose", Pose,  callback, queue_size=10)

output = Twist()
output.linear.x = 1
output.angular.z = 0

loop_hz = rospy.Rate(10) 

while not rospy.is_shutdown():
    if (flag):
        output.angular.z = 1
    else :
        output.angular.z = 0    

    pub_handle_py.publish(output)
    loop_hz.sleep()
    