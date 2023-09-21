# turtlesim tutorials

### Start the turtlesim node with rosrun:
```
 $ rosrun turtlesim turtlesim_node 
```
Enter command below in the new terminal:
```
 $ rosnode list
```

Output

/rosout 
/turtlesim 


### turtlesim services
```
 $ rosnode info /turtlesim 
```
To get the info of a specific rosservice use the command below:
```
$ rosservice info /spawn
```
**Example rosservices turtlesim**

**teleport_absolute**

The absolute teleport option moves the turtle to the absolute position in de screen. The arguments are [x y theta].
```
$ rosservice call /turtle1/teleport_absolute 1 1 0 
```
**teleport_relative**

The relative teleport option moves the turtle with respect to its present position. The arguments are [linear angle].
```
$ rosservice call /turtle1/teleport_relative  1 0 

$ rosservice call /turtle1/teleport_absolute 2 1 0 

$ rosservice info /turtle1/set_pen

$ rosservice call /turtle1/set_pen 100 150 100 3 0

$ rosservice call /turtle1/teleport_relative  1 0 
```

turtlesim pose topic:
```
$rostopic info /turtle1/pose 
```

turtlesim pose topic:
```
$ rostopic info /turtle1/pose 
```
Output

Type: turtlesim/Pose

Publishers: 
* /turtlesim (http://127.0.0.1:33755/)

Subscribers: None
    

A rostopic is always based on a rosmsg (ROS message) structure.

To get the ROS message name used by the ROS topic use the command below.
```
$ rostopic type /turtle1/pose 
```  

Output:

turtlesim/Pose

 
To get the specific structure of the ros message use the command below.
```
$ rosmsg show turtlesim/Pose 
```

Output:

turtlesim/Pose
root@74b5eecc7ce2:/# rosmsg show turtlesim/Pose
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity


To read the actual values of the ROS topic use the rostopic echo command:
```
$ rostopic echo /turtle1/pose 
```    

Output:

x: 6.0
y: 1.0
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
x: 6.0
y: 1.0
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---

### overview of the existing rosparam use the list argument.
```
$ rosparam list
```
Output

/rosdistro
/roslaunch/uris/host_127_0_0_1__46711
/rosversion
/run_id
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r

### Read a parameter value

We want to get parameters and change color of background to red.
```
$ rosparam get /turtlesim/background_r
```

To change a rosparam use the argument set
```
rosparam set /turtlesim/background_r 0
```
### Set a parameter value

This changes the parameter value, now we have to call the clear service for the parameter change to take effect:
```
$ rosservice call /clear 

$ rosnode info /turtlesim 
```
 
### Get info rostopic cmd_vel

To get the message type of the rostopic /turtle1/cmd_vel we use the argument type below.
```
$ rostopic type /turtle1/cmd_vel 
```

Output

geometry_msgs/Twist

    

To get the specific info of the rosmsg we can extend the command with rosmsg show
```
$ rostopic type /turtle1/cmd_vel | rosmsg show 
```
 
Output

geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
 
 ``` 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 

rostopic echo /turtle1/cmd_vel
```
linear: 
  x: 1.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 1.8
---

```
rostopic echo /turtle1/pose 
```
---
x: 0.332792580128
y: 4.38678216934
theta: -0.580785274506
linear_velocity: 0.0
angular_velocity: 0.0
---\

publish a steady stream of commands using rostopic pub -r
``` 
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 

rosservice call /reset 
```
Show the rate in Hz of the published topic /turtle1/pose (CTRL-C to stop data stream):
```
rostopic hz /turtle1/pose 

rostopic hz /turtle1/cmd_vel
```
### Move the turtle with teleop

roscore
rosrun turtlesim turtlesim_node 

rosrun turtlesim turtle_teleop_key 

rosnode list 

rosnode info /teleop_turtle 

rqt

rqt stand for ROS QT applications. It is not one tool but multiple interesting tools bundled together to make it easy for the programmer.

You can start rqt with the command rqt in the terminal. By selecting the item Plugins you can select many different tools

rqt

Make the turtle to move in circles

rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 

 
rqt_plot

The information of all nodes and topics is available by CLI but most of the times not so usable for visualization of values. With rqt_plot you can plot values into graph.

Make sure that the turtle is moving by execute the command below. The turtle should move in circles

rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
    

Now we can plot information about the nodes and topics.

rqt_plot /turtle1/pose/x:y:z 


roscore

rosrun turtlesim turtlesim_node

The following command is the entire one. 

After entering a part, use the [Tab] key to complete it, and then modify the content

rosservice call /spawn "x: 3.0

y: 3.0

theta: 90.0

name: 'my_turtle'" 

The node is the main calculation execution process. ROS is composed of many nodes.

rqt_graph


### collision avoidance

```
roscore


rosrun turtlesim turtlesim_node 


rosrun automate_turtlesim collision

```
```

roscore


rosrun turtlesim turtlesim_node 


rosrun automate_turtlesim collision.py
```