### First Launch
To launch the project use the following command
```
$ roslaunch m2wr_description rviz.launch
```
Once the node is launched we need to open Graphical Tools from the Tools menu, it will help us to see the rviz window.
Once the rviz window loads, do the following:

* Change the frame to link_chasis in Fixed Frame option
* Add a Robot Description display

Next we will spawn our robot with the launch file in the empty gazebo world. Use the following command
```
$ roslaunch m2wr_description spawn.launch
```
The robot should load in the gazebo window.

### Control the robot using keyboard
To control the motion of the robot we can use the keyboard_teleop to publish motion commands using the keyboard. Use the following command in Shell
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
### Read the laser data from the robot 
To test the changes we can start the rviz visualization with command
```
$ roslaunch m2wr_description rviz.launch
```
Use Graphical Tool to see the rviz output

To see whether everything works, we can launch the gazebo simulation of and empty world and spawn the robot. First we will start the gazebo simulation from the Simulations menu option and then spawn a robot with the following command
```
$ roslaunch m2wr_description spawn.launch
```
We will make this script executable with following commands
```
$ cd ~/catkin_ws/src/motion_plan/scripts/ $ chmod +x reading_laser.py
```
Use the following commands
```
$ rosrun motion_plan reading_laser.py
```

### Obstacle Avoidance
To test the logic lets run the simulation. We have the world loaded, now we will spawn the differential drive robot with following command
```
$ roslaunch m2wr_description spawn.launch
```
Finally we launch the obstacle avoidance script to move the robot around and avoid obstacles
```
$ rosrun motion_plan obstacle_avoidance.py
```
### Run wall following

Let us run the wall following algorithm and see the robot in action. We already have the simulation window open with robot spawned into it. Open Tools>Shell and enter the following command
```
$ rosrun motion_plan follow_wall
```
### Test bug0 and bug1 algorithms

Let us now run the simulation and see the robot in action

Spawn the robot with following command
```
$ roslaunch m2wr_description spawn.launch
```
Launch the behavior nodes
```
$ roslaunch motion_plan behaviors.launch des_x:=0 des_y:=8
```
Notice the last two arguments are the co-ordinates of the goal location

Run the bug0 algorithm
```
$ rosrun motion_plan bug0.py
```

Open Tools > Shell and spawn the robot with following commands on shell
```
$ roslaunch m2wr_description spawn.launch y:=8
```
Once the robot is loaded into the world. Lets run our Bug 0 algorithm to navigate to a point x = 2 and y = -3
```
$ roslaunch motion_plan bug0.launch des_x:=2 des_y:=-3
```
Lets spawn the robot at x=0, y=8. Open Tools > Shell and enter the following commands
```
$ roslaunch m2wr_description spawn.launch y:=8
```
Next we set the goal (point x=0, y=-3) for our robot and launch the Bug 1 behavior with following command
```
$ roslaunch motion_plan bug1.launch des_x:=0 des_y:=-3
```

