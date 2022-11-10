# where_am_i
MCL algorithm applied to the localize a simulated robot using ROS nodes

## Running Environment
First launch the simulation:

```
roslaunch where_am_i world.launch
```

In a new terminal, launch the amcl launch file:

```
roslaunch where_am_i amcl.launch
```
 
## Sending Navigation Goal:
You can send a 2D Nav Goal from RViz: click the 2D Nav Goal button in the toolbar, then click and drag on the map to send the goal to the robot. It will start moving and localize itself in the process.

## Testing Using teleop Node:
Open a terminal and launch the teleop script:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
