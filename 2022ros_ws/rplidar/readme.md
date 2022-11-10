# About RPLIDAR

## First Read this
RPLIDAR WIKI: http://wiki.ros.org/rplidar

RPLIDAR VENDOR: https://www.slamtec.com/en

***
   RPLIDAR is a low cost 2D LIDAR solution developed by RoboPeak Team, SlamTec company. It can scan 360° environment within 6meter radius. The output of RPLIDAR is very suitable to build map, do slam, or build 3D model.
>    You can know more information aboud rplidar from SlamTec HomePage(http://www.slamtec.com/en).

* [RPLIDAR A2](http://www.slamtec.com/en/Lidar)  
* [RPLIDAR ](http://www.slamtec.com/en/Lidar/A1)
 
***
# How to build rplidar ros package
* > 1) Clone this project to your catkin's workspace src folder
* > 2) Running catkin_make to build rplidarNode and rplidarNodeClient

## How to run rplidar ros package

Check the authority of rplidar's serial-port :

`ls -l /dev |grep ttyUSB`

Add the authority of write: (such as /dev/ttyUSB0)

`sudo chmod 666 /dev/ttyUSB0`
 
There're two ways to run rplidar ros package
### I. Run rplidar node and view in the rviz

`roslaunch rplidar_ros view_rplidar.launch`

You should see rplidar's scan result in the rviz.

### II. Run rplidar node and view using test application

`roslaunch rplidar_ros rplidar.launch`

`rosrun rplidar_ros rplidarNodeClient`

You should see rplidar's scan result in the console

##  How to remap the USB serial port name
>  Maybe you should change the usb device port mode with the authority of read and write, more than make the port remap to an fixed name. 
>  install USB port remap:
./scripts/create_udev_rules.sh

> chack the remap using following command: 
`ls -l /dev | grep ttyUSB `
![usb_port_remap](http://img.blog.csdn.net/20160416163111816)

> Once you have change the USB port remap, you can change the launch file about the serial_port value.  
`  <param name="serial_port"         type="string" value="/dev/rplidar"/> `

### RPLidar frame
RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png


***
# How to install rplidar to your robot
> rplidar rotates in clockwise direction. The first range comes from the front (the tail with the line). 

> you can install rplidar A2 in your robot like this. 
![rplidar_a2_install_robot](https://github.com/robopeak/rplidar_ros/blob/master/rplidar_A2.png)

> you can install rplidar A1 in your robot like this. 
![rplidar_a1_install_robot](https://github.com/robopeak/rplidar_ros/blob/master/rplidar_A1.png)
