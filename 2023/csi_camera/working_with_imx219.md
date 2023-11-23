### Camera support for Jetson nano 
Jetson developer kits have multiple interfaces for connecting a camera, including USB, Ethernet, and MIPI CSI-2. Popular cameras are supported out of the box, and Jetson ecosystem partners support a broad portfolio of additional cameras .

Popular cameras supported out of the box include IMX219 camera modules such as Raspberry Pi Camera Module V2, the Intel Realsense and StereoLabs Zed 3D cameras, and standard USB webcams.
### Install drivers and apps

Install gstreamer1.0:

```
sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
```

### using nvgstcapture
The examples below use nvgstcapture gstreamer application to access the camera features via the NVIDIA API. More info and commands of nvgstcapture can be found in the L4T Guide under the Multimedia section.
Start Capture and Preview display on the screen
CSI camera
For information about how to connect the ribbon cable for MIPI CSI-2 cameras, see Jetson Nano 2GB Developer Kit User Guide .

In order to check that the CSI camera is working, you can run the following command, which will start capture and preview display it on the screen.

`nvgstcapture-1.0`
This example command will rotate the image 180 degrees (vertical flip)

`nvgstcapture-1.0 --orientation 2`


### Install gscam

In your catkin_ws install the following ros package 
```
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/gscam.git
```
Then edit ./gscam/Makefile and add the CMake flag -DGSTREAMER_VERSION_1_x=On to the first line of the file, so that it reads:
```
EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1 -DGSTREAMER_VERSION_1_x=On
```
While this flag is only necessary if you have both gstreamer-0.1 and gstreamer-1.0 installed simultaneously, it is good practice to include.
```
cd ~/catkin_ws/src
git clone https://github.com/rt-net/jetson_nano_csi_cam_ros.git 
```
compile the workspace and execute the node by using the command to publish the camera stream to the ROS topic /csi_cam_0/image_raw, use this command in the terminal:
```
roslaunch jetson_csi_cam jetson_csi_cam.launch width:=<image width> height:=<image height> fps:=<desired framerate>
```
roslaunch jetson_csi_cam jetson_csi_cam.launch width:=3840 height:=2160 fps:=15
```
View the image by using the following command in a new twerminal
```
rqt_image_view 
```

