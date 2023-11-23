### Camera support for Jetson nano 
Jetson developer kits have multiple interfaces for connecting a camera, including USB, Ethernet, and MIPI CSI-2. Popular cameras are supported out of the box, and Jetson ecosystem partners support a broad portfolio of additional cameras .

Popular cameras supported out of the box include IMX219 camera modules such as Raspberry Pi Camera Module V2, the Intel Realsense and StereoLabs Zed 3D cameras, and standard USB webcams.
### Install drivers and apps

Install gstreamer1.0:

```
sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
```

Install gscam

In your catkin_ws install the following ros package 
```
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/gscam.git
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
