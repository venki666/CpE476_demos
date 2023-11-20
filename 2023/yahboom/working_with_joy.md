## Move your robot using the PS4 controller and BLE Dongle 

### RF PS2 Controller
Some students might also have the RF controller. If you have the Yahboom RF controller follow instruction here @ http://www.yahboom.net/study/USB-PS2. Insert the supplied RF dongle (see the 
battery bay of the RF controller) in the jetson nano usb slot. Insert batteries and press start button to pair the devices. Do not mix the RF 
dongle and controller with other teams.

### Follow the instructions if you have an PS4 controller.
Before you start make sure you have the PS4 controller and BLE USB dongle. Follow the steps below, you can also use the jetson nano GUI or ubuntu 
Bluetooth icon in the top conner to connect to the PS4 controller if you have the HDMI+Input Devices connected. Here we assume you are running a 
headless sytem. 

Install ds4dr: 
`$sudo pip install ds4drv`
Check BLE dongle:
`$lsusb`

Install ros-joy: http://wiki.ros.org/joy - assume working on jenson nano running melodic
`$ sudo apt-get install ros-melodic-joy`
Go into pairing mode with PS4: Playstation button + share button for ~5 sec
Run 
`$ds4drv` from command line to connect to PS4
This will output something like _Created devices /dev/input/jsX
remember /dev/input/js__X__ and update the launch file (default X=0)
`$sudo chmod a+rw /dev/input/jsX`
`$ls /dev/input/`
Test Joy
`$ sudo jstest /dev/input/jsX`

Calibrate and re-map joy_stick
To install jstest enter the following command in the terminal
`$ sudo apt-get install jstest-gtk`

Once installed you can start jstest from your terminal by entering the following command below, or from your menu
`$ jstest-gtk & `
Play around and claibrate the joy_stick

Ros runs:run commands in seperate terminals
`$ds4drv if it is not running already.
$ roscore 
$ rosparam set joy_node/dev "/dev/input/jsX"
$ rosrun joy joy_node
$ rosrun ps4_ros ps4_ros
$ rostopic echo joy`





