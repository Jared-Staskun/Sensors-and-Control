# Sensors-and-Control

Individual Code Contribution:

*Jared Staskun - 13211863 - 33.3%
*Vearinama Kila - 12582336 - 33.3%
*Cameron Sarkis - 12578833 - 33.3%

The essential libraries must first be setup up in order to complete initialistaion of the code. The following links

https://github.com/gapaul/dobot_magician_driver
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

Dobot Start up Commands

1)	source /home/jared/catkin_ws/devel/setup.bash

*change -username- to your file path

2)	ls /dev -l

*search for ttyUSBX -> will most likely be ttyUSB0

3)	sudo chmod 666 /dev/ttyUSB0

4)	roslaunch dobot_magician_driver dobot_magician.launch

Camera Start up Commands

1) 	source /home/jared/catkin_ws/devel/setup.bash

*change -username- to your file path

2) 	roslaunch realsense2_camera rs_camera.launch
