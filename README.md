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


Code Outline:

Using rostopics to obtain data from the Dobot Magician and the Intel RealSense Camera, the environment of the setup is perceived. After ROS subscribers and the Dobot are initialised, the Dobot moves out of the camera feed, as to not obstruct the camera from seeing the blocks. 
The RGB image of the camera is stored and crossed check by Colour profiles (Hue, Saturation and Value, high and low thresholds), which were calibrated against precaptured images of the coloured blocks. By applying each colour profile, we can segregate the shapes of objects detected in the image by their colour. When segregated, 4 images are stored each only containing the shapes of the coloured blocks which are converted to black and white.
Then the cube centroids are found and plotted on the RGB image, displaying their coordinates in pixels. 

Afterwards, algorithms are applied, including the Disparity formula used in triangulation change the origin of the image from the top-left corner, to the principal point (centre). Then by using some calibration and further algorithms, the Cube centroids were converted from pixel coordinates to world coordinates. Furthermore, these are transformed to the end-effector coordinate frame of the Dobot Magician. Here on the code is set out to send movement commands to the Dobot to prick up and stack the cloloured blocks in order of Red, Yellow, Green, Blue. The stacking is dont at the principal point of the Camera, so the blocks cannot be placed on the origin point, except for the Red block. Then having stack the four blocks in this order, the end-effector is moved next to the tower of blocks, and knocks them over.




