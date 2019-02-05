ROS package repository containing an interface to the Barrett WAM/Hand.  

Barrett's btclient code is made into a ROS package in BarrettWAM.

The arm/hand interface and inverse kinematics library are in WAMinterface.

The interface to the Nano17 6-axis force/torque fingertip sensors is in Nano17interface.

If you're using ROS, make sure you add this directory (absolute directory name, e.g /home/robot/ros/lis-ros-pkg, colon-separated from what's already in there) to the export ROS_PACKAGE_PATH line in your .bashrc.ros file.

See the READMEs throughout the directories for more information.

