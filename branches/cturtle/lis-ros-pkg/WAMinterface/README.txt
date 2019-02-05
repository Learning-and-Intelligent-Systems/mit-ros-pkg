Barrett WAM and Hand interface and inverse kinematics library.
author: Kaijen Hsiao (kaijenhsiao@gmail.com)

NOTE: Most of these libraries will only compile on the WAM's computer (they require the BarrettWAM package, which depends on Xenomai/RTAI), but to use the ROS interface, all you need to compile is the ROS services, so to call the ROS services from another computer, comment out everything under the line "# Build WAMinterfacelib, WAMinversekinematics, and btdiag" in CMakelists.txt, and type 'make' in this directory.  This package is configured for a Xenomai/Peak CAN bus system.  It will probably work in other configurations if you edit the Makefiles, but no guarantees.

To use the non-ROS interfaces, look at the READMEs in WAMinterfacelib and WAMinversekinematics.  To use the ROS interface, install ROS as described at http://www.ros.org/wiki/ROS/Installation and look at the README in the ROS directory.

In this directory:

ROS: ROS server and sample client for the arm and hand

WAMinterfacelib: 
    -functions to interface with the Barrett arm and hand directly (can be used without ROS if the Makefile is changed to include the paths to btclient) 
    -a simple socket interface (can also be used without ROS, from the same or a different computer, or from any language that you can write a socket server in.  The provided socket server is written in Python.)

WAMinversekinematics: 
    -C analytical inverse kinematics function for the 7-DOF Barrett WAM (written by Manfred Huber.  Searches over joint 2 and solves the rest analytically.)
    -Python optimization inverse kinematics using scipy's fmin_cobyla, which is good for those barely-out-of-reach goals.  (also, python ctypes bindings for the C IK library.)

btdiag: Barrett's standard btdiag, with a couple added extras (BHand arbitrary command, move arm home).  Mostly used for Makefile testing.

srv: ROS services

manifest.xml: ROS manifest file

Makefile: runs CMake when you type 'make'

Makefile.other: recursive make for all the relevant subdirectories.

More specific details are in README files scattered throughout the directories.



