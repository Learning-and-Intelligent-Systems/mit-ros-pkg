Inverse kinematics for the 7-DOF Barrett Arm
author: Kaijen Hsiao (kaijenhsiao@gmail.com)

In this directory:
C analytical inverse kinematics function for the 7-DOF Barrett WAM (IK written by Manfred Huber, interface written by Kaijen.  Searches over joint 2 and solves the rest analytically.)

WAMKinematics.c contains Manfred's analytical IK code.  wamik.c contains the code to run the search over joint 2.  testwamik.c has a test program that goes through a really long list of joint angles to make sure it works (to save you the trouble, in its current form, it gets 54 wrong out of 1206576.)

In either SOlib (linux, uses wamik.so.1.0) or MSVC6/WAMinversekinematicsdll/Debug (Windows, uses wamik.dll):
Python optimization inverse kinematics using scipy's fmin_cobyla, which is good for those barely-out-of-reach goals.  (also, python ctypes bindings for the C IK library.)

To use:
As straight C library: link to libwamik.a.  Example in testwamik.c.
In Python in linux: use the wamik.py module in SOlib, example in SOlib/testwamik.py.
In Python in Windows: use the wamik.py module in MSVC6/WAMinversekinematicsdll/Debug.  Example in MSVC6/WAMinversekinematicsdll/Debug/testwamik.py.
(wamik.py and testwamik.py are the same in both MSVC and SOlib versions, they just detect whether you're in Windows or not.  wamik_interface.c is the ctypes interface.)  
From ROS: IK services are included in ../ROS/WAMServerROS.py.

To create wamik.so.1.0 for your linux system, run 'sh ./create_wamik_so.sh' in the SOlib directory, or type 'make' in this directory.  To make wamik.dll, build the MSVC6 workspace in Visual Studio 6.   

Enjoy!
