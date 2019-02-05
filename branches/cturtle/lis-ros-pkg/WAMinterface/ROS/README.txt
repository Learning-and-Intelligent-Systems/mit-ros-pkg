ROS wrapper for the waminterface library for running the Barrett WAM and Hand.
author: Kaijen Hsiao (kaijenhsiao@gmail.com)

The ROS server is in WAMServerROS.py, which in addition to being a ROS server, is a plain-socket server that talks to a plain-socket client (socketwamif) that in turn calls waminterface functions.  After starting the ROS server, you will be prompted to start WAMinterface/WAMinterfacelib/socketwamif/socketwamif, which will in turn prompt you to activate the WAM.

To talk to the ROS server, you can use the python functions in WAMClientROSFunctions.py or write your own ROS client.  Sample code in testprograms; run the main test program with 'python testprograms/WAMtestROS.py' after starting the ROS server with 'python WAMServerROS.py'.

wamik.so.1.0 is auto-generated when you run 'make' in WAMinterface, or by running 'make' in WAMinterface/WAMinversekinematics.

