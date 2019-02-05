Simple socket interface to run the Barrett WAM and Hand (through the waminterface library)
author: Kaijen Hsiao (kaijenhsiao@gmail.com)

Because linking to Barrett's libraries seem to cause seg faults for any socket code but socket clients, the socket interface is a socket client, not a server.
This means that you need to run a socket server (example code and interface functions in socketservercmds.py: run with 'python socketservercmds.py') first, then you can run socketwamif (as root: 'sudo ./socketwamif'), which will prompt you to activate the WAM.

To use this socket layer directly, you can use the commands in socketservercmds.py, or write your own socket code to talk to socketwamif in the same way.

If you're looking for the ROS interface, go to WAMinterface/ROS.
