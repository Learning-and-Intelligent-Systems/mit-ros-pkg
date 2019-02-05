/*!

\mainpage Barrett WAM control library

\section qs Programming Quickstart

To learn to write software for the WAM, Barrett recommends the following course.
- Read \ref intro, \ref fctly, \subpage pcv, \subpage boc
- Follow the quickstart sheet shipped with the WAM to get the WAM set up and running.
- Compile and run each of the example programs
- Read the code of the example programs
- Scan through the list of Barrett code library functions
- Read the code for btdiag 

 \section intro Introduction

 The Barrett Technology software library provides the following functionality:

-# Communication with the safety module and motor controllers on the CAN bus using Barrett’s proprietary communications protocol.
-# Automatic recognition puck+motor pairs (actuators) attached to the CAN bus. Simplified command and control of actuators.
-# Initialization and control of 4DOF and 7DOF WAM robots
-# A real-time control loop. (Note: Presently the CAN driver is not hard-realtime.)
-# Basic control routines and heuristics (decision making routines). 
 -# Control routines include a PID algorithm and Trapezoidal trajectory generator. 
 -# Heuristics  include a set of functions and state variables to enable, disable, and switch between control methods safely.
 -# Teach and play: recording and playback of trajectories
-# A real-time framework for simplifying the process of insuring that control happens consistently and at a high sample rate.

To best utilize the WAM system the programmer will want to develop some familiarity with all of the above functionality. 
Barrett Technology has provided access to our proprietary code to help the programmer learn about the WAM system and 
eliminate guesswork at what exactly is happening when you call one of our library functions. If you would like to use 
or modify any of the code in the library for yourself; please contact Barrett Technology to find out what licensing 
requirements (if any) we have.

The following is an overview of the library functionality and what code files provide the functionality. 

\section fctly Barrett Module Functionality
  
  
  \dot
  digraph Structure {
      node [shape=record];
      vn [label="{{<p1> btwam}|{<p2>btrobot|<p3>btstatecontrol|<p4>btcontrol|<p5>bthaptics}|{btos|btmath|btcan}}"];

  }
  \enddot
Essential:
- btwam.h: This is the primary file used for controlling the WAM.

Useful funtionality:
- btmath.h: vector and matrix library
- btrobot.h: robot kinematics & dynamics
- btlogger.h: realtime data logging
- btpath.h: space curves for use as trajectories or haptic objects
- btcontrol.h: Control objects. PID, Multi-via trajectories.
- btstatecontrol.h: virtualize objects for btcontrol
- bthaptics.h: simple haptics library
- btjointcontrol.h: jointspace state controller

Mostly internal use:
- btcan.h: CAN bus communication code
- btos.h: OS abstractions for easier porting
- btparser.h: Config file parser
- btsystem.h: Bussed actuators communication & management code

- playlist.h: point to point playlist funcionality for btjointcontrol
- serial.h: serial comm library

 
*/

/**
\page pcv Programming Conventions
Based on our experience with RTAI, Linux, and our code, here are some words of advice.
\subsection pcv1 GDB
Learn to use gdb! If you are getting segfaults in your code (our rampant use of 
objects in C makes that likely) then gdb will help you alot. 

Turn on core dumps
\code
#ulimit -c 5000
\endcode
Compile your program with the gcc -g option to include debugging info.
Run your program again and get it to crash.
Run gdb.
\code
#gdb yourprogramname corefilename
\endcode

The core file stores a snapshot of all program memory when it crashed. 
"p variablename" will display the values in you variables.

\subsection pcv2 Process/Thread control

Learn to use exit() and atexit(). These are part of gnu libc.

Always put a sleep() or usleep() command inside loops. This gives other threads
a chance to run....








*/
