Sample code/test programs for waminterface
author: Kaijen Hsiao (kaijenhsiao@gmail.com)

WAMtest.c: the main sample code for all waminterface functions

WAMtestcpp.cpp: same as WAMtest.c, just making sure that the same code compiles as C++ code with g++ instead of gcc (all except the Ctrl-C handler.  I haven't bothered to try to find the equivalent C++ code for that.)

runcalibration.c: just runs waminterface's joint calibration for optical encoders
calibrationangles.txt: if you want to use runcalibration, you should change the angles in this file to ones where your arm is free to move to its joint limits

RThandtest.c: just tests the real-time hand functions in waminterface

socketlinkdebugging: debugging what sort of socket code can and can't be linked to Barrett's libraries directly.  See the README in that directory for details.  If you know how to link Barrett's code to a socket server and make it not segfault, please email me!!!
