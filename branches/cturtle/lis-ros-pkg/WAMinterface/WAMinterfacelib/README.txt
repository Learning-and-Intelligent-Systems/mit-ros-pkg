Interface for Barrett Arm and Hand and plain-socket interface

Directories:
waminterface: functions that use Barrett's btclient code to control the arm and simple functions to talk to the Barrett hand in non-threaded mode
handinterface: functions to talk to the Barrett Hand in a separate thread
socketwamif: a simple socket interface to run the arm/hand from another computer, or just without having to link to Barrett's code (which causes all kinds of problems)
testprograms: sample code
log: logging library
lib: where the waminterface library is compiled to
