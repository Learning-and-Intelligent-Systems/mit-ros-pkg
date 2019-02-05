from openravepy import *
from numpy import *
import time
import os, shutil

def sendvelcmd(robot,vel,env=None):
    cmd="setvelocity "
    joints=robot.GetActiveManipulator().GetArmIndices()
    for i in range(len(vel)):
      cmd=cmd+" "+str(joints[i])+" "+str(vel[i])+" "
    #print cmd
    if robot.GetController().GetXMLId() == 'IdealController':
#      robot.SetActiveDOFValues(robot.GetActiveDOFValues()+vel*.1)
      vals=robot.GetActiveDOFValues()
      vals[robot.GetActiveManipulator().GetArmIndices()]+=array(vel)*.1

      robot.SetActiveDOFValues(vals)
      #time.sleep(.045)
      time.sleep(.045)
      if env!=None:
        boxavoid(env)
    else:
      robot.GetController().SendCommand(cmd)

def movejoint(robot,degrees):
  rad=array(degrees)*3.1415/180
  joints=robot.GetActiveDOFValues()[robot.GetActiveManipulator().GetArmIndices()]
  #vel=sign(joints-rad)*.05
  #vel=[abs(d)>.01 ? sign(d)*-.05:0 for d in joints-rad]
  err=(joints-rad)
  while dot(err,err) >.001:
    vel=(joints-rad)*-.5
    sendvelcmd(robot,vel)
    joints=robot.GetActiveDOFValues()[robot.GetActiveManipulator().GetArmIndices()]
    err=(joints-rad)
    time.sleep(.01)
    #print "err:",err
    #print vel
  sendvelcmd(robot,[0,0,0,0,0,0,0])