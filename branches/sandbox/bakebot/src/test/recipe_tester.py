#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import sys
from services.bakebot_recipe_service import *

if __name__ == '__main__':
    args = sys.argv
    if len(args) < 2:
        print 'usage: python recipe_tester.py recipe.txt'
    else:
        f = open(args[1], 'r')
        string = f.read()
        status = BakebotClient.execute_recipe(string, debug=True)
        print status
        print 'done'
