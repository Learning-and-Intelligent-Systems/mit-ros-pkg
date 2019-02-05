#!/usr/bin/env python
import roslib; roslib.load_manifest('pushing_test')
import rospy
from furniture.msg import Table_Poses
from furniture.msg import Table_Polygons

import sys
import socket
import threading
import os
import signal
import select

class SocketServer(threading.Thread):
    def __init__(self, host, port, backlog):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        self.backlog = backlog
        self.client = None
        self.address = None
        self.isSocketOpen = False
        self.server = None

        self.setDaemon(True)
        self.start()

    def run(self):
        running = 1
        while running:
            while not self.isSocketOpen:
                try:
                    print 'trying to open socket'
                    self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.server.bind((host, port))
                    self.server.listen(backlog)
                    self.isSocketOpen = True
                    print 'server inited'
                except socket.error, (value, message):
                    if self.server:
                        self.server.close()
                    print 'caught exception', value, message
                    print 'retrying'
                    rospy.sleep(1.0)

            while not self.client:
                print 'waiting for client'
                inputs = [self.server, sys.stdin] # server.accept() blocks, make sure sys.stdin is available for ctrl+c

                inputready, outputready, exceptionready = select.select(inputs, [], [])
                for s in inputready:
                    if s == self.server:
                        client, address = self.server.accept()
                        self.client = client
                        self.address = address
                        print 'accepted client',client, address



if __name__=='__main__':
    host = ''
    port = 50000
    backlog = 1
    socketServer = SocketServer(host,port,backlog)
    
    threads = []
    threads.append(socketServer)

    # close the server port when ctrl+c is hit
    def on_exit(sig, func=None):
        print 'ctrl+c handler triggered'
        print 'closing server'
        socketServer.server.close()

    signal.signal(signal.SIGINT, on_exit)


    # send the latest box pose to matlab
    def tablePosesCallback(d):
        if len(d.poses)>1:
            print 'received more than 1 pose'
            print 'ignoring all but the first one'

        msg="%.2f "%d.poses[0].x+"%.2f "%d.poses[0].y+"%.2f "%d.poses[0].theta
        print 'received',msg
        try:
            if socketServer.client:
                print 'sending pose in table_init frame'
                socketServer.client.send(msg)
        except socket.error, (value, message):
            print 'failed to send message'
            print value,message
            print 'closing client'
            socketServer.client.close()
            socketServer.client=None

        
    rospy.init_node('box_pose_listener')
    rospy.Subscriber('table_poses',Table_Poses, tablePosesCallback)
    #rospy.init_node('box_polygons_listener')
    #rospy.Subscriber('table_polygons',Table_Polygons,callback)
  
    rospy.spin()
    print 'haha'
