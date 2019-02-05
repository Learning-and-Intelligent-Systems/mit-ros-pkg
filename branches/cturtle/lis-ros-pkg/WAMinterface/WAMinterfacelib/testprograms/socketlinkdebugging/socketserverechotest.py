#!/usr/bin/env python
"USAGE: echoserver.py <port>"
from socket import *    # import *, but we'll avoid name conflict
import sys

def handleClient(sock):
    data = sock.recv(32)
    while data:
        sock.sendall(data)
        data = sock.recv(32)
        sock.close()

if len(sys.argv) != 2:
    print __doc__
else:
    sock = socket(AF_INET, SOCK_STREAM)
    sock.bind(('',int(sys.argv[1])))
    sock.listen(5)
    while 1:    # Run until cancelled
        newsock, client_addr = sock.accept()
        print "Client connected:", client_addr
        handleClient(newsock)
           
