#python server commands to talk to socketwamif
#see bottom for example usage (run if this script is main)
#written by Kaijen Hsiao (questions? email kaijenhsiao@gmail.com)

from socket import *   
import sys
import time
import math

#pretty-print a double array
def ppdoublearray(array):
    strlist = ["%4.2f"%x for x in array]
    return " ".join(strlist)


#Client/server commands


#connect to the server as a client
def start_client(port, ipaddr = "localhost"):
    clientsock = socket(AF_INET, SOCK_STREAM)
    clientsock.connect((ipaddr, port))
    return clientsock


#connect to a new client (blocks until new client shows up)
def connect_client(sockserver):
    print "waiting for client"
    #connect to client (socketwamif)
    sock, client_addr = sockserver.accept()
    print "Client connected:", client_addr

    #send confirmation 'c' to tell client we're connected 
    #sock.send("c")
    #print "sent confirmation c" 

    return sock


#start up the socket server
def start_server(port):   
    sockserver = socket(AF_INET, SOCK_STREAM)
    sockserver.bind(('',port))
    sockserver.listen(5)
    print "started up sockserver"
    return sockserver


#shut down client/arm/hand and close the connection
def shutdown_all(sock):
    sock.send("shu")
    #get 'q' of socket client closing
    closeq = receive_block(sock, 1)
    print "received", closeq
    sock.close()
    print "closed socket"



#Socket read/write commands

#send a string to the socket
def send(sock, msg):
    totalsent = 0
    msglen = len(msg)
    while totalsent < msglen:
        sent = sock.send(msg[totalsent:])
        #if sent == 0:
        #    print "socket connection broken"
            #sys.exit(1)
        totalsent = totalsent + sent


#receive a block of length blocklen
def receive_block(sock, blocklen):
    msg = ''
    while len(msg) < blocklen:
        chunk = sock.recv(blocklen-len(msg))
        #if chunk == '':
        #    print "socket connection broken"
            #sys.exit(1)
        if chunk != '':
            msg = msg + chunk
    return msg


#write an array to the socket
def write_array(sock, array):
    arraystr = ' '.join([str(x) for x in array])
    arraystr += '\0'
    send(sock, arraystr)


#read an array of doubles from the socket
def read_double_array(sock, length):
    array = [0]*length
    for i in range(length):
        data = receive_block(sock, 1)
        numstr = ''
        while data != ' ' and data != '\0' and data != '\n':
            numstr = numstr + data
            data = receive_block(sock, 1)
        try:
            array[i] = float(numstr)
        except:
            print "not a float:", numstr
    return array


#read a char from the socket
def read_char(sock):
    data = []
    while len(data) == 0:
        data = receive_block(sock, 1)
    return data


#read an int from the socket
def read_int(sock):
    numstr = ''
    readint = 0
    data = receive_block(sock, 1)
    while data != ' ' and data != '\0' and data != '\n':
        numstr = numstr + data
        data = receive_block(sock, 1)
    try:
        readint = int(numstr)
    except:
        print "not an int:", numstr
    return readint


#read an array of ints from the socket
def read_int_array(sock, length):
    array = [0]*length
    for i in range(length):
        array[i] = read_int(sock)
    return array


#write an int or double to the socket
def write_num(sock, num):
    numstr = str(num) + '\n'
    #print "numstr:",  numstr
    try:
        send(sock, numstr)
    except:
        print "sock.send failed!"
        print "trying to send numstr:", numstr


#get an acknowledgement character ('d', returns 1 if success, 0 if failure)
def get_ack_d(sock):
    retval = read_char(sock)
    if retval != 'd':
        print "error! retval:", retval    
        return 0
    #print "got ack"
    return 1    


#send just a header, receive 'd' in reponse (returns 1 if success, 0 if failure)
def header_only_cmd(sock, string):
    send(sock, string)
    return get_ack_d(sock)



#General arm commands

#connect the arm (must be run before other arm commands!)
#connect the hand separately with connect_hand
def connect_arm(sock):
    return header_only_cmd(sock, "act")


#shut down the arm (but not the socket connection; reconnect with connect_arm)
def disconnect_arm(sock):
    return header_only_cmd(sock, "asd")


#turn on gravity comp (returns 1 if success, 0 if failure)
def set_gcomp(sock):
    return header_only_cmd(sock, "gra")


#run joint angle calibration for optical encoders (put arm in home position first)
#joint angle sequence for calibration should be in calibrationangles.txt in the directory where socketwamif is run
def run_joint_calibration(sock):
    return header_only_cmd(sock, "cal")


#wait for trajectory to finish (returns 1 if success, 0 if failure)
def wait_until_traj_done(sock):
    return header_only_cmd(sock, "trf")


#check if trajectory is finished (non-blocking) (1 if done, 0 if not, -1 if err)
def check_traj_done(sock):
    send(sock, "ckt")
    retval = read_char(sock)
    if retval == '0':
        return 0
    elif retval == '1':
        return 1
    else:
        print "error! retval:", retval
        return -1


#disable controllers (returns 1 if success, 0 if failure)
def stop_controllers(sock):
    return header_only_cmd(sock, "dis")
        


#Arm joint control


#move to joint angles (returns 1 if success, 0 if failure)
def move_joint(sock, angles):
    send(sock, "moj")
    write_array(sock, angles)    
    return get_ack_d(sock)


#get current joint angles
def get_joint(sock):
    send(sock, "gja")
    jointangles = read_double_array(sock, 7)
    return jointangles


#get current motor angles (rad from zero pos)
def get_motor_angles(sock):
    send(sock, "gmc")
    motorcounts = read_double_array(sock, 7)
    return motorcounts


#send the arm back home (returns 1 if success, 0 if failure)
def go_home(sock):
    return header_only_cmd(sock, "hom")


#set the WAMcallback torque limits for the arm (Nm)
#[6.31, 6.31, 6.31, 6.31, 1.826, 1.826, 0.613] are Barrett's stated peak values
#[7.75,7.75,7.75,7.75,2.5,2.5,2] gives you most of the possible torque while avoiding most torque faults
def set_torque_limits(sock, torquelimits):
    send(sock, "tlm")
    write_array(sock, torquelimits)
    return get_ack_d(sock)



#Arm Cartesian control


#move to Cartesian position/orientation (returns 1 if success, 0 if failure)
#pos is [x,y,z] in meters
#rot is a 3x3 rotation matrix as a 9-vector [R11, R12, R13, R21... R33]
#(origin is the point where joint 0,1,2 axes meet, frame as defined in manual)
def move_cartesian(sock, pos, rot):
    send(sock, "moc")
    write_array(sock, pos)
    write_array(sock, rot)
    return get_ack_d(sock)


#get Cartesian position/orientation
#pos is [x,y,z] in meters
#rot is a 3x3 rotation matrix as a 9-vector [R11, R12, R13, R21... R33]
#(origin is the point where joint 0,1,2 axes meet, frame as defined in manual)
def get_cartesian(sock):
    send(sock, "gcp")
    pos = read_double_array(sock, 3)
    rot = read_double_array(sock, 9)
    return (pos, rot)


#Arm joint trajectory control

#move through a joint angle trajectory (list of double[7]s of length length)
#e.g: len = 2, traj = [0,1,2,3,4,5,6,0,1,2,3,4,5,6]
def move_joint_trajectory(sock, length, traj):
    if len(traj) % length != 0:
        return  
    send(sock, "jtc")
    write_num(sock, length)
    # uncomment code below to allow traj format traj = [[0,1,2,3,4,5,6],[0,1,2,3,4,5,6]]
    #otraj = []
    #for x in traj:
    #    otraj.extend(x)
    #write_array(sock, otraj)
    write_array(sock,traj)
    return get_ack_d(sock)


#Hand supervisory commands

#connect to the hand (must be run before all other hand commands!)
def connect_hand(sock):
    return header_only_cmd(sock, "hct")


#disconnect from hand (can be reconnected with connect_hand)
def disconnect_hand(sock):
    return header_only_cmd(sock, "hsd")


#open the hand (GO) (returns 1 if success, 0 if failure)
def open_fingers(sock):
    return header_only_cmd(sock, "opn")


#close the hand (GC) (returns 1 if success, 0 if failure)
def close_fingers(sock):
    return header_only_cmd(sock, "cls")


#open spread as well as grasp (returns 1 if success, 0 if failure)
def spread_grasp_open(sock):
    return header_only_cmd(sock, "sgo")


#make the hand go back into its home position (1 if success, 0 if failure)
def hand_home(sock):
    return header_only_cmd(sock, "hhe")


#set the position (int, 0-18000) for one motor 
#motor is 0 for spread, 1-3 for finger bend
#waits for termination (1 if success, 0 if failure)
def set_finger_position(sock, motor, position):
    send(sock, "omp")
    write_num(sock, int(position))
    write_num(sock, int(motor))
    return get_ack_d(sock)


#set the positions for all four motors ([S F1 F2 F3])
#waits for termination (1 if success, 0 if failure)
def set_all_finger_positions(sock, positions):
    send(sock, "amp")
    write_array(sock, positions)
    return get_ack_d(sock)


#set the angle (double, joint radians) for one motor 
#motor is 0 for spread, 1-3 for finger bend
#waits for termination (1 if success, 0 if failure)
def set_finger_angle(sock, motor, angle):
    send(sock, "oma")
    write_num(sock, float(angle))
    write_num(sock, int(motor))
    return get_ack_d(sock)


#set the angles for all four motors ([S F1 F2 F3])
#waits for termination (1 if success, 0 if failure)
def set_all_finger_angles(sock, angles):
    send(sock, "ama")
    write_array(sock, angles)
    return get_ack_d(sock)


#get the positions (0-17800 for bend, 0-3150 for spread) for all four motors ([S F1 F2 F3])
def get_finger_positions(sock):
    send(sock, "gmp")
    positions = read_int_array(sock, 4)
    return positions


#get the breakaway status and positions for the three fingers [F1 F2 F3]
def get_breakaway_positions(sock):
    send(sock, "brk")
    breakawaystatus = read_int_array(sock, 3)
    breakawaypositions = read_int_array(sock, 3)
    return (breakawaystatus, breakawaypositions)


#get the joint angles for all seven joints ([S F1 F2 F3 F1tip F2tip F3tip])
def get_finger_angles(sock):
    send(sock, "gma")
    angles = read_double_array(sock, 7)
    return angles


#get the strain gauge values for all three fingers ([F1 F2 F3])
def get_strain_gauge(sock):
    send(sock, "gsg")
    strainvals = read_int_array(sock, 3)
    return strainvals


#send raw hand command (returns 1 if success, 0 if failure)
#don't put the \r at the end (it gets added later)
def hand_raw(sock, cmd):
    send(sock, "cmd")
    send(sock, cmd + "\n")
    return get_ack_d(sock)



#Hand realtime commands


#start hand realtime mode
#takes in the parameter string and the loop string (leave out to use defaults)
def start_hand_realtime(sock, parameterstring = "0", loopstring = "0"):
    parameterstring = parameterstring + "\n"
    loopstring = loopstring + "\n"

    send(sock, "hrs")
    send(sock, parameterstring)
    send(sock, loopstring)

    retval = read_char(sock)
    if retval != 'd':
        print "error! retval:", retval    
        return 0
    return 1


#terminate hand realtime mode (back to supervisory mode)
def terminate_hand_realtime(sock):
    return header_only_cmd(sock, "hre")


#bend the fingers in realtime mode
#assumes the default startup in wamif_hand_start_realtime
#(use raw hand commands to send other realtime commands)
#takes in vels[3] and gains[3], both between -127 and 127
#returns the default feedback (strainvals[3], fingerpositions[3]
def hand_bend_realtime(sock, vels, gains):
    send(sock, "hrb")
    write_array(sock, vels)
    write_array(sock, gains)
    strainvals = read_int_array(sock, 3)
    fingerpositions = read_int_array(sock, 3)
    return (strainvals, fingerpositions)


#send realtime command and get feedback block (must be in realtime mode already)
#blocklen is the expected length of the feedback block (not including the *)
#returns the feedback block (None if error)
#blocklen can be 0 if you just want the acknowledgement char (*)
def send_realtime_command(sock, cmd, blocklen):
    send(sock, "hrf")
    send(sock, cmd + '\0')
    write_num(sock, blocklen)
    feedbackblock = receive_block(sock, blocklen)
    return feedbackblock


#clip a value to be between +- mag
def clipVal(val, mag):
    if val > mag:
        return mag
    if val < -mag:
        return -mag
    return val


#same as hand_bend_realtime (demonstrates send_realtime_command)
#assuming the default hand startup, send vels and gains 
#and get back the strainvals and fingerpositions
def default_realtime_demo(sock, vels, gains):
    import struct
    cvels = [clipVal(vel, 127) for vel in vels]
    cgains = [clipVal(gain, 127) for gain in gains]

    #pack the command
    cmd = struct.pack('cbbbbbb', 'C', cvels[0], cgains[0], cvels[1], cgains[1], cvels[2], cgains[2])
    feedbackblock = send_realtime_command(sock, cmd, 9)
    
    #unpack the feedback block into strainvals and fingerpositions
    if feedbackblock != None:
        strainvals = [0]*3
        fingerpositions = [0]*3
        feedbackarray = [ord(x) for x in feedbackblock]
        strainvals = [0]*3
        fingerpositions = [0]*3
        for fingernum in range(3):
            strainvals[fingernum] = feedbackarray[fingernum*3]
            fingerpositions[fingernum] = (feedbackarray[fingernum*3+1] << 8) | feedbackarray[fingernum*3+2]
        #print "strainvals:", strainvals
        #print "fingerpositions:", fingerpositions
        return (strainvals, fingerpositions)
    else:
        print "feedbackblock was empty!"
        return (0,0)


#pause for enter
def keypause():
    print "press enter twice to continue"
    raw_input()
    raw_input()


#sample code showing how to use the above functions    
if __name__ == '__main__':
    sockserver = start_server(4321)
    movearm = 1
    movehand = 0
    pause = 0
    while 1:    # Run until cancelled with Ctrl-C

        sock = connect_client(sockserver)

        if movearm:
            print "connecting to arm"
            connect_arm(sock)
            
            
            print "getting motor counts"
            motorangles = get_motor_angles(sock)
            print "motorangles:", ppdoublearray(motorangles)

            torquelimits = [9.99,9.99,9.99,9.99,9.99,9.99,9.99]
            print "setting torque limits to", torquelimits
            set_torque_limits(sock, torquelimits)

            print "turning on gravity compensation"
            set_gcomp(sock)
            if pause:
                keypause()

            print "moving to just above home (in joint mode)"
            justabovehome = [0, -1.99, 0, 2.4, 0, 0.25, 0]
            move_joint(sock, justabovehome)
            
            print "waiting for trajectory to finish"
            wait_until_traj_done(sock)
            
            print "getting current joint angles"
            currentjointangles = get_joint(sock)
            print "jointangles:", ppdoublearray(currentjointangles)
            if pause:
                keypause()

            print "moving farther out (in Cartesian mode)"
            fartheroutpos = [.31, 0, .42]
            fartheroutrot = [0,0,1,	 
                             0,1,0, 
                             -1,0,0]
            move_cartesian(sock, fartheroutpos, fartheroutrot)
            
            print "waiting for trajectory to finish"
            wait_until_traj_done(sock)
            
            print "get the current Cartesian position/orientation"
            (pos, rot) = get_cartesian(sock)
            print "pos:", pos
            print "rot:", rot
            if pause:
                keypause()
            
            print "get the current joint angles"
            currentjointangles = get_joint(sock)
            print "jointangles:", ppdoublearray(currentjointangles)

            print "carrying out a trajectory"
            length = 4
            traj = [
                   [0.009007, -2.005244, 0.041125, 3.049417, 0.081722, -0.131585, -0.035286],
                   [-0.000298, -1.762723, 0.034756, 2.848247, 0.062907, -0.094783, 0.002270],
                   [-0.017828, -0.875831, 0.030786, 2.565059, 0.145275, 0.031809, -0.054683],
                   [-0.017418, -1.459543, 0.024555, 2.865306, 0.103445, 0.092045, -0.078104]
                ]
            t=[]
            for a in traj:
                t.extend(a)
            success = move_joint_trajectory(sock,length,t)
            print "traj move success?", success!= None
            
            print "get the current joint angles"
            currentjointangles = get_joint(sock)
            print "jointangles:", ppdoublearray(currentjointangles)
            
            if pause:
                keypause()

        if movehand:
            print "connecting to hand"
            connect_hand(sock)

            print "spread grasp open"
            spread_grasp_open(sock)
    
            print "closing fingers"
            close_fingers(sock)

            print "check breakaway status and positions"
            (breakawaystatus, breakawaypositions) = get_breakaway_positions(sock)
            print "breakawaystatus:", breakawaystatus
            print "breakawaypositions:", breakawaypositions

            print "getting strain gauge values"
            strainvals = get_strain_gauge(sock)
            print "strainvals:", strainvals

            print "opening fingers (raw command)"
            hand_raw(sock, "GO")
            if pause:
                keypause()

            print "moving just finger 3 to bend 5000"
            set_finger_position(sock, 3, 5000)

            print "getting all finger positions"
            currentpositions = get_finger_positions(sock)
            print "currentpositions:", currentpositions
            if pause:
                keypause()

            print "moving just finger 3 to angle PI/2"
            set_finger_angle(sock, 3, math.pi/2)

            print "getting all finger angles"
            currentangles = get_finger_angles(sock)
            print "currentangles:", currentangles
            if pause:
                keypause()

            print "moving fingers to positions spread 0, bend 5000, 10000, 15000"
            positions = [0, 5000, 10000, 15000]
            set_all_finger_positions(sock, positions)
            
            print "getting all finger positions"
            currentpositions = get_finger_positions(sock)
            print "currentpositions:", currentpositions
            if pause:
                keypause()

            print "moving fingers to angles spread PI, bend PI/2 (1.57), PI/3 (1.0), PI/4 (.79)"
            angles = [math.pi, math.pi/2, math.pi/3, math.pi/4]
            set_all_finger_angles(sock, angles)

            print "getting all finger angles"
            currentangles = get_finger_angles(sock)
            print "currentangles:", currentangles
            if pause:
                keypause()

            print "opening fingers"
            open_fingers(sock)
            
            print "starting realtime mode"
            start_hand_realtime(sock)
            
            gains = [100,100,100]
            print "closing hand in realtime mode (using send_realtime_command)"
            for i in range(25):
                vels = [35, 30, 30]
                (strainvals, currentpositions) = default_realtime_demo(sock, vels, gains)
                if not strainvals or not currentpositions:
                    print "error!"
                    break
                print "currentpositions:", currentpositions
                if currentpositions[0]>16000 and currentpositions[1]>16000 and currentpositions[2]>16000:
                    break

            print "opening hand in realtime mode (using hand_bend_realtime)"
            for i in range(25):
                vels = [-55, -50, -50]
                (strainvals, currentpositions) = hand_bend_realtime(sock, vels, gains)
                print "currentpositions:", currentpositions
                if currentpositions[0] == 0 and currentpositions[1] == 0 and currentpositions[2] == 0:
                    break
                
            print "switching back to hand supervisory mode"
            terminate_hand_realtime(sock)
            if pause:
                keypause()

            print "hand home"
            hand_home(sock)

            print "disconnecting hand"
            disconnect_hand(sock)
            if pause:
                keypause()

        if movearm:
            print "sending robot home"
            go_home(sock)

            print "waiting for trajectory to finish (nonblocking)"
            while not check_traj_done(sock):
                time.sleep(1)

            print "stopping controllers"
            stop_controllers(sock)

            print "disconnecting arm"
            disconnect_arm(sock)

        print "shutting down"
        shutdown_all(sock)
        


    
    





















           
