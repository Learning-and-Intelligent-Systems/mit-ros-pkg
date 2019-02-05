import roslib
roslib.load_manifest('bakebot')
import rospy
from Tkinter import *
from bakebot.srv import *
from bakebot_pick_and_place_manager import *
from unified_tf_client import UnifiedTFClient
import tf
import actionlib
import itertools
import math
import sys
import time
from bakebot_controller_manager import *
from mixing_client import *

class MixingUI(Frame):
    def __init__(self, dryrun = False, master=None):
        self.dryrun = dryrun
        Frame.__init__(self, master)
        self.grid()
        self.createWidgets()
        self.lower = False
        self.lin = False
        self.circ = False
        self.rse = False
        self.ismix = False
        self.isdf = False
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        if not self.dryrun:
            self.mc = MixingClient(True)

    def createWidgets(self):
        Label(self, text = 'mbc x').grid(row=0,column=0)
        Label(self, text = 'mbc y').grid(row=0,column=1)
        Label(self, text = 'mbc z').grid(row=0,column=2)
        Label(self, text = 'rad').grid(row=0,column=3)
        Label(self, text = 'rpad').grid(row=0,column=4)

        self.xbox = Entry(self)
        self.ybox = Entry(self)
        self.zbox = Entry(self)
        self.rbox = Entry(self)
        self.rpbox = Entry(self)
        self.xbox.grid(row=1, column = 0)
        self.ybox.grid(row=1, column = 1)
        self.zbox.grid(row=1, column = 2)
        self.rbox.grid(row=1, column = 3)
        self.rpbox.grid(row=1, column = 4)

        Label(self, text = ' ').grid(row=2,column=1)

        self.ismixbut = Checkbutton(self, text='is MIX', command = self.setmix)
        self.ismixbut.grid(row=3, column = 0)
        self.singleactbut = Checkbutton(self, text='single action only')
        self.singleactbut.grid(row=3, column = 2)

        Label(self, text = ' ').grid(row=4,column=1)

        Label(self, text = 'laps').grid(row=5, column = 1)
        Label(self, text = 'period').grid(row=5, column = 2)
        Label(self, text = 'force').grid(row=5, column = 3)
        self.islowerbut = Checkbutton(self, text='lower', command = self.setlower)
        self.iscircbut = Checkbutton(self, text='circular', command = self.setcirc)
        self.islinbut = Checkbutton(self, text='linear', command = self.setlin)
        self.israisebut = Checkbutton(self, text='raise', command = self.setraise)
        self.islowerbut.grid(row=6, column=0, sticky='W')
        self.iscircbut.grid(row=7, column=0, sticky='W')
        self.islinbut.grid(row=8, column=0, sticky='W')
        self.israisebut.grid(row=9, column=0, sticky='W')

        self.defaultsbut = Button(self, text = 'restore defaults', command = self.resetdefaults)
        self.defaultsbut.grid(row=5,column=5)
        
        self.clapsbox = Entry(self)
        self.llapsbox = Entry(self)
        self.clapsbox.grid(row = 7, column = 1)
        self.llapsbox.grid(row = 8, column = 1)
        self.cperbox = Entry(self)
        self.lperbox = Entry(self)
        self.lfbox = Entry(self)
        self.cperbox.grid(row = 7, column = 2)
        self.lperbox.grid(row = 8, column = 2)
        self.lfbox.grid(row = 8, column = 3)

        Label(self, text = ' ').grid(row=9,column=1)
        Label(self, text = 'force').grid(row=10, column=1)
        Label(self, text = 'stiffness').grid(row=10, column=3)

        self.isdownforcebut = Checkbutton(self, text = 'is down force', command = self.setisdf)
        self.isdownforcebut.grid(row=12, column = 0, sticky='W')

        self.downforcebox = Entry(self)
        self.radstiffbox = Entry(self)
        self.angstiffbox = Entry(self)
        self.downforcebox.grid(row=12, column = 1)
        self.radstiffbox.grid(row=11, column = 3)
        self.angstiffbox.grid(row=12, column = 3)
        Label(self, text='position').grid(row=11, column=4)
        Label(self, text='orientation').grid(row=12, column=4)

        Label(self, text = ' ').grid(row=13,column=1)
        self.gobutton = Button(self, text = 'SEND MIXING SERVICE REQUEST', command = self.sendrequest)
        self.gobutton.grid(row=14,column=0, columnspan=6, sticky='nsew')

    def setlower(self):
        self.lower = not self.lower

    def setcirc(self):
        self.circ = not self.circ

    def setlin(self):
        self.lin = not self.lin

    def setraise(self):
        self.rse = not self.rse

    def setmix(self):
        self.ismix = not self.ismix

    def setisdf(self):
        self.isdf = not self.isdf

    def resetdefaults(self):
        self.xbox.delete(0,END)
        self.xbox.insert(0,'0.4')

        self.ybox.delete(0,END)
        self.ybox.insert(0,'-0.25')

        self.zbox.delete(0,END)
        self.zbox.insert(0,'-0.25')

        self.rbox.delete(0,END)
        self.rbox.insert(0,'0.15')

        self.rpbox.delete(0,END)
        self.rpbox.insert(0,'0.04')

        self.clapsbox.delete(0,END)
        self.clapsbox.insert(0,'5')

        self.llapsbox.delete(0,END)
        self.llapsbox.insert(0,'3')

        self.cperbox.delete(0,END)
        self.cperbox.insert(0,'7')

        self.lperbox.delete(0,END)
        self.lperbox.insert(0,'1')

        self.lfbox.delete(0,END)
        self.lfbox.insert(0,'10')

        self.downforcebox.delete(0,END)
        self.downforcebox.insert(0,'10')

        self.radstiffbox.delete(0,END)
        self.radstiffbox.insert(0,'500')

        self.angstiffbox.delete(0,END)
        self.angstiffbox.insert(0,'50')

        self.lower = False
        self.lin = False
        self.circ = False
        self.rse = False
        self.ismix = False
        self.isdf = False
        self.ismixbut.deselect()
        self.ismixbut.invoke()
        self.singleactbut.deselect()
        self.isdownforcebut.deselect()
        #self.isdownforcebut.invoke()
        self.islinbut.deselect()
        self.iscircbut.deselect()
        self.iscircbut.invoke()
        self.israisebut.deselect()




    def sendrequest(self):
        print 'setting up the service request'
        try:
            x = float(self.xbox.get())
            y = float(self.ybox.get())
            z = float(self.zbox.get())
            r = float(self.rbox.get())
            rp = float(self.rpbox.get())
            print 'x', x
            print 'y', y
            print 'z', z
            print 'r', r
            print 'rp', rp
            im = self.ismix
            lower = self.lower
            circ = self.circ
            lin = self.lin
            rse = self.rse
            isdf = self.isdf
            print im
            print lower
            print circ
            print lin
            print rse
            print isdf
            claps = float(self.clapsbox.get())
            llaps = float(self.llapsbox.get())
            cperd = float(self.cperbox.get())
            lperd = float(self.lperbox.get())
            print claps
            print llaps
            print cperd
            print lperd
            df = float(self.downforcebox.get())
            radstiff = float(self.radstiffbox.get())
            angstiff = float(self.angstiffbox.get())
            print df
            print radstiff
            print angstiff

            lf = float(self.lfbox.get())
            print 'lf', lf

            print 'closing left gripper'
            self.papm.close_gripper(1)
            print 'closing right gripper'
            self.papm.close_gripper(0)

            if not self.dryrun:
                self.mc.call_service((x,y,z), r, .4, im, lower, circ, lin, False, rse, claps, llaps, 1, cperd, lperd, 10, 3, 7, df, lf, radstiff, angstiff, rp, False, isdf)

        except Exception as e:
            print 'an error has occurred'
            print e





if __name__ == '__main__':
    rospy.init_node('mixing_client_tester', anonymous=True)
    app = MixingUI(dryrun=False)
    app.master.title('sample')
    app.mainloop()
