#!/usr/local/bin/python
from Tkinter import *

class Application(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.grid()
        self.createWidgets()

    def pose(self):
        print 'click', self.e.get()

    def createWidgets(self):
        self.quitButton = Button(self, text = 'print', command = self.pose)
        self.quitButton.grid(row=10,column=10)
        self.txt = Label(self, text = 'lable')
        self.txt.grid(row=11,column=11)
        self.e = Entry()
        self.e.grid()

app = Application()
app.master.title('sample')
app.mainloop()
