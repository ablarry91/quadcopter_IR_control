#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray
from Tkinter import *

pub = rospy.Publisher('sliderData', Int16MultiArray, queue_size=10)
rospy.init_node('tkinterGUI', anonymous=True)

def publishData(junk):
	a = Int16MultiArray()
	a.data.append(var1.get())
	a.data.append(var2.get())
	a.data.append(var3.get())
	a.data.append(var4.get())
	a.data.append(var5.get())
	a.data.append(var6.get())
	pub.publish(a)
	# print a.data

root = Tk()
frame = Frame(root)
frame.pack()

topFrame = Frame(frame)
botFrame = Frame(frame)

topFrame.pack(side = TOP)
botFrame.pack(side = BOTTOM)

var1 = IntVar()
var2 = IntVar()
var3 = IntVar()
var4 = IntVar()
var5 = IntVar()
var6 = IntVar()

scale1 = Scale(topFrame, label = 'lower1', variable = var1, from_ = 0, to = 255, command = publishData)
scale1.set(0)
scale1.pack(side=LEFT)
scale2 = Scale(topFrame, label = 'lower2', variable = var2, from_ = 0, to = 255, command = publishData)
scale2.set(124)
scale2.pack(side=LEFT)
scale3 = Scale(topFrame, label = 'lower3', variable = var3, from_ = 0, to = 255, command = publishData)
scale3.set(66)
scale3.pack(side=LEFT)
scale4 = Scale(botFrame, label = 'upper1', variable = var4, from_ = 0, to = 255, command = publishData)
scale4.set(91)
scale4.pack(side = LEFT)
scale5 = Scale(botFrame, label = 'upper2', variable = var5, from_ = 0, to = 255, command = publishData)
scale5.set(255)
scale5.pack(side=LEFT)
scale6 = Scale(botFrame, label = 'upper3', variable = var6, from_ = 0, to = 255, command = publishData)
scale6.set(255)
scale6.pack(side=LEFT)

root.mainloop()