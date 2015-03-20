#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from Tkinter import *

sliderLength = 160

pub = rospy.Publisher('sliderData', Float32MultiArray, queue_size=20)
rospy.init_node('tkinterGUI', anonymous=True)

def publishData(junk):
	a = Float32MultiArray()
	a.data.append(float(var1.get()))
	a.data.append(float(var2.get()))
	a.data.append(float(var3.get()))
	a.data.append(float(var4.get()))
	a.data.append(float(var5.get()))
	a.data.append(float(var6.get()))
	a.data.append(float(var7.get()))
	a.data.append(float(var8.get()))
	a.data.append(float(var9.get()))
	a.data.append(float(var10.get()))
	a.data.append(float(var11.get()))
	a.data.append(float(var12.get()))
	pub.publish(a)
	# print a.data

def abort():
	print 'bloop!'
	# a = Int16MultiArray
	pass

def reset():
	pass

root = Tk()
frame = Frame(root)
frame.pack()

frame0 = Frame(frame)
frame1 = Frame(frame)
frame2 = Frame(frame)
frame3 = Frame(frame)
frame4 = Frame(frame)

frame0.pack(side = TOP)
frame1.pack(side = TOP)
frame2.pack(side = TOP)
frame3.pack(side = TOP)
frame4.pack(side = TOP)

var1 = DoubleVar()
var2 = DoubleVar()
var3 = DoubleVar()
var4 = DoubleVar()
var5 = DoubleVar()
var6 = DoubleVar()
var7 = DoubleVar()
var8 = DoubleVar()
var9 = DoubleVar()
var10 = DoubleVar()
var11 = DoubleVar()
var12 = DoubleVar()

b0 = Button(frame0, text="Abort!", command=abort)
b0.pack(side=LEFT)

b1 = Button(frame0, text="Reset", command=abort)
b1.pack(side=RIGHT)

scale1 = Scale(frame1, label = 'kPThrust', variable = var1, from_ = 0.00, to = 5.00, length=sliderLength, resolution=0.01, command = publishData)
scale1.set(0)
scale1.pack(side=LEFT)
scale2 = Scale(frame1, label = 'kiThrust', variable = var2, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale2.set(0)
scale2.pack(side=LEFT)
scale3 = Scale(frame1, label = 'kdThrust', variable = var3, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale3.set(0)
scale3.pack(side=LEFT)
scale4 = Scale(frame2, label = 'kpRoll', variable = var4, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale4.set(0)
scale4.pack(side = LEFT)
scale5 = Scale(frame2, label = 'kiRoll', variable = var5, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale5.set(0)
scale5.pack(side=LEFT)
scale6 = Scale(frame2, label = 'kdRoll', variable = var6, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale6.set(0)
scale6.pack(side=LEFT)
scale7 = Scale(frame3, label = 'kpPitch', variable = var7, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale7.set(0)
scale7.pack(side=LEFT)
scale8 = Scale(frame3, label = 'kiPitch', variable = var8, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale8.set(0)
scale8.pack(side=LEFT)
scale9 = Scale(frame3, label = 'kdPitch', variable = var9, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale9.set(0)
scale9.pack(side=LEFT)
scale10 = Scale(frame4, label = 'kpYaw', variable = var10, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale10.set(0)
scale10.pack(side = LEFT)
scale11 = Scale(frame4, label = 'kiYaw', variable = var11, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale11.set(0)
scale11.pack(side=LEFT)
scale12 = Scale(frame4, label = 'kdYaw', variable = var12, from_ = 0, to = 5, length=sliderLength, resolution=0.01, command = publishData)
scale12.set(0)
scale12.pack(side=LEFT)

root.mainloop()