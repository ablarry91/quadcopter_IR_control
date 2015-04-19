#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool,UInt8MultiArray
from Tkinter import *

sliderLength = 160 #the physical length of a slider
abortStatus = True #emergency stop for the quadcopter
abortText = "Enable motors"  #is there a word for robot phobia?
resetStatus = False #resets the gains



#any time the slider changes, this function is called
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

#force stop the quadcopter
def abortCommand():
	global abortStatus
	if abortStatus:
		# rospy.loginfo("published")
		abortStatus = False
		b0["text"] = "I REGRET THIS DECISION"
	else:
		# rospy.loginfo("published")
		abortStatus = True
		b0["text"] = "Enable motors."
	a = Bool()
	a.data = abortStatus
	kill.publish(a)

#reset the gains
def resetCommand():
	a = Bool()
	a.data=False
	reset.publish(a)

#connect the RF controller to the quadcopter
def syncCommand():
	a = Bool()
	a.data=True
	sync.publish(a)

def pwmControl():
	data1 = int(e1.get())
	data2 = int(e2.get())
	data3 = int(e3.get())
	data4 = int(e4.get())
	dataOut = UInt8MultiArray()
	dataOut.data = [data1,data2,data3,data4]
	rospy.loginfo("PWM published: %s\n    ", dataOut)
	pwm.publish(dataOut)

#create the GUI window
root = Tk()
frame = Frame(root)
frame.pack()

#build subframes into the main GUI frame
frame0 = Frame(frame)
frame01 = Frame(frame,borderwidth=2,relief=RAISED)
frame1 = Frame(frame)
frame2 = Frame(frame)
frame3 = Frame(frame)
frame4 = Frame(frame)

#indicate where you want the frames to be oriented
frame0.pack(side = TOP)
frame01.pack(side = TOP)
frame1.pack(side = TOP)
frame2.pack(side = TOP)
frame3.pack(side = TOP)
frame4.pack(side = TOP)

#reserve a float for each gain value
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

#create buttons
b0 = Button(frame0, text=abortText, command=abortCommand)
b0.pack(side=RIGHT)
b1 = Button(frame0, text="Reset gains", command=resetCommand)
b1.pack(side=RIGHT)
b2 = Button(frame0, text="Sync",command=syncCommand)
b2.pack(side=RIGHT)
b3 = Button(frame0, text="PWM Update",command=pwmControl)
b3.pack(side=RIGHT)

#create text entries
frameE1 = Frame(frame01)
frameE2 = Frame(frame01)
frameE3 = Frame(frame01)
frameE4 = Frame(frame01)
frameE1.pack(side = LEFT)
frameE2.pack(side = LEFT)
frameE3.pack(side = LEFT)
frameE4.pack(side = LEFT)

#PWM labels
label1 = Label(frameE1,text="thrust")
label2 = Label(frameE2,text="roll")
label3 = Label(frameE3,text="pitch")
label4 = Label(frameE4,text="yaw")
label1.pack(side=TOP)
label2.pack(side=TOP)
label3.pack(side=TOP)
label4.pack(side=TOP)

#PWM values being sent to quad
e1 = Entry(frameE1,width = 10,justify=CENTER)
e1.pack(side=RIGHT)
e1.insert(0, "0")
e2 = Entry(frameE2,width = 10,justify=CENTER)
e2.pack(side=RIGHT)
e2.insert(0, "0")
e3 = Entry(frameE3,width = 10,justify=CENTER)
e3.pack(side=RIGHT)
e3.insert(0, "0")
e4 = Entry(frameE4,width = 10,justify=CENTER)
e4.pack(side=RIGHT)
e4.insert(0, "0")

#insert the scales into the appropriate frames and associate to your desired variable
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

def lostSignal(data):
	global abortStatus
	if data.data == True:
		rospy.loginfo("LOST SIGNAL.  ENSURE DATA IS BEING GATHERED AND REENABLE MOTORS.")
		abortStatus = False
		abortCommand()

def updatePWM(data):
	e1.delete(0, END)
	e2.delete(0, END)
	print "thrust is",data.data
	print "yaw is ",data.data[1]
	print data
	# e1.insert(0, int(data.data[0]))
	# e2.insert(0, int(data.data[1]))
	# e3.insert(0, str(data.data[2]))
	# e4.insert(0, str(data.data[3]))
	# print "data is",int(data.data[1])
	# print "data is",str(data.data[1])

#create the publishers
pub = rospy.Publisher('sliderData', Float32MultiArray, queue_size=20)
kill = rospy.Publisher('killCommand', Bool, queue_size=5)
reset = rospy.Publisher('resetCommand', Bool, queue_size=5)
sync = rospy.Publisher('syncCommand', Bool, queue_size=5)
pwm = rospy.Publisher('pwmInput', UInt8MultiArray, queue_size=5)
rospy.Subscriber("fail_safe",Bool, lostSignal)
# rospy.Subscriber("pwm_control", UInt8MultiArray, updatePWM)
rospy.init_node('tkinterGUI', anonymous=True)

#run the loop indefinitely
root.mainloop()

