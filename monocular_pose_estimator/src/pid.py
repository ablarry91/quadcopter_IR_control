#!/usr/bin/env python
import rospy
import time
import numpy as np
from std_msgs.msg import UInt8, UInt8MultiArray, Float32MultiArray, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

# create the target at which the quadcopter will attempt to hold
target = Pose()
target.position.x = 0
target.position.y = 0
target.position.z = .32
target.orientation.x = 0
target.orientation.y = 0
target.orientation.z = 0
target.orientation.w = 0

# variables used throughout the code
k = np.zeros([4,3])  #4x3 atrix of PID gains, for thrust, roll, pitch, yaw
inputs = np.zeros([4,3]) #4x3 matrix of integrator error, differential error, and previous error, respectively.  4 channels.
maxI = np.zeros([.5,.5,.5,.5]) #max integrator error
PWM = np.array([0,255,255,255]) #the integer PWM values sent over to an arduino
wait = True #toggled with a button, stops the PID controller from publishing PWM signals if necessary
syncData = True #used for connecting the RF controller to the quad.  You should only have to do this once every session.
upperLimit = 255  #upper limit for PWM command that can be published
lowerLimit = 0   #lower limit for PWM command that can be published

# create the publisher for the arduino
pub = rospy.Publisher('pwm_control', UInt8MultiArray, queue_size=50)


def pidPrep(data):
	pid(data,target)

def pid(meas, target):
	global maxI, PWM, inputs

	# Convert quaternion to euler
	q0 = meas.pose.pose.orientation.x
	q1 = meas.pose.pose.orientation.y
	q2 = meas.pose.pose.orientation.z
	q3 = meas.pose.pose.orientation.w
	roll = np.arctan2(2*(q0*q1+q2*q3),1-2*(q1**2+q2**2))
	pitch = np.arcsin(2*(q0*q2-q3*q1))
	yaw = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**3))
	# print roll,pitch,yaw

	if roll < 0:  #this is a hack to deal with a singularity scenario.  please dont judge
		roll = np.pi+(np.pi+roll)
		print "INVERTING"

	# calculate the roll and pitch corrections needed using a 2D rigid transformation
	rotation = np.matrix([[np.cos(np.radians(yaw)),np.sin(np.radians(yaw))],[-np.sin(np.radians(yaw)),np.cos(np.radians(yaw))]])
	rollPitchE = np.dot(rotation, np.array([meas.pose.pose.position.x, meas.pose.pose.position.y])) #roll and pitch error

	# Update PID controls.  This is really onerous, I know
	inputs[0,0] = inputs[0,0] + meas.pose.pose.position.z - target.position.z
	inputs[0,1] = meas.pose.pose.position.z - target.position.z - inputs[0,2]
	inputs[0,2] = meas.pose.pose.position.z - target.position.z
	# print "z = ",meas.pose.pose.position.z
	inputs[1,0] = inputs[1,0] + rollPitchE[0,0] - target.position.x
	inputs[1,1] = rollPitchE[0,0] - target.position.x - inputs[1,2]
	inputs[1,2] = rollPitchE[0,0] - target.position.x
	inputs[2,0] = inputs[2,0] + rollPitchE[0,1] - target.position.y
	inputs[2,1] = rollPitchE[0,1] - target.position.y - inputs[2,2]
	inputs[2,2] = rollPitchE[0,1] - target.position.y
	inputs[3,0] = inputs[3,0] + yaw
	inputs[3,1] = yaw - inputs[3,2]
	inputs[3,2] = yaw

	# PID equation
	u0 = np.dot(k[0,:],np.transpose(inputs[0,:]))
	u1 = np.dot(k[1,:],np.transpose(inputs[1,:]))
	u2 = np.dot(k[2,:],np.transpose(inputs[2,:]))
	u3 = np.dot(k[3,:],np.transpose(inputs[3,:]))
	u = np.array([u0,u1,u2,u3])

	# # Integrator anti windup in case it grows too much
	# if thrustI > maxI:
	# 	thrustI = maxI
	
	# Convert to PWM and be mindful of limits
	for i in range(len(PWM)):
		val = PWM[i] + int(u[i])
		if val > upperLimit:
			PWM[i] = upperLimit
		elif val < lowerLimit:
			PWM[i] = lowerLimit
		else:
			PWM[i] = PWM[i] + int(u[i])

	#Publish to the arduino
	if wait == True:
		publish(np.zeros(4))
	elif wait == False:
		publish(PWM)

def publish(data):
	"""Publishes PWM values to the microcontroller"""

	dataOut = UInt8MultiArray()
	dataOut.data = [data[0],data[1],data[2],data[3]]
	# rospy.loginfo("PWM published: %s\n    ", dataOut.data)
	pub.publish(dataOut)

def manualPublish(data):
	dataOut = UInt8MultiArray()
	dataOut.data = [data.data[0],data.data[1],data.data[2],data.data[3]]
	rospy.loginfo("PWM published: %s\n    ", dataOut.data)
	pub.publish(data)

def GUI(data):
	"""This is called any time the GUI reports one of the K gains for the PID controller being changed"""

	for i in range(len(data.data)):
		# for j in range(len(data.data)):
		# k[i%4-4,j%3-3] = data.data[i]
		k[np.floor(i/3),i%3-3] = data.data[i]
   
def kill(data):
	"""This switches PWM values to zero if reported by the GUI, as a safety precaution."""

	global wait
	if data.data == False:
		wait = False
	else:
		wait = True

def resetCommand(data):
	"""Resets gains that may have accumulated, if we're resetting an experiment."""

	global thrustI, thrustD, thrustPrev, PWM
	thrustI = 0
	thrustD = 0
	thrustPrev = 0
	PWM = np.zeros([4])

def sync(data):
	"""This function must be called in order for the RF controller to connect to the quadcopter.  You should only have to call this function once per session."""

	global syncData
	syncData = False # I was hoping this could stop a subscriber from working, but no cigar
	for i in range(255):
		publish([i,0,0,0])
		time.sleep(0.01)
	for i in range(255):
		publish([255-i,0,0,0])
		time.sleep(0.01)
	syncData = True  

	
def listener():
	rospy.init_node('pid', anonymous=True)
	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, pidPrep)
	rospy.Subscriber("sliderData", Float32MultiArray, GUI)
	rospy.Subscriber("killCommand",Bool, kill)
	rospy.Subscriber("resetCommand", Bool, resetCommand)
	rospy.Subscriber("syncCommand", Bool, sync)
	rospy.Subscriber("pwmInput",UInt8MultiArray,manualPublish)

	rospy.spin()

if __name__ == '__main__':
	listener()







