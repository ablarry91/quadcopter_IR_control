#!/usr/bin/env python
import rospy
import time
import numpy as np
from std_msgs.msg import UInt8, UInt8MultiArray, Float32MultiArray, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

target = Pose()
target.position.x = 0
target.position.y = 0
target.position.z = .32
target.orientation.x = 0
target.orientation.y = 0
target.orientation.z = 0
target.orientation.w = 0

thrustI = 0
thrustD = 0
thrustPrev = 0
maxI = .5

k = np.zeros([4,3])  #4x3 matrix of PID gains
inputs = np.zeros([4,3]) #4x3 matrix of integrator error, differential error, and previous error, respectively.  4 channels.
maxI = np.zeros([.5,.5,.5,.5]) #max integrator error
PWM = np.zeros([4])

PWM = 0
PWMWait = PWM
wait = True

syncData = True

pub = rospy.Publisher('pwm_control', UInt8MultiArray, queue_size=10)


def callback(data):
	pid(data,target)

def pid(meas, target):
	global thrustI, thrustD, thrustPrev, maxI, PWM
	# global controls, PWM
	# rospy.loginfo("I heard %s", meas.pose.pose.position)

	# Determine errors
	thrustP = meas.pose.pose.position.z - target.position.z
	thrustI = thrustI + thrustP
	thrustD = thrustP - thrustPrev
	thrustPrev = thrustP


	# Convert quaternion to euler
	q0 = meas.pose.pose.orientation.x
	q1 = meas.pose.pose.orientation.y
	q2 = meas.pose.pose.orientation.z
	q3 = meas.pose.pose.orientation.w


	roll = np.arctan2(2*(q0*q1+q2*q3),1-2*(q1**2+q2**2))
	pitch = np.arcsin(2*(q0*q2-q3*q1))
	yaw = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**3))
	print roll,pitch,yaw

	rospy.loginfo("thrust error is %s", thrustP)

	# Integrator anti windup in case it grows too much
	if thrustI > maxI:
		thrustI = maxI

	# Control effort
	uThrust = kpZ*thrustP + kiZ*thrustI + kdZ*thrustD
	rospy.loginfo("thrust effort is %s", uThrust)

	# Convert for PWM
	PWM = PWM + uThrust
	PWMout = int(PWM)

	# Publish the data
	if wait == True:
		publish(0)
	elif wait == False:
		publish(PWMout)

def publish(data):
	# pwm = UInt8()
	# pwm.data = data
	# pub.publish(pwm)
	# rospy.loginfo("PWM published: %s\n    ", data)

	dataOut = UInt8MultiArray()
	dataOut.data = [data,data,data,data]
	rospy.loginfo("PWM published: %s\n    ", data)
	pub.publish(dataOut)

def GUI(data):
    global kpZ, kiZ, kdZ, kpR, kiR, kdR, kpP, kiP, kdP, kpY, kiY, kdY, k
    kpZ = data.data[0]
    kiZ = data.data[1]
    kdZ = data.data[2]
    kpR = data.data[3]
    kiR = data.data[4]
    kdR = data.data[5]
    kpP = data.data[6]
    kiP = data.data[7]
    kdP = data.data[8]
    kpY = data.data[9]
    kiY = data.data[10]
    kdY = data.data[11]

    for i in range(len(data.data)):
    	for j in range(len(data.data)):
    		k[i%4-4,j%3-3] = data.data[i]
    
def kill(data):
	global wait
	if data.data == False:
		wait = False
	else:
		wait = True
	# rospy.loginfo("I heard %s", data)

def resetCommand(data):
	global thrustI, thrustD, thrustPrev, PWM
	thrustI = 0
	thrustD = 0
	thrustPrev = 0
	PWM = 0

def sync(data):
	global syncData
	# rospy.loginfo("I heard %s", data)

	syncData = False
	for i in range(255):
		publish(i)
		time.sleep(0.01)
	for i in range(255):
		publish(255-i)
		time.sleep(0.01)
	syncData = True

	
def listener():
	rospy.init_node('pid', anonymous=True)

	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, callback)
	rospy.Subscriber("sliderData", Float32MultiArray, GUI)
	rospy.Subscriber("killCommand",Bool, kill)
	rospy.Subscriber("resetCommand", Bool, resetCommand)
	rospy.Subscriber("syncCommand", Bool, sync)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()