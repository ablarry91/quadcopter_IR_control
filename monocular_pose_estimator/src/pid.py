#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

target = Pose()
target.position.x = 0
target.position.y = 0
target.position.z = .32
target.orientation.x = 0
target.orientation.y = 0
target.orientation.z = 0
target.orientation.w = 0

kpZ = 1
kiZ = 0.01
kdZ = 0.1
kpR = 1
kiR = 0.01
kdR = 0.1
kpP = 1
kiP = 0.01
kdP = 0.1
kpY = 1
kiY = 0.01
kdY = 0.1

thrustI = 0
thrustD = 0
thrustPrev = 0
maxI = .5

PWM = 50


def callback(data):
	# you can find more info about a particular topic by typing in 'rosmsg show geometry_msgs/PoseWithCovarianceStamped'
	# rospy.loginfo("I heard %s", data.pose.pose.position)
	pid(data,target)

def pid(meas, target):
	global thrustI, thrustD, thrustPrev, maxI, PWM
	# rospy.loginfo("I heard %s", meas.pose.pose.position)

	# Determine errors
	thrustP = meas.pose.pose.position.z - target.position.z
	thrustI = thrustI + thrustP
	thrustD = thrustP - thrustPrev
	thrustPrev = thrustP
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
	rospy.loginfo("PWM is %s\n", PWMout)

	# Publish the data
	publish(PWMout)

def publish(data):
	pub = rospy.Publisher('pwm_control', UInt8, queue_size=10)
	# rospy.init_node('talker', anonymous=True)
	pwm = UInt8()
	pwm.data = data
	pub.publish(pwm)
	rospy.loginfo("published %s\n", data)
	pass

def GUI(data):
    global kpZ, kiZ, kdZ, kpR, kiR, kdR, kpP, kiP, kdP, kpY, kiY, kdY
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
    print data

	
def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('pid', anonymous=True)

	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, callback)
	rospy.Subscriber("sliderData", Int16MultiArray, GUI)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()