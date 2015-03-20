#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, Float32MultiArray, Bool
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

PWM = 0
PWMWait = PWM
wait = True


def callback(data):
	pid(data,target)

def pid(meas, target):
	global thrustI, thrustD, thrustPrev, maxI, PWM
	# rospy.loginfo("I heard %s", meas.pose.pose.position)

	# Determine errors
	thrustP = meas.pose.pose.position.z - target.position.z
	thrustI = thrustI + thrustP
	thrustD = thrustP - thrustPrev
	thrustPrev = thrustP
	# rospy.loginfo("thrust error is %s", thrustP)

	# Integrator anti windup in case it grows too much
	if thrustI > maxI:
		thrustI = maxI

	# Control effort
	uThrust = kpZ*thrustP + kiZ*thrustI + kdZ*thrustD
	# rospy.loginfo("thrust effort is %s", uThrust)

	# Convert for PWM
	PWM = PWM + uThrust
	PWMout = int(PWM)

	# Publish the data
	if wait == True:
		publish(0)
	elif wait == False:
		publish(PWMout)

def publish(data):
	pub = rospy.Publisher('pwm_control', UInt8, queue_size=10)
	pwm = UInt8()
	pwm.data = data
	pub.publish(pwm)
	# rospy.loginfo("PWM published: %s\n    ", data)

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
    # print data
    
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
	
def listener():
	rospy.init_node('pid', anonymous=True)

	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, callback)
	rospy.Subscriber("sliderData", Float32MultiArray, GUI)
	rospy.Subscriber("killCommand",Bool, kill)
	rospy.Subscriber("resetCommand", Bool, resetCommand)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()