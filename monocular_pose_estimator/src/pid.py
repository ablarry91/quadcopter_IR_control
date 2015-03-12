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

Kp = 1
Ki = 0.01
Kd = 0.1

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
	uThrust = Kp*thrustP + Ki*thrustI + Kd*thrustD
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
	# rate = rospy.Rate(10) # 10hz
	# while not rospy.is_shutdown():
	pwm = UInt8()
	pwm.data = data
	# rospy.loginfo(hello_str)
	pub.publish(pwm)
	rospy.loginfo("published %s\n", data)
	# rate.sleep()
	pass
	
def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('pid', anonymous=True)

	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()