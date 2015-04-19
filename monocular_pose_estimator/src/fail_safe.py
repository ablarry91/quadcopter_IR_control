#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

timeNew = 0
timeOld = 0
failTime = 1 #in seconds

def updateTime(data):
	"""This is called every time a measurement update is published."""

	global timeOld
	now = rospy.get_rostime()
	timeOld = now.secs + float(now.nsecs)/10**9

def check():
	"""This is called at a much higher frequency than updateTime.  It updates the current time stamp, and if we notice that the quad has not updated its time for too long, we cut the motors."""

	global timeNew,timeOld

	rate = rospy.Rate(100) # 100hz
	pub = rospy.Publisher('fail_safe', Bool, queue_size=10)
	now = rospy.get_rostime()
	timeNew = now.secs + float(now.nsecs)/10**9

	a = Bool()

	if timeNew - timeOld >= failTime:
		#we've lost signal for too long
		a.data=True
		pub.publish(a)

	rate.sleep()

def main():
	"""Mainly initiates the ROS node, timer, and then indefinitely loops in check() function"""
	
	global timeOld,timeNew
	rospy.init_node('fail_safe', anonymous=True)

	now = rospy.get_rostime()

	timeNew = now.secs + float(now.nsecs)/10**9
	timeOld = timeNew

	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, updateTime)

	while not rospy.is_shutdown():
		check()
	# rospy.spin()

if __name__ == '__main__':
	main()