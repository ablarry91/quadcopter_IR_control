#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

def filterNode(data):

	pub = rospy.Publisher('filter_output', String, queue_size=10)
	hello_str = "should be slow %s" % rospy.get_time()
	rospy.loginfo(hello_str)
	pub.publish(hello_str)
	# print "hi"

def talker():
	rospy.init_node('filter_node', anonymous=True)
	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, filterNode)
	# pub = rospy.Publisher('filter_output', String, queue_size=10)

	while not rospy.is_shutdown():
		pub = rospy.Publisher('filter_test', String, queue_size=10)
		hello_str = "should be fast %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate = rospy.Rate(100) # 10hz
		rate.sleep()

	# hello_str = "hi world %s" % rospy.get_time()
	# rospy.loginfo(hello_str)
	# pub.publish(hello_str)

if __name__ == '__main__':
	talker()
