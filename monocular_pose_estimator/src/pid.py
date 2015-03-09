#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

def callback(data):
	# you can find more info about a particular topic by typing in 'rosmsg show geometry_msgs/PoseWithCovarianceStamped'
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.pose.position)
	
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