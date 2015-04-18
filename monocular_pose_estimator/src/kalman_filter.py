#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import numpy as np

currentPos = [0,0,0,0,0,0,0] #x,y,z,x,y,z,w

def poseUpdate(data):
	"""This function is called when the pose estimator publishes a coordinate update.  It will likely publish slower than we'd like in the context of controlling a quadcopter, which is why we create a separate publisher to estimate pose and publish more frequently."""

	pub = rospy.Publisher('filter_output', String, queue_size=10)
	# hello_str = "should be slow %s" % rospy.get_time()
	# rospy.loginfo(hello_str)
	# pub.publish(hello_str)
	currentPos[0] = data.pose.pose.position.x
	currentPos[1] = data.pose.pose.position.y
	currentPos[2] = data.pose.pose.position.z
	currentPos[3] = data.pose.pose.orientation.x
	currentPos[4] = data.pose.pose.orientation.y
	currentPos[5] = data.pose.pose.orientation.z
	currentPos[6] = data.pose.pose.orientation.w

def poseOutput():
	"""This function is called at a higher frequency than poseUpdate.  It implements a linear kalman filter to try and estimate the quad's pose when data is noisy, unavailable, or slow."""

	# data = getData(directory)
	# [truthData,measData] = parseData(data)

	#constants
	F = np.matrix([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]])  #constant vel
	H = np.matrix([[1,0,0,0],[0,1,0,0]])

	#initializers
	posEst = np.transpose(np.matrix([0,50,10,10]))
	cov = np.identity(4)  #just a guess
	q=.01
	Q = np.matrix([[q,0,0,0],[0,q,0,0],[0,0,q,0],[0,0,0,q]])
	r=100
	R = np.matrix([[r,0],[0,r]])

	#state prediction
	statePred = F*posEst
	statePredCov = F*cov*np.transpose(F)+Q

	#measurement prediction
	measPred = H*statePred
	measPredCov = H*statePredCov*np.transpose(H)+R	

	#associate the data together
	measurement = np.matrix(measData[i])
	measurement = np.transpose(measurement)
	innovation = measurement-measPred

	#update
	gain = statePredCov*np.transpose(H)*np.linalg.inv(measPredCov)
	posEst = statePred+gain*innovation
	cov = (np.identity(4)-gain*H)*statePredCov

	try:
		estimation = np.vstack((estimation,np.transpose(posEst)))
	except:
		estimation = np.transpose(posEst)

	plotData(truthData,measData,estimation,"Kalman Filter - Constant Velocity")

	return truthData,measData,estimation

	pub = rospy.Publisher('filter_test', String, queue_size=10)






	# hello_str = "should be fast %s" % rospy.get_time()
	# rospy.loginfo(hello_str)
	# pub.publish(hello_str)




def talker():
	rospy.init_node('filter_node', anonymous=True)
	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, poseUpdate)
	rate = rospy.Rate(100) # 10hz
	
	while not rospy.is_shutdown():
		poseOutput()
		rate.sleep()

if __name__ == '__main__':
	talker()
