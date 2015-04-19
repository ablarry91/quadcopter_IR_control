#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float32,Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import numpy as np

currentPos = [0,0,0,0,0,0,0] #x,y,z,x,y,z,w

def poseUpdate(data):
	"""This function is called when the pose estimator publishes a coordinate update.  It will likely publish slower than we'd like in the context of controlling a quadcopter, which is why we create a separate publisher to estimate pose and publish more frequently."""

	# pub = rospy.Publisher('filter_output', String, queue_size=10)
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

def kalmanVel():
	rospy.init_node('filter_node', anonymous=True)
	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, poseUpdate)
	rate = rospy.Rate(100) # 10hz

	#initializers for the kalman filter, representing 7 parameters, first 7 being positions, next 7 being velocities
	# posEst = np.transpose(np.matrix([0,0,0,0,0,0,0,0,0,0,0,0,0,0]))
	posEst = np.zeros([14,1])
	cov = np.identity(14)  #just a guess
	q=.01
	Q = np.zeros([14,14])
	np.fill_diagonal(Q,q)
	# Q = np.matrix([[q,0,0,0],[0,q,0,0],[0,0,q,0],[0,0,0,q]])
	r=100
	R = np.zeros([7,7])
	np.fill_diagonal(R,r)
	# R = np.matrix([[r,0],[0,r]])

	#constants
	f = np.identity(7)
	F = np.identity(14)
	for i in range(len(f)):
		for j in range(len(f)):
			F[i+7,j+7] = f[i,j]
	h = np.identity(7)
	H = np.zeros([7,14])
	for i in range(len(h)):
		for j in range(len(h)):
			H[i,j] = h[i,j]
	
	#keep looping until all hell breaks loose
	while not rospy.is_shutdown():

		#state prediction
		statePred = F*posEst
		statePredCov = F*cov*np.transpose(F)+Q

		#measurement prediction
		measPred = np.dot(H,statePred)
		measPredCov = np.dot(H,statePredCov)
		measPredCov = np.dot(H,np.transpose(H)) + R	

		#associate the data together
		measurement = np.matrix([currentPos[0],currentPos[1],currentPos[2],currentPos[3],currentPos[4],currentPos[5],currentPos[6]])
		measurement = np.transpose(measurement)
		innovation = measurement-measPred

		#update
		gain = np.dot(statePredCov,np.transpose(H))
		gain = np.dot(gain,np.linalg.inv(measPredCov))
		posEst = statePred+gain*innovation
		cov = np.identity(14) - np.dot(gain,H)
		cov = np.dot(cov,statePredCov)

		#get your final result
		estimation = np.transpose(posEst)

		#set up the publisher
		pub = rospy.Publisher('filter_output', Pose, queue_size=10)
		# test = Float32()
		# test.data = estimation[0,0]

		#write to a ROS message array
		a = Pose()
		a.position.x = estimation[0,0]
		a.position.y = estimation[0,1]
		a.position.z = estimation[0,2]
		a.orientation.x = estimation[0,3]
		a.orientation.y = estimation[0,4]
		a.orientation.z = estimation[0,5]
		a.orientation.w = estimation[0,6]
		pub.publish(a)
		# hello_str = "should be fast %s" % rospy.get_time()
		# rospy.loginfo(hello_str)
		# pub.publish(hello_str)

		#wait at prescribed frequency
		rate.sleep()

def kalmanAccel():
	rospy.init_node('filter_node', anonymous=True)
	rospy.Subscriber("/monocular_pose_estimator/estimated_pose", PoseWithCovarianceStamped, poseUpdate)
	rate = rospy.Rate(100) # 10hz

	#initializers for the kalman filter
	posEst = np.transpose(np.matrix([0,50,10,10]))
	cov = np.identity(4)  #just a guess
	q=.01
	Q = np.matrix([[q,0,0,0],[0,q,0,0],[0,0,q,0],[0,0,0,q]])
	r=100
	R = np.matrix([[r,0],[0,r]])

	#constants
	F = np.matrix([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]])  #constant vel
	H = np.matrix([[1,0,0,0],[0,1,0,0]])
	
	#keep looping until all hell breaks loose
	while not rospy.is_shutdown():

		#state prediction
		statePred = F*posEst
		statePredCov = F*cov*np.transpose(F)+Q

		#measurement prediction
		measPred = H*statePred
		measPredCov = H*statePredCov*np.transpose(H)+R	

		#associate the data together
		measurement = np.matrix([currentPos[0],currentPos[1]])
		measurement = np.transpose(measurement)
		innovation = measurement-measPred

		#update
		gain = statePredCov*np.transpose(H)*np.linalg.inv(measPredCov)
		posEst = statePred+gain*innovation
		cov = (np.identity(4)-gain*H)*statePredCov

		#get your final result
		estimation = np.transpose(posEst)
		# print "estimation is", estimation

		pub = rospy.Publisher('filter_output', Float32, queue_size=10)
		test = Float32()
		test.data = estimation[0,0]
		# test.data = 5
		pub.publish(test)
		# hello_str = "should be fast %s" % rospy.get_time()
		# rospy.loginfo(hello_str)
		# pub.publish(hello_str)

		#wait at prescribed frequency
		rate.sleep()

if __name__ == '__main__':
	kalmanVel()
