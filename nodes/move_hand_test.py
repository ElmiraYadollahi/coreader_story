	# -*- encoding: UTF-8 -*- 

'''Cartesian control: Arm trajectory example'''

import sys
import motion
import almath
from naoqi import ALProxy
import rospy
import math
from std_msgs.msg import Int64, Int64MultiArray, String
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
from numpy.linalg import inv
import time

global midP
midP = []
global path
path = []
global relatPath
relatPath = []
global poses
poses = []


# Initialize the node
rospy.init_node('read_lines_event')

def StiffnessOn(proxy):
	#We use the "Body" name to signify the collection of all joints
	pNames = "Body"
	pStiffnessLists = 1.0
	pTimeLists = 1.0
	proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def lineFunc(PA, PB, t):
	newP = Pose()
	newP.position.x = PA.position.x + (PB.position.x - PA.position.x)* t
	newP.position.y = PA.position.y + (PB.position.y - PA.position.y)* t
	newP.position.z = PA.position.z + (PB.position.z - PA.position.z)* t
	print t
	print newP
	return newP

def addMatrix(ori, added):
	#orig = []
	#print "before"
	#print ori
	ori.append(added)
	#print "after"
	#print ori
	return ori
def calculateWhereToPointAt(wordCount, point1, point2, effector):

	global path
	global midP
	global relatPath
	global poses

	PA = Pose()
	PB = Pose()
	PA.position.x = point1.item(0, 0)
	PA.position.y = point1.item(1, 0)
	PA.position.z = point1.item(2, 0)

	PB.position.x = point2.item(0, 0)
	PB.position.y = point2.item(1, 0)
	PB.position.z = point2.item(2, 0)

	for i in range(wordCount):
		print wordCount
		print i
		const = (i + 1) / float (1 + wordCount) 
		#const /= 4
		print const
		newP = lineFunc(PA, PB, const)
		midP = [newP.position.x, newP.position.y, newP.position.z, newP.orientation.x, newP.orientation.y, newP.orientation.z]
		path = addMatrix(path, midP)
	print "path"
	print path
	print len(path)

	maxSpeed = 0.05
	#effector = "LArm"
	frame = motion.FRAME_ROBOT #FRAME ROBOT}
	if wordCount > 1:
		dy = (path[0][1] - path[1][1])/2
		print "dy"
		print dy
	

	#pose = [0, 0, 0]
	pose2 = [0, 0, 0]
	useWholeBody = False
	dx_corr = 0.1
	dz_corr = 0.1
	if effector == "LArm":
		dy_corr = -0.03
	else:
		dy_corr = 0.03

	for i in range(len(path)):
		pose = []
		pose.append(path[i][0] + dx_corr)
		pose.append(path[i][1] + dy_corr)
		pose.append(path[i][2] + dz_corr)
		#tracker.pointAt(effector, pose, frame, maxSpeed)
		#tracker.lookAt(pose, frame, maxSpeed, useWholeBody)
		poses = addMatrix(poses, pose)
		#poses.append(pose)
		print "pose"
		print pose
		#time.sleep(1)
		#pose[1] = path[i][1] - 0.03
		#pose2[0] = pose[0]
		#pose2[1] = pose[1]
		poseM = []
		if i != len(path)-1:
			poseM.append(pose[0])
			poseM.append(pose[1] - (dy/2))
			poseM.append(pose[2] + ((4/3) * dy))
			#tracker.pointAt(effector, poseM, frame, maxSpeed)
			#time.sleep(1)
			poses = addMatrix(poses, poseM)	
			poseM = []
			poseM.append(pose[0])
			poseM.append(pose[1] - dy)
			poseM.append(pose[2] + 2 * dy)
			#tracker.pointAt(effector, poseM, frame, maxSpeed)
			poses = addMatrix(poses, poseM)
			#time.sleep(1)
			poseM = []
			poseM.append(pose[0])
			poseM.append(pose[1] - (3 * dy)/2)
			poseM.append(pose[2] + ((4/3) * dy))
			#tracker.pointAt(effector, poseM, frame, maxSpeed)
			poses = addMatrix(poses, poseM)
			#time.sleep(1)
		#poses = addMatrix(poses, pose2)	
		#poses.append(pose2)
		print len(poses)
		print len(poses[0])
	for i in range(len(poses)):
			print poses[i],
			print ''
			
		#print "pose m"
		#print pose2
	#print poses
	for i in range(len(poses)):

		tracker.pointAt(effector, poses[i], frame, maxSpeed)	
		#print "pose"
		#print poses[i]
		if i%4 == 0:
			time.sleep(1)
			tracker.pointAt(effector, poses[i], frame, maxSpeed)
	#pose = []
	#poses = []

	#pos = [0.69854686, 0.12905443, -0.02585498]

	wordMin = wordCount - 1
	for i in range(wordMin):
		print i
		dy = (path[i][1] - path[i+1][1])/float(2)
		dz = float(dy)/2
		dx = 0.005 + 0.002 * (2*i)
		midP = [-dx, -dy, dz]
		relatPath = addMatrix(relatPath, midP)
		dx = 0.005 + 0.002 * (2*i + 1)
		midP = [-dx, -dy, -dz]
		relatPath = addMatrix(relatPath, midP)
		#print relatPath
	print "Path List"
	print relatPath 

	return relatPath


def calculateEachWordLocaion(wordCount, point1, point2, roll):
	
	PA = Pose()
	PB = Pose()

	"""PA.position.x = msg.poses[0].position.x
	PA.position.y = msg.poses[0].position.y
	PA.position.z = msg.poses[0].position.z
	PA.orientation.x = msg.poses[0].orientation.x
	PA.orientation.y = msg.poses[0].orientation.y
	PA.orientation.z = msg.poses[0].orientation.z
	PA.orientation.w = msg.poses[0].orientation.w

	PB.position.x = msg.poses[1].position.x
	PB.position.y = msg.poses[1].position.y
	PB.position.z = msg.poses[1].position.z
	PB.orientation.x = msg.poses[1].orientation.x
	PB.orientation.y = msg.poses[1].orientation.y
	PB.orientation.z = msg.poses[1].orientation.z
	PB.orientation.w = msg.poses[1].orientation.w"""

	PA.position.x = point1.item(0, 0)
	PA.position.y = point1.item(1, 0)
	PA.position.z = point1.item(2, 0)

	PB.position.x = point2.item(0, 0)
	PB.position.y = point2.item(1, 0)
	PB.position.z = point2.item(2, 0)

	global path
	global midP
	global relatPath

	for i in range(wordCount):
		print wordCount
		print i
		const = (i + 1) / float (1 + wordCount) 
		#const /= 4
		print const
		newP = lineFunc(PA, PB, const)
		midP = [newP.position.x, newP.position.y, newP.position.z, newP.orientation.x, newP.orientation.y, newP.orientation.z]
		path = addMatrix(path, midP)
	#print path 

	midP = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	#relatPath = addMatrix(relatPath, midP)

	wordMin = wordCount - 1
	for i in range(wordMin):
		print i
		dy = (path[i][1] - path[i+1][1])/float(2)
		dz = float(dy)/2
		dx = 0.005 + 0.002 * (2*i)
		midP = [-dx, -dy, dz, 0.1, 0.0, 0.0]
		relatPath = addMatrix(relatPath, midP)
		dx = 0.005 + 0.002 * (2*i + 1)
		midP = [-dx, -dy, -dz, 0.1, 0.0, 0.0]
		relatPath = addMatrix(relatPath, midP)
		#print relatPath
	print "Path List"
	print relatPath 

	return relatPath

def transformMatrix(camBotTransform):
	""" A is the coordinate system recieved from TF
		B is the coordinate system only rotated
		C is the coordinate system rotate dand moved to FRAME_BODY
	"""
	matAB = [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1]
	#matAB = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
	invAB = np.matrix(np.reshape(matAB, (4, 4)))

	matBC = camBotTransform
	matBC = np.matrix(np.reshape(camBotTransform, (4, 4)))
	invBC = inv(np.matrix(matBC))
	
	matAC = np.dot(matBC, invAB)

	return matAC

def firstPosition():

	names = list()
	times = list()
	keys = list()

	names.append("LElbowRoll")
	times.append([0.36, 0.68, 1, 1.28, 2.04])
	keys.append([-0.684122, -1.40664, -0.68719, -1.38516, -0.381923])

	names.append("LElbowYaw")
	times.append([0.36, 0.68, 1, 1.28, 2.04])
	keys.append([-0.883625, -1.03856, -0.888228, -1.02475, -1.13213])

	names.append("LHand")
	times.append([0.68, 1.28, 2.04])
	keys.append([0.110572, 0.110935, 0.112026])

	names.append("LShoulderPitch")
	times.append([0.36, 0.68, 1, 1.28, 2.04])
	keys.append([1.4818, 1.4051, 1.48027, 1.42965, 1.4864])

	names.append("LShoulderRoll")
	times.append([0.36, 0.68, 1, 1.28, 2.04])
	keys.append([0.62583, 0.14262, 0.593616, 0.162562, 0.085862])

	names.append("LWristYaw")
	times.append([0.68, 1.28, 2.04])
	keys.append([-0.112024, -0.112024, -0.128898])



def main(robotIp):
	''' Example showing a path of two positions
	Warning: Needs a PoseInit before executing
	'''

	nao_ip = "10.0.0.13"

	global story
	story = ALProxy("ALTextToSpeech", nao_ip, 9559)

	global conversration
	conversation = ALProxy("ALTextToSpeech", nao_ip, 9559)

	global reaction
	motionProxy = ALProxy("ALMotion", nao_ip, 9559)

	global postureProxy
	postureProxy = ALProxy("ALRobotPosture", nao_ip, 9559)

	global audioreaction
	audioreaction = ALProxy("ALAudioPlayer", nao_ip, 9559)

	global tracker
	tracker = ALProxy("ALTracker", nao_ip, 9559)

	global proxy
	proxy = ALProxy("ALLeds", nao_ip, 9559)

	motionProxy.setExternalCollisionProtectionEnabled("All", True)
	postureProxy.goToPosture("Crouch", 0.5)

	a = np.array([[1., 2.], [3., 4.]])

	effector = "LArm"

	if(effector == "LArm"):
		motionProxy.openHand("LHand")
		roll = -1.7 #rotate wrist to the left (about the x axis, w.r.t. robot frame)
	else:
		motionProxy.openHand("RHand")
		roll = 1.7 #rotate wrist to the right (about the x axis, w.r.t. robot frame)


	name            = "CameraBottom"
	space           = motion.FRAME_ROBOT
	useSensorValues = True
	result          = motionProxy.getPosition(name, space, useSensorValues) 
	print result
	print 

	name            = effector
	space           = motion.FRAME_ROBOT
	useSensorValues = True
	result          = motionProxy.getPosition(name, space, useSensorValues)
	print "hand"
	print result
	print


	name  = 'CameraBottom'
	space  = 2
	useSensorValues  = True
	camBotTransform = motionProxy.getTransform(name, space, useSensorValues)
	for i in range(0, 4):
		for j in range(0, 4):
			print camBotTransform[4*i + j],
	print ''
	print camBotTransform
	print
	camBotTransform = np.matrix(np.reshape(camBotTransform, (4, 4)))
	print camBotTransform

	ainv = inv(np.matrix(camBotTransform))
	print ainv

	trans1 = [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1]
	INV1 = np.matrix(np.reshape(trans1, (4, 4)))
	print INV1
	print 

	mult = np.dot(ainv, INV1)
	print mult 
	print

	tranAC = transformMatrix(camBotTransform)

	oldPoint = np.matrix([[-0.117167759435], [0.180355769749], [0.406543636401], [1]])
	oldPoint = np.matrix([[0.128377211506], [0.130287323539], [0.44565108369], [1]])
	oldPoint = np.matrix([[0.100028354724], [0.117019713083], [0.484881340195], [1]])
	oldPoint = np.matrix([[-0.0795520328283], [0.113370332341], [0.476543570067], [1]])
	newPoint1 = np.dot(tranAC, oldPoint)
	print "newPoint1"
	print newPoint1
	print 

	oldPoint = np.matrix([[0.0910058372973], [0.180042399881], [0.407498793539], [1]])
	oldPoint = np.matrix([[-0.0758025405705], [0.0944930856717], [0.458260370057], [1]])
	oldPoint = np.matrix([[-0.105395166379], [0.0958002714073], [0.471818255403], [1]])
	
	oldPoint = np.matrix([[0.126619950559], [0.140763824819], [0.479968283814], [1]])
	newPoint2 = np.dot(tranAC, oldPoint)
	print "newPoint2"
	print newPoint2
	print 

	#INV1 = np.matrix()
    # Set NAO in Stiffness On
	#StiffnessOn(motionProxy)

    # Send NAO to Pose Init
	
    # Example showing how to close the right hand.
	#handName  = 'RHand'
	#motionProxy.openHand(handName)

	#rospy.Subscriber('target_pose', PoseArray, calculateEachWordLocaion)
	#pathList = calculateEachWordLocaion(newPoint1, newPoint2)

	#effector   = "LArm"
	space      = motion.FRAME_ROBOT
	axisMask   = almath.AXIS_MASK_VEL    # just control position
	axisMaskList  = almath.AXIS_MASK_X+almath.AXIS_MASK_Y+almath.AXIS_MASK_Z+almath.AXIS_MASK_WX   # just control position
	axisMaskList  = almath.AXIS_MASK_X+almath.AXIS_MASK_Y+almath.AXIS_MASK_Z   # just control position
	isAbsolute = True

    # Since we are in relative, the current position is zero
	currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	dx         =  0.09      # translation axis X (meters)
	dy         =  0.09      # translation axis Y (meters)
	dz         =  0.20      # translation axis Z (meters)
	dwx        =  0.00      # rotation axis X (radians)
	dwy        =  0.00      # rotation axis Y (radians)
	dwz        =  0.30      # rotation axis Z (radians)
	targetPos  = [dx, dy, dz, dwx, dwy, dwz]

	dx         =  0.20      # translation axis X (meters)
	dy         =  0.066      # translation axis Y (meters)
	dz         =  0.20      # translation axis Z (meters)
	dwx        =  0.22      # rotation axis X (radians)
	dwy        =  0.014      # rotation axis Y (radians)
	dwz        =  0.20      # rotation axis Z (radians)
	dvx        =  0.22      # rotation axis X (radians)
	dvy        =  -0.03      # rotation axis Y (radians)
	dvz        =  0.20      # rotation axis Z (radians)
	targetPos  = [dx, dy, dz, roll, 0.0, 0.0]
	time_s = [2.0]
	#axisMask  = almath.AXIS_MASK_ALL  # just control position

	#targetPos = [0.18758109211921692, 0.1094910278916359, 0.1940743625164032, 0.3532320559024811, 0.6077926754951477, -0.04876329377293587]
	#motionProxy.positionInterpolation(effector, space, targetPos, axisMaskList, time, isAbsolute)

	#axisMaskList  = almath.AXIS_MASK_ALL   # just control position
	isAbsolute = False

	pathList	= 	[[dx, dy, dz, 0.0, 0.0, 0.0], # point 1
					[dwx, dwy, dwz, 0.0, 0.0, 0.0], # point 2
					[dvx, dvy, dvz, 0.0, 0.0, 0.0], # point 3
					[dwy, dwy, dz, 0.0, 0.0, 0.0] # point 4
					]

	
	pathList =	[[-0.005, -0.02080870790232492, 0.01040435395116246, 0.1, 0.0, 0.0],
	 			 [-0.007, -0.02080870790232492, -0.01040435395116246, 0.1, 0.0, 0.0], 
	 			 [-0.009, -0.02080870790232492, 0.01040435395116246, 0.1, 0.0, 0.0], 
	 			 [-0.011, -0.02080870790232492, -0.01040435395116246, 0.1, 0.0, 0.0], 
				 [-0.013, -0.02080870790232492, 0.01040435395116246, 0.1, 0.0, 0.0], 
	 			 [-0.015, -0.02080870790232492, -0.01040435395116246, 0.1, 0.0, 0.0]]


	wordCount = 3


	if newPoint1[1] >= newPoint2[1]:
		pathList = calculateWhereToPointAt( wordCount, newPoint1, newPoint2, effector)
	else:
		pathList = calculateWhereToPointAt( wordCount, newPoint2, newPoint1, effector)
	#pathList = calculateEachWordLocaion( wordCount, newPoint1, newPoint2, roll)
	

    # Go to the target and back again
	#pathList       = [targetPos, currentPos]
	timeLists      = [[2.0], [4.0], [6.0], [8.0], [10.0], [12.0]] # seconds
	timeList      = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0] # seconds

	timeListss      = [timeList] # seconds
	targetPos  = [0.0, -dy, dwy, 0.0, 0.0, 0.0]
	targetPos  = [-0.005, -0.020808086793345053, 0.010404043396672526, 0.0, 0.0, 0.0]
	#motionProxy.positionInterpolation(effector, space, pathList, axisMaskList, timeList, isAbsolute)
	effectors = [effector]
	pathLists = [pathList]
	axisMaskLists = [axisMaskList]
	#motionProxy.positionInterpolation(effector, space, pathList, axisMaskList, timeList, isAbsolute)
	#motionProxy.positionInterpolations(effectors, space, pathLists, axisMaskLists, timeListss, isAbsolute)

	chainName        = effector
	space            = 2
	position         = [0.18758109211921692, 0.1094910278916359, 0.1940743625164032,  0.1532320559024811, 0.6077926754951477, -0.04876329377293587] # Absolute Position
	fractionMaxSpeed = 0.2
	axisMask         = 63

	#motionProxy.setPosition(chainName, space, position, fractionMaxSpeed, axisMask)

	"""position         = [0.18058109211921692, 0.0894910278916359, 0.1840743625164032,  0.4532320559024811, 0.6077926754951477, -0.04876329377293587] # Absolute Position
	motionProxy.setPosition(chainName, space, position, fractionMaxSpeed, axisMask)

	position         = [0.17358109211921692, 0.0694910278916359, 0.1940743625164032,  0.4532320559024811, 0.6077926754951477, -0.04876329377293587] # Absolute Position
	motionProxy.setPosition(chainName, space, position, fractionMaxSpeed, axisMask)

	position         = [0.16458109211921692, 0.0494910278916359, 0.1840743625164032,  0.4532320559024811, 0.6077926754951477, -0.04876329377293587] # Absolute Position
	motionProxy.setPosition(chainName, space, position, fractionMaxSpeed, axisMask)"""

	"""motionProxy.positionInterpolation(effector, space, targetPos, axisMaskList, time, isAbsolute)

	targetPos  = [-0.007, -0.020808086793345053, -0.010404043396672526, 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation(effector, space, targetPos, axisMaskList, time, isAbsolute)

	targetPos1  = [-0.009, -0.020808086793345046, 0.010404043396672523, 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation(effector, space, targetPos1, axisMaskList, time, isAbsolute)
	
	pathList       = [targetPos, targetPos1]
	times 	= [2.0, 4.0]
	targetPos  = [-0.011, -0.020808086793345046, -0.010404043396672523, 0.0, 0.0, 0.0]
	#motionProxy.positionInterpolation(effector, space, pathList, axisMaskList, times, isAbsolute)
	
	motionProxy.positionInterpolation(effector, space, targetPos, axisMaskList, time, isAbsolute)

	targetPos  = [-0.013, -0.020808086793345046, 0.010404043396672523, 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation(effector, space, targetPos, axisMaskList, time, isAbsolute)

	targetPos  = [-0.015, -0.020808086793345046, -0.010404043396672523, 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation(effector, space, targetPos, axisMaskList, time, isAbsolute)"""

	#postureProxy.goToPosture("Crouch", 0.5)

	pos = [0.69854686, 0.12905443, -0.02585498]


	"""maxSpeed = 0.05
	frame = 2 #FRAME ROBOT}
	tracker.pointAt(effector, pos, frame, maxSpeed)
	#sleep(2)

	pos = [0.63258937, -0.00441637, 0.607334]
	#time.sleep(0.2)

	maxSpeed = 0.05
	frame = 2 #FRAME ROBOT}
	tracker.pointAt(effector, pos, frame, maxSpeed)


	pos = [0.63258937, -0.12441637, -0.00197334]
	time.sleep(0.2)

	maxSpeed = 0.05
	frame = 2 #FRAME ROBOT}
	tracker.pointAt(effector, pos, frame, maxSpeed)

	pos = [0.63258937, -0.24441637, 0.207334]
	#time.sleep(0.2)

	maxSpeed = 0.05
	frame = 2 #FRAME ROBOT}
	tracker.pointAt(effector, pos, frame, maxSpeed)


	pos = [0.63258937, -0.36441637, -0.00197334]
	time.sleep(0.2)

	maxSpeed = 0.05
	frame = 2 #FRAME ROBOT}
	tracker.pointAt(effector, pos, frame, maxSpeed)

	pos = [0.63258937, -0.48441637, 0.207334]
	time.sleep(0.2)

	maxSpeed = 0.05
	frame = 2 #FRAME ROBOT}
	tracker.pointAt(effector, pos, frame, maxSpeed)

	pos = [0.63258937, -0.60441637, -0.00197334]
	time.sleep(0.2)

	maxSpeed = 0.05
	frame = 2 #FRAME ROBOT}
	tracker.pointAt(effector, pos, frame, maxSpeed)"""
	time.sleep(5)
	postureProxy.goToPosture("Crouch", 0.5)

if __name__ == "__main__":
    robotIp = 'nao.local'
    if len(sys.argv) <= 1:
        print "Usage python motion_cartesianArm1.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
	

