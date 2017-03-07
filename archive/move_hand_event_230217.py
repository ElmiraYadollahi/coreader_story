	# -*- encoding: UTF-8 -*- 

'''Move hand from data recieved by ar_tags_location_detect'''

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
import re

global midP
midP = []
global path
path = []
global relatPath
relatPath = []
global poses
poses = []
global tag_pairs
tag_pairs = '[0, 0]'
global counter 
counter = 0

pairs_dict = {	'[0, 0]' : True,
				'[0, 1]' : False,
				'[2, 3]' : False,
				'[4, 5]' : False,
				'[6, 7]' : False,
				'[8, 9]' : False
			 }

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

def lineFunc(PA, PB, t):
	newP = Pose()
	newP.position.x = PA.position.x + (PB.position.x - PA.position.x)* t
	newP.position.y = PA.position.y + (PB.position.y - PA.position.y)* t
	newP.position.z = PA.position.z + (PB.position.z - PA.position.z)* t
	return newP

def addMatrix(ori, added):
	ori.append(added)
	return ori

def calculateWhereToPointAt(wordCount, point1, point2, effector, frame):

	global selectedStory
	global path
	global midP
	global relatPath
	global poses
	poses = []

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
		newP = lineFunc(PA, PB, const)
		midP = [newP.position.x, newP.position.y, newP.position.z, newP.orientation.x, newP.orientation.y, newP.orientation.z]
		path = addMatrix(path, midP)
	
	print "path"
	print path
	print len(path)

	maxSpeed = 0.1
	maxSpeedHead = 0.05


	if wordCount > 1:
		dy = (path[0][1] - path[1][1])/2
		print "dy"
		print dy
	
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
			poseM.append(pose[2] + ((8/3) * dy))
			#tracker.pointAt(effector, poseM, frame, maxSpeed)
			#time.sleep(1)
			poses = addMatrix(poses, poseM)	
			poseM = []
			poseM.append(pose[0])
			poseM.append(pose[1] - dy)
			poseM.append(pose[2] + 4 * dy)
			#tracker.pointAt(effector, poseM, frame, maxSpeed)
			poses = addMatrix(poses, poseM)
			#time.sleep(1)
			poseM = []
			poseM.append(pose[0])
			poseM.append(pose[1] - (3 * dy)/2)
			poseM.append(pose[2] + ((8/3) * dy))
			#tracker.pointAt(effector, poseM, frame, maxSpeed)
			poses = addMatrix(poses, poseM)
			#time.sleep(1)

		print len(poses)
		print len(poses[0])
	for i in range(len(poses)):
			print poses[i],
			print ''


	taggedStory = readTheTaggedStory(selectedStory, True)
	words = taggedStory.split() 
	print words
	toSay = taggedStory.encode("utf-8")
	
	#story.post.say(toSay)
	for i in range(len(poses)):
		
		tracker.lookAt(poses[i], frame, maxSpeed, useWholeBody)
		tracker.pointAt(effector, poses[i], frame, maxSpeedHead)

		if i == 0:
			story.post.say(toSay)
		if (i ) %4 == 0:
			print i
			time.sleep(0.2)
			#tracker.lookAt(poses[i], frame, maxSpeed, useWholeBody)

		#print i
		print poses[i]
	poses = []
	path = []
	print "poses"
	print poses

def getLocations(msg):
	global counter
	counter += 1
	print counter

	tag = "2, 3"
	tag = tag_pairs.replace('[', '').replace(']', '')
	wordCount = storySelection(tag)
	print wordCount

	if counter == 1:

		if(effector == "LArm"):
			motionProxy.openHand("LHand")
		else:
			motionProxy.openHand("RHand")

		point1 = msg.poses[0]
		point2 = msg.poses[1]
		P1 = []
		P1.append([point1.position.x])
		P1.append([point1.position.y])
		P1.append([point1.position.z])
		P1.append([1])
		P2 = []
		P2.append([point2.position.x])
		P2.append([point2.position.y])
		P2.append([point2.position.z])
		P2.append([1])
		#print "point1"
		#print P1
		#print "point2"
		#print P2
		points = []
		points.append(P1)
		points.append(P2)
		print "point"
		print points

		name  = 'CameraBottom'
		space  = 2
		useSensorValues  = True
		camBotTransform = motionProxy.getTransform(name, space, useSensorValues)

		tranAC = transformMatrix(camBotTransform)
			# sample Point
		oldPoint = np.matrix(points[0])
		#oldPoint = np.matrix([[-0.0795520328283], [0.113370332341], [0.476543570067], [1]])
		newPoint1 = np.dot(tranAC, oldPoint)
		print "newPoint1"
		print newPoint1
		print 

		oldPoint = np.matrix(points[1])
		#oldPoint = np.matrix([[0.126619950559], [0.140763824819], [0.479968283814], [1]])
		newPoint2 = np.dot(tranAC, oldPoint)
		print "newPoint2"
		print newPoint2
		print 

		space      = motion.FRAME_ROBOT

		#wordCount = 3


		if newPoint1[1] >= newPoint2[1]:
			calculateWhereToPointAt( wordCount, newPoint1, newPoint2, effector, space)
		else:
			calculateWhereToPointAt( wordCount, newPoint2, newPoint1, effector, space)

		time.sleep(2)
		postureProxy.goToPosture("Crouch", 0.5)
		pairs_dict[tag_pairs] = True
		print pairs_dict[tag_pairs]


def tagState(msg):
	global counter
	global tag_pairs
	global poses 
	if tag_pairs != msg.data:
		counter = 0
		tag_pairs = msg.data
		poses = []
	print tag_pairs
	#print tag_pairs

def sayFromFile(story, filename, encoding):
	"""

	"""
	#with codecs.open(filename, encoding=encoding) as fp:
	#contents = filename.read()
		# warning: print contents won't work
	toSay = filename.encode("utf-8")
	story.post.say(toSay)

def readTheTaggedStory(taggedStory, correctFlag):
	""" Read a story containing the tags and based on correctFlag change the tags approprietly

	"""
	global moveHand
	moveHand = 0		
	if correctFlag == True:

		tag = "=RTag"
		taggedStory = removeTheTag(tag, taggedStory)
		tag = "=WTag"
		taggedStory = removeTheWordWithTag(tag, taggedStory)

	elif correctFlag == False:
		tag = "=WTag"
		taggedStory = removeTheTag(tag, taggedStory)
		tag = "=RTag"
		taggedStory = removeTheWordWithTag(tag, taggedStory)
	
	#sayFromFile(story, taggedStory, 'ascii')
	return taggedStory


def storySelection(tag):
	""" Select a story from the text files of each story and return it

	"""
	global selectedStory
	"""with open('chick_story_en.txt') as f:
		lines_content = f.read().splitlines()
		#print lines_content

	for line in lines_content:
		found = re.search("2, 3", line)
		if not found == None:
			#print line
			#print line[:found.start()] + line[found.end():]
			print
			selectedStory = line[:found.start()] + line[found.end():]
			selectedStory = selectedStory.replace('[', '').replace(']', '')
			break"""

	with open('chick_story_en.txt') as f:
		lines_array = f.read().splitlines()

	#story_loaded = rospy.get_param("story_text_en")
	#lines_array = story_loaded.splitlines()

	for line in lines_array:
		found = re.search(tag, line)
		if not found == None:
			selectedStory = line[:found.start()] + line[found.end():]
			selectedStory = selectedStory.replace('[', '').replace(']', '')
			#print selectedStory
			break

	wordCount = getTheWordCount(selectedStory)
	tag = "=Num"
	selectedStory = removeTheWordWithTag(tag, selectedStory)
	return wordCount	
	

def getTheWordCount(storyContent):
			
	tag = "=Num"
	tagWithWord = "\w+(?=" + tag + ")"
	foundTag = re.search(tagWithWord + tag, storyContent)
	wordCountString = foundTag.group(0)
	wordCountString = removeTheTag(tag, wordCountString)
	wordCount = int(wordCountString)

	#print wordCount

	return wordCount



def removeTheTag(tag, storyContent):
	""" Find and remove the tag given to the function and leave the word connected to them intact

	"""
	while True:
		foundTag = re.search(tag, storyContent)
		if foundTag == None:
			break
		storyContent = storyContent[:foundTag.start(0)] + storyContent[foundTag.end(0):]

	return storyContent


def removeTheWordWithTag(tag, storyContent):
	""" Find and remove the tag given to the function and remove the tag and the word connected to it as well

	"""
	while True:
		tagWithWord = "\w+(?=" + tag + ")"
		foundTag = re.search(tagWithWord + tag, storyContent)
		if foundTag == None:
			break
		storyContent = storyContent[:foundTag.start(0)] + storyContent[foundTag.end(0):]

	return storyContent


def main(nao_ip):
	rospy.init_node("ar_tags_poses")
	global tag_pairs

	global story
	story = ALProxy("ALTextToSpeech", nao_ip, 9559)

	global conversration
	conversation = ALProxy("ALTextToSpeech", nao_ip, 9559)

	global motionProxy
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

	global effector
	effector = "RArm"

	global selectedStory
	
	story.setLanguage('English')
	# To create transforamtion Matrix


	rospy.Subscriber("tag_id_state", String, tagState)
	#if pairs_dict[tag_pairs] == False:
		#print "hello"
	rospy.Subscriber("target_pose", PoseArray, getLocations)
		#print points[0]
		
		#print "hey"

	tag = String(tag_pairs)
	
	#time.sleep(5)
	#postureProxy.goToPosture("Crouch", 0.5)
	rospy.spin()

if __name__ == "__main__":
    robotIp = 'nao.local'
    if len(sys.argv) <= 1:
        print "Usage python motion_cartesianArm1.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)