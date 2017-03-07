#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import animations.embarassed_seated_pose
import animations.scratchHead_seated_pose
import animations.IdontKnow_seated_pose
import animations.IdontKnow2_seated_pose
import animations.hesitation_seated_pose
import animations.hesitation2_seated_pose
import animations.hesitation3_seated_pose
import animations.thinking5_seated_pose
import animations.thinking6_seated_pose
import animations.thinking7_seated_pose
import animations.thinking8_seated_pose
import animations.monster_pose
import animations.disappointed_seated_pose
import animations.excited_seated_pose
import animations.excited2_seated_pose
import animations.happy_seated_pose
import animations.happy2_seated_pose
import animations.happy3_seated_pose
import animations.introduction_pose
import animations.proud_seated_pose
import animations.nod_pose
import animations.relieved_seated_pose
import animations.winner_seated_pose
import animations.winner2_seated_pose
import animations.pensive_seated_pose

from naoqi import ALProxy 
import codecs
import time
import re
import random

# Debug
import rospy
from memory.msg import Animation

import sys
import motion
import almath
import math
from std_msgs.msg import Int64, Int64MultiArray, String
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
from numpy.linalg import inv

""" It is a dictionary for the states of the tag:
	False: 						means the story have not be read yet, it will go into reading with wrong tag 
	"Waiting for Red-card":		means the robot is waiting to recieve a green card
	"Waiting for Yellow-card":	robot repeats reading the text
	"Waiting for Green-card":	robot reacts happily
	"correcting Mode": 			means the story have been read and the robot have recieved a red card
	"Corrected Mode":			
	True:						means the story have been read and the robot have recieved a green card

"""
pairs_dict = {	'[0, 1]' : False,
				'[2, 3]' : False,
				'[4, 5]' : False,
				'[6, 7]' : False,
				'[8, 9]' : False,
				'[10, 11]' : False,
				'[12, 13]' : False
			 }


story_dict ={	'[200, 201]': 1, 
				'[210, 211]': 2,
				'[220, 221]': 3,
				'[230, 231]': 4,
				'[240, 241]': 5,
				'[2, 3]': 7
			}

card_id = { 39, 40}

motionProxy_states = { "low": [1, 2, 3, 4, 5],
				  	"medium": [6, 7, 8, 9, 10],
				  	"high": [11, 12, 13, 14, 15] }


mistake_level_states = { "low": [1],
						 "medium": [3],
						 "high": [5]}

story_selection_state = {}

""" Recieve the key stroke from key publisher node
	[1]: tell the robot made a mistake in reading the story
	[2]: tell the robot to choose a new story and repeat the process
	[3]: tell the robot to stop the tracker and go to resting mode

"""
KEY_MAPPING = { 'f': [2], 
				'g': [2], 
				'h': [2],
				'b': [3],
				'n': [3],
				'm': [3],
				'e': [4],
				'r': [4],
				't': [4],
				's': [1]  
				}

global counter
counter = 0

global tag_stream_buffer
tag_stream_buffer = []

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
global moveHand 
moveHand = 0
global selectedStory
global mainPoints
mainPoints = []
global trajComplete
trajComplete = []


class TRANSFORMATION:
	""" MATRIC TRANSFORMATION """

	def __init__(self):
		
		self.newP = Pose()
		self.matAB = [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1]
		self.space  = motion.FRAME_ROBOT
		self.useSensorValues  = True

	def transformMatrix(self, name):
		""" A is the coordinate system recieved from TF
			B is the coordinate system only rotated
			C is the coordinate system rotate dand moved to FRAME_BODY
		"""
		
		
		camBotTransform = motionProxy.getTransform(name, self.space, self.useSensorValues)

		invAB = np.matrix(np.reshape(self.matAB, (4, 4)))

		matBC = np.matrix(np.reshape(camBotTransform, (4, 4)))
		invBC = inv(np.matrix(matBC))
		
		matAC = np.dot(matBC, invAB)

		return matAC


	def lineFunc(self, PA, PB, t):
		""" line function created using the position of tags
			PA: point A
			PB: point B
			t: variable to find points on the line function

		"""

		self.newP.position.x = PA.position.x + (PB.position.x - PA.position.x)* t
		self.newP.position.y = PA.position.y + (PB.position.y - PA.position.y)* t
		self.newP.position.z = PA.position.z + (PB.position.z - PA.position.z)* t
		
		return self.newP


	def calculateTrajectory(self, mainPoints, wordCount):
		""" Calculate the main trajectory for the robot to point at

		"""
			

		""" Calculate the correction values to be added to the original points for increasing the pointing accuracy"""
		dx_corr = 0.1
		dz_corr = 0.1

		if wordCount > 1:
			# The distance between the main points
			dy = (mainPoints[0][1] - mainPoints[1][1])/2

		if effector == "LArm":
			dy_corr = -0.03
		else:
			dy_corr = 0.03


		for i in range(len(mainPoints)):
			trajMainP = []
			trajMainP.append(mainPoints[i][0] + dx_corr)
			trajMainP.append(mainPoints[i][1] + dy_corr)
			trajMainP.append(mainPoints[i][2] + dz_corr)

			trajComplete.append(trajMainP)
			
			if i != len(mainPoints)-1:

				trajMidP = []
				trajMidP.append(trajMainP[0])
				trajMidP.append(trajMainP[1] - (dy/2))
				trajMidP.append(trajMainP[2] + ((8/3) * dy))

				trajComplete.append(trajMidP)

				trajMidP = []
				trajMidP.append(trajMainP[0])
				trajMidP.append(trajMainP[1] - dy)
				trajMidP.append(trajMainP[2] + 4 * dy)

				trajComplete.append(trajMidP)

				trajMidP = []
				trajMidP.append(trajMainP[0])
				trajMidP.append(trajMainP[1] - (3 * dy)/2)
				trajMidP.append(trajMainP[2] + ((8/3) * dy))

				trajComplete.append(trajMidP)	

		for i in range(len(trajComplete)):
				print trajComplete[i],
				print ''

		return trajComplete






class WORDPROCESSING:
	""" Processing the text and read """

	def __init__(self):

		self.tag = "=Num"

	def storySelection(self, tag):
		""" Select a story from the text files of each story and return it

		"""
		global selectedStory

		
		""" creates a array containing the line related to detected tags """
		with open('chick_story_en.txt') as f:
			lines_array = f.read().splitlines()

		for line in lines_array:
			found = re.search(tag, line)
			if found != None:
				selectedStory = line[:found.start()] + line[found.end():]
				selectedStory = selectedStory.replace('[', '').replace(']', '')
				break

		#print selectedStory
		wordCount = self.getTheWordCount(selectedStory)

		return wordCount	

	def getTheWordCount(self, storyContent):
		""" Reads the number of words given in the text and then remove the tag and number and returns wordCount

		"""		
		global selectedStory

		tag = "=Num"
		tagWithWord = "\w+(?=" + tag + ")"
		foundTag = re.search(tagWithWord + tag, storyContent)

		wordCountString = foundTag.group(0)
		wordCountString = self.removeTheTag(tag, wordCountString)

		wordCount = int(wordCountString)
		selectedStory = self.removeTheWordWithTag(tag, selectedStory)

		return wordCount


	def removeTheTag(self, tag, storyContent):
		""" Find and remove the tag given to the function and leave the word connected to them intact

		"""
		while True:
			foundTag = re.search(tag, storyContent)
			if foundTag == None:
				break
			storyContent = storyContent[:foundTag.start(0)] + storyContent[foundTag.end(0):]

		return storyContent


	def removeTheWordWithTag(self, tag, storyContent):
		""" Find and remove the tag given to the function and remove the tag and the word connected to it as well

		"""
		while True:
			tagWithWord = "\w+(?=" + tag + ")"
			foundTag = re.search(tagWithWord + tag, storyContent)
			if foundTag == None:
				break
			storyContent = storyContent[:foundTag.start(0)] + storyContent[foundTag.end(0):]

		return storyContent

	def readTheTaggedStory(self, taggedStory, correctFlag):
		""" Read a story containing the tags and based on correctFlag change the tags approprietly

		"""
		#global moveHand
		#moveHand = 0	
		if correctFlag == True:
			tag = "=RTag"
			taggedStory = self.removeTheTag(tag, taggedStory)
			tag = "=WTag"
			taggedStory = self.removeTheWordWithTag(tag, taggedStory)

		elif correctFlag == False:
			tag = "=WTag"
			taggedStory = self.removeTheTag(tag, taggedStory)
			tag = "=RTag"
			taggedStory = self.removeTheWordWithTag(tag, taggedStory)
		
		self.sayFromFile(story, taggedStory, 'ascii')


	def sayFromFile(self, story, filename, encoding):
		"""

		"""

		toSay = filename.encode("utf-8")
		story.post.say(toSay)







class EMOTIONS:
	""" Processing the text and read """

	def __init__(self):
		a = 1

	def emotion_anger(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed2", "FaceLed3" ]
		section2 = ["FaceLed4", "FaceLed5", "FaceLed6", "FaceLed7", "ChestLeds", "FeetLeds", "EarLeds"]
		proxy.createGroup("half_up", section1)
		proxy.createGroup("half_down", section2)

		# Initiate the emotion
		proxy.fadeRGB("half_up", 0, 0, 0, 0.1)
		proxy.fadeRGB("half_down", 0x00FA0000, 0.1)
		proxy.fadeRGB("half_down", 0x00640000, 1)
		proxy.fadeRGB("half_down", 0x00FA0000, 1)

		# Turn off the emotion
		proxy.fadeRGB("FaceLeds", 0x00000000, 1)
		proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_surprise(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed2", "FaceLed3", "FaceLed4", "FaceLed5", "FaceLed6", "FaceLed7", "ChestLeds", "FeetLeds"]
		proxy.createGroup("whole", section1)

		# Initiate the emotion
		proxy.fadeRGB("whole", 0x00323200, 0.2)
		proxy.fadeRGB("whole", 0x00000000, 0.1)
		proxy.fadeRGB("whole", 0x003C3C00, 0.1)
		proxy.fadeRGB("whole", 0x00FAFA00, 0.1)
		proxy.fadeRGB("whole", 0x00FAFA00, 1)

		# turn off the emotion 
		proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_disgust(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed4", "FaceLed5", "ChestLeds", "FeetLeds"]
		section2 = ["FaceLed2", "FaceLed3", "FaceLed6", "FaceLed7"]
		proxy.createGroup("Updisgust", section1)
		proxy.createGroup("Downdisgust", section2)

		# Initiate the emotion
		proxy.fadeRGB("Downdisgust", 0x00000000, 0.1)
		proxy.fadeRGB("Updisgust", 0x0000FA00, 0.1)
		proxy.fadeRGB("Updisgust", 0x00004B00, 0.5)
		proxy.fadeRGB("Updisgust", 0x0000FA00, 0.1)
		proxy.fadeRGB("Updisgust", 0x00004B00, 0.5)
		
		# turn off the emotion 
		proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_sadness(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed2", "FaceLed3", "FaceLed6", "FaceLed7", "FeetLeds"]
		section2 = [ "FaceLed4", "FaceLed5", "ChestLeds"]
		proxy.createGroup("dark", section1)
		proxy.createGroup("sadness", section2)

		# Initiate the emotion
		proxy.fadeRGB("dark", 0x00000000, 0)
		proxy.fadeRGB("sadness", 0x00000000, 0)
		proxy.fadeRGB("sadness", 0x000000FF, 5.0)
		proxy.fadeRGB("sadness", 0x00000000, 2.0)

		# turn off the emotion 
		proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_happiness(self, dur=1):

		namesup = ["FaceLed0", "FaceLed1", "ChestLeds"]
		namesright = [ "FaceLed2", "FaceLed3", "ChestLeds"]
		namesdown = [ "FaceLed4", "FaceLed5", "ChestLeds"]
		namesleft = [ "FaceLed6", "FaceLed7", "ChestLeds"]

		proxy.createGroup("happyup",namesup)
		proxy.createGroup("happydown",namesdown)
		proxy.createGroup("happyright",namesright)
		proxy.createGroup("happyleft",namesleft)

		# Initiate the emotion
		i = 0
		while i <= 7:
			proxy.fadeRGB("happyup", 0x00FF00FF, 0)
			proxy.fadeRGB("happyright", 0x00969600, 0)
			proxy.fadeRGB("happydown", 0x00FF8C00, 0)
			proxy.fadeRGB("happyleft", 0x00FF0000, 0)
			time.sleep(0.21)
			proxy.fadeRGB("happyright", 0x00FF00FF, 0)
			proxy.fadeRGB("happydown", 0x00969600, 0)
			proxy.fadeRGB("happyleft", 0x00FF8C00, 0)
			proxy.fadeRGB("happyup", 0x00FF0000, 0)
			time.sleep(0.21)
			proxy.fadeRGB("happydown", 0x00FF00FF, 0)
			proxy.fadeRGB("happyleft", 0x00969600, 0)
			proxy.fadeRGB("happyup", 0x00FF8C00, 0)
			proxy.fadeRGB("happyright", 0x00FF0000, 0)
			time.sleep(0.21)
			proxy.fadeRGB("happyleft", 0x00FF00FF, 0)
			proxy.fadeRGB("happyup", 0x00969600, 0)
			proxy.fadeRGB("happyright", 0x00FF8C00, 0)
			proxy.fadeRGB("happydown", 0x00FF0000, 0)
			i = i + 1

		# turn off the emotion 
		proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_fear(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed2", "FaceLed3", "FeetLeds"]
		section2 = [ "FaceLed4", "FaceLed5", "FaceLed6", "FaceLed7", "ChestLeds"]
		proxy.createGroup("dark",section1)
		proxy.createGroup("fear",section2)

		# Initiate the emotion
		proxy.fadeRGB("dark", 0x00000000, 0)
		proxy.fadeRGB("fear", 0x00000046, 0)
		proxy.fadeRGB("fear", 0x00000064, 0.5)
		proxy.fadeRGB("fear", 0x00000046, 0.3)

		# turn off the emotion 
		proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)

	def turn_on_eye(self):
		section1 = ["FaceLeds", "ChestLeds" ]
		proxy.createGroup("turn",section1)
		proxy.fadeRGB("turn", 0x00FFFFFF, 0.3)


	def set_emotion(self, emotional_state):

		if emotional_state == "anger":
			self.emotion_anger()

		if emotional_state == "surprise":
			self.emotion_surprise()	

		if emotional_state == "disgust":
			self.emotion_disgust()

		if emotional_state == "sad":
			self.emotion_sadness()

		if emotional_state == "happy":
			self.emotion_happiness()

		if emotional_state == "fear":
			self.emotion_fear()


def getTagLocations(msg, cameraName):
	
	global moveHand
	moveHand += 1
	global wordCount

	changeCoordinate = TRANSFORMATION()
	wordProc = WORDPROCESSING()

	tag = tag_pairs
	tag = tag.replace('[', '').replace(']', '')
	#wordCount = wordProc.storySelection(tag)

	if moveHand == 1:

		if(effector == "LArm"):
			motionProxy.openHand("LHand")
		else:
			motionProxy.openHand("RHand")

		P1 = []
		P1.append([msg.poses[0].position.x])
		P1.append([msg.poses[0].position.y])
		P1.append([msg.poses[0].position.z])
		P1.append([1])
		P2 = []
		P2.append([msg.poses[1].position.x])
		P2.append([msg.poses[1].position.y])
		P2.append([msg.poses[1].position.z])
		P2.append([1])

		pointsAB = []
		pointsAB.append(P1)
		pointsAB.append(P2)

		""" Transformation Matrix"""
		tranAC = changeCoordinate.transformMatrix(cameraName)

		oldPoint = np.matrix(pointsAB[0])
		newPoint1 = np.dot(tranAC, oldPoint) 

		oldPoint = np.matrix(pointsAB[1])
		newPoint2 = np.dot(tranAC, oldPoint)

		""" to make sure the robot's hand move from left to write
			start from the tag located in the left toward the one in the right
			compare their y value
		"""
		if newPoint1[1] >= newPoint2[1]:
			pointsAB[0] = newPoint1
			pointsAB[1] = newPoint2
		else:
			pointsAB[0] = newPoint2
			pointsAB[1] = newPoint1

		calculateWhereToPointAt(wordCount, pointsAB, effector, space)

		time.sleep(2)
		postureProxy.goToPosture("Crouch", 0.5)
		pairs_dict[tag_pairs] = True
		print pairs_dict[tag_pairs]




def calculateWhereToPointAt(wordCount, pointsAB, effector, frame):

	global selectedStory
	global mainPoints
	global trajComplete
	trajComplete = []

	changeCoordinate = TRANSFORMATION()
	wordProc = WORDPROCESSING()

	PA = Pose()
	PB = Pose()

	PA.position.x = pointsAB[0].item(0, 0)
	PA.position.y = pointsAB[0].item(1, 0)
	PA.position.z = pointsAB[0].item(2, 0)

	PB.position.x = pointsAB[1].item(0, 0)
	PB.position.y = pointsAB[1].item(1, 0)
	PB.position.z = pointsAB[1].item(2, 0)

	for i in range(wordCount):
		""" Based on the number of words, calcultae the positions for the robot to point at"""
		const = (i + 1) / float (1 + wordCount) 
		newP = changeCoordinate.lineFunc(PA, PB, const)
		midP = [newP.position.x, newP.position.y, newP.position.z, newP.orientation.x, newP.orientation.y, newP.orientation.z]
		mainPoints.append(midP)

	maxSpeed = 0.1
	maxSpeedHead = 0.05
	useWholeBody = False


	trajComplete = changeCoordinate.calculateTrajectory(mainPoints, wordCount)

	
	#storyToSay = taggedStory.encode("utf-8")
	
	for i in range(len(trajComplete)):
	
		tracker.lookAt(trajComplete[i], frame, maxSpeed, useWholeBody)
		tracker.pointAt(effector, trajComplete[i], frame, maxSpeedHead)

		if i == 0:
			wordProc.readTheTaggedStory(selectedStory, True)
		if (i ) %4 == 0:
			print i
			time.sleep(0.2)

		print trajComplete[i]

	# Empty the arrays
	trajComplete = []
	mainPoints = []

	print "trajComplete"
	print trajComplete

def readAndMoveInstruction(selectedStory, correctPermission, cameraName, handMovePermission, tag):

	wordProc = WORDPROCESSING()
	wordProc.readTheTaggedStory(selectedStory, correctPermission)
	if handMovePermission == True:
		wordCount = wordProc.storySelection(tag)
		rospy.Subscriber("target_pose", PoseArray, getTagLocations, cameraName)

def tag_detection(msg):
	"""

	"""
	#faceTrackingEnded()
	global wordCount
	global moveHand
	# initializing classes
	wordProc = WORDPROCESSING()
	#global selectedStory

	tag = msg.data
	tag = tag.replace('[', '').replace(']', '')
	#print "tag"
	#print tag

	cameraName = 'CameraBottom'
	
	wordCount = wordProc.storySelection(tag)
	#rospy.Subscriber("target_pose", PoseArray, getTagLocations, cameraName)

	pitch_angle = 0.3
	global counter
	if counter == 0:
		LookAtTheBook(pitch_angle)
		counter = 1

	global tag_stream_buffer

	if len(tag_stream_buffer) == 0:
		tag_stream_buffer.append(msg.data)
	if len(tag_stream_buffer) >= 1:
		if msg.data == tag_stream_buffer[len(tag_stream_buffer)-1]:
			tag_stream_buffer.append(msg.data)

		else:
			tag_stream_buffer = []
			tag_stream_buffer.append(msg.data)

	#if len(tag_stream_buffer) >= 2:
		#current_tag = tag_stream_buffer[len(tag_stream_buffer)-1]
	
		# Tilt the roobot's head to the front


	for i in pairs_dict:
		if i != msg.data:
			if pairs_dict[i] == "Waiting for Red-card":
				pairs_dict[i] = False

			elif pairs_dict[i] == True:
				pairs_dict[i] = "Corrected Mode"

			elif pairs_dict[i] == "Waiting for Green-card":
				pairs_dict[i] = "Corrected Mode"

			elif pairs_dict[i] == "Happy Mode":
				pairs_dict[i] = "Corrected Mode"




	rospy.Subscriber('card_id_state', String, card_detection, msg.data)
	print pairs_dict[msg.data]


	if pairs_dict[msg.data] == False:
		moveHand = 0
		correctPermission = pairs_dict[msg.data]
		handMovePermission = True
		readAndMoveInstruction(selectedStory, correctPermission, cameraName, handMovePermission, tag)
		# story.say(selectedStory)
		pairs_dict[msg.data] = "Waiting for Red-card"

	elif pairs_dict[msg.data] == "Correcting Mode":
		pairs_dict[msg.data] = "Waiting for Green-card"
		
		#motionProxy.setBreathEnabled('Arms', False)
		#motionProxy.setBreathEnabled('Head', False)
		sadmotionProxySelection()
		wordProc.readTheTaggedStory(selectedStory, True)
		#pairs_dict[msg.data] = True

	elif pairs_dict[msg.data] == "Happy Mode":
		pairs_dict[msg.data] = True
		
		#motionProxy.setBreathEnabled('Arms', False)
		#motionProxy.setBreathEnabled('Head', False)
		happymotionProxySelection()
		#story.say("\\rspd=70\\ Lets go to the next page \\pau=500\\ ")
	
	elif pairs_dict[msg.data] == "Corrected Mode":
		pairs_dict[msg.data] = "Waiting for Green-card"
		wordProc.readTheTaggedStory(selectedStory, True)
		#pairs_dict[msg.data] = True



def card_detection(msg, tag):
	red_card = "40"
	green_card = "39"

	card_id = msg.data
	global red_counter
	#print red_counter
	if card_id == red_card:
		if pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Correcting Mode"
			#reac = 2
			#motionProxySelection(reac)

		#if pairs_dict[tag] == "corrected":
			#pairs_dict[tag] = True


	elif card_id == green_card:
		if pairs_dict[tag] == "Waiting for Green-card" :
			#red_counter = 0
			pairs_dict[tag] = "Happy Mode"

			

def sadmotionProxySelection():
	""" Select a motionProxy from the available motionProxys after receiving 

	"""
	
	motionProxy.setExternalCollisionProtectionEnabled("All", True)
	motionProxyNum = random.randint(1,5)
	#motionProxyNum = 3

	if motionProxyNum == 6:
		wordsBefore = "\\rspd=90\\ Oh Really???"
		sleepTime = 3
		wordsAfter = "\\rspd=90\\wait!! \\pau=500\\ I'll try again"
		emotion = "sad"
		reactToTheMistake(emotion, animations.embarassed_seated_pose, wordsBefore, wordsAfter, sleepTime)

	if motionProxyNum == 2:
		wordsBefore = "\\rspd=60\\ Aaahhh!! \\pau=600\\ \\rspd=90\\ I didn't know!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ let me try again"
		emotion = "surprise"
		reactToTheMistake(emotion, animations.scratchHead_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if motionProxyNum == 7:
		wordsBefore = "\\rspd=80\\ Oh!! sorry!!"		
		sleepTime = 1
		wordsAfter = "\\rspd=90\\ I will read it again"
		emotion = "sad"
		reactToTheMistake(emotion, animations.disappointed_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if motionProxyNum == 4:
		wordsBefore = "\\rspd=70\\ Oh!! sorry!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ I will read it again"
		emotion = "sad"
		reactToTheMistake(emotion, animations.pensive_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

	if motionProxyNum == 5:
		wordsBefore = "\\rspd=70\\ hmm!!"		
		sleepTime = 1
		wordsAfter = "\\rspd=80\\ I need to read it again"
		emotion = "sad"
		reactToTheMistake(emotion, animations.thinking6_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if motionProxyNum == 1:
		wordsBefore = "\\rspd=80\\ oh  \\pau=700\\ \\rspd=60\\  really?!!"		
		sleepTime = 2
		wordsAfter = " \\rspd=80\\ \\pau=200\\ let me read it again"
		emotion = "surprise"
		reactToTheMistake(emotion, animations.hesitation2_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if motionProxyNum == 3:
		wordsBefore = "\\rspd=60\\ Oh!!! \\rspd=80\\ \\pau=700\\ I was wrong"		
		sleepTime = 2
		wordsAfter = "\\rspd=90\\ I will try again"
		emotion = "sad"
		reactToTheMistake(emotion, animations.thinking5_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)


def happymotionProxySelection():
	""" Select a motionProxy from the available motionProxys after receiving the green card 

	"""
	
	motionProxy.setExternalCollisionProtectionEnabled("All", True)
	motionProxyNum = random.randint(1,5)
	#motionProxyNum = 4
	emotion = "happy"

	if motionProxyNum == 1:
		pitch_angle = -0.9
		LookAtTheBook(pitch_angle)
		wordsBefore = "\\rspd=80\\ Yeaaah!!!"
		sleepTime = 3
		wordsAfter = "\\rspd=70\\ Thank you"
		reactToTheMistake(emotion, animations.winner_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if motionProxyNum == 2:
		pitch_angle = -0.9
		LookAtTheBook(pitch_angle)
		wordsBefore = "\\rspd=60\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ Thank you"
		reactToTheMistake(emotion, animations.winner2_seated_pose, wordsBefore, wordsAfter, sleepTime, 1.0)

	if motionProxyNum == 3:
		pitch_angle = -0.9
		LookAtTheBook(pitch_angle)
		wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ I made it "
		reactToTheMistake2(animations.relieved_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if motionProxyNum == 4:
		pitch_angle = -0.9
		LookAtTheBook(pitch_angle)
		wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ "
		reactToTheMistake(emotion, animations.proud_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

	if motionProxyNum == 5:
		wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
		sleepTime = 3
		wordsAfter = "\\rspd=80\\ "
		reactToTheMistake(emotion, animations.happy_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if motionProxyNum == 6:
		wordsBefore = "\\rspd=70\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=70\\ "
		reactToTheMistake(emotion, animations.happy2_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)
		pitch_angle = 0.5
		LookAtTheBook(pitch_angle)

	if motionProxyNum == 7:
		wordsBefore = "\\rspd=70\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ "
		reactToTheMistake(emotion, animations.happy3_pose, wordsBefore, wordsAfter, sleepTime)



def reactToTheMistake( emotion, pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
	""" If the keys pressed are due to detection of mistake, the robot reacts.
		The motionProxy is a physical movement and certain words which shows robot's remorse

	"""

	emotionReaction = EMOTIONS()

	times = changeSpeed(pose.times, factorSpeed)
	id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
	story.post.say(wordsBefore)
	emotionReaction.set_emotion(emotion)
	turn_on_eye()
	#time.sleep(pause)
	#motionProxy.wait(id,0)
	#time.sleep(1)
	story.say(wordsAfter)
	#time.sleep(5)
	correctFlag = True
	pitch_angle = 0.5
	LookAtTheBook(pitch_angle)
	#motionProxy.setBreathEnabled('Arms', True)
	#motionProxy.setBreathEnabled('Head', True)
	#postureProxy.goToPosture("Stand", 1.0)
	#readTheTaggedStory(selectedStory, correctFlag)
	#readTheTaggedStoryWithLevel(selectedStory, correctFlag)


def reactToTheMistake2( pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
	""" If the keys pressed are due to detection of mistake, the robot reacts.
		The motionProxy is a physical movement and certain words which shows robot's remorse

	"""
	emotionReaction = EMOTIONS()

	times = changeSpeed(pose.times, factorSpeed)
	id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
	story.post.say(wordsBefore)
	emotionReaction.set_emotion("happy")
	turn_on_eye()
	#time.sleep(pause)
	#motionProxy.wait(id,0)
	#time.sleep(1)
	story.say(wordsAfter)
	#time.sleep(5)
	correctFlag = True
	pitch_angle = 0.5
	LookAtTheBook(pitch_angle)
	#motionProxy.setBreathEnabled('Arms', True)
	#motionProxy.setBreathEnabled('Head', True)
	#postureProxy.goToPosture("Stand", 1.0)
	#readTheTaggedStory(selectedStory, correctFlag)
	#readTheTaggedStoryWithLevel(selectedStory, correctFlag)


def turn_on_eye():
	section1 = ["FaceLeds", "ChestLeds" ]
	proxy.createGroup("turn",section1)
	proxy.fadeRGB("turn", 0x00FFFFFF, 0.3)


def changeSpeed(times, factor):
	""" It changes the speed of predefined times for each pose movement

	"""

	for i in xrange(len(times)):
		times[i] = [x / float(factor) for x in times[i]]

	return times




def faceTrackingStarted(faceSize):
	""" Robot starts to track the users face

	"""

	# First, wake up
	#motionProxy.wakeUp()
	#motionProxy.rest()

	# Add target to track
	targetName = "Face"
	faceWidth = faceSize
	tracker.registerTarget(targetName, faceWidth)

	# Then, start tracker
	tracker.track(targetName)


def faceTrackingEnded():
	""" Robot stops to track the users face and go into resting mode after certain keys are pressed

	"""

	tracker.stopTracker()
	tracker.unregisterAllTargets()
	#motionProxy.rest()


def LookAtTheBook(pitch_angle):
	""" Move the robot's head to look at the camera

	"""

	motionProxy.setStiffnesses("Head", 1.0)

	# Example showing how to set angles, using a fraction of max speed
	names  = ["HeadYaw", "HeadPitch"]
	angles  = [0, pitch_angle]
	fractionMaxSpeed  = 0.05
	motionProxy.setAngles(names, angles, fractionMaxSpeed)

	"""names               = "HeadYaw"
	changes             = -0.5
	fractionMaxSpeed    = 0.05
	motionProxy.changeAngles(names, changes, fractionMaxSpeed)"""

	time.sleep(1.0)
	motionProxy.setStiffnesses("Head", 0.0)


def IntroduceNao():
	"""
	Nao starts introducing itself when the book cover is in front of him 
	"""

	# First, wake up
	#motionProxy.wakeUp()
	postureProxy.goToPosture("Crouch", 0.5)
	turn_on_eye()
	#motionProxy.setBreathEnabled('Arms', True)
	#motionProxy.setBreathEnabled('Head', True)
	#motionProxy.rest()


	'''if msg.data in story_dict:
		storyNum = story_dict[msg.data]
		print storyNum'''

	'''if msg.data == '[0, 1]' or msg.data == '[1, 0]':
		if pairs_dict['[0, 1]'] == False:
			print "test         [0, 1]"
			pairs_dict['[0, 1]'] = True'''
	story.setLanguage('English')
	#story.say("\\rspd=90\\ Hello \\pau=500\\ My name is nao \\pau=500\\ I really like reading short stories")
	#story.say("\\rspd=90\\ Do you want to listen to them?")
	#story.say("\\rspd=90\\ sometimes I make mistakes, can you help me to correct them?")
	time.sleep(1)
	#story.say("\\rspd=90\\ If you want to read with me, please bring the book")
	story.say("\\rspd=90\\ Hello")
	pitch_angle = 0.1
	LookAtTheBook(pitch_angle)
	time.sleep(2)





def tagState(msg):
	global counter
	global tag_pairs
	global poses 
	if tag_pairs != msg.data:
		moveHand = 0
		tag_pairs = msg.data
		poses = []
	print tag_pairs
	#print tag_pairs

	
			


def main():
	
	global story
	story = ALProxy("ALTextToSpeech", "nao.local", 9559)
	
	global conversration
	conversation = ALProxy("ALTextToSpeech", "nao.local", 9559)

	global motionProxy
	motionProxy = ALProxy("ALMotion", "nao.local", 9559)

	global postureProxy
	postureProxy = ALProxy("ALRobotPosture", "nao.local", 9559)
	
	global audiomotionProxy
	audiomotionProxy = ALProxy("ALAudioPlayer", "nao.local", 9559)
	
	global tracker
	tracker = ALProxy("ALTracker", "nao.local", 9559)

	global proxy
	proxy = ALProxy("ALLeds", "nao.local", 9559)

	# Initialize the node
	rospy.init_node('read_lines_event')

	#storyNum = 1
	#pub = rospy.Publisher('story_number', String, queue_size=1)
	#pub.publish(str(storyNum))
	#storySelection()

	# Face tracking activated
	facesize = 0.1
	global restingEnabled 
	restingEnabled = False
	faceTrackingStarted(facesize)
	
	global space
	space      = motion.FRAME_ROBOT

	# Select a story and activity level
	global selectedStory
	#selectedStory = storySelection()
	correctFlag = False
	activityLevel = rospy.get_param('actlevel', 'medium')
	#mistakeNum = numberOfMistakesSelection(activityLevel)

	time.sleep(3)
	IntroduceNao()

	#rospy.Subscriber('tag_id_state', String, IntroduceNao)
	faceTrackingEnded()
	pitch_angle = 0.8
	LookAtTheBook(pitch_angle)
	
	global effector
	effector = "LArm"

	

	rospy.Subscriber('tag_id_state', String, tag_detection)
	

		
	#rospy.Subscriber('card_id_state', String, card_detection)

	#readTheTaggedStory(selectedStory, correctFlag)

	#readTheTaggedStoryWithLevel(selectedStory, correctFlag, mistakeNum)

	#rospy.Subscriber('button_start', String, startTheStoryReading)

	#rospy.Subscriber('button_wrong', String, mistakeDetected)

	#rospy.Subscriber('keys', String, faceTrackingEnded)

	#rospy.Subscriber('keys', String, repeatTheStoryReading)
	
	try:
		while True:
			time.sleep(1)
    	except restingEnabled == True:
        	print
        	print "Interrupted by user, shutting down"
        	myBroker.shutdown()
        	sys.exit(0)



if __name__ == "__main__":


	main()