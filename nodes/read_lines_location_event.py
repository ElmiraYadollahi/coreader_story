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
from EEClass import EYEEMOTIONS
from MTClass import TRANSFORMATION
from WPClass import WORDPROCESSING

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
				'[12, 13]' : False,
				'[14, 15]' : False,
				'[16, 17]' : False,
				'[18, 19]' : False,
				'[20, 21]' : False,
				'[250, 251]' : False,
				'[252, 253]' : False,
				'[200, 201, 202, 203]' : False,
				'[204, 205, 206, 207]' : False,
				'[208, 209, 210, 211]' : False,
				'[212, 213, 214, 215]' : False,
				'[216, 217, 218, 219]' : False,
				'[220, 221, 222, 223]' : False,
				'[256, 257]' : False,
				'[258, 259]' : False,
				'[260, 261]' : False,
				'[262, 263]' : False,
				'[264, 265]' : False,
				'[266, 267]' : False,
				'[268, 269]' : False,
				'[270, 271]' : False,
				'[272, 273]' : False,
				'[274, 275]' : False,
				'[276, 277]' : False,
				'[278, 279]' : False
			 }


story_dict ={	'[200, 201]': 1, 
				'[210, 211]': 2,
				'[220, 221]': 3,
				'[230, 231]': 4,
				'[240, 241]': 5,
				'[2, 3]': 7
			}

card_id = { 38, 39, 40}


instruction_tags_dict = {	'=LineNum',
							'=wordNum',
							'=MistakeNum',
							'=NextLine',
							'=L1',
							'=L2',
							'=L3',
							'=L4',
							'=L5',
							'=L6',
							'=B1',
							'=B1',
							'=B2',
							'=B3',
							'=B4',
							'=B5',
							'=B6'
							
						}


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
global trajComplete
trajComplete = []
global correctPermission
correctPermission = False
global cardPosition
cardPosition = 0
global reactionPermission 
reactionPermission = False
global ARTag
global REDCardAllert
REDCardAllert = False

class MOTION_ANIMATION_SELECTION:
	def __init__(self):
		n = 1

	def reactionToREDCard(self):
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


	def reactionToGREENCard(self):
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


def cardPointedAt(msg):
	"""

	"""
	global cardPosition
	cardPosition += 1
	print "cardPosition"
	print cardPosition

	if cardPosition == 10:

		cameraName = 'CameraBottom'
		#changeCoordinate = TRANSFORMATION(motionProxy)
		#pointedWord = Pose()
		#pointedWord.position = msg.position
		PW = []
		PW.append([msg.position.x])
		PW.append([msg.position.y])
		PW.append([msg.position.z])
		PW.append([1])

		#P1 = changeCoordinate.transformPoint(PW, cameraName)
		print "P1"
		#print P1



def getTagLocations(msg):
	
	global trajComplete
	#trajComplete = [1]
	global ARTag
	changeCoordinate = TRANSFORMATION(motionProxy)
	global moveHand
	moveHand += 1
	cameraName = 'CameraBottom'
	#ARTag = tag_pairs
	#ARTag = ARTag.replace('[', '').replace(']', '')
	#wordProc = WORDPROCESSING(ARTag)


	if moveHand == 1:

		if(effector == "LArm"):
			motionProxy.openHand("LHand")
		else:
			motionProxy.openHand("RHand")

		if len(ARTag) == 1:
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

			
			#pointsAB.append(P1)
			#pointsAB.append(P2)

			""" Transformation Matrix"""
			#tranAC = changeCoordinate.transformMatrix(cameraName)
			newP1 = changeCoordinate.transformPoint(P1, cameraName)
			newP2 = changeCoordinate.transformPoint(P2, cameraName)

			#oldPoint = np.matrix(pointsAB[0])
			#newPoint1 = np.dot(tranAC, oldPoint) 

			#oldPoint = np.matrix(pointsAB[1])
			#newPoint2 = np.dot(tranAC, oldPoint)

			""" to make sure the robot's hand move from left to write
				start from the tag located in the left toward the one in the right
				compare their y value
			"""
			pointsAB = []
			if newP1[1] >= newP2[1]:
				pointsAB.append(newP1)
				pointsAB.append(newP2)
			else:
				pointsAB.append(newP2)
				pointsAB.append(newP1)

		elif len(ARTag) == 2:
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
			P3 = []
			P3.append([msg.poses[2].position.x])
			P3.append([msg.poses[2].position.y])
			P3.append([msg.poses[2].position.z])
			P3.append([1])
			P4 = []
			P4.append([msg.poses[3].position.x])
			P4.append([msg.poses[3].position.y])
			P4.append([msg.poses[3].position.z])
			P4.append([1])

			
			#pointsAB.append(P1)
			#pointsAB.append(P2)

			""" Transformation Matrix"""
			#tranAC = changeCoordinate.transformMatrix(cameraName)
			newP1 = changeCoordinate.transformPoint(P1, cameraName)
			newP2 = changeCoordinate.transformPoint(P2, cameraName)
			newP3 = changeCoordinate.transformPoint(P3, cameraName)
			newP4 = changeCoordinate.transformPoint(P4, cameraName)

			#oldPoint = np.matrix(pointsAB[0])
			#newPoint1 = np.dot(tranAC, oldPoint) 

			#oldPoint = np.matrix(pointsAB[1])
			#newPoint2 = np.dot(tranAC, oldPoint)

			""" to make sure the robot's hand move from left to write
				start from the tag located in the left toward the one in the right
				compare their y value
			"""
			pointsAB = []
			pointsAB.append(newP1)
			pointsAB.append(newP2)
			pointsAB.append(newP3)
			pointsAB.append(newP4)
			


		calculateWhereToPointAt(pointsAB, effector, space, changeCoordinate)

		time.sleep(2)
		postureProxy.goToPosture("Crouch", 0.5)
		pairs_dict[tag_pairs] = True
		print pairs_dict[tag_pairs]




def calculateWhereToPointAt( pointsAB, effector, frame, changeCoordinate):

	global correctPermission
	global selectedStory
	global trajComplete
	global story
	trajComplete = []
	global ARTag
	print "ARTag" 
	print ARTag
	global REDCardAllert
	global reactionPermission

	for j in range(len(ARTag)):
		print j 
		wordProc = WORDPROCESSING(story, ARTag[j])

		INTag = "=WordNum"
		wordCount = wordProc.getTheInstructionTagData(INTag)
		INTag = "=LineNum"
		lineCount = wordProc.getTheInstructionTagData(INTag)
		LiWoCount = []
		lineDistanceCoef = []
		#if lineCount > 1:
		for i in range(lineCount):
			INTag = "=L" + str((i+1))
			LiWoCount.append(wordProc.getTheInstructionTagData(INTag))

			INTag = "=S" + str((i+1))
			lineDistanceCoef.append(0.01 * wordProc.getTheInstructionTagData(INTag))

		#elif lineCount == 1:
			#LiWoCount.append(wordCount)
		
		print LiWoCount
		print lineDistanceCoef

		print "lineCount"
		print lineCount


		wordProc.getTheLineMatrix()
		#wordCount = wordProc.getTheWordCount()
		wordProc.clearAllTheInstructionTags()

		#changeCoordinate = TRANSFORMATION( motionProxy)
		#wordProc = WORDPROCESSING()
		print "LiWoCount = wordProc.getLineWordCount()"
		print LiWoCount
		print "pointsAB"
		print pointsAB

		maxSpeed = 0.1
		maxSpeedHead = 0.05
		useWholeBody = False

		trajComplete2 = []

		PA = Pose()
		PB = Pose()
		PC = Pose()
		PD = Pose()
		
		PA.position.x = pointsAB[2*j].item(0, 0)
		PA.position.y = pointsAB[2*j].item(1, 0)
		PA.position.z = pointsAB[2*j].item(2, 0)

		PB.position.x = pointsAB[2*j+1].item(0, 0)
		PB.position.y = pointsAB[2*j+1].item(1, 0)
		PB.position.z = pointsAB[2*j+1].item(2, 0)
		trajComplete2 = changeCoordinate.calculateEachWordPosition(PA, PB, effector, LiWoCount, lineDistanceCoef)



		print "PA"
		print PA
		print "PB"
		print PB
		print "PC"
		print PC
		print "PD"
		print PD


		print "len trajComplete2"
		print len(trajComplete2)
		#wordProc.readTheTaggedStory(correctPermission)
		for i in range(len(trajComplete2)):

			for j in range(len(trajComplete2[i][0])):
				if REDCardAllert == True:
					break
		
				tracker.lookAt(trajComplete2[i][0][j], frame, maxSpeed, useWholeBody)
				tracker.pointAt(effector, trajComplete2[i][0][j], frame, maxSpeedHead)

				if j == 0:
					if REDCardAllert == True:
						break
					wordProc.readFromMatrixLine(correctPermission, i+1, REDCardAllert)
					print "correct permission"
					print correctPermission
					x = True
					if REDCardAllert == True:
						break
				if (j ) %4 == 0:
			
					time.sleep(0.2)

			if REDCardAllert == True:
				break


		time.sleep(1)
		# Empty the arrays
		trajComplete2 = []
		wordProc.cleanStory()
		changeCoordinate.clearCoordinate()
		reactionPermission = True


def readAndMoveInstruction( cameraName, handMovePermission, detected_tag):

	global reactionPermission
	global ARTag
	ARTag = []
	ARRec = detected_tag
	ARRec = ARRec.replace('[', '').replace(']', '')
	foundAR = re.split(",", ARRec)
	print foundAR
	if len(foundAR) == 2:
		tagNum = 2
		ARTag.append(foundAR[0] +","+foundAR[1])

	elif len(foundAR) == 4:
		tagNum = 4
		ARTag.append(foundAR[0] +","+foundAR[1])
		ARTag.append(foundAR[2] +","+foundAR[3])
	print "tagNum"
	print tagNum
	print ARTag
	#wordProc = WORDPROCESSING(story, ARTag)
	motionProxy.setBreathEnabled('Arms', False)
	#motionProxy.setBreathEnabled('Head', False)
	#wordProc.readTheTaggedStory(selectedStory, correctPermission)
	if handMovePermission == True:
		reactionPermission = False


		"""INTag = "=WordNum"
		wordCount = wordProc.getTheInstructionTagData(INTag)
		INTag = "=LineNum"
		lineCount = wordProc.getTheInstructionTagData(INTag)
		LiWoCount = []
		lineDistanceCoef = []
		#if lineCount > 1:
		for i in range(lineCount):
			INTag = "=L" + str((i+1))
			LiWoCount.append(wordProc.getTheInstructionTagData(INTag))

			INTag = "=S" + str((i+1))
			lineDistanceCoef.append(0.01 * wordProc.getTheInstructionTagData(INTag))

		#elif lineCount == 1:
			#LiWoCount.append(wordCount)
		
		print LiWoCount
		print lineDistanceCoef

		print "lineCount"
		print lineCount

		wordProc.saveLineWordCount(LiWoCount, lineDistanceCoef)
		LiWoCount = wordProc.getLineWordCount()
		print LiWoCount

		wordProc.getTheLineMatrix()
		#wordCount = wordProc.getTheWordCount()
		wordProc.clearAllTheInstructionTags()"""

		rospy.Subscriber("target_pose", PoseArray, getTagLocations)

		#motionProxy.setBreathEnabled('Arms', True)




def tagDetection(msg):
	"""

	"""
	#faceTrackingEnded()
	global wordCount
	global moveHand
	global correctPermission
	global reactionPermission
	global REDCardAllert
	animationSelection = MOTION_ANIMATION_SELECTION()
	# initializing classes
	
	#global selectedStory


	#wordProc = WORDPROCESSING(ARTag)
	#print "tag"
	#print tag

	cameraName = 'CameraBottom'
	
	#wordCount = wordProc.storySelection(tag)
	#rospy.Subscriber("target_pose", PoseArray, getTagLocations, cameraName)

	pitch_angle = 0.2
	global counter
	if counter == 0:
		yaw_angle = 0.4
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
			
			elif pairs_dict[i] == "Repeat Correct Mode":
				pairs_dict[i] = "Corrected Mode"

			elif pairs_dict[i] == "Repeat Wrong Mode":
				pairs_dict[i] = False				



	rospy.Subscriber('card_id_state', String, cardDetection, msg.data)
	print pairs_dict[msg.data]


	if pairs_dict[msg.data] == False:
		moveHand = 0
		print "False added"
		correctPermission = False
		handMovePermission = True
		readAndMoveInstruction(cameraName, handMovePermission, msg.data)
		reactionPermission = True
		REDCardAllert = False
		# story.say(selectedStory)
		pairs_dict[msg.data] = "Waiting for Red-card"



	elif pairs_dict[msg.data] == "Correcting Mode":
		
		
		#motionProxy.setBreathEnabled('Arms', False)
		#motionProxy.setBreathEnabled('Head', False)
		if reactionPermission == True:
			animationSelection.reactionToREDCard()
			moveHand = 0
			print "Correcting Mode added"
			correctPermission = True
			handMovePermission = True
			REDCardAllert = False
			readAndMoveInstruction(cameraName, handMovePermission, msg.data)
		#wordProc.readTheTaggedStory(selectedStory, True)
		#pairs_dict[msg.data] = True
		pairs_dict[msg.data] = "Waiting for Green-card"



	elif pairs_dict[msg.data] == "Happy Mode":
		pairs_dict[msg.data] = True
		print "Happy Mode added"
		#motionProxy.setBreathEnabled('Arms', False)
		#motionProxy.setBreathEnabled('Head', False)
		if reactionPermission == True:
			animationSelection.reactionToGREENCard()
		#story.say("\\rspd=70\\ Lets go to the next page \\pau=500\\ ")
	


	elif pairs_dict[msg.data] == "Corrected Mode":
		
		moveHand = 0
		print "corrected Mode added"
		correctPermission = True
		handMovePermission = True
		REDCardAllert = False
		readAndMoveInstruction(cameraName, handMovePermission, msg.data)
		#pairs_dict[msg.data] = True
		pairs_dict[msg.data] = "Waiting for Green-card"



	elif pairs_dict[msg.data] == "Repeat Correct Mode":
				
		moveHand = 0
		print "Repeat Correct Mode added"
		correctPermission = True
		handMovePermission = True
		REDCardAllert = False
		readAndMoveInstruction(cameraName, handMovePermission, msg.data)
		pairs_dict[msg.data] = "Repeated Correct Mode"



	elif pairs_dict[msg.data] == "Repeat Wrong Mode":
				
		moveHand = 0
		print "Repeat Wrong Mode added"
		correctPermission = False
		handMovePermission = True
		REDCardAllert = False
		readAndMoveInstruction( cameraName, handMovePermission, msg.data)
		pairs_dict[msg.data] = "Repeated Wrong Mode"



	elif pairs_dict[msg.data] == "Repeated Wrong Mode" :
		moveHand = 1
		pairs_dict[msg.data] = "Waiting for Red-card"


	elif pairs_dict[msg.data] == "Repeated Correct Mode" :
		moveHand = 1
		pairs_dict[msg.data] = "Waiting for Green-card"



def cardDetection(msg, tag):
	global cardPosition
	global REDCardAllert

	red_card = "40"
	green_card = "39"
	yellow_card = "38"

	card_id = msg.data
	global red_counter
	#print red_counter
	if card_id == red_card:
		if pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Correcting Mode"
			REDCardAllert = True
			#reac = 2
			#motionProxySelection(reac)

	elif card_id == yellow_card:

		if pairs_dict[tag] == False:
			pairs_dict[tag] = "Repeat Wrong Mode"

		elif pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Repeat Wrong Mode"

		elif pairs_dict[tag] == "Waiting for Green-card":
			pairs_dict[tag] = "Repeat Correct Mode"

		elif pairs_dict[tag] == "Corrected Mode":
			pairs_dict[tag] = "Repeat Correct Mode"

		elif pairs_dict[tag] == True:
			pairs_dict[tag] = "Repeat Correct Mode"	


	elif card_id == green_card:
		if pairs_dict[tag] == "Waiting for Green-card" :
			#red_counter = 0
			pairs_dict[tag] = "Happy Mode"

	cardPosition = 0

	#rospy.Subscriber('card_pose', Pose, cardPointedAt)

			


def reactToTheMistake( emotion, pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
	""" If the keys pressed are due to detection of mistake, the robot reacts.
		The motionProxy is a physical movement and certain words which shows robot's remorse

	"""
	motionProxy.setBreathEnabled('Arms', False)

	emotionReaction = EYEEMOTIONS(proxy)

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
	motionProxy.setBreathEnabled('Arms', True)
	#motionProxy.setBreathEnabled('Head', True)
	#postureProxy.goToPosture("Stand", 1.0)
	#readTheTaggedStory(selectedStory, correctFlag)
	#readTheTaggedStoryWithLevel(selectedStory, correctFlag)


def reactToTheMistake2( pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
	""" If the keys pressed are due to detection of mistake, the robot reacts.
		The motionProxy is a physical movement and certain words which shows robot's remorse

	"""
	motionProxy.setBreathEnabled('Arms', False)

	emotionReaction = EYEEMOTIONS(proxy)

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
	motionProxy.setBreathEnabled('Arms', True)
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


def LookAtTheBook(pitch_angle, yaw_angle=0):
	""" Move the robot's head to look at the camera

	"""

	motionProxy.setStiffnesses("Head", 1.0)

	# Example showing how to set angles, using a fraction of max speed
	names  = ["HeadYaw", "HeadPitch"]
	angles  = [yaw_angle, pitch_angle]
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
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	#motionProxy.setBreathEnabled('Head', True)
	#motionProxy.rest()


	story.setLanguage('English')
	#story.say("\\rspd=90\\ Hello \\pau=500\\ My name is nao \\pau=500\\ I really like reading short stories")
	#story.say("\\rspd=90\\ Do you want to listen to them?")
	#story.say("\\rspd=90\\ sometimes I make mistakes, can you help me to correct them?")
	time.sleep(1)
	#story.say("\\rspd=90\\ If you want to read with me, please bring the book")
	story.say("\\rspd=90\\ Hello")
	pitch_angle = 0.2
	LookAtTheBook(pitch_angle)
	time.sleep(2)



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
	rospy.init_node('read_lines_location_event')

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
	pitch_angle = 0.1
	yaw_angle = 0.8
	LookAtTheBook(pitch_angle)
	
	global effector
	effector = "LArm"

	

	rospy.Subscriber('tag_id_state', String, tagDetection)
	
	
	
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