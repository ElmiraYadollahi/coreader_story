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

global mainPoints
mainPoints = []
global trajComplete
trajComplete = []
global tag_pairs
tag_pairs = '[0, 0]'
global moveHand 
moveHand = 1

pairs_dict = {	'[0, 0]' : True,
				'[0, 1]' : False,
				'[2, 3]' : False,
				'[4, 5]' : False,
				'[6, 7]' : False,
				'[8, 9]' : False
			 }

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




class EMOTIONS:
	""" Processing the text and read """

	def __init__(self):
		a = 1

	def emotion_anger(self, dur=1):
		"""

		"""

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
		"""

		"""

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
		"""

		"""

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
		"""

		"""

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
		"""

		"""

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
		"""

		"""

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
		"""

		"""

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


	def calculateTrajectory(self, mainPoints, wordCount, lineNum=1):
		""" Calculate the main trajectory for the robot to point at

		"""
			

		""" Calculate the correction values to be added to the original points for increasing the pointing accuracy"""
		dx_corr = 0.1 - (0.03 * (lineNum - 1))
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

	def __init__(self, ARTag):

		self.tag = "=WordNum"
		self.selectedStory = self.storySelection(ARTag)
		self.lineMatrix = []

	def storySelection(self, tag):
		""" Select a story from the text files of each story and return it

		"""

		
		""" creates a array containing the line related to detected tags """
		with open('chick_story_en.txt') as f:
			lines_array = f.read().splitlines()

		for line in lines_array:
			found = re.search(tag, line)
			if found != None:
				self.selectedStory = line[:found.start()] + line[found.end():]
				self.selectedStory = self.selectedStory.replace('[', '').replace(']', '')
				break

		#wordCount = self.getTheWordCount()
		#print "story selection"
		#print self.selectedStory

		return self.selectedStory	


	def getTheLineMatrix(self):
		""" Reads the number of words given in the text and then remove the tag and number and returns wordCount

		"""		

		tag = "=NextLine"
		tagWithWord = "\w+(?=" + tag + ")"
		self.lineMatrix = re.split( tagWithWord + tag, self.selectedStory)
		#print "line matrix"
		#print self.lineMatrix
		#wordCountString = foundTag.group(0)
		#wordCountString = self.removeTheTag(tag, wordCountString)

		#wordCount = int(wordCountString)
		#self.selectedStory = self.removeTheWordWithTag(tag, self.selectedStory)

		#print "get the word count"
		#print self.selectedStory

		#return wordCount

	def getTheInstructionTagData(self, INTag):
		"""

		"""

		tag = INTag
		tagWithWord = "\w+(?=" + tag + ")"
		foundTag = re.search(tagWithWord + tag, self.selectedStory)

		instructionString = foundTag.group(0)
		instructionString = self.removeTheTag(tag, instructionString)

		instruct = int(instructionString)
		self.selectedStory = self.removeTheWordWithTag(tag, self.selectedStory)

		return instruct


	def clearAllTheInstructionTags(self):
		"""

		"""

		for inTag in instruction_tags_dict:
			if re.search(inTag, self.selectedStory) != None:
				self.selectedStory = self.removeTheWordWithTag(inTag, self.selectedStory)

		for inTag in instruction_tags_dict:
			for i in range(len(self.lineMatrix)):
				if re.search(inTag, self.lineMatrix[i]) != None:
					self.lineMatrix[i] = self.removeTheWordWithTag(inTag, self.lineMatrix[i])

		#print "clear all tags"
		#print self.selectedStory

		#print "clear all tags matrix"
		#print self.lineMatrix

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

	def readFromMatrixLine(self, correctFlag, line):

		if correctFlag == True:
			tag = "=RTag"
			self.lineMatrix[line] = self.removeTheTag(tag, self.lineMatrix[line])
			tag = "=WTag"
			self.lineMatrix[line] = self.removeTheWordWithTag(tag, self.lineMatrix[line])

		elif correctFlag == False:
			tag = "=WTag"
			self.lineMatrix[line] = self.removeTheTag(tag, self.lineMatrix[line])
			tag = "=RTag"
			self.lineMatrix[line] = self.removeTheWordWithTag(tag, self.lineMatrix[line])

		print "read line"
		print line

		self.sayFromFile(story, self.lineMatrix[line], 'ascii')

	def readTheTaggedStory(self, correctFlag):
		""" Read a story containing the tags and based on correctFlag change the tags approprietly

		"""
		
		if correctFlag == True:
			tag = "=RTag"
			taggedStory = self.removeTheTag(tag, self.selectedStory)
			tag = "=WTag"
			taggedStory = self.removeTheWordWithTag(tag, self.selectedStory)

		elif correctFlag == False:
			tag = "=WTag"
			taggedStory = self.removeTheTag(tag, self.selectedStory)
			tag = "=RTag"
			taggedStory = self.removeTheWordWithTag(tag, self.selectedStory)
		
		self.sayFromFile(story, taggedStory, 'ascii')


	def sayFromFile(self, story, filename, encoding):
		"""

		"""

		toSay = filename.encode("utf-8")
		story.post.say(toSay)






def getTagLocations(msg, cameraName):
	"""

	"""
	
	global moveHand
	moveHand += 1
	print "movehand"
	print moveHand

	changeCoordinate = TRANSFORMATION()
	

	ARTag = tag_pairs
	ARTag = ARTag.replace('[', '').replace(']', '')
	wordProc = WORDPROCESSING(ARTag)

	INTag = "=WordNum"
	wordCount = wordProc.getTheInstructionTagData(INTag)
	INTag = "=LineNum"
	lineCount = wordProc.getTheInstructionTagData(INTag)
	LiWoCount = []
	if lineCount > 1:
		for i in range(lineCount):
			INTag = "=L" + str((i+1))
			LiWoCount.append(wordProc.getTheInstructionTagData(INTag))
	elif lineCount == 1:
		LiWoCount.append(wordCount)
	
	print LiWoCount

	print "lineCount"
	print lineCount
	wordProc.getTheLineMatrix()
	#wordCount = wordProc.getTheWordCount()
	wordProc.clearAllTheInstructionTags()

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

		calculateWhereToPointAt( wordCount, pointsAB, effector, space, wordProc, lineCount, LiWoCount)

		time.sleep(2)
		postureProxy.goToPosture("Crouch", 0.5)
		pairs_dict[tag_pairs] = True
		print pairs_dict[tag_pairs]




def calculateWhereToPointAt(wordCount, pointsAB, effector, frame, wordProc, lineCount, LiWoCount):

	global selectedStory
	global mainPoints
	global trajComplete
	trajComplete = []

	changeCoordinate = TRANSFORMATION()
	#wordProc = WORDPROCESSING()

	PA = Pose()
	PB = Pose()

	PA.position.x = pointsAB[0].item(0, 0)
	PA.position.y = pointsAB[0].item(1, 0)
	PA.position.z = pointsAB[0].item(2, 0)

	PB.position.x = pointsAB[1].item(0, 0)
	PB.position.y = pointsAB[1].item(1, 0)
	PB.position.z = pointsAB[1].item(2, 0)



	maxSpeed = 0.1
	maxSpeedHead = 0.05
	useWholeBody = False

	trajComplete2 = []


	

	for i in range(len(LiWoCount)):
		mainPoints = []
		for j in range(LiWoCount[i]):
			""" Based on the number of words, calcultae the positions for the robot to point at"""
			const = (j + 1) / float (1 + LiWoCount[i]) 
			newP = changeCoordinate.lineFunc(PA, PB, const)
			midP = [newP.position.x, newP.position.y, newP.position.z, newP.orientation.x, newP.orientation.y, newP.orientation.z]
			mainPoints.append(midP)

		trajComplete = []
		trajComplete = changeCoordinate.calculateTrajectory(mainPoints, LiWoCount[i], i+1)
		print "trajComplete"
		print LiWoCount[i]

		print "len trajComplete"
		print len(trajComplete)
		trajComplete2.append([trajComplete])
			




	#print "trajComplete2"
	#print trajComplete2

	print "len trajComplete2"
	print len(trajComplete2)

	
	#storyToSay = taggedStory.encode("utf-8")
	#if lineCount == 1:
	for i in range(len(trajComplete2)):
		#print "i"
		#print i
		for j in range(len(trajComplete2[i][0])):
			#print "j"
			#print j
	
			tracker.lookAt(trajComplete2[i][0][j], frame, maxSpeed, useWholeBody)
			tracker.pointAt(effector, trajComplete2[i][0][j], frame, maxSpeedHead)

			if j == 0:
				wordProc.readFromMatrixLine(True, i+1)
				x = True
			if (j ) %4 == 0:
				#print j
				time.sleep(0.2)

			#print trajComplete[j]

	# Empty the arrays
	trajComplete = []
	mainPoints = []

	#print "trajComplete"
	#print trajComplete


def tagState(msg):
	global moveHand
	global tag_pairs
	global trajComplete 
	if tag_pairs != msg.data:
		moveHand = 0
		tag_pairs = msg.data
		trajComplete = []
	print tag_pairs
	#print tag_pairs





def main(nao_ip):
	rospy.init_node("ar_tags_poses")
	global tag_pairs

	global story
	story = ALProxy("ALTextToSpeech", nao_ip, 9559)
	story.setLanguage('English')

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
		
	global space
	space      = motion.FRAME_ROBOT

	global effector
	effector = "RArm"

	global selectedStory

	cameraName  = 'CameraBottom'


	# Start the movement 
	motionProxy.setExternalCollisionProtectionEnabled("All", True)
	
	postureProxy.goToPosture("Crouch", 0.5)

	rospy.Subscriber("tag_id_state", String, tagState)

	rospy.Subscriber("target_pose", PoseArray, getTagLocations, cameraName)
	
	rospy.spin()

if __name__ == "__main__":
    robotIp = 'nao.local'
    if len(sys.argv) <= 1:
        print "Usage python motion_cartesianArm1.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)