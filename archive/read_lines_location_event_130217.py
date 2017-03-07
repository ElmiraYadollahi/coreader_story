#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import animations.embarassed_pose
import animations.scratchHead_pose
import animations.IdontKnow_pose
import animations.IdontKnow2_pose
import animations.hesitation_pose
import animations.hesitation2_pose
import animations.hesitation3_pose
import animations.thinking1_pose
import animations.thinking2_pose
import animations.thinking3_pose
import animations.thinking4_pose
import animations.thinking5_pose
import animations.thinking6_pose
import animations.thinking7_pose
import animations.thinking8_pose
import animations.monster_pose
import animations.disappointed_pose
import animations.excited_pose
import animations.excited2_pose
import animations.happy_pose
import animations.happy2_pose
import animations.happy3_pose
import animations.introduction_pose
import animations.proud_pose
import animations.nod_pose
import animations.relieved_pose
import animations.winner_pose
import animations.winner2_pose


from naoqi import ALProxy 
import codecs
import time
import re
import random

# Debug
import rospy
from std_msgs.msg import String
from memory.msg import Animation


""" It is a dictionary for the states of the tag:
	False: 		means the story have not be read yet, it will go into reading with wrong tag 
	"correcting": means the story have been read and the robot have recieved a red card
	True:		means the story have been read and the robot have recieved a green card

"""
pairs_dict = {	'[0, 1]' : False,
				'[2, 3]' : False,
				'[4, 5]' : False,
				'[6, 7]' : False,
				'[8, 9]' : False
			 }


story_dict ={	'[200, 201]': 1, 
				'[210, 211]': 2,
				'[220, 221]': 3,
				'[230, 231]': 4,
				'[240, 241]': 5,
				'[2, 3]': 7
			}

card_id = { 39, 40}

reaction_states = { "low": [1, 2, 3, 4, 5],
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

def storySelection():
	""" Select a story from the text files of each story and return it

	"""

	with open('chick_story_en.txt') as f:
		lines_content = f.read().splitlines()
		print lines_content

	for line in lines_content:
		found = re.search("2, 3", line)
		if not found == None:
			print line
			print line[:found.start()] + line[found.end():]
			print
			selectedStory = line[:found.start()] + line[found.end():]
			break

	return selectedStory

def checkList(list):
	"""

	"""

	for item in list[1:]:
		if item != list[0]:
			return False
		return True 

def tag_detection(msg):
	"""

	"""
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

	tag = msg.data
	tag = tag.replace('[', '').replace(']', '')
	
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


	rospy.Subscriber('card_id_state', String, card_detection, msg.data)
	print pairs_dict[msg.data]

	if pairs_dict[msg.data] == False:
		readTheTaggedStory(selectedStory, pairs_dict[msg.data])
		# story.say(selectedStory)
		pairs_dict[msg.data] = "Waiting for Red-card"

	elif pairs_dict[msg.data] == "Correcting Mode":
		pairs_dict[msg.data] = "Waiting for Green-card"
		reac = 2
		sadReactionSelection(reac)
		readTheTaggedStory(selectedStory, True)
		#pairs_dict[msg.data] = True

	elif pairs_dict[msg.data] == "Happy Mode":
		pairs_dict[msg.data] = True
		reac = 2
		happyReactionSelection(reac)
		#story.say("\\rspd=70\\ Lets go to the next page \\pau=500\\ ")
	
	elif pairs_dict[msg.data] == "Corrected Mode":
		pairs_dict[msg.data] = "Waiting for Green-card"
		readTheTaggedStory(selectedStory, True)
		#pairs_dict[msg.data] = True


	"""if msg.data == '[2, 3]' or msg.data == '[3, 2]':
		if pairs_dict[msg.data] == False:
			print "test         [2, 3]"
			#if counter == 1:
			#rospy.loginfo("read the story...")
			story.say("\\rspd=70\\ A red Fox \\pau=500\\ in green socks \\pau=500\\")
			pairs_dict['[2, 3]'] = True

	if msg.data == '[4, 5]' or msg.data == '[4, 5]':
		if pairs_dict['[4, 5]'] == False:
			print "test         [4, 5]"
			#if counter == 1:
			#rospy.loginfo("read the story...")
			story.say("\\rspd=70\\ doing tricks with orange rocks")
			pairs_dict['[4, 5]'] = True"""


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
			#reactionSelection(reac)

		#if pairs_dict[tag] == "corrected":
			#pairs_dict[tag] = True

		"""if red_counter == 1:
			pairs_dict[tag] = "correcting"
			print "hello"
			print tag
		elif pairs_dict[tag] >= 2:
			pairs_dict[tag] = True
		red_counter = red_counter + 1"""

	elif card_id == green_card:
		if pairs_dict[tag] == "Waiting for Green-card" :
			#red_counter = 0
			pairs_dict[tag] = "Happy Mode"

			

def readTheTaggedStory(taggedStory, correctFlag):
	""" Read a story containing the tags and based on correctFlag change the tags approprietly

	"""
			
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
	
	sayFromFile(story, taggedStory, 'ascii')


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


def sayFromFile(story, filename, encoding):
	"""

	"""
	#with codecs.open(filename, encoding=encoding) as fp:
	#contents = filename.read()
		# warning: print contents won't work
	toSay = filename.encode("utf-8")
	story.say(toSay)



def sadReactionSelection(reac):
	""" Select a reaction from the available reactions after receiving 

	"""
	
	reaction.setExternalCollisionProtectionEnabled("All", True)
	reactionNum = random.randint(1,8)
	reactionNum = 2

	if reactionNum == 1:
		wordsBefore = "\\rspd=90\\ Oh Really???"
		sleepTime = 3
		wordsAfter = "\\rspd=90\\wait!! \\pau=500\\ I'll try again"
		reactToTheMistake(reac, animations.embarassed_pose, wordsBefore, wordsAfter, sleepTime)

	if reactionNum == 2:
		wordsBefore = "\\rspd=60\\ hmm!! \\pau=300\\ \\rspd=90\\ I didn't know!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ let me try again"
		reactToTheMistake(reac, animations.scratchHead_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if reactionNum == 3:
		wordsBefore = "\\rspd=80\\ Oh!! sorry!!"		
		sleepTime = 1
		wordsAfter = "\\rspd=90\\ I will read it again"
		reactToTheMistake(reac, animations.disappointed_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if reactionNum == 4:
		wordsBefore = "\\rspd=80\\ Oh!!! I was wrong"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ wait!!! I'll try again"
		reactToTheMistake(reac, animations.thinking5_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

	if reactionNum == 5:
		wordsBefore = "\\rspd=70\\ hmm!!"		
		sleepTime = 1
		wordsAfter = "\\rspd=80\\ I need to read it again"
		reactToTheMistake(reac, animations.thinking6_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if reactionNum == 6:
		wordsBefore = "\\rspd=80\\ let me see!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=90\\ you are right!! \\rspd=80\\ \\pau=200\\ let me read it again"
		reactToTheMistake(reac, animations.thinking7_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if reactionNum == 7:
		wordsBefore = "\\rspd=80\\ Oh!!! \\pau=700\\ was I wrong?"		
		sleepTime = 2
		wordsAfter = "\\rspd=90\\ I will try again"
		reactToTheMistake(reac, animations.thinking8_pose, wordsBefore, wordsAfter, sleepTime, 0.8)


def happyReactionSelection(reac):
	""" Select a reaction from the available reactions after receiving the green card 

	"""
	
	reaction.setExternalCollisionProtectionEnabled("All", True)
	reactionNum = random.randint(1,8)
	#reactionNum = 7

	if reactionNum == 1:
		wordsBefore = "\\rspd=80\\ Yeaaah!!!"
		sleepTime = 3
		wordsAfter = "\\rspd=80\\ Thanks"
		reactToTheMistake(reac, animations.winner_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if reactionNum == 2:
		wordsBefore = "\\rspd=60\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ "
		reactToTheMistake(reac, animations.winner2_pose, wordsBefore, wordsAfter, sleepTime, 1.0)

	if reactionNum == 3:
		wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ "
		reactToTheMistake(reac, animations.relieved_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if reactionNum == 4:
		wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ "
		reactToTheMistake(reac, animations.proud_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

	if reactionNum == 5:
		wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
		sleepTime = 3
		wordsAfter = "\\rspd=80\\ "
		reactToTheMistake(reac, animations.happy_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

	if reactionNum == 6:
		wordsBefore = "\\rspd=70\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=70\\ "
		reactToTheMistake(reac, animations.happy2_pose, wordsBefore, wordsAfter, sleepTime, 0.8)
		postureProxy.goToPosture("Stand", 0.7)
		pitch_angle = 0.5
		LookAtTheBook(pitch_angle)

	if reactionNum == 7:
		wordsBefore = "\\rspd=70\\ Yeaaah!!!"		
		sleepTime = 2
		wordsAfter = "\\rspd=80\\ "
		reactToTheMistake(reac, animations.happy3_pose, wordsBefore, wordsAfter, sleepTime)


def reactToTheMistake(reac, pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
	""" If the keys pressed are due to detection of mistake, the robot reacts.
		The reaction is a physical movement and certain words which shows robot's remorse

	"""


	times = changeSpeed(pose.times, factorSpeed)
	id = reaction.post.angleInterpolationBezier(pose.names, times, pose.keys)
	time.sleep(pause)
	story.post.say(wordsBefore)
	reaction.wait(id,0)
	time.sleep(1)
	story.say(wordsAfter)
	time.sleep(5)
	correctFlag = True
	pitch_angle = 0.5
	LookAtTheBook(pitch_angle)
	#postureProxy.goToPosture("Stand", 1.0)
	#readTheTaggedStory(selectedStory, correctFlag)
	#readTheTaggedStoryWithLevel(selectedStory, correctFlag)

	
def changeSpeed(times, factor):
	""" It changes the speed of predefined times for each pose movement

	"""

	for i in xrange(len(times)):
		times[i] = [x / float(factor) for x in times[i]]

	return times


def mistakeDetected(msg):
	""" Determin if the user has detected a mistake in reading the story
		with checking if certain keys have been pressed

	"""
	
	if len(msg.data) == 0 or not KEY_MAPPING.has_key(msg.data[0]):
		return
	reac = KEY_MAPPING[msg.data[0]]
	print KEY_MAPPING[msg.data[0]]
	#reactToTheMistake(reac)
	reactionSelection(reac)


def faceTrackingStarted(faceSize):
	""" Robot starts to track the users face

	"""

	# First, wake up
	#reaction.wakeUp()
	reaction.rest()

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
	reaction.rest()


def LookAtTheBook(pitch_angle):
	""" Move the robot's head to look at the camera

	"""

	reaction.setStiffnesses("Head", 1.0)

	# Example showing how to set angles, using a fraction of max speed
	names  = ["HeadYaw", "HeadPitch"]
	angles  = [0, pitch_angle]
	fractionMaxSpeed  = 0.05
	reaction.setAngles(names, angles, fractionMaxSpeed)

	"""names               = "HeadYaw"
	changes             = -0.5
	fractionMaxSpeed    = 0.05
	motionProxy.changeAngles(names, changes, fractionMaxSpeed)"""

	time.sleep(1.0)
	reaction.setStiffnesses("Head", 0.0)


def IntroduceNao():
	"""
	Nao starts introducing itself when the book cover is in front of him 
	"""

	# First, wake up
	reaction.wakeUp()
	postureProxy.goToPosture("Crouch", 0.8)
	#reaction.rest()


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
	#time.sleep(1)
	story.say("\\rspd=90\\ If want to read with me, please bring the story book")
	pitch_angle = 0.1
	LookAtTheBook(pitch_angle)
	time.sleep(2)

			


def main():
	
	global story
	story = ALProxy("ALTextToSpeech", "nao.local", 9559)
	
	global conversration
	conversation = ALProxy("ALTextToSpeech", "nao.local", 9559)

	global reaction
	reaction = ALProxy("ALMotion", "nao.local", 9559)

	global postureProxy
	postureProxy = ALProxy("ALRobotPosture", "nao.local", 9559)
	
	global audioreaction
	audioreaction = ALProxy("ALAudioPlayer", "nao.local", 9559)
	
	global tracker
	tracker = ALProxy("ALTracker", "nao.local", 9559)

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
	#faceTrackingStarted(facesize)
	#faceTrackingEnded()

	# Select a story and activity level
	global selectedStory
	#selectedStory = storySelection()
	correctFlag = False
	activityLevel = rospy.get_param('actlevel', 'medium')
	#mistakeNum = numberOfMistakesSelection(activityLevel)

	time.sleep(3)
	IntroduceNao()

	#rospy.Subscriber('tag_id_state', String, IntroduceNao)


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