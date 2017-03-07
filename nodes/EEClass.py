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
#from EClass import EMOTIONS

class EYEEMOTIONS:
	""" Processing the text and read """

	def __init__(self, prox):
		a = 1
		self.proxy = prox

	def emotion_anger(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed2", "FaceLed3" ]
		section2 = ["FaceLed4", "FaceLed5", "FaceLed6", "FaceLed7", "ChestLeds", "FeetLeds", "EarLeds"]
		self.proxy.createGroup("half_up", section1)
		self.proxy.createGroup("half_down", section2)

		# Initiate the emotion
		self.proxy.fadeRGB("half_up", 0, 0, 0, 0.1)
		self.proxy.fadeRGB("half_down", 0x00FA0000, 0.1)
		self.proxy.fadeRGB("half_down", 0x00640000, 1)
		self.proxy.fadeRGB("half_down", 0x00FA0000, 1)

		# Turn off the emotion
		self.proxy.fadeRGB("FaceLeds", 0x00000000, 1)
		self.proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_surprise(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed2", "FaceLed3", "FaceLed4", "FaceLed5", "FaceLed6", "FaceLed7", "ChestLeds", "FeetLeds"]
		self.proxy.createGroup("whole", section1)

		# Initiate the emotion
		self.proxy.fadeRGB("whole", 0x00323200, 0.2)
		self.proxy.fadeRGB("whole", 0x00000000, 0.1)
		self.proxy.fadeRGB("whole", 0x003C3C00, 0.1)
		self.proxy.fadeRGB("whole", 0x00FAFA00, 0.1)
		self.proxy.fadeRGB("whole", 0x00FAFA00, 1)

		# turn off the emotion 
		self.proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		self.proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_disgust(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed4", "FaceLed5", "ChestLeds", "FeetLeds"]
		section2 = ["FaceLed2", "FaceLed3", "FaceLed6", "FaceLed7"]
		self.proxy.createGroup("Updisgust", section1)
		self.proxy.createGroup("Downdisgust", section2)

		# Initiate the emotion
		self.proxy.fadeRGB("Downdisgust", 0x00000000, 0.1)
		self.proxy.fadeRGB("Updisgust", 0x0000FA00, 0.1)
		self.proxy.fadeRGB("Updisgust", 0x00004B00, 0.5)
		self.proxy.fadeRGB("Updisgust", 0x0000FA00, 0.1)
		self.proxy.fadeRGB("Updisgust", 0x00004B00, 0.5)
		
		# turn off the emotion 
		self.proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		self.proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_sadness(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed2", "FaceLed3", "FaceLed6", "FaceLed7", "FeetLeds"]
		section2 = [ "FaceLed4", "FaceLed5", "ChestLeds"]
		self.proxy.createGroup("dark", section1)
		self.proxy.createGroup("sadness", section2)

		# Initiate the emotion
		self.proxy.fadeRGB("dark", 0x00000000, 0)
		self.proxy.fadeRGB("sadness", 0x00000000, 0)
		self.proxy.fadeRGB("sadness", 0x000000FF, 5.0)
		self.proxy.fadeRGB("sadness", 0x00000000, 2.0)

		# turn off the emotion 
		self.proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		self.proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_happiness(self, dur=1):

		namesup = ["FaceLed0", "FaceLed1", "ChestLeds"]
		namesright = [ "FaceLed2", "FaceLed3", "ChestLeds"]
		namesdown = [ "FaceLed4", "FaceLed5", "ChestLeds"]
		namesleft = [ "FaceLed6", "FaceLed7", "ChestLeds"]

		self.proxy.createGroup("happyup",namesup)
		self.self.proxy.createGroup("happydown",namesdown)
		self.proxy.createGroup("happyright",namesright)
		self.proxy.createGroup("happyleft",namesleft)

		# Initiate the emotion
		i = 0
		while i <= 7:
			self.proxy.fadeRGB("happyup", 0x00FF00FF, 0)
			self.proxy.fadeRGB("happyright", 0x00969600, 0)
			self.proxy.fadeRGB("happydown", 0x00FF8C00, 0)
			self.proxy.fadeRGB("happyleft", 0x00FF0000, 0)
			time.sleep(0.21)
			self.proxy.fadeRGB("happyright", 0x00FF00FF, 0)
			self.proxy.fadeRGB("happydown", 0x00969600, 0)
			self.proxy.fadeRGB("happyleft", 0x00FF8C00, 0)
			self.proxy.fadeRGB("happyup", 0x00FF0000, 0)
			time.sleep(0.21)
			self.proxy.fadeRGB("happydown", 0x00FF00FF, 0)
			self.proxy.fadeRGB("happyleft", 0x00969600, 0)
			self.proxy.fadeRGB("happyup", 0x00FF8C00, 0)
			self.proxy.fadeRGB("happyright", 0x00FF0000, 0)
			time.sleep(0.21)
			self.proxy.fadeRGB("happyleft", 0x00FF00FF, 0)
			self.proxy.fadeRGB("happyup", 0x00969600, 0)
			self.proxy.fadeRGB("happyright", 0x00FF8C00, 0)
			self.proxy.fadeRGB("happydown", 0x00FF0000, 0)
			i = i + 1

		# turn off the emotion 
		self.proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		self.proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)


	def emotion_fear(self, dur=1):

		section1 = ["FaceLed0", "FaceLed1", "FaceLed2", "FaceLed3", "FeetLeds"]
		section2 = [ "FaceLed4", "FaceLed5", "FaceLed6", "FaceLed7", "ChestLeds"]
		self.proxy.createGroup("dark",section1)
		self.proxy.createGroup("fear",section2)

		# Initiate the emotion
		self.proxy.fadeRGB("dark", 0x00000000, 0)
		self.proxy.fadeRGB("fear", 0x00000046, 0)
		self.proxy.fadeRGB("fear", 0x00000064, 0.5)
		self.proxy.fadeRGB("fear", 0x00000046, 0.3)

		# turn off the emotion 
		self.proxy.post.fadeRGB("FaceLeds", 0x00000000, 1)
		self.proxy.fadeRGB("ChestLeds", 0x00000000, 1)
		time.sleep(1)

	def turn_on_eye(self):
		section1 = ["FaceLeds", "ChestLeds" ]
		self.proxy.createGroup("turn",section1)
		self.proxy.fadeRGB("turn", 0x00FFFFFF, 0.3)


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
