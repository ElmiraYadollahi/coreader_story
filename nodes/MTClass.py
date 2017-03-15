import codecs
import time
import re
import random

import motion
import almath
import math
from std_msgs.msg import Int64, Int64MultiArray, String
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
from numpy.linalg import inv
#global trajComplete
#trajComplete = []

class TRANSFORMATION:
	""" MATRIC TRANSFORMATION """

	def __init__(self,  mProxy):
		
		self.matAB = [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1]
		self.space  = motion.FRAME_ROBOT
		self.useSensorValues  = True
		self.motionProxy = mProxy
		self.pointsMatrix = []
		#self.trajcomplete = trajcompl

	def transformMatrix(self, name):
		""" A is the coordinate system recieved from TF
			B is the coordinate system only rotated
			C is the coordinate system rotate dand moved to FRAME_BODY
		"""
		
		
		camBotTransform = self.motionProxy.getTransform(name, self.space, self.useSensorValues)

		invAB = np.matrix(np.reshape(self.matAB, (4, 4)))

		matBC = np.matrix(np.reshape(camBotTransform, (4, 4)))
		invBC = inv(np.matrix(matBC))
		
		matAC = np.dot(matBC, invAB)

		return matAC


	def transformPoint(self, point, name):
		"""

		"""

		tranAC = self.transformMatrix(name)

		oldPoint = np.matrix(point)
		newPoint = np.dot(tranAC, oldPoint)

		return newPoint


	def lineFunc(self, PA, PB, t):
		""" line function created using the position of tags
			PA: point A
			PB: point B
			t: variable to find points on the line function

		"""
		newP = Pose()
		newP.position.x = PA.position.x + (PB.position.x - PA.position.x)* t
		newP.position.y = PA.position.y + (PB.position.y - PA.position.y)* t
		newP.position.z = PA.position.z + (PB.position.z - PA.position.z)* t
		
		return newP


	def calculateEachWordPosition(self, PA, PB, effector, LiWoCount, lineDistanceCoef):
		"""

		"""
		print "PA"
		#print PA
		print "PB"
		#print PB
		print "LiWoCount"
		print LiWoCount
		print "lineDistanceCoef"
		print lineDistanceCoef
		trajComplete2 = []
		for i in range(len(LiWoCount)):
			mainPoints = []
			for j in range(LiWoCount[i]):
				""" Based on the number of words, calcultae the positions for the robot to point at"""
				const = (j + 1) / float (1 + LiWoCount[i]) 
				newP = self.lineFunc(PA, PB, const)
				midP = [newP.position.x, newP.position.y, newP.position.z, newP.orientation.x, newP.orientation.y, newP.orientation.z]
				mainPoints.append(midP)

			self.pointsMatrix.append(mainPoints)
			print "mainPointsMatrix"
			print self.pointsMatrix

			trajComplete = []
			trajComplete = self.calculateTrajectory(effector, mainPoints, LiWoCount[i], i+1, lineDistanceCoef)

			
			print "trajComplete"
			print LiWoCount[i]

			print "len trajComplete"
			print len(trajComplete)
			trajComplete2.append([trajComplete])


		return trajComplete2


	def calculateWordBoxes(locCardPoint):
		"""

		"""
		locWord = []
		for i in range(len(self.pointsMatrix)):
			for j in range(self.pointsMatrixt[i]):
				if (locCardPoint.position.x < self.pointsMatrixt[i][1]+ dx) and (locCardPoint.position.x > self.pointsMatrixt[i][1]-dx):
					if (locCardPoint.position.y < self.pointsMatrixt[i][2]+ dx) and (locCardPoint.position.y > self.pointsMatrixt[i][2]-dx):
						locWord = [i, j]
						print "locatedWord"
						print locWord


		return locatedWord



 





	def calculateTrajectory(self, effector, mainPoints, wordCount, lineNum=1, lineDistanceCoef=0.03):
		""" Calculate the main trajectory for the robot to point at

		"""
			

		""" Calculate the correction values to be added to the original points for increasing the pointing accuracy"""
		print "lineDistanceCoef"
		print lineDistanceCoef
		trajComplete = []
		#if len(lineDistanceCoef) >= 1:
		dx_corr = 0.1 + (lineDistanceCoef[lineNum-1] )
		#else:
			#dx_corr = 0.1 
		dz_corr = 0.1 + (lineDistanceCoef[lineNum-1] )


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
		
		print "trajComplete[i]"
		
		for i in range(len(trajComplete)):
				print trajComplete[i],
				print ''

		return trajComplete


	def clearCoordinate(self):
		self.pointsMatrix = []
