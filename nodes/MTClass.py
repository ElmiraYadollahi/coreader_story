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
		
		self.newP = Pose()
		self.matAB = [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1]
		self.space  = motion.FRAME_ROBOT
		self.useSensorValues  = True
		self.motionProxy = mProxy
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


	def calculateTrajectory(self, effector, mainPoints, wordCount, lineNum=1):
		""" Calculate the main trajectory for the robot to point at

		"""
			

		""" Calculate the correction values to be added to the original points for increasing the pointing accuracy"""
		trajComplete = []
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