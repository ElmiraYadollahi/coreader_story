#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int64, Int64MultiArray, String
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np



tag_states = { 1, 2 , 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 
				17, 18, 19, 20, 21, 22, 23, 
				200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 
				210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
				220, 221, 222, 223, 224, 225, 226, 227, 228, 229,
				250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 
				260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 
				270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 
				280,
				2222, 4444}
card_tag_states = { 38, 39, 40}
tag_pairs = [	[0, 1], 
				[2, 3], [4, 5], 
				[6, 7], [8, 9],
				[10, 11], [12, 13],
				[14, 15], [16, 17],
				[18, 19], [20, 21],
				[22, 23],
				[250, 251], [252, 253], [200, 201, 202, 203], [204, 205, 206, 207],
				[208, 209, 210, 211], [212, 213, 214, 215], [216, 217, 218, 219],
				[260, 261], [262, 263], [264, 265], [266, 267], [268, 269],
				[270, 271], [272, 273], [274, 275], [276, 277], [278, 279],
				[220, 221], [230, 231],
				[240, 241], [250, 251]
			]

tag_number_dict = {	'[0, 1]' : False,
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
				'250' : 2,
				'251' : 2,
				'252' : 2,
				'253' : 2,
				'200' : 4,
				'201' : 4,
				'202' : 4,
				'203' : 4,
				'204' : 4,
				'205' : 4,
				'206' : 4,
				'207' : 4,
				'208' : 4,
				'209' : 4,
				'210' : 4,
				'211' : 4,
				'212' : 4,
				'213' : 4,
				'214' : 4,
				'215' : 4,
				'216' : 4,
				'217' : 4,
				'218' : 4,
				'219' : 4,
				'[252, 253]' : False,
				'[254, 255]' : False,
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
				'[278, 279]' : False,
			 }

global avail_tags
avail_tags = []
global avail_pair
avail_pair = []
global tag_marker_array
tag_marker_array = []
global tag_cog_array
tag_cog_array = []
global tagCount
tagCount = 2


class TagsCOG():

	def __init__(self):
		rospy.init_node("ar_tags_poses")

		# Read in an optional list of valid tag ids
		self.tag_ids = rospy.get_param('~tag_ids', None)

		# Publish the COG on the /target_pose topic as a PoseStamped message
		self.tag_pub = rospy.Publisher("target_pose", PoseArray, queue_size=5)
		self.card_pose_pub = rospy.Publisher("card_pose", Pose, queue_size=5)

		rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)

		rospy.loginfo("Publishing combined tag COG on topic /target_pose...")
		print "test point 1"


	def get_tags(self, msg):

		# Initialize the COG as a PoseStamped message
		tag_cog = PoseStamped()
		tag_cog_array = PoseArray()
		tag_cog_array4 = PoseArray()
		middle_pose = Pose()

		# Get the number of markers
		n = len(msg.markers)

		global avail_pair
		global avail_tags
		global counter
		global tag_marker_array
		global tagCount

		# If no markers detected, just retutn 
		if n == 0:
			return

		#print "test point 2"

		#tag_cog_array.poses = []
		# Iterate through the tags and sum the x, y and z coordinates
		for tag in msg.markers:

			# Skip any tags that are not in our list
			if self.tag_ids is not None and not tag.id in self.tag_ids:
				continue

			#calculate_distance(tag.pose.pose.position.x, tag.pose.pose.position.y)
			if tag.id in tag_states:
				print "in it"
				if tag.id not in avail_tags:
					avail_tags.append(tag.id)
					#tagCount = tag_number_dict[String(avail_tags)]
					print "String(tag.id)"
					x = str(tag.id)
					print x
					if str(tag.id) in tag_number_dict:
						tagCount = tag_number_dict[str(tag.id)]
						print "tagCount"
						print tagCount
				print "avail_tags"
				print sorted(avail_tags)
				#tag_cog_array.poses.append(middle_pose)



				if tag.id not in avail_pair:
					avail_pair.append(tag.id)
					tag_marker_array.append(tag)
					tag_cog_array.poses.append(middle_pose)
					#print "loop"
					#print len(tag_cog_array.poses)

				#tag_state_pub = rospy.Publisher("id_state", Int64, queue_size=5)
				#tag_state_pub.publish(tag.id)

				if len(avail_pair) == (tagCount+1):
					avail_pair = []
					tag_cog_array.poses = []
					tag_marker_array = []



				avail_pair = sorted(avail_pair)
				print "avail_pair "
				print avail_pair 
				if avail_pair in tag_pairs:
					#print "tag_marker_array"
					#print tag_marker_array

					#print len(tag_marker_array)

					"""for eachpair in tag_marker_array:
						middle_pose.position.x = eachpair.pose.pose.position.x
						middle_pose.position.y = eachpair.pose.pose.position.y
						middle_pose.position.z = eachpair.pose.pose.position.z
						middle_pose.orientation.x = eachpair.pose.pose.orientation.x
						middle_pose.orientation.y = eachpair.pose.pose.orientation.y
						middle_pose.orientation.z = eachpair.pose.pose.orientation.z
						middle_pose.orientation.w = eachpair.pose.pose.orientation.w

						#tag_cog_array.poses.append(middle_pose)
						tag_cog_array.poses = addMatrix(tag_cog_array.poses, middle_pose)
						tag_cog_array.header.stamp = rospy.Time.now()
						tag_cog_array.header.frame_id = msg.markers[0].header.frame_id
						print eachpair.id

					i = 1
					tag_cog_array.poses[i].position.x = tag_marker_array[i].pose.pose.position.x
					tag_cog_array.poses[i].position.y = tag_marker_array[i].pose.pose.position.y
					tag_cog_array.poses[i].position.z = tag_marker_array[i].pose.pose.position.z
					tag_cog_array.poses[i].orientation.x = tag_marker_array[i].pose.pose.orientation.x
					tag_cog_array.poses[i].orientation.y = tag_marker_array[i].pose.pose.orientation.y
					tag_cog_array.poses[i].orientation.z = tag_marker_array[i].pose.pose.orientation.z
					tag_cog_array.poses[i].orientation.w = tag_marker_array[i].pose.pose.orientation.w
					#tag_cog_array.poses.append(middle_pose)
					#tag_cog_array.poses[i] = middle_pose"""
					if tagCount==2:
						i = 0
						mid = Pose()
						middle_pose.position.x = tag_marker_array[i].pose.pose.position.x
						middle_pose.position.y = tag_marker_array[i].pose.pose.position.y
						middle_pose.position.z = tag_marker_array[i].pose.pose.position.z
						middle_pose.orientation.x = tag_marker_array[i].pose.pose.orientation.x
						middle_pose.orientation.y = tag_marker_array[i].pose.pose.orientation.y
						middle_pose.orientation.z = tag_marker_array[i].pose.pose.orientation.z
						middle_pose.orientation.w = tag_marker_array[i].id

						i = 1
						mid.position.x = tag_marker_array[i].pose.pose.position.x
						mid.position.y = tag_marker_array[i].pose.pose.position.y
						mid.position.z = tag_marker_array[i].pose.pose.position.z
						mid.orientation.x = tag_marker_array[i].pose.pose.orientation.x
						mid.orientation.y = tag_marker_array[i].pose.pose.orientation.y
						mid.orientation.z = tag_marker_array[i].pose.pose.orientation.z
						mid.orientation.w = tag_marker_array[i].pose.pose.orientation.w

						if len(tag_cog_array.poses) == 1:
							tag_cog_array.poses.append(mid)

						print "length"
						print len(tag_cog_array.poses)

						if len(tag_cog_array.poses) == 2:

							tag_cog_array.poses[0] = middle_pose
							tag_cog_array.poses[1] = mid

							tag_cog_array.header.stamp = rospy.Time.now()
							tag_cog_array.header.frame_id = msg.markers[0].header.frame_id
								#print eachpair.id

							A = np.array((tag_marker_array[0].pose.pose.position.x, tag_marker_array[0].pose.pose.position.y, tag_marker_array[0].pose.pose.position.z))
							B = np.array((tag_marker_array[1].pose.pose.position.x, tag_marker_array[1].pose.pose.position.y, tag_marker_array[1].pose.pose.position.z))

							A1 = np.array((tag_cog_array.poses[0].position.x, tag_cog_array.poses[0].position.y, tag_cog_array.poses[0].position.z))
							B1 = np.array((tag_cog_array.poses[1].position.x, tag_cog_array.poses[1].position.y, tag_cog_array.poses[1].position.z))

							print tag_cog_array

							dist_AB = np.linalg.norm(A-B)
							print dist_AB

							dist_A1B1 = np.linalg.norm(A1-B1)
							print dist_A1B1

							story_pub.publish(str(avail_pair))
							self.tag_pub.publish(tag_cog_array)
						print avail_pair
						avail_pair = []
						avail_tags = []
						#print tag_cog_array
						tag_cog_array.poses = []
						tag_marker_array = []
						#tag_cog_array = []

						
					elif tagCount == 4:
						for j in range(tagCount):
							if tag_marker_array[j].id == avail_pair[0]:
								
								mid0 = Pose()
								mid0.position.x = tag_marker_array[j].pose.pose.position.x
								mid0.position.y = tag_marker_array[j].pose.pose.position.y
								mid0.position.z = tag_marker_array[j].pose.pose.position.z
								mid0.orientation.x = tag_marker_array[j].pose.pose.orientation.x
								mid0.orientation.y = tag_marker_array[j].pose.pose.orientation.y
								mid0.orientation.z = tag_marker_array[j].pose.pose.orientation.z
								mid0.orientation.w = tag_marker_array[j].id
								tag_cog_array4.poses.append(mid0)
						
						for j in range(tagCount):	
							if tag_marker_array[j].id == avail_pair[1]:
								
								mid1 = Pose()
								mid1.position.x = tag_marker_array[j].pose.pose.position.x
								mid1.position.y = tag_marker_array[j].pose.pose.position.y
								mid1.position.z = tag_marker_array[j].pose.pose.position.z
								mid1.orientation.x = tag_marker_array[j].pose.pose.orientation.x
								mid1.orientation.y = tag_marker_array[j].pose.pose.orientation.y
								mid1.orientation.z = tag_marker_array[j].pose.pose.orientation.z
								mid1.orientation.w = tag_marker_array[j].id
								tag_cog_array4.poses.append(mid1)
						for j in range(tagCount):

							if tag_marker_array[j].id == avail_pair[2]:
								
								mid2 = Pose()
								mid2.position.x = tag_marker_array[j].pose.pose.position.x
								mid2.position.y = tag_marker_array[j].pose.pose.position.y
								mid2.position.z = tag_marker_array[j].pose.pose.position.z
								mid2.orientation.x = tag_marker_array[j].pose.pose.orientation.x
								mid2.orientation.y = tag_marker_array[j].pose.pose.orientation.y
								mid2.orientation.z = tag_marker_array[j].pose.pose.orientation.z
								mid2.orientation.w = tag_marker_array[j].id
								tag_cog_array4.poses.append(mid2)
						
						for j in range(tagCount):
							if tag_marker_array[j].id == avail_pair[3]:
								
								mid3 = Pose()
								mid3.position.x = tag_marker_array[j].pose.pose.position.x
								mid3.position.y = tag_marker_array[j].pose.pose.position.y
								mid3.position.z = tag_marker_array[j].pose.pose.position.z
								mid3.orientation.x = tag_marker_array[j].pose.pose.orientation.x
								mid3.orientation.y = tag_marker_array[j].pose.pose.orientation.y
								mid3.orientation.z = tag_marker_array[j].pose.pose.orientation.z
								mid3.orientation.w = tag_marker_array[j].id
								tag_cog_array4.poses.append(mid3)								
						#if len(tag_cog_array.poses) == 1:
							#tag_cog_array.poses.append(mid)

						print "length"
						print len(tag_cog_array4.poses)

						if len(tag_cog_array4.poses) == 4:

							#tag_cog_array4.poses[0] = middle_pose
							#tag_cog_array4.poses[1] = mid

							tag_cog_array4.header.stamp = rospy.Time.now()
							tag_cog_array4.header.frame_id = msg.markers[0].header.frame_id
								#print ea4chpair.id

							A = np.array((tag_marker_array[0].pose.pose.position.x, tag_marker_array[0].pose.pose.position.y, tag_marker_array[0].pose.pose.position.z))
							B = np.array((tag_marker_array[1].pose.pose.position.x, tag_marker_array[1].pose.pose.position.y, tag_marker_array[1].pose.pose.position.z))

							A1 = np.array((tag_cog_array4.poses[0].position.x, tag_cog_array4.poses[0].position.y, tag_cog_array4.poses[0].position.z))
							B1 = np.array((tag_cog_array4.poses[1].position.x, tag_cog_array4.poses[1].position.y, tag_cog_array4.poses[1].position.z))

							print tag_cog_array4

							dist_AB = np.linalg.norm(A-B)
							print dist_AB

							dist_A1B1 = np.linalg.norm(A1-B1)
							print dist_A1B1

							print "tag_cog_array"
							print tag_cog_array4

							story_pub.publish(str(avail_pair))
							self.tag_pub.publish(tag_cog_array4)
						print avail_pair
						avail_pair = []
						avail_tags = []
						#print tag_cog_array
						#tag_cog_array = []
						tag_cog_array4.poses = []
						tag_marker_array = []

				
			if tag.id in card_tag_states:
				card_id = tag.id
				print card_id
				card_pub.publish(str(card_id))

				card_pos = Pose()
				card_pos.position.x = tag.pose.pose.position.x
				card_pos.position.y = tag.pose.pose.position.y
				card_pos.position.z = tag.pose.pose.position.z
				card_pos.orientation.x = tag.pose.pose.orientation.x
				card_pos.orientation.y = tag.pose.pose.orientation.y
				card_pos.orientation.z = tag.pose.pose.orientation.z
				
				self.card_pose_pub.publish(card_pos)

					



			# Compute the COG
			tag_cog.pose.position.x /= n
			tag_cog.pose.position.y /= n
			tag_cog.pose.position.z /= n

			# Give the tag a unit orientation
			tag_cog.pose.orientation.w = 1

			# Add a time stamp and frame_id
			tag_cog.header.stamp = rospy.Time.now()
			tag_cog.header.frame_id = msg.markers[0].header.frame_id
			#tag_cog.header.seq = self.tag_ids

			# Publish the COG
			#self.tag_pub.publish(tag_cog)


	def timer_callback(avail_pair):
		avail_pair = []
		print 'timer called at ' + str(event.current_real)

		return avail_pair
	#def recognize_text(self):

		# Recognize the tag and read associated text
		#if 


	#def calculate_distance(x, y):
		
		# find the tags 

def addMatrix(ori, added):
	#orig = []
	#print "before"
	#print ori
	ori.append(added)
	#print "after"
	#print ori
	return ori

if  __name__ == '__main__':
	story_pub = rospy.Publisher('tag_id_state', String, queue_size=10)
	card_pub = rospy.Publisher('card_id_state', String, queue_size=10)
	try:
		TagsCOG()
		#story()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AR Tag Tracker node terminated.")