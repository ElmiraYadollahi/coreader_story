#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int64, Int64MultiArray, String
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np



tag_states = { 0, 1, 2 , 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 
				17, 18, 19, 20, 21, 22, 23, 2222, 4444}
card_tag_states = { 38, 39, 40}
tag_pairs = [	[0, 1], 
				[2, 3], [4, 5], 
				[6, 7], [8, 9],
				[10, 11], [12, 13],
				[14, 15], [16, 17],
				[18, 19], [20, 21],
				[22, 23],
				[200, 201], [210, 211],
				[220, 221], [230, 231],
				[240, 241], [250, 251]
			]

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
				'[22, 23]' : False,
			 }

global avail_tags
avail_tags = []
global avail_pair
avail_pair = []
global tag_marker_array
tag_marker_array = []
global tag_cog_array
tag_cog_array = []


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
		middle_pose = Pose()

		# Get the number of markers
		n = len(msg.markers)

		global avail_pair
		global avail_tags
		global counter
		global tag_marker_array

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
				avail_tags.append(tag.id)
				#tag_cog_array.poses.append(middle_pose)


				if tag.id not in avail_pair:
					avail_pair.append(tag.id)
					tag_marker_array.append(tag)
					tag_cog_array.poses.append(middle_pose)
					#print "loop"
					#print len(tag_cog_array.poses)

				#tag_state_pub = rospy.Publisher("id_state", Int64, queue_size=5)
				#tag_state_pub.publish(tag.id)

				if len(avail_pair) == 3:
					avail_pair = []
					tag_cog_array.poses = []
					tag_marker_array = []



				avail_pair = sorted(avail_pair)
				if avail_pair in tag_pairs:
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
					#print tag_cog_array
					tag_cog_array.poses = []
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