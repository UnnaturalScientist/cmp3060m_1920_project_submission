#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
__________________________________________________________________________________________________
Brief Description
__________________________________________________________________________________________________

This mux_select script can be found in the mux package for Ros. This is the exact script, in its
entirity, which is available for manipulation. This is a significant discovery. There was also
some supporting literature to help implement the package properly and to repair any issues with
the mux nodelet. In addition, the yaml found in the book, which is used to pass strings as 
alterantive topics, can be found here: cmp3060m_1920_project/mux_params/mux_config.yaml

(Goebel, 2014, 227-236)

Goebel, R.P., (2014) Ros by Example Vol 2 Indigo. CA: A Pi Robot Production.

"""

import rospy
from geometry_msgs.msg import Twist
from topic_tools.srv import MuxSelect
import threading

class Decision_Criteria:
	def __init__(self):
		rospy.init_node("blended_control")

		# The rate at which to update the input selection
		rate = rospy.get_param('~rate', 5)

		# Convert to a ROS rate
		r = rospy.Rate(rate)

		# Get a lock for updating the selected cmd_vel input
		self.lock = threading.Lock()

		# Set the default input control
		self.move_base = True
		self.joystick = False

		# Track the last input control
		self.last_joystick = self.joystick
		self.last_move_base = self.move_base

		# Subscribe to the individual control topics and set a callback for each
		rospy.Subscriber('joystick_cmd_vel', Twist, self.joystick_cb) # callback for joy
		rospy.Subscriber('move_base_cmd_vel', Twist, self.move_base_cb) # callback for auto

		# Wait for the mux select service
		rospy.loginfo("Waiting for mux select service...")
		rospy.wait_for_service('cmd_vel_mux/select')

		# Create a proxy for the mux select service
		mux_select = rospy.ServiceProxy('cmd_vel_mux/select', MuxSelect) # calls MuxSelect from where?
	   	# is this just something in ROS?

		rospy.loginfo("Connected to mux select service.")

		rospy.loginfo("Ready for input.")

		# Main loop to switch inputs if user move joystick
		while not rospy.is_shutdown(): # so ROS is NOT shutdown...
			if self.joystick and self.joystick != self.last_joystick: # consider past and present
				mux_select('joystick_cmd_vel')
			elif self.move_base and self.move_base != self.last_move_base: # same logic
				mux_select('move_base_cmd_vel')

			self.last_joystick = self.joystick
			self.last_move_base = self.move_base

			r.sleep() # sleep mode

	# If the joystick is moved, get the message here
	def joystick_cb(self, msg): # where is "msg" called to
			self.lock.acquire()
			self.joystick = True # True that the joystick
			self.move_base = False
			self.lock.release() # is lock release the same?

	# If move_base is active, get the message here
	def move_base_cb(self, msg): # so what does the autonomy even relate to?
			self.lock.acquire()
			self.joystick = False
			self.move_base = True
			self.lock.release()
			# do i need a script to reach a set goal?

if __name__ == '__main__':
	Decision_Criteria() # simply calls back the class
	rospy.spin()