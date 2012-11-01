#!/usr/bin/env python
#
# tactile sensor display
# 
# subscribes to HandleSensors topic and displays sensor values
#
# based on example http://www.pygame.org/docs/ref/draw.html#pygame.draw.rect
#
# Leif Jentoft, May 2012

import roslib; roslib.load_manifest('sensors')
import rospy
from handle_msgs.msg import HandleSensors

# This is an example that uses pygame.draw.rect:
import os, sys
import random
import pygame
from pygame.locals import *
from numpy import array

class TactileDisplay():
	def __init__(self, size_x=None, size_y=None):
		if size_x == None or size_y == None:
			self.size_x = 2
			self.size_y = 4
		else:
			self.size_x = size_x
			self.size_y = size_y

		pygame.init()

		self.pixel_size = 20
		screen_size = (self.size_x * self.pixel_size, self.size_y * self.pixel_size)
		APPLICATION_x_size = screen_size[0]
		APPLICATION_y_size = screen_size[1]
		self.screen = pygame.display.set_mode((APPLICATION_x_size, APPLICATION_y_size))
		pygame.display.set_caption('TactileSensorDisplay')
		pygame.mouse.set_visible(True)
		#pygame.mouse.set_visible(False)
		black_square_that_is_the_size_of_the_screen = pygame.Surface(self.screen.get_size())
		black_square_that_is_the_size_of_the_screen.fill((0, 0, 0))
		self.screen.blit(black_square_that_is_the_size_of_the_screen, (0, 0))
		pygame.display.flip()
	
	def set_tile(self,x,y,val):
		# x and y are zero-indexed
		rect = (x * self.pixel_size, y * self.pixel_size, self.pixel_size, self.pixel_size)
		pygame.draw.rect(self.screen, ([val])*3, rect)

	def show(self,values):
		# set up as row major for now
		
		for i in range(len(values)):
			if values[i] > 255:
				print 'warning: clipping value'
				values[i] = 255
			if values[i] < 0:
				print 'warning: clipping value'
				values[i] = 255
			y = (i/self.size_x)%self.size_y
			x = i%self.size_x
			self.set_tile(x,y,int(values[i]))

		pygame.display.flip()

# Base Class

class DisplayNode():
	def __init__(self):
		# subscribe to Korg messages, set up publishing channel for controlling hand
		rospy.init_node('TactileDisplay', anonymous=True)
		self.display = TactileDisplay(2,6)

		self.init = [False]
		
		rospy.loginfo(rospy.get_name() + " subscribing to topic 'HandleSensors'")
		rospy.Subscriber("HandleSensors", HandleSensors, self.callback)
		rospy.loginfo(rospy.get_name() + " subscribed")

		rospy.loginfo(rospy.get_name() + " node initialized, awaiting orders")

		rospy.spin()
		
	def callback(self, data):
		if not self.init[0]:
			self.init = array(data.fingerTactile[2].proximal)
		values = 20 * (self.init - array(data.fingerTactile[2].proximal)) + 127
		self.display.show(values)

display = DisplayNode()
