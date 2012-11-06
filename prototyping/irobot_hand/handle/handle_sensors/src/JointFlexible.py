#
# Base class for flexible sensor
#
# Yaroslav Tenzer, May 2012

import rospy
from handle_msgs.msg import HandleSensors
from handle_msgs.msg import Finger

# This is an example that uses pygame.draw.rect:
import os, sys
import random
import time
import numpy as np

#from pygame.locals import *

DEBUG=False

LOWPASS = 0.1

class JointFlexible():
	def __init__(self):
		self.dataLength = 4
		self.dataProximal = np.array([0 for n in range(4)])
		self.dataDistal = np.array([0 for n in range(4)])
		self.display = None		# this is to save the display module

		self.PROXIMAL_flex=0
		self.PROXIMAL_twist=1
		self.DISTAL_flex=2
		self.DISTAL_twist=3

		self.ratios = np.array([0.0,0.0,0.0,0.0])

		self.dataCalibration=0
		self.flagCalibrated=False

		self.messageAngle=Finger()

	def set_values(self,val):
		self.dataProximal = np.array(val.proximal)
		self.dataDistal = np.array(val.distal)

	def calibrate(self, dataCalibrationNew):
		self.dataCalibration=dataCalibrationNew['distalJointAngle']
		self.flagCalibrated=True

	# this method returns the calibrated parameters
	def get_converted_values(self):
		if (self.flagCalibrated):
			# flexion / twist rate at both ends of joint
			raw = np.append(self.dataProximal,self.dataDistal)
			ratios = [0.0, 0.0, 0.0, 0.0]
			flex =  [0.0, 0.0]
			twist = [0.0, 0.0]
			for i in range(4):
				ratios[i] = np.polyval([self.dataCalibration[i]['a'], self.dataCalibration[i]['b']], (raw[2*i] - raw[2*i+1]) / (raw[2*i] + raw[2*i+1]))
				# if stuff is messed up, fail more gracefully
				if np.abs(ratios[i]) > np.pi / 3:
					ratios[i] = np.sign(ratios[i])*np.pi/3
				if not np.isfinite(ratios[i]): 
					ratios[i] = 0
					if (DEBUG): print "ERROR: No calibration was applied to :", self.__class__.__name__
				self.ratios[i] = LOWPASS * ratios[i] + (1 - LOWPASS) * self.ratios[i]
			for i in range(2):
				flex[i] = self.dataCalibration[i]['kflex']*(self.ratios[2*i] + self.ratios[2*i + 1])
				twist[i] = self.dataCalibration[i]['ktwist']*(self.ratios[2*i] - self.ratios[2*i + 1])		
			return np.array([flex[0], twist[0], flex[1], twist[1]])
		else:
			if (DEBUG): print "ERROR: No calibration was applied to :", self.__class__.__name__
			return [0,0,0,0]

	def get_angle_message(self):
		tmp=self.get_converted_values()
		self.messageAngle.distal=[tmp[0], tmp[1]]
		self.messageAngle.proximal=[tmp[2], tmp[3]]
		return self.messageAngle

	@staticmethod
	def flextwist_to_steps(flextwist):
		steps = 10
		flex = np.linspace(flextwist[0], flextwist[2], steps) / steps
		twist = np.linspace(flextwist[1], flextwist[3], steps) / steps
		return [flex,twist]
