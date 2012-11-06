#!/usr/bin/env python

import roslib; roslib.load_manifest('handle_sensors')
import rospy
import sys
import numpy as np

from yaml import dump

import ConfigUtilities
from handle_msgs.msg import HandleSensors
import matplotlib.pyplot as plt

SAMPLES_EACH = 10
SAMPLES_TOTAL = 10

class AveragedBuffer:
	def __init__(self, datasize):
		rospy.Subscriber('/handle/sensors/raw', HandleSensors, self.callback)
		self.buffer = np.zeros([SAMPLES_EACH,datasize])
		self.datasize = datasize
		self.i = 0
		self.record = False
	
	def callback(self, new_data):
		self.buffer[self.i] = np.array(new_data)
		if self.i + 1 >= len(self.buffer):
			self.record = False
		self.i += 1
	
	def get_sample(self, index):
		self.index = index
		self.i = 0
		self.record = True
		while self.record and not rospy.is_shutdown():
			rospy.sleep(0.01)
		return np.mean(self.buffer, 0)

class JointBuffer(AveragedBuffer):
	def callback(self, data):
		if self.record:
			new_data = data.distalJointAngle[self.index].proximal
			new_data += data.distalJointAngle[self.index].distal
			AveragedBuffer.callback(self, new_data)

class AccelBuffer(AveragedBuffer):
	def callback(self, data):
		if self.record:
			new_data = [data.fingerAcceleration[self.index].x, data.fingerAcceleration[self.index].y, data.fingerAcceleration[self.index].z]
			AveragedBuffer.callback(self, new_data)

def get_ratios(raw):
	ratios = np.zeros([4])
	for i in range(4):
		ratios[i] = (raw[2*i]	 - raw[2*i+1]) / (raw[2*i] + raw[2*i+1])
	return ratios

def get_flextwist(reference_axis, current_axis):
	#if np.sqrt(np.sum(reference_axis**2)) == 0: # testing situation -- when no accel data
	#	reference_axis = np.array([0,0,1])
	#	current_axis = np.array([0,1,1])
	
	reference = reference_axis / np.sqrt(np.sum(reference_axis**2))
	current = current_axis / np.sqrt(np.sum(current_axis**2))
	
	flex = np.arctan2(current[2], current[1]) - np.arctan2(reference[2], reference[1])
	twist = 0#np.arcsin(proj[0])
	return flex, twist

## Do stuff ##
joint_sensor = JointBuffer(8)
accel_sensor = AccelBuffer(3)
rospy.init_node('calibrate')

#finger_index = int(sys.argv[1])
finger_index = int(raw_input('please enter the finger index you want to calibrate\n'))

reference_axis = accel_sensor.get_sample(finger_index)

print "make sure finger surface is pointed up"

ratios = np.zeros([4, SAMPLES_TOTAL])
flex = np.zeros([SAMPLES_TOTAL])
twist = np.zeros([SAMPLES_TOTAL])

for i in range(SAMPLES_TOTAL):
	print i
	rospy.sleep(1)
	#raw_input('please hit enter to capture joint angle values\n')
	
	new_axis = accel_sensor.get_sample(finger_index)
	
	flex[i], twist[i] = get_flextwist(reference_axis, new_axis)
	ratios[0:4,i] = get_ratios(joint_sensor.get_sample(finger_index))
	print 180 / np.pi * flex[i]

kflex = {0:'', 1:'', 2:'', 3:''}
kk = range(4)

for i in range(4):
	#print ratios[i,:]
	kfx = np.polyfit(ratios[i,:], flex, 1).tolist()
	#ktwist[i] = np.polyfit(ratios[i,:], twist, 1).tolist()
	kk[i] = kfx
	kflex[i] = {'a':kfx[0], 'b':kfx[1], 'kflex':0.5, 'ktwist':-0.5}
	#ktwist[i] ={'a':ktw[0], 'b':ktw[1]}
 
for i in range(4):
	print '    %i :'%i, kflex[i]
#print ktwist

#print ratios
#print flex
#print kflex
#print ktwist
#print dump(kflex, default_flow_style=False)
#print dump(ktwist, default_flow_style=False)
#print flex 

plt.figure(2).show()
flex = flex * 180 / np.pi
plt.plot(flex, np.polyval(kk[0], ratios[0,:]), flex, np.polyval(kk[1], ratios[1,:]), flex, np.polyval(kk[2], ratios[2,:]), flex, np.polyval(kk[3], ratios[3,:]), marker='o')
plt.figure(2).show()
raw_input()
