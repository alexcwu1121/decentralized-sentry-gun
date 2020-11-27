import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from CameraTurret import CameraTurret
from comms import Comms
import numpy as np
import time


"""
Main camera motion publisher
Responsible for sending camera paths based on current state.
"""
class CameraMotion():
	def __init__(self):
		self.name = "camera"

		self.cameraTurret = CameraTurret()

		self.Comms = Comms()
		self.Comms.add_publisher_port('127.0.0.1', '3002', 'cameraPath')

	def sendPath(self, path):
		self.Comms.define_and_send(self.name, 'cameraPath', path)

	def getFullSweep(self):
		duration = 5
		# Testing required to find q1_range and q2_range
		# Eventually should be declared as constants
		q_mat = self.cameraTurret.sweepPath(duration, q1_range=(-np.pi/4,np.pi/4), q2_range=(-np.pi/4,np.pi/4))

		# Make duplicate for reverse direction
		q_mat2 = self.cameraTurret.sweepPath(duration, q1_range=(-np.pi/4,np.pi/4), q2_range=(-np.pi/4,np.pi/4))
		q_mat2 = np.flip(q_mat2, 1)[:,1:]
		q_size = int((q_mat2.size)/3)
		# Change timestamps of reverse array
		for i in range(q_size):
			q_mat2[0,i] = duration + (duration - q_mat2[0,i])

		return np.concatenate((q_mat, q_mat2), axis=1), duration*2

	def run(self):
		path, duration = self.getFullSweep()
		#print(path, duration)

		while(True):
			self.sendPath(path)

			# Estimate of how long simulation actually takes vs expected duration
			# Need to test to get accurate scale
			# Also add conditional to see if target is found before duration is over
			time.sleep(duration*2)

