import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from CameraTurret import CameraTurret
from comms import Comms
import numpy as np
import time
import queue


"""
Main camera motion publisher
Responsible for sending camera paths based on current state.
"""
class CameraMotion():
	def __init__(self):
		self.name = "camera"

		self.cameraTurret = CameraTurret()
		self.currentPos = np.array([0, 0]).reshape(2, 1)
		self.currentTarget = np.array([])
		self.gunPath = np.array([])

		self.Comms = Comms()
		self.Comms.add_subscriber_port('127.0.0.1', '3000', 'cState')
		self.Comms.add_publisher_port('127.0.0.1', '3002', 'cameraPath')
		self.Comms.add_subscriber_port('127.0.0.1', '3003', 'gunPath')
		self.Comms.add_subscriber_port('127.0.0.1', '3004', 'targetPos')

	def sendPath(self, path):
		self.Comms.define_and_send(self.name, 'cameraPath', path)

	def receive(self):
		try:
			self.currentTarget = self.Comms.get('targetPos').payload
		except queue.Empty:
			pass

		try:
			self.currentPos = self.Comms.get('cState').payload
			#print("new state: ", self.currentPos)
		except queue.Empty:
			pass

		try:
			self.gunPath = self.Comms.get('gunPath').payload
		except queue.Empty:
			pass

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

	def runS(self):
		sweepPath, duration = self.getFullSweep()
		sweepInit = np.array(sweepPath[1:3, 0]).reshape(2, 1)
		print(sweepInit)
		self.sendPath(sweepPath)

		startTime = time.time()
		#print(path, duration)

		# self.currentPos is incorrect, fixes should happen in hardware_interface
		while(True):
			self.receive()

			if self.currentTarget.size:
				targetPos = self.cameraTurret.inverseKin(self.currentTarget)
				targetPath = self.cameraTurret.scurvePath(self.currentPos, targetPos, 10, 3, 0.1)
				#print(self.currentPos)
				self.sendPath(targetPath)

				# Hardcoded to duration of scurve * 2, may need to switch to when gun shoots (another connection)
				time.sleep(6)
				self.currentTarget = np.array([])
			else:
				if time.time() - startTime >= duration*1.1:
					# Destination is start of sweep path, hardcoded to (-np.pi/4, -np.pi/4)
					# because sweep path init is hardcoded to this point
					self.sendPath(self.cameraTurret.scurvePath(self.currentPos, sweepInit, 10, 2, 0.1))
					# Hardcoded to duration of scurve * 2
					time.sleep(4)
					self.sendPath(sweepPath)
					startTime = time.time()

				# Estimate of how long simulation actually takes vs expected duration
				# Need to test to get accurate scale
				# Also add conditional to see if target is found before duration is over

			time.sleep(0.02)

	def runR(self):
		pass

	def run(self, is_sim):
		if is_sim:
			self.runS()
		else:
			self.runR()