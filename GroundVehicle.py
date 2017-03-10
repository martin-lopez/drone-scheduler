# GroundVehicle.py
# Assignment 3

# Author: Alessandro Lira

import math, threading, sys, random

from IllegalArgumentException import *
from DeadlockTesterException import *
from DeadlockTester import *

class GroundVehicle(threading.Thread):
	# static class members (call using: Class.<member>)
	totalNumVehicles = 0
	# A simple re-entrant lock suffices here becasue this
	# lock is only used for gated access to the resource
	gv_class_lock = threading.RLock()

	def __init__(self,pos,dx,dy,dtheta):
		threading.Thread.__init__(self)

		# check for legal arguments
		if len(pos) != 3:
			raise IllegalArgumentException("Incorrect size Pos array")

		# initiate Ground Vehicle
		self.__x = pos[0]
		self.__y = pos[1]
		self.__theta = pos[2]

		self.__dx = dx
		self.__dy = dy
		self.__dtheta = dtheta

		# intrinsic "self" lock
		self.gv_lock = threading.RLock()

		self.clampPosition()
		self.clampVelocity()

		self.__sim = None

		self.__lastCheckedSec = 0
		self.__lastCheckedMSec = 0

		# synchronize incrementation on all GV objects
		GroundVehicle.gv_class_lock.acquire()
		self.__vehicleID = GroundVehicle.totalNumVehicles
		GroundVehicle.totalNumVehicles += 1
		GroundVehicle.gv_class_lock.release()

	def addSimulator(self, sim):
		self.__sim = sim

	def getVehicleID(self):
		return self.__vehicleID

	def clampPosition(self): # clamps position values (useful immediately after values have been changed)

		# clamp X&Y values if necessary
		self.__x = min(max(self.__x,0),100)
		self.__y = min(max(self.__y,0),100)
		
		# wrap Theta angle value if necessary
		self.__theta = min(max(self.__theta, -math.pi), math.pi)
		if (math.fabs(self.__theta-math.pi) < 1e-6):
			self.__theta = -math.pi

	def clampVelocity(self): # clamps velocity values (useful immediately after values have been changed)
		
		speed = math.sqrt(self.__dx*self.__dx + self.__dy*self.__dy)

		# clamp dx & dy values if necessary
		if (speed < 5):
			if speed == 0: speed = 1 # condition to catch div/0 error
			self.__dx = self.__dx*(5/speed)
			self.__dy = self.__dy*(5/speed)
		elif (speed > 10):
			self.__dx = self.__dx*(10/speed)
			self.__dy = self.__dy*(10/speed)
	

		# clamp dtheta value (rotational velocity) if necessary
		if (self.__dtheta < -math.pi/4):
			self.__dtheta = -math.pi/4
		elif (self.__dtheta > math.pi/4):
			self.__dtheta = math.pi/4

		self.__dtheta = min(max(self.__dtheta,-math.pi/4),math.pi/4)

	def checkIfNoLock(self):
		if self.__sim is None:
			return False
		try:
			return DeadlockTester.testLock(self,self.__sim)
		except DeadlockTesterException, e:
			print e
			sys.exit()

		return False

	def getPosition(self):
		pos = []
		if self.checkIfNoLock():
			self.gv_lock.acquire() # start critical region
			pos = [self.__x, self.__y, self.__theta]
			self.gv_lock.release() # end critical region
			return pos

		return pos

	def getVelocity(self):
		vel = []
		if self.checkIfNoLock():
			self.gv_lock.acquire() # start critical region
			vel = [self.__dx, self.__dy, self.__dtheta]
			self.gv_lock.release() # end critical region
			return vel

		return vel

	def setPosition(self,pos):
		if len(pos) != 3:
			raise IllegalArgumentException("new Pos array must be of length 3")

		self.gv_lock.acquire() # start critical region
		self.__x = pos[0]
		self.__y = pos[1]
		self.__theta = pos[2]

		self.clampPosition()
		self.gv_lock.release() # end critical region

	def setVelocity(self,vel):
		if len(vel) != 3:
			raise IllegalArgumentException("new Vel array must be of length 3")

		self.gv_lock.acquire() # start critical region
		self.__dx = vel[0]
		self.__dy = vel[1]
		self.__dtheta = vel[2]

		self.clampVelocity()
		self.gv_lock.release() # end critical region

	def controlVehicle(self,c):
		speed = c.getSpeed()
		dtheta = c.getRotVel()

		self.gv_lock.acquire() # start critical region
		# modify internal dx and dy values
		self.__dx = speed*math.cos(self.__theta)
		self.__dy = speed*math.sin(self.__theta)

		# change dtheta to supplied rotational velocity
		self.__dtheta = c.getRotVel()

		self.clampVelocity()
		self.gv_lock.release() # end critical region

	@staticmethod
	def normalizeAngle(theta):

		rtheta = math.fmod(theta - math.pi, 2*math.pi)
		if rtheta < 0:
			rtheta += 2*math.pi

		rtheta -= math.pi

		return rtheta

	def advance(self,sec,msec):
		t = sec + msec*1e-3;

		self.gv_lock.acquire() # start critical region

		noiseScaleFactor = 200 # adjust to determine noise magnitude

		errc = random.gauss(0,10)/noiseScaleFactor
		errd = random.gauss(0,20)/noiseScaleFactor

		self.__x = self.__x + self.__dx*t + errd*math.cos(self.__theta) - errc*math.sin(self.__theta)
		self.__y  = self.__y + self.__dy*t + errd*math.sin(self.__theta) + errc*math.cos(self.__theta)
		self.__theta = self.normalizeAngle(self.__theta + self.__dtheta*t)

		s = math.sqrt(self.__dx*self.__dx + self.__dy*self.__dy)
		self.__dx = s*math.cos(self.__theta)
		self.__dy = s*math.sin(self.__theta)
		self.__dtheta = self.__dtheta

		self.clampPosition()
		self.clampVelocity()

		self.gv_lock.release() # end critical region

	def advanceNoiseFree(self,sec,msec):
		t = sec + msec*1e-3;

		# -- Linear approximation --
		#	see previous solution for linear approximation model

		# -- Curve model --
		
		self.gv_lock.acquire() # start critical region

		# Assuming that  dx,  dy, and  dtheta was set beforehand by controlVehicle()
		s = math.sqrt(self.__dx*self.__dx + self.__dy*self.__dy)

		if (abs(self.__dtheta) > 1e-3): # The following model is not well defined when dtheta = 0
			# Circle center and radius
			r = s/self.__dtheta

			xc = self.__x - r * math.sin(self.__theta)
			yc = self.__y + r * math.cos(self.__theta)

			self.__theta = self.__theta + self.__dtheta*t

			rtheta = ((self.__theta-math.pi) % (2*math.pi))
			if (rtheta < 0): # Note that % in java is remainder, not modulo.
				rtheta += 2*math.pi

			self.__theta = rtheta - math.pi;

			# Update Values
			self.__x = xc + r * math.sin(self.__theta)
			self.__y = yc - r * math.cos(self.__theta)
			self.__dx = s * math.cos(self.__theta)
			self.__dy = s * math.sin(self.__theta)
		
		else:	# Straight motion. No change in theta.
			self.__x = self.__x + self.__dx*t
			self.__y = self.__y + self.__dy*t

		self.clampPosition()
		self.clampVelocity()

		self.gv_lock.release() # end critical region

	def run(self):

		print "GV: %i thread started" % self.__vehicleID

		currentSec = 0
		currentMSec = 0

		while(currentSec < 200):
			if not self.__sim.getDisplayClient().isConnected():
				break

			# Start Condition Critical Region
			self.__sim.simulator_lock.acquire()

			currentSec = self.__sim.getCurrentSec()
			currentMSec = self.__sim.getCurrentMSec()

			while 1:

				# check if time has changed since last update
				if not (self.__lastCheckedSec == currentSec and 
						self.__lastCheckedMSec == currentMSec):

					currentSec = self.__sim.getCurrentSec()
					currentMSec = self.__sim.getCurrentMSec()
					break # exit "while 1" loop once current time is updated

				# wait until Simulator notifies all threads that time has passed
				self.__sim.simulator_lock.wait() 
												 
				currentSec = self.__sim.getCurrentSec()
				currentMSec = self.__sim.getCurrentMSec()

			#self.advanceNoiseFree(0,10)
			self.advance(0,10)

			# End Condition Critical Region
			self.__sim.simulator_lock.release()

			# update the time of the last control
			self.__lastCheckedSec = currentSec
			self.__lastCheckedMSec = currentMSec

			# start critical region
			self.__sim.simulator_lock.acquire()

			# decrement number of controllers left to update
			if self.__sim.numVehicleToUpdate > 0:
				self.__sim.numVehicleToUpdate -= 1
			self.__sim.simulator_lock.notify_all()

			# end critical region
			self.__sim.simulator_lock.release() 


	# The following three methods (getVehicleLock, compareId,
	# reverseCompareId) is is needed when you try resource-hierarchy
	# solution for deadlock prevention

	# def compareID(self,gv):
	# 	if self.getVehicleID < gv.getVehicleID:
	# 		return -1
	# 	elif self.getVehicleID < gv.getVehicleID:
	# 		return 1
	# 	else:
	# 		return 0

	# def reverseCompareID(self,gv):
	# 	return -(self.compareID(gv))


