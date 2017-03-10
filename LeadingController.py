# LeadingController.py
# Assignment 3

# Author: Alessandro Lira

import math, threading, sys

from Simulator import *
from GroundVehicle import *
from VehicleController import *
from IllegalArgumentException import *

class LeadingController(VehicleController):
	
	def __init__(self,sim,gv):
		# initializes super class
		super(LeadingController, self).__init__(sim,gv)
		
		self.__gv = gv
		self.__gvList = []

	# Adds a GroundVehicle to the list of chasers considered by this
    # groundVehicle.
	def addFollower(self,gv):
		self.__gvList.append(gv)

	# Return a GroundVehicle at index
	def getFollower(self,index):
		if index<0 or index>(len(self.__gvList)-1):
			return None
		return self.__gvList[index]

	# Returns a control negating the output for the FollowingControler. Added
    # special controls when the GroundVehicle is close to the walls.
	def getControl(self,sec,msec):
		closestGV = self.getClosestVehicle()

		# if no closest vehicle return None
		if closestGV is None:
			return None

		desiredOmega = 0

		chaserPos = closestGV.getPosition() # Shared Resource
		myPos = self.__gv.getPosition() # Shared Resource

		# Attempt to get more than one lock - uncomment below to see exception thrown*/
		
		# leadGV = self.__gv
		# print '---Aquiring Multiple GV-Locks---\n'
		# closestGV.gv_lock.acquire()
		# leadGV.gv_lock.acquire()
		# print '---Both GV-Locks Acquired---\n'
		# # The following call to getPosition() will cause DeadLockTester to
		# # throw an exception, since two different gv-locks have been acquired 
		# # by a single thread. The program execution will hault becasue of this
		# closestGV.getPosition()
		# leadGV.getPosition()
		# leadGV.gv_lock.release()
		# closestGV.gv_lock.release()

	    
		xDiff = chaserPos[0] - myPos[0]
		yDiff = chaserPos[1] - myPos[1]
		targetTheta = math.atan2(yDiff, xDiff)

		desiredOmega = VehicleController.normalizeAngle(targetTheta - myPos[2] + math.pi)

		gain = 5.0
		desiredOmega *= gain
		if desiredOmega > math.pi/4:
			desiredOmega = math.pi/4
		if desiredOmega < -math.pi/4:
			desiredOmega = -math.pi/4

		desiredSpeed = 10

		# Wall cases and corner cases
		a = VehicleController.avoidWalls(myPos)
		if a is not None:
			return a

		c = Control(desiredSpeed, desiredOmega)
		return c

	def getClosestVehicle(self):

		leaderPos = self.__gv.getPosition()
		closestDist = sys.float_info.max
		closestGV = None
		for gv in self.__gvList:
			followerPos = gv.getPosition()
			xDist = leaderPos[0] - followerPos[0]
			yDist = leaderPos[1] - followerPos[1]
			totalDist = math.hypot(xDist, yDist)

			if totalDist < closestDist:
				closestDist = totalDist
				closestGV = gv

		assert (closestGV is not None)
		return closestGV
