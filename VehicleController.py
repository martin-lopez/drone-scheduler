# VehicleController.py
# Assignment 3

# Author: Alessandro Lira

import math, threading, random

from IllegalArgumentException import *
from GroundVehicle import *
from Simulator import *
from Control import *

class VehicleController(threading.Thread):
	# static class members
	avoidWallDist = 15

	def __init__(self,sim,gv):
		threading.Thread.__init__(self)

		self.__numSides = 5

		self.__minTransSpeed = 5.0
		self.__maxTransSpeed = 10.0
		self.__minRotSpeed = -math.pi/4
		self.__maxRotSpeed = math.pi/4
		self.__circumCircleRadius = 25.0

		self.__isTurning = False
		self.__controllerInitialized = False
		
		self.__turnDuration = None
		self.__edgeTravelDuration = None
		
		self.__timeOfManoeuverStart = None

		# special control mode for n=3,4 
		# because there are no inner turns
		# it is not possible to do the triangle/square with the same logic
		
		self.__q = 2 # how the star is drawn - every _q th point after is connected
		
		self.__isAtOuter = False # flag indicating the vehicle is at outer turn

		self.__count = 0
		self.__ID = 0

		self.__isSpecialMode = False

		self.__turnDurationOuter = 0 # turn time in the outer point
		self.__turnDurationInner = 0 # turn duration in the inner point

		self.__sim = sim
		self.__gv = gv

		self.__lastCheckedSec = 0
		self.__lastCheckedMSec = 0

	def setNumSides(self,numSides):
		if (numSides>=3) and (numSides<=10):
			self.__numSides = numSides
		return self.__numSides

	def initializeController(self):
		# The bulk of this method is to determine how long to spend turning at
		# each corner of the polygon, and how long to spend driving along each
		# edge. We calculate turnDuration and edgeTravelDuration, and then use
		# these inside getControl to decide when to switch from turning to
		# traveling straight and back again. 

		minTurningRadius = self.__minTransSpeed / self.__maxRotSpeed

		if self.__numSides < 5: # for stars with less than 5 points the path needs to do something different

			self.__isSpecialMode = True

			turningAngle = math.pi - math.pi*(self.__numSides-2)/self.__numSides
			arcLengthOuter = turningAngle * minTurningRadius

			# in special mode, we only use the outer turning angle
			self.__turnDurationOuter = arcLengthOuter/self.__minTransSpeed

			alpha = math.pi*(self.__numSides-2)/self.__numSides/2
			beta = turningAngle/2

			l1 = minTurningRadius/math.tan(alpha)

			r_act = (l1*math.cos(alpha) + self.__circumCircleRadius -
				(minTurningRadius - minTurningRadius*math.cos(beta)) )

			# Size of a single edge on the star - see diagram 
			gamma = math.pi/self.__numSides
			d = 2*r_act*math.sin(gamma)

			# Length of edge traveled
			edgeLength = d - 2*l1

			# duration of the straight travel
			self.__edgeTravelDuration = edgeLength/self.__maxTransSpeed

		else:
			# Refer to the PDF for the defined angles 
			# alpha, beta, theta, gamma, R_act


			#----------Outer Turnining angle--------------

			# irstly, we know we need to turn the vehicle by PI - the internal angle's at the outer edge of the star

			# Internal angle of the star is given by Math.PI*(_numSides-2*_q)/_numSides
			turningAngleOuter = math.pi - math.pi*(self.__numSides - 2*self.__q)/self.__numSides

			# And we're going to turn the vehicle along the circumference of the
			# smallest circle we can make.

			# The distance we have to travel along that smallest circle is a function
			# of the angle and the radius, and is an arc along that small circle.
			arcLengthOuter = turningAngleOuter*minTurningRadius

			# We can work out how long each outer turn will take based on the arcLength and
			# how fast we are travelling. Of course, we could also work it out based
			# on the angle and our maximum angular velocity.
			self.__turnDurationOuter = arcLengthOuter/self.__minTransSpeed


			#----------InnerTurnining angle--------------

			# Simmilar scenario for the inner turning angle

			alpha = (math.pi*(self.__numSides - 2*self.__q)/self.__numSides)/2
			gamma = math.pi/self.__numSides
			theta = alpha + gamma

			turningAngleInner = math.pi - 2*theta

			# The distance we have to travel along that smallest circle is a function
			# of the angle and the radius, and is an arc along that small circle.
			arcLengthInner = turningAngleInner*minTurningRadius

			# We can work out how long each inner turn will take based on the arcLength and
			# how fast we are travelling. Of course, we could also work it out based	 
			# on the angle and our maximum angular velocity.

			self.__turnDurationInner = arcLengthInner/self.__minTransSpeed

			#---------Edge Length Calculation-------------

			beta = turningAngleOuter/2

			# Distances not traveled on the star edge because of the curved trajectories
			l1 = minTurningRadius/math.tan(alpha)
			l2 = minTurningRadius/math.tan(theta)

			# R_act (Actual Radius) - distance from the center to any outer corner of the star 
			# (Larger than circumCircleRadius because of the curved trajectories)

			r_act = (l1*math.cos(alpha) + self.__circumCircleRadius - 
					(minTurningRadius - minTurningRadius*math.cos(beta)) )

			# Size of a single edge on the star - see diagram
			d = r_act*math.cos(alpha)/(1 + math.sin(alpha))

			# Length of edge travelled
			edgeLength = d - l1 - l2

			# duration of the straight travel

			self.__edgeTravelDuration = edgeLength/self.__maxTransSpeed

			# Also in method, we initialize the controller state. It's a little ugly,
			# but we'll start as if we're half-way through an outer turn, and tangent to the
			# outer circle. This makes it easy to put the vehicle on a legal part of
			# the star, rather than having to drive to it.

		self.__isTurning = True
		self.__timeOfManoeuverStart = -self.__turnDurationOuter/2
		self.__isAtOuter = True

		self.__controllerInitialized = True

	def getControl(self,sec,msec):
		controlTime = sec+msec*1e-3
		nextControl = None

		if (not self.__controllerInitialized):
			self.initializeController()

		# if we're are currently turning
		if (self.__isTurning):
			currentTurnDuration = 0.0

			# special mode we only do the outer turn (to outline triangle and square)
			# otherwise we do two types of turns alternating between minRotationalVelocity and 
			# maxRotationalVelocity for the outer and inner edges
			if (self.__isSpecialMode):
				currentTurnDuration = self.__turnDurationOuter
			else:
				if (self.__isAtOuter):
					currentTurnDuration = self.__turnDurationOuter
				else:
					currentTurnDuration = self.__turnDurationInner

			# if turn is NOT complete
			if (controlTime-self.__timeOfManoeuverStart < currentTurnDuration):
				if (self.__isSpecialMode):
					nextControl = Control(self.__minTransSpeed,self.__maxRotSpeed)
				else:
					if (self.__isAtOuter):
						nextControl = Control(self.__minTransSpeed,self.__maxRotSpeed)
					else: 
						#we need to turn in the other direction if performing an inner turn
						nextControl = Control(self.__minTransSpeed,self.__minRotSpeed)
			
			# we are done turning - go over to travelling in straight line
			else: 
				self.__isTurning = False
				self.__isAtOuter = not self.__isAtOuter # flip the parameter as we alternate between inner and outer turns
				self.__timeOfManoeuverStart = controlTime
				nextControl = Control(self.__maxTransSpeed, 0)
		
		# we are currently moving in a straight line
		else:
			if (controlTime - self.__timeOfManoeuverStart < self.__edgeTravelDuration):
				nextControl = Control(self.__maxTransSpeed, 0)
			# done with the edge travel - start making the turn
			else:
				self.__isTurning = True
				self.__timeOfManoeuverStart = controlTime
				
				if (self.__isSpecialMode):
					nextControl = Control(self.__minTransSpeed,self.__maxRotSpeed)
				else:
					if(self.__isAtOuter):
						nextControl = Control(self.__minTransSpeed,self.__maxRotSpeed)
					else:
						nextControl = Control(self.__minTransSpeed,self.__minRotSpeed)
				
		return nextControl
		# done deciding controls! :D for now...

	def run(self):

		print "VC controlling GV: %i thread started" % self.__gv.getVehicleID()

		currentSec = 0
		currentMSec = 0

		while(currentSec < 200):
			
			#[NOT NECESSARY] Implemented for convenience of having the VC and 
			# Sim threads ends when quit is called on the DisplayServer
			if not self.__sim.getDisplayClient().isConnected():
				print 'VC: display client NOT connected'
				break

			# Start Condition Critical Region
			self.__sim.simulator_lock.acquire()

			currentSec = self.__sim.getCurrentSec()
			currentMSec = self.__sim.getCurrentMSec()

			while 1:

				# check if time has changed since last update
				if not (self.__lastCheckedSec == currentSec and self.__lastCheckedMSec == currentMSec):
					currentSec = self.__sim.getCurrentSec()
					currentMSec = self.__sim.getCurrentMSec()
					break # use to exit "while 1" loop when the current time has been updated

				self.__sim.simulator_lock.wait() # wait until Simulator notifies all threads that time has passed

				currentSec = self.__sim.getCurrentSec()
				currentMSec = self.__sim.getCurrentMSec()

			# End Conditional Critical Region
			self.__sim.simulator_lock.release()

			# generate a new control 
			nextControl = self.getControl(currentSec,currentMSec)

			if nextControl != None:
				self.__gv.controlVehicle(nextControl)


			# update the time of the last control
			self.__lastCheckedSec = currentSec
			self.__lastCheckedMSec = currentMSec

			# start critical region
			self.__sim.simulator_lock.acquire()
			
			# decrement number of controllers left to update
			if self.__sim.numControlToUpdate > 0:
				self.__sim.numControlToUpdate -= 1
			
			self.__sim.simulator_lock.notify_all()
			
			# end critical region
			self.__sim.simulator_lock.release() 

	@staticmethod
	def normalizeAngle(theta):

		rtheta = math.fmod(theta - math.pi, 2*math.pi)
		if rtheta < 0:
			rtheta += 2*math.pi

		rtheta -= math.pi

		return rtheta

	@staticmethod
	def avoidWalls(pos):
		if (pos[0] > 100 - VehicleController.avoidWallDist and pos[1] > 100 - VehicleController.avoidWallDist):
			if pos[2] > -3*math.pi/4:
				return Control(5,-math.pi/4)
			else:
				return Control(5,+math.pi/4)

		if (pos[0] > 100 - VehicleController.avoidWallDist and pos[1] < 0 + VehicleController.avoidWallDist):
			if pos[2] >  3*math.pi/4:
				return Control(5,-math.pi/4)
			else:
				return Control(5,+math.pi/4)

		if (pos[0] < 0 + VehicleController.avoidWallDist and pos[1] > 100 - VehicleController.avoidWallDist):
			if pos[2] > -math.pi/4:
				return Control(5,-math.pi/4)
			else:
				return Control(5,+math.pi/4)

		if (pos[0] < 0 + VehicleController.avoidWallDist and pos[1] < 0 + VehicleController.avoidWallDist):
			if pos[2] >  math.pi/4:
				return Control(5,-math.pi/4)
			else:
				return Control(5,+math.pi/4)

		if (pos[0] > 100 - VehicleController.avoidWallDist):
			if pos[2] > 0:
				return Control(5,+math.pi/4)
			else:
				return Control(5,-math.pi/4)

		if (pos[0] < 0 + VehicleController.avoidWallDist):
			if pos[2] > 0:
				return Control(5,-math.pi/4)
			else:
				return Control(5,+math.pi/4)

		if (pos[1] < 0 + VehicleController.avoidWallDist):
			if pos[2] > math.pi/2:
				return Control(5,-math.pi/4)
			else:
				return Control(5,+math.pi/4)

		if (pos[1] > 100 - VehicleController.avoidWallDist):
			if pos[2] > -math.pi/2:
				return Control(5,-math.pi/4)
			else:
				return Control(5,+math.pi/4)

		return None

				