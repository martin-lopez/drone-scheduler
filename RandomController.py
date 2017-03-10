# RandomController.py
# Assignment 3

# Author: Alessandro Lira

import math, threading, random

from IllegalArgumentException import *
from VehicleController import *
from GroundVehicle import *
from Simulator import *

class RandomController(VehicleController):
	
	def __init__(self,sim,gv):
		# initializes super class
		super(RandomController, self).__init__(sim,gv)

		self.__gv = gv

		self.__minTransSpeed = 5
		self.__maxTransSpeed = 8
		self.__maxRotSpeed = math.radians(10)
		self.__controllerInitialized = False
		self.__avoidWallDist = 15

	def getControl(self,sec,msec):
		# avoid walls if we're too close
		a = VehicleController.avoidWalls(self.__gv.getPosition())

		if a is not None:
			return a
		# otherwise generate a random control
		randSpeed = random.random()*5 + 5
		randOmega = random.random()*math.pi/2.0 - math.pi/4.0
		c = Control(randSpeed,randOmega)

		return c

