# Control.py
# Assignment 3

# Author: Alessandro Lira

import math

from IllegalArgumentException import *

class Control:
	
	def __init__(self,s,omega):
		# Check to make sure speed and omega are in range
		if (s<-10 or s>10):
			raise IllegalArgumentException("Speed out of range")
		if (omega<-math.pi or omega>=math.pi):
			raise IllegalArgumentException("Omega out of range")

		self.__s = s
		self.__omega = omega

	def getSpeed(self):
		return self.__s

	def getRotVel(self):
		return self.__omega

