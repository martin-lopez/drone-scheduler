# TestVehicleController.py
# Assignment 3

# Author: Alessandro Lira

import unittest, math

from IllegalArgumentException import *
from FollowingController import *
from VehicleController import *
from GroundVehicle import *
from Simulator import *
from Control import *

class TestVehicleController(unittest.TestCase):

	# Test constructor does not throw an error
	def testConstructor(self):
		sim = Simulator()
		pos = [0,0,0]
		gv = GroundVehicle(pos,1,0,math.pi); gv.addSimulator(sim)
		vc = VehicleController(sim,gv)

	# Tests if after calling the method "setNumSides (int n)" the variable
    # "sides" gets correctly updated.
	def testSetSides(self):
		sim = Simulator()
		pose = [0,0,0]
		gv = GroundVehicle(pose,1,0,math.pi)
		vc = VehicleController(sim,gv)

		# values within boundaries
		self.assertEqual(7,vc.setNumSides(7))
		self.assertEqual(4,vc.setNumSides(4))

		# above upper boundry, value does not change
		self.assertEqual(4,vc.setNumSides(25))
		#below lower boundart, value still doesn't change
		self.assertEqual(4,vc.setNumSides(2))

if __name__ == '__main__':
	unittest.main()
