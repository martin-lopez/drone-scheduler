# TestLeadingController.py
# Assignment 3

# Author: Alessandro Lira

import unittest, math

from IllegalArgumentException import *
from FollowingController import *
from VehicleController import *
from LeadingController import *
from GroundVehicle import *
from Simulator import *
from Control import *

class TestLeadingController(unittest.TestCase):

	# Test constructor does not throw an error
	def testConstructor(self):
		sim = Simulator()
		pos = [0,0,0]
		gv = GroundVehicle(pos,1,0,math.pi); gv.addSimulator(sim)
		vc = VehicleController(sim,gv)

	# Tests that following vehicles are correctly added to the 
	# Leading Controller
	def testAddFollower(self):
		sim = Simulator()

		pos = [1,2,3]

		gv1 = GroundVehicle(pos,0,0,0); gv1.addSimulator(sim)
		gv2 = GroundVehicle(pos,1,1,1); gv2.addSimulator(sim)

		lc = LeadingController(sim, gv1)
		lc.addFollower(gv2)

		self.assertEqual(gv2, lc.getFollower(0))

	def testGetClosestVehicle(self):
		sim = Simulator()

		pos1 = [50,50,0]
		pos2 = [10,10,0]
		pos3 = [100,100,0]

		gv1 = GroundVehicle(pos1,5,0,0); gv1.addSimulator(sim)
		gv2 = GroundVehicle(pos2,0,5,0); gv2.addSimulator(sim)
		gv3 = GroundVehicle(pos3,10,0,0); gv3.addSimulator(sim)


		lc = LeadingController(sim, gv1)
		lc.addFollower(gv2)
		lc.addFollower(gv3)

		self.assertEqual(gv2, lc.getClosestVehicle())

if __name__ == '__main__':
	unittest.main()