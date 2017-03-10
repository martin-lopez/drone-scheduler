# Authors: Martin Lopez, Patrick Lowe
# 16.35 Final Project
# May 13, 2016

import unittest, math

from IllegalArgumentException import *
from FollowingController import *
from VehicleController import *
from FlightController import *
from FlightVehicle import *
from Route import *
from GroundVehicle import *
from Simulator import *
from Control import *

class TestFlightController(unittest.TestCase):

    def testConstructor(self):
        sim = Simulator()
        pos = [5,5,0,0]
        fv = FlightVehicle(pos,[1,0,math.pi,0])
        fv.addSimulator(sim)
        fc = FlightController(sim,fv)

    def testBadConstructor(self):
        sim = Simulator()
        pos = [5, 5, 0, 0]
        fv = FlightVehicle(pos, [1, 0, math.pi, 0])
        fv.addSimulator(sim)
        sit = None
        ft = None
        with self.assertRaises(IllegalArgumentException):
            fc = FlightController(sim,ft)

        with self.assertRaises(IllegalArgumentException):
            fc = FlightController(sit, fv)

    def testDefaultValues(self):
        sim = Simulator()
        pos = [5,5,0,0]
        fv = FlightVehicle(pos,[1,0,math.pi,0])
        fv.addSimulator(sim)
        fc = FlightController(sim, fv)

        self.assertEqual(5, fc._FlightController__maxForwardVel)
        self.assertEqual(1, fc._FlightController__minForwardVel)
        self.assertEqual(5,fc._FlightController__maxUpVel)
        self.assertEqual(-5,fc._FlightController__minUpVel)

    def testInitializeController(self):
        sim = Simulator()
        pos = [5, 5, 0, 0]
        fv = FlightVehicle(pos, [1, 0, math.pi, 0])
        fv.addSimulator(sim)
        fc = FlightController(sim,fv)
        route = Route((5,5),(10,5),(10,10),10,5)
        fv.setRoute(route)
        minTurnRadius = 1 / (math.pi/4)

        fc.initializeController()
        self.assertEqual(6, fc._FlightController__climbTime)
        self.assertAlmostEqual((5 - (minTurnRadius * math.tan(math.pi / 4))) / 5, fc._FlightController__firstSegTime[0])
        self.assertAlmostEqual((5 - (minTurnRadius * math.tan(math.pi / 4))) / 5, fc._FlightController__secondSegTime[0])
        self.assertAlmostEqual(math.pi / 2 * (2/6) / 2,fc._FlightController__climbOmega[0])



if __name__ == '__main__':
	unittest.main()