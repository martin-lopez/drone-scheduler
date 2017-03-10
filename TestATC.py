import unittest

from ATC import *
from FlightVehicle import *
from Route import *

class TestATC(unittest.TestCase):

	def testGoodConstructor(self):
		atc = ATC()
		self.assertFalse(atc.canHandleMore())
		self.assertEqual(5, atc.getVMax())
		self.assertEqual(1, atc.getVMin())

	def testSetVMax(self):
		atc = ATC()
		atc.setVMax(2)
		self.assertEqual(2, atc.getVMax())
		with self.assertRaises(IllegalArgumentException):
			atc.setVMax(-3)
			atc.setVMax(0)
		atc.setVMin(0)
		atc.setVMax(0)
		self.assertEqual(0, atc.getVMin())
		self.assertEqual(0, atc.getVMax())

	def testSetVMin(self):
		atc = ATC()
		atc.setVMin(0)
		self.assertEqual(0, atc.getVMin())
		with self.assertRaises(IllegalArgumentException):
			atc.setVMin(-2)
			atc.setVMin(10)
		atc.setVMin(5)
		self.assertEqual(5, atc.getVMax())
		self.assertEqual(5, atc.getVMin())

	def testCanHandleMore(self):
		atc = ATC()
		fv = FlightVehicle([0,0,0,0], [0,0,0,0])
		atc.addVehicle(fv)
		self.assertTrue(atc.canHandleMore())
		fv.setAvailable(False)
		self.assertFalse(atc.canHandleMore())

	def testRmVehicle(self):
		atc = ATC()
		fv = FlightVehicle([0,0,0,0], [0,0,0,0])
		with self.assertRaises(IllegalArgumentException):
			atc.rmVehicle()
		atc.addVehicle(fv)
		fv.setAvailable(False)
		with self.assertRaises(IllegalArgumentException):
			atc.rmVehicle()
		fv.setAvailable(True)
		atc.rmVehicle()
		self.assertFalse(atc.canHandleMore())

	def testCheckRoutes(self):
		atc = ATC()
		fv1 = FlightVehicle([0,0,0,0], [0,0,0,0])
		atc.addVehicle(fv1)
		with self.assertRaises(IllegalArgumentException):
			atc.checkRoutes((10,10))
			atc.handleRequest((10,10), 0)
			self.assertFalse(atc.canHandleMore())
			r1 = Route((0,0), (0,10), (10,10), 0, 5)
			self.assertFalse(atc.checkRoutes(r1))

	def testHandleRequest(self):
		atc = ATC()
		fv1 = FlightVehicle([0,0,0,0], [0,0,0,0])
		fv2 = FlightVehicle([0,0,0,0], [0,0,0,0])
		atc.addVehicle(fv1)
		atc.addVehicle(fv2)
		atc.handleRequest((10,10), 0)
		self.assertTrue(atc.canHandleMore())
		atc.handleRequest((10,10), 0)
		self.assertFalse(atc.canHandleMore())
		self.assertNotEqual(fv1.getRoute(), fv2.getRoute())
		

if __name__ == "__main__":
	unittest.main()
