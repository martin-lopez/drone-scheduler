# TestControl.py
# Assignment 3

# Author: Alessandro Lira

import unittest

from IllegalArgumentException import *
from Control import *

class TestControl(unittest.TestCase):

	# test illegal arguments
	def testLowS(self):
		with self.assertRaises(IllegalArgumentException):
			c = Control(4,0)	
	
	def testHighS(self):
		with self.assertRaises(IllegalArgumentException):
			c = Control(11,0)
	
	def testLowOmega(self):
		with self.assertRaises(IllegalArgumentException):
			c = Control(5,-10)

	def testHighOmega(self):
		with self.assertRaises(IllegalArgumentException):
			c = Control(5,10)

	# test legal arguments
	def testCorrectOutputs(self):
		# values which should not throw errors
		test_s = 7.5
		test_omega = .2
		test_c = Control(test_s,test_omega)
		
		self.assertEqual(test_s, test_c.getSpeed())
		self.assertEqual(test_omega, test_c.getRotVel())


# main method which executes unit tests when TestControl.py is run directly
if __name__ == "__main__":
	unittest.main()


