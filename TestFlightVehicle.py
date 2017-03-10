# TestFlightVehicle
# Assignment 3

# Author: Alessandro Lira

import unittest, math

from IllegalArgumentException import *
from FlightVehicle import *
from Simulator import *
from Control import *

class TestGoundVehicle(unittest.TestCase):

	# Contructs valid GV to ensusre that no exceptions are thrown;
	# also confirms that the GV arguments are properly set
	def testConstructor(self):
		pose = [1,2,0,0]
		dx = 5
		dy = 0
		dt = 0
		fv = FlightVehicle(pose, [dx, dy, dt, 0])
		sim = Simulator()
		fv.addSimulator(sim)
		
		# Note: self.assertAlmostEqual() compares two arguments up
		#       to 7 decimal places by default. This amount can be 
		#       modified by including a third argument

		newPose = fv.getPosition()
		self.assertAlmostEqual(pose[0], newPose[0])
		self.assertAlmostEqual(pose[1], newPose[1])
		self.assertAlmostEqual(pose[2], newPose[2])
		self.assertAlmostEqual(pose[3], newPose[3])

		newVel = fv.getVelocity()
		self.assertAlmostEqual(dx, newVel[0])
		self.assertAlmostEqual(dy, newVel[1])
		self.assertAlmostEqual(dt, newVel[2])
		self.assertAlmostEqual(0, newVel[3])

	def testTooManyArgumentsConstructor(self):
		with self.assertRaises(IllegalArgumentException):
			pose = [0,0,0,0,0]
			fv = FlightVehicle(pose, [0, 0, 0, 0])

	def testTooFewArgumentsConstructor(self):
		with self.assertRaises(IllegalArgumentException):
			pose = [0,0,0]
			fv = FlightVehicle(pose, [0, 0, 0, 0])

	def testNonArrayPoseArgument(self):
		with self.assertRaises(IllegalArgumentException):
			pose = 'not an array'
			fv = FlightVehicle(pose, [0, 0, 0, 0])

	def testTooManyArgumentsSetPosition(self):
		with self.assertRaises(IllegalArgumentException):
			fv = FlightVehicle([0,0,0,0], [5, 0, 0, 0])
			fv.setPosition([0,0,0,0,0])

	def testTooFewArgumentsSetPosition(self):
		with self.assertRaises(IllegalArgumentException):
			fv = FlightVehicle([0,0,0,0], [5, 0, 0, 0])
			fv.setPosition([0,0,0])

	def testTooManyArgumentsSetVelocity(self):
		with self.assertRaises(IllegalArgumentException):
			fv = FlightVehicle([0,0,0,0], [5, 0, 0, 0])
			fv.setVelocity([0,0,0,0,0])

	def testTooFewArgumentsSetVelocity(self):
		with self.assertRaises(IllegalArgumentException):
			fv = FlightVehicle([0,0,0,0], [5, 0, 0, 0])
			fv.setVelocity([0,0,0])

	# Test get/set Position/Velocity at all legal position bounds
	def testGetSetPosVelValid(self):
		pose = [1,2,0,0]
		dx = 5
		dy = 0
		dt = 0
		fv = FlightVehicle(pose, [dx, dy, dt, 0])
		sim = Simulator()
		fv.addSimulator(sim)		
		
		# test get pos/vel values match inputs of constructor
		newPose = fv.getPosition()
		self.assertAlmostEqual(pose[0], newPose[0])
		self.assertAlmostEqual(pose[1], newPose[1])
		self.assertAlmostEqual(pose[2], newPose[2])
		self.assertAlmostEqual(pose[3], newPose[3])

		newVel = fv.getVelocity()
		self.assertAlmostEqual(dx, newVel[0])
		self.assertAlmostEqual(dy, newVel[1])
		self.assertAlmostEqual(dt, newVel[2])
		self.assertAlmostEqual(0, newVel[3])

		# First, test getPosition and setPostion at legal bounds
		
		# x-position near-boundary conditions
		pose[0] = 0
		fv.setPosition(pose)
		newPose = fv.getPosition()
		self.assertAlmostEqual(pose[0], newPose[0])

		pose[0] = 99
		fv.setPosition(pose)
		newPose = fv.getPosition()
		self.assertAlmostEqual(pose[0], newPose[0])

		# y-position near-boundary conditions
		pose[1] = 0
		fv.setPosition(pose)
		newPose = fv.getPosition()
		self.assertAlmostEqual(pose[1], newPose[1])

		pose[1] = 99
		fv.setPosition(pose)
		newPose = fv.getPosition()
		self.assertAlmostEqual(pose[1], newPose[1])

		# theta near-boundary conditions
		pose[2] = -math.pi
		fv.setPosition(pose)
		newPose = fv.getPosition()
		self.assertAlmostEqual(pose[2], newPose[2])

		pose[2] = math.radians(179)
		fv.setPosition(pose)
		newPose = fv.getPosition()
		self.assertAlmostEqual(pose[2], newPose[2])

		# Test getVelocity and setVelocity at all legal position bounds
		vel = fv.getVelocity()

		# x-velocity near-boundary conditions
		vel[0] = 5
		vel[1] = 0
		fv.setVelocity(vel)
		newVel = fv.getVelocity()
		self.assertAlmostEqual(vel[0], newVel[0])

		vel[0] = 10
		vel[1] = 0
		fv.setVelocity(vel)
		newVel = fv.getVelocity()
		self.assertAlmostEqual(vel[0], newVel[0])

		# y-velocity near-boundary conditions
		vel[0] = 0
		vel[1] = 5
		fv.setVelocity(vel)
		newVel = fv.getVelocity()
		self.assertAlmostEqual(vel[1], newVel[1])

		vel[0] = 0
		vel[1] = 10
		fv.setVelocity(vel)
		newVel = fv.getVelocity()
		self.assertAlmostEqual(vel[1], newVel[1])	

		# omega near-boundary conditions
		vel[2] = -math.pi/4
		fv.setVelocity(vel)
		newVel = fv.getVelocity()
		self.assertAlmostEqual(vel[2], newVel[2])	

		vel[2] = math.pi/4
		fv.setVelocity(vel)
		newVel = fv.getVelocity()
		self.assertAlmostEqual(vel[2], newVel[2])	

	def testGetSetPosVelInvalid(self):
		pose = [1,2,0,0]
		dx = 5
		dy = 0
		dt = 0
		fv = FlightVehicle(pose, [dx, dy, dt, 0])
		sim = Simulator()
		fv.addSimulator(sim)

		# Test getPosition and setPosition at illegal bounds. Since all bounds
		# violations get clamped to legal limits, we can test all three
		# dimensions of position at once. 

		# lower bounds
		pose[0] = -1
		pose[1] = -1
		pose[2] = -2*math.pi
		fv.setPosition(pose)
		newPose = fv.getPosition()
#		self.assertAlmostEqual(0, newPose[0])
#		self.assertAlmostEqual(0, newPose[1])
#		self.assertAlmostEqual(-math.pi, newPose[2])

		# upper bounds
		pose[0] = 101
		pose[1] = 101
		pose[2] = math.pi
		fv.setPosition(pose)
		newPose = fv.getPosition()
#		self.assertAlmostEqual(100, newPose[0])
#		self.assertAlmostEqual(100, newPose[1])
#		self.assertAlmostEqual(-math.pi, newPose[2])

		# Test getVelocity and setVelocity at illegal bounds. Since all bounds
		# violations get clamped to legal limits, we can test all three
		# dimensions of velocity at once.

		vel = fv.getVelocity()

		# lower bounds
		vel[0] = 0
		vel[1] = 1
		vel[2] = -math.pi
		fv.setVelocity(vel)
		newVel = fv.getVelocity()
		self.assertAlmostEqual(0, newVel[0])
#		self.assertAlmostEqual(5, newVel[1])
#		self.assertAlmostEqual(-math.pi/4, newVel[2])

		# upper bounds
		vel[0] = 0
		vel[1] = 20
		vel[2] = math.pi
		fv.setVelocity(vel)
		newVel = fv.getVelocity()
		self.assertAlmostEqual(0, newVel[0])
#		self.assertAlmostEqual(10, newVel[1])
#		self.assertAlmostEqual(math.pi/4, newVel[2])

	# controlVehicle and advanceNoiseFree are tricky to test. You 
	# have to use your judgement as to how to test these. Typically 
	# what happens is that as you develop, you discover edge cases 
	# that need to be added. 

#	def testControlVehicle(self):
#		pose = [0,0,0,0]
#		dx = 5
#		dy = 0
#		dt = 0
#		fv = FlightVehicle(pose, [dx, dy, dt,0])
#		sim = Simulator()
#		fv.addSimulator(sim)

		# Acceleration in x

#		c = Control(10,0)
#		fv.controlVehicle(c)

#		newVel = fv.getVelocity()
#		self.assertAlmostEqual(10, newVel[0])
#		self.assertAlmostEqual(0, newVel[1])
#		self.assertAlmostEqual(0, newVel[2])

		# Acceleration in y

#		pose = [0,0,math.pi/2]
#		fv.setPosition(pose)
#		vel = [10,0,0]
#		fv.setVelocity(vel)

#		c = Control(10,0)
#		fv.controlVehicle(c)
		
#		newVel = fv.getVelocity()
#		self.assertAlmostEqual(0, newVel[0])
#		self.assertAlmostEqual(10, newVel[1])
#		self.assertAlmostEqual(0, newVel[2])

		# Acceleration at PI/4 from 5m/s to 10 m/s

#		vel[0] = math.sqrt(12.5)
#		vel[1] = math.sqrt(12.5)
#		vel[2] = math.pi/4
#		fv.setVelocity(vel)
#		c = Control(10,0)
#		fv.controlVehicle(c)

#		newVel = fv.getVelocity()
#		self.assertAlmostEqual(10, math.sqrt(newVel[0]*newVel[0] + newVel[1]*newVel[1]))

		# Rotational acceleration

#		vel[2] = 0
#		fv.setVelocity(vel)
#		c = Control(5, math.pi/8)
#		fv.controlVehicle(c)

#		newVel = fv.getVelocity()
#		self.assertAlmostEqual(math.pi/8, newVel[2])


	def testAdvance(self):
		pose = [0,0,0,0]
		dx = 5
		dy = 0
		dt = 0
		fv = FlightVehicle(pose, [dx, dy, dt, 0])
		sim = Simulator()
		fv.addSimulator(sim)

		# Straight-line motion along x
		
		fv.advance(1, 0)

		newPose = fv.getPosition()
		self.assertTrue(math.fabs(5 - newPose[0]) < 0.5)
		self.assertTrue(math.fabs(0 - newPose[1]) < 0.5)
		self.assertTrue(math.fabs(0 - newPose[2]) < 0.5)

		# Straight-line motion along y
		
		pose = [0,0,0,0]
		fv.setPosition(pose)
		vel = [0,5,0,0]
		fv.setVelocity(vel)

		fv.advance(1, 0)

		newPose = fv.getPosition()
		self.assertTrue(math.fabs(0 - newPose[0]) < 0.5)
		self.assertTrue(math.fabs(5 - newPose[1]) < 0.5)
		self.assertTrue(math.fabs(0 - newPose[2]) < 0.5)

		# Straight-line motion along PI/4

		pose = [0,0,0,0]
		fv.setPosition(pose)

		# Set vehicle moving at 5 m/s along PI/4

		vel[0] = math.sqrt(12.5)
		vel[1] = math.sqrt(12.5)
		vel[2] = 0
		fv.setVelocity(vel)

		newVel = fv.getVelocity()
		self.assertTrue(math.fabs(vel[0] - newVel[0]) < 0.5)
		self.assertTrue(math.fabs(vel[1] - newVel[1]) < 0.5)
		self.assertTrue(math.fabs(vel[2] - newVel[2]) < 0.5)

		fv.advance(1, 0)

		newPose = fv.getPosition()
		self.assertTrue(math.fabs(math.sqrt(12.5) - newPose[0]) < 0.5)
		self.assertTrue(math.fabs(math.sqrt(12.5) - newPose[1]) < 0.5)
		self.assertTrue(math.fabs(0 - newPose[2]) < 0.5)

		# Rotational Motion

		pose = [0,0,0,0]
		fv.setPosition(pose)

		vel[0] = math.sqrt(5)
		vel[1] = math.sqrt(5)
		vel[2] = math.pi/8

		fv.setVelocity(vel)

		fv.advance(1,0)

		newPose = fv.getPosition()
		self.assertTrue(math.fabs(math.pi/8 - newPose[2]) < 0.5)

	# Tests if the returned angle is in the range [-Pi, Pi)

	def testNormalizeAngle(self):
		# Within range boundaries
		self.assertAlmostEqual(0, FlightVehicle.normalizeAngle(0))

		# Near upper boundary
		self.assertAlmostEqual(math.radians(179), FlightVehicle.normalizeAngle(math.radians(179)))

		# Near lower boundary
		self.assertAlmostEqual(-math.pi, FlightVehicle.normalizeAngle(-math.pi))

		# Above upper boundary
		self.assertAlmostEqual(-math.pi/2, FlightVehicle.normalizeAngle(3.5*math.pi))

		# Below lower boundary
		self.assertAlmostEqual(math.pi/2, FlightVehicle.normalizeAngle(-3.5*math.pi))




# main method which executes unit tests when TestFlightVehicle.py is run directly
if __name__ == "__main__":
	unittest.main()
