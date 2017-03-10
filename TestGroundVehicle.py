# TestGroundVehicle
# Assignment 3

# Author: Alessandro Lira

import unittest, math

from IllegalArgumentException import *
from GroundVehicle import *
from Simulator import *
from Control import *

class TestGoundVehicle(unittest.TestCase):

	# Contructs valid GV to ensusre that no exceptions are thrown;
	# also confirms that the GV arguments are properly set
	def testConstructor(self):
		pose = [1,2,0]
		dx = 5
		dy = 0
		dt = 0
		gv = GroundVehicle(pose, dx, dy, dt)
		sim = Simulator()
		gv.addSimulator(sim)
		
		# Note: self.assertAlmostEqual() compares two arguments up
		#       to 7 decimal places by default. This amount can be 
		#       modified by including a third argument

		newPose = gv.getPosition()
		self.assertAlmostEqual(pose[0], newPose[0])
		self.assertAlmostEqual(pose[1], newPose[1])
		self.assertAlmostEqual(pose[2], newPose[2])

		newVel = gv.getVelocity()
		self.assertAlmostEqual(dx, newVel[0])
		self.assertAlmostEqual(dy, newVel[1])
		self.assertAlmostEqual(dt, newVel[2])

	def testTooManyArgumentsConstructor(self):
		with self.assertRaises(IllegalArgumentException):
			pose = [0,0,0,0]
			gv = GroundVehicle(pose, 0, 0, 0)

	def testTooFewArgumentsConstructor(self):
		with self.assertRaises(IllegalArgumentException):
			pose = [0,0]
			gv = GroundVehicle(pose, 0, 0, 0)

	def testNonArrayPoseArgument(self):
		with self.assertRaises(IllegalArgumentException):
			pose = 'not an array'
			gv = GroundVehicle(pose, 0, 0, 0)

	def testTooManyArgumentsSetPosition(self):
		with self.assertRaises(IllegalArgumentException):
			gv = GroundVehicle([0,0,0], 5, 0, 0)
			gv.setPosition([0,0,0,0])

	def testTooFewArgumentsSetPosition(self):
		with self.assertRaises(IllegalArgumentException):
			gv = GroundVehicle([0,0,0], 5, 0, 0)
			gv.setPosition([0,0])

	def testTooManyArgumentsSetVelocity(self):
		with self.assertRaises(IllegalArgumentException):
			gv = GroundVehicle([0,0,0], 5, 0, 0)
			gv.setVelocity([0,0,0,0])

	def testTooFewArgumentsSetVelocity(self):
		with self.assertRaises(IllegalArgumentException):
			gv = GroundVehicle([0,0,0], 5, 0, 0)
			gv.setVelocity([0,0])

	# Test get/set Position/Velocity at all legal position bounds
	def testGetSetPosVelValid(self):
		pose = [1,2,0]
		dx = 5
		dy = 0
		dt = 0
		gv = GroundVehicle(pose, dx, dy, dt)
		sim = Simulator()
		gv.addSimulator(sim)		
		
		# test get pos/vel values match inputs of constructor
		newPose = gv.getPosition()
		self.assertAlmostEqual(pose[0], newPose[0])
		self.assertAlmostEqual(pose[1], newPose[1])
		self.assertAlmostEqual(pose[2], newPose[2])

		newVel = gv.getVelocity()
		self.assertAlmostEqual(dx, newVel[0])
		self.assertAlmostEqual(dy, newVel[1])
		self.assertAlmostEqual(dt, newVel[2])

		# First, test getPosition and setPostion at legal bounds
		
		# x-position near-boundary conditions
		pose[0] = 0
		gv.setPosition(pose)
		newPose = gv.getPosition()
		self.assertAlmostEqual(pose[0], newPose[0])

		pose[0] = 99
		gv.setPosition(pose)
		newPose = gv.getPosition()
		self.assertAlmostEqual(pose[0], newPose[0])

		# y-position near-boundary conditions
		pose[1] = 0
		gv.setPosition(pose)
		newPose = gv.getPosition()
		self.assertAlmostEqual(pose[1], newPose[1])

		pose[1] = 99
		gv.setPosition(pose)
		newPose = gv.getPosition()
		self.assertAlmostEqual(pose[1], newPose[1])

		# theta near-boundary conditions
		pose[2] = -math.pi
		gv.setPosition(pose)
		newPose = gv.getPosition()
		self.assertAlmostEqual(pose[2], newPose[2])

		pose[2] = math.radians(179)
		gv.setPosition(pose)
		newPose = gv.getPosition()
		self.assertAlmostEqual(pose[2], newPose[2])

		# Test getVelocity and setVelocity at all legal position bounds
		vel = gv.getVelocity()

		# x-velocity near-boundary conditions
		vel[0] = 5
		vel[1] = 0
		gv.setVelocity(vel)
		newVel = gv.getVelocity()
		self.assertAlmostEqual(vel[0], newVel[0])

		vel[0] = 10
		vel[1] = 0
		gv.setVelocity(vel)
		newVel = gv.getVelocity()
		self.assertAlmostEqual(vel[0], newVel[0])

		# y-velocity near-boundary conditions
		vel[0] = 0
		vel[1] = 5
		gv.setVelocity(vel)
		newVel = gv.getVelocity()
		self.assertAlmostEqual(vel[1], newVel[1])

		vel[0] = 0
		vel[1] = 10
		gv.setVelocity(vel)
		newVel = gv.getVelocity()
		self.assertAlmostEqual(vel[1], newVel[1])	

		# omega near-boundary conditions
		vel[2] = -math.pi/4
		gv.setVelocity(vel)
		newVel = gv.getVelocity()
		self.assertAlmostEqual(vel[2], newVel[2])	

		vel[2] = math.pi/4
		gv.setVelocity(vel)
		newVel = gv.getVelocity()
		self.assertAlmostEqual(vel[2], newVel[2])	

	def testGetSetPosVelInvalid(self):
		pose = [1,2,0]
		dx = 5
		dy = 0
		dt = 0
		gv = GroundVehicle(pose, dx, dy, dt)
		sim = Simulator()
		gv.addSimulator(sim)

		# Test getPosition and setPosition at illegal bounds. Since all bounds
		# violations get clamped to legal limits, we can test all three
		# dimensions of position at once. 

		# lower bounds
		pose[0] = -1
		pose[1] = -1
		pose[2] = -2*math.pi
		gv.setPosition(pose)
		newPose = gv.getPosition()
		self.assertAlmostEqual(0, newPose[0])
		self.assertAlmostEqual(0, newPose[1])
		self.assertAlmostEqual(-math.pi, newPose[2])

		# upper bounds
		pose[0] = 101
		pose[1] = 101
		pose[2] = math.pi
		gv.setPosition(pose)
		newPose = gv.getPosition()
		self.assertAlmostEqual(100, newPose[0])
		self.assertAlmostEqual(100, newPose[1])
		self.assertAlmostEqual(-math.pi, newPose[2])

		# Test getVelocity and setVelocity at illegal bounds. Since all bounds
		# violations get clamped to legal limits, we can test all three
		# dimensions of velocity at once.

		vel = gv.getVelocity()

		# lower bounds
		vel[0] = 0
		vel[1] = 1
		vel[2] = -math.pi
		gv.setVelocity(vel)
		newVel = gv.getVelocity()
		self.assertAlmostEqual(0, newVel[0])
		self.assertAlmostEqual(5, newVel[1])
		self.assertAlmostEqual(-math.pi/4, newVel[2])

		# upper bounds
		vel[0] = 0
		vel[1] = 20
		vel[2] = math.pi
		gv.setVelocity(vel)
		newVel = gv.getVelocity()
		self.assertAlmostEqual(0, newVel[0])
		self.assertAlmostEqual(10, newVel[1])
		self.assertAlmostEqual(math.pi/4, newVel[2])

	# controlVehicle and advanceNoiseFree are tricky to test. You 
	# have to use your judgement as to how to test these. Typically 
	# what happens is that as you develop, you discover edge cases 
	# that need to be added. 

	def testControlVehicle(self):
		pose = [0,0,0]
		dx = 5
		dy = 0
		dt = 0
		gv = GroundVehicle(pose, dx, dy, dt)
		sim = Simulator()
		gv.addSimulator(sim)

		# Acceleration in x

		c = Control(10,0)
		gv.controlVehicle(c)

		newVel = gv.getVelocity()
		self.assertAlmostEqual(10, newVel[0])
		self.assertAlmostEqual(0, newVel[1])
		self.assertAlmostEqual(0, newVel[2])

		# Acceleration in y

		pose = [0,0,math.pi/2]
		gv.setPosition(pose)
		vel = [10,0,0]
		gv.setVelocity(vel)

		c = Control(10,0)
		gv.controlVehicle(c)
		
		newVel = gv.getVelocity()
		self.assertAlmostEqual(0, newVel[0])
		self.assertAlmostEqual(10, newVel[1])
		self.assertAlmostEqual(0, newVel[2])

		# Acceleration at PI/4 from 5m/s to 10 m/s

		vel[0] = math.sqrt(12.5)
		vel[1] = math.sqrt(12.5)
		vel[2] = math.pi/4
		gv.setVelocity(vel)
		c = Control(10,0)
		gv.controlVehicle(c)

		newVel = gv.getVelocity()
		self.assertAlmostEqual(10, math.sqrt(newVel[0]*newVel[0] + newVel[1]*newVel[1]))

		# Rotational acceleration

		vel[2] = 0
		gv.setVelocity(vel)
		c = Control(5, math.pi/8)
		gv.controlVehicle(c)

		newVel = gv.getVelocity()
		self.assertAlmostEqual(math.pi/8, newVel[2])

	def testAdvanceNoiseFree(self):
		pose = [0,0,0]
		dx = 5
		dy = 0
		dt = 0
		gv = GroundVehicle(pose, dx, dy, dt)
		sim = Simulator()
		gv.addSimulator(sim)

		# Straight-line motion along x
		
		gv.advanceNoiseFree(1, 0)

		newPose = gv.getPosition()
		self.assertAlmostEqual(5,newPose[0])
		self.assertAlmostEqual(0,newPose[1])
		self.assertAlmostEqual(0,newPose[2])

		# Straight-line motion along y
		
		pose = [0,0,0]
		gv.setPosition(pose)
		vel = [0,5,0]
		gv.setVelocity(vel)

		gv.advanceNoiseFree(1, 0)

		newPose = gv.getPosition()
		self.assertAlmostEqual(0,newPose[0])
		self.assertAlmostEqual(5,newPose[1])
		self.assertAlmostEqual(0,newPose[2])

		# Straight-line motion along PI/4

		pose = [0,0,0]
		gv.setPosition(pose)

		# Set vehicle moving at 5 m/s along PI/4

		vel[0] = math.sqrt(12.5)
		vel[1] = math.sqrt(12.5)
		vel[2] = 0
		gv.setVelocity(vel)

		newVel = gv.getVelocity()
		self.assertAlmostEqual(vel[0], newVel[0])
		self.assertAlmostEqual(vel[1], newVel[1])
		self.assertAlmostEqual(vel[2], newVel[2])

		gv.advanceNoiseFree(1, 0)

		newPose = gv.getPosition()
		self.assertAlmostEqual(math.sqrt(12.5), newPose[0])
		self.assertAlmostEqual(math.sqrt(12.5), newPose[1])
		self.assertAlmostEqual(0, newPose[2])

		# Rotational Motion

		pose = [0,0,0]
		gv.setPosition(pose)

		vel[0] = math.sqrt(5)
		vel[1] = math.sqrt(5)
		vel[2] = math.pi/8

		gv.setVelocity(vel)

		gv.advanceNoiseFree(1,0)

		newPose = gv.getPosition()
		self.assertAlmostEqual(math.pi/8, newPose[2])

	# Because advance() has noise, we need to adjust the margin of error 
	# allowed in the assert statements 

	def testAdvance(self):
		pose = [0,0,0]
		dx = 5
		dy = 0
		dt = 0
		gv = GroundVehicle(pose, dx, dy, dt)
		sim = Simulator()
		gv.addSimulator(sim)

		# Straight-line motion along x
		
		gv.advance(1, 0)

		newPose = gv.getPosition()
		self.assertTrue(math.fabs(5 - newPose[0]) < 0.5)
		self.assertTrue(math.fabs(0 - newPose[1]) < 0.5)
		self.assertTrue(math.fabs(0 - newPose[2]) < 0.5)

		# Straight-line motion along y
		
		pose = [0,0,0]
		gv.setPosition(pose)
		vel = [0,5,0]
		gv.setVelocity(vel)

		gv.advance(1, 0)

		newPose = gv.getPosition()
		self.assertTrue(math.fabs(0 - newPose[0]) < 0.5)
		self.assertTrue(math.fabs(5 - newPose[1]) < 0.5)
		self.assertTrue(math.fabs(0 - newPose[2]) < 0.5)

		# Straight-line motion along PI/4

		pose = [0,0,0]
		gv.setPosition(pose)

		# Set vehicle moving at 5 m/s along PI/4

		vel[0] = math.sqrt(12.5)
		vel[1] = math.sqrt(12.5)
		vel[2] = 0
		gv.setVelocity(vel)

		newVel = gv.getVelocity()
		self.assertTrue(math.fabs(vel[0] - newVel[0]) < 0.5)
		self.assertTrue(math.fabs(vel[1] - newVel[1]) < 0.5)
		self.assertTrue(math.fabs(vel[2] - newVel[2]) < 0.5)

		gv.advance(1, 0)

		newPose = gv.getPosition()
		self.assertTrue(math.fabs(math.sqrt(12.5) - newPose[0]) < 0.5)
		self.assertTrue(math.fabs(math.sqrt(12.5) - newPose[1]) < 0.5)
		self.assertTrue(math.fabs(0 - newPose[2]) < 0.5)

		# Rotational Motion

		pose = [0,0,0]
		gv.setPosition(pose)

		vel[0] = math.sqrt(5)
		vel[1] = math.sqrt(5)
		vel[2] = math.pi/8

		gv.setVelocity(vel)

		gv.advance(1,0)

		newPose = gv.getPosition()
		self.assertTrue(math.fabs(math.pi/8 - newPose[2]) < 0.5)

	# Tests if the returned angle is in the range [-Pi, Pi)

	def testNormalizeAngle(self):
		# Within range boundaries
		self.assertAlmostEqual(0, GroundVehicle.normalizeAngle(0))

		# Near upper boundary
		self.assertAlmostEqual(math.radians(179), GroundVehicle.normalizeAngle(math.radians(179)))

		# Near lower boundary
		self.assertAlmostEqual(-math.pi, GroundVehicle.normalizeAngle(-math.pi))

		# Above upper boundary
		self.assertAlmostEqual(-math.pi/2, GroundVehicle.normalizeAngle(3.5*math.pi))

		# Below lower boundary
		self.assertAlmostEqual(math.pi/2, GroundVehicle.normalizeAngle(-3.5*math.pi))




# main method which executes unit tests when TestGroundVehicle.py is run directly
if __name__ == "__main__":
	unittest.main()