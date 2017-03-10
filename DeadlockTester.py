# DeadlockTester.py
# Assignment 3

# Author: Alessandro Lira

import thread, threading

from DeadlockTesterException import *

class DeadlockTester:

	# Used to test for potential deadlock scenarios 

	# Checks that the current thread that is trying to access GroundVehicle
	# 'gv' method does not hold a different lock except the built-in lock of
	# 'gv'

	# Args:
	# 	arg1 (GroundVehicle): groundVehicle that is calling this method
	# 	arg2 (Simulator): simulator in which this GroundVehicle is added

	# Returns:
	# 	bool: True if it doesn't hold any lock (different from built-in 
	# 	lock of 'gv')

	# Raises:
	# 	DeadlockTesterException
	

	@staticmethod
	def testLock(targetGV,sim):
		gvList = sim._gvList # note that sim._gvList is psuedo-protected
		for gv in gvList:
			# check if current threaad holds each GV's intrinsic lock
			if gv.gv_lock._RLock__owner == thread.get_ident():
				print "Holds Lock to %i" % gv.getVehicleID()
				if gv != targetGV:
					raise DeadlockTesterException()
					return False
		return True
	

	# ----------------------------------------------------------------------------

	# Alternatively, we could use the methods below to ensure that we acquire
	# locks in a specific order (and release in reverse order). 

	# NOTE: The following methods are not a complete deadlock prevention solution,
	# but are potential building blocks to deadlock prevention system


	# Each vehicle has a distinct Id. The order to get the locks is
    # from lowest to highest. If this ground vehicle has a lower id
    # than the leader, it gets its own lock first and then the
    # leaders lock. Else the order to get the locks will be reversed.
	
	# @staticmethod
	# def lockgvLocks(gvList, ID):
	# 	orderedList = sortVehiclesByID(gvList)
	# 	for gv in orderedList:
	# 		gv.gv_lock.acquire()

    # Unlocks all the 'mygvLocks' lock of each GroundVehicle object in the
    # gvList.
	
	# @staticmethod
	# def unlockgvLocks(gvList, ID):
	# 	orderedList = reverseSortVehiclesByID(gvList)
	# 	for gv in orderedList:
	# 		gv.gv_lock.release()


	# Sorting lists in Python
	# Python has built-in list sorting functionality. The mothods written here
	# are merely wrappers for the built in 'sorted' function. Lists of Objects
	# can be sorted by Object attribute (even provate attributes) by providing
	# logic which returns the value of said attrbute in the 'key' argument of 
	# the 'sorted' function

	# @staticmethod
	# def sortVehcilesByID(gvList):
	# 	return sorted(gvList, key = GroundVehicle.getVehicleID)

	# @staticmethod
	# def reverseSortVehcilesByID(gvList):
	# 	return sorted(gvList, key = GroundVehicle.getVehicleID, reverse=True)
				
	# For more info, check here: 
	# https://wiki.python.org/moin/HowTo/Sorting#Key_Functions











