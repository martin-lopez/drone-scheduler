# DeadlockTesterException.py
# Assignment 3

# Author: Alessandro Lira

# Create DeadlockTesterException that inherits from Exception class
class DeadlockTesterException(Exception):
	
	# DeadlockTesterException prints out  a notification of deadlock 
	# to the text-stream. Otherwise behaves the same as an Exception

	def __init__(self):
		self.__exception = ("DeadlockTester found that a GV held the "
							"lock to a GV other than itself")

	def __str__(self):
		return repr(self.__exception)