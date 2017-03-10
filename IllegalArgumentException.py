# IllegalArgumentException.py
# Assignment 3

# Author: Alessandro Lira

# Create IllegalArgumentException that inherits from Exception class
# this class take a string as an exception argument and returns that 
# string when it is raised as an exception
class IllegalArgumentException(Exception):
	def __init__(self,exception):
		self.__exception = exception

	def __str__(self):
		return repr(self.__exception)


