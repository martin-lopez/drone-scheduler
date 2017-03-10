# DisplayClient.py
# Assignment 2

# Author: Alessandro Lira

import socket, select, string, sys, cPickle, time, math

#Functions

#1. start with Server IP providedin args[]
#2. Attempt establish communication with DisplayServer
#3. Connect with DisplayServer
#4. Repeatedly "pull" data from server? (alternatively server pushes to all DisplayClients)
#5. [PLACEHOLDER] constantly print vehicle positions from DisplayServer

class DisplayClient:

	def __init__(self, host):


		self.__host = host
		self.__port = 1635

		# initialize socket 
		self.__mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 		#self.__mySocket.settimeout(2) # set timeout length to 2 seconds

		# connect to remote host
		try:
			self.__mySocket.connect((self.__host,self.__port))
			#[PLACEHOLDER] check to see if display server is running
		except:
			print 'Unable to connect to host. Did you start Display Server?'
			sys.exit()

		# connection was a success!
		print 'Connected to remote host.'
		self.__connected = True

	#[PLACEHOLDER]
	def close(self):
		try:
			data = cPickle.dumps(('close',)) # make a tupe to be assembled by cPickle
			self.__mySocket.send(data)
		except Exception as e:
			print e
			sys.exit()

	def clear(self):
		try:	
			data = cPickle.dumps(('clear',)) # make a tupe to be assembled by cPickle
			self.__mySocket.send(data)
		except Exception as e:
			print e
			print 'Disconnected from Display Server(%s)' % self.__host
			self.__connected = False

	def traceOn(self):
		try:
			data = cPickle.dumps(('traceon',)) # make a tupe to be assembled by cPickle
			self.__mySocket.send(data)
		except Error as e:
			print e
			print 'Disconnected from Display Server(%s)' % self.__host
			self.__connected = False

	def traceOff(self):
		try:
			data = cPickle.dumps(('traceoff',)) # make a tupe to be assembled by cPickle
			self.__mySocket.send(data)
		except Exception as e:
			print e
			print 'Disconnected from Display Server(%s)' % self.__host
			self.__connected = False

	def update(self,numVehicles, gvX, gvY, gvTheta):
		#time.sleep(.0001)
		try:	
			message = (numVehicles, gvX, gvY, gvTheta) # assemble all info into a tuple "message"
			data = cPickle.dumps(message) # then "pickle" (serialize) that message into byte-code data
			self.__mySocket.send(data)
		except Exception, e:
			print e
			print 'Disconnected from Display Server(%s)' % self.__host
			self.__connected = False

	def sendDemoText(self):
		try:	
			message = ('demo',) # assemble all info into a tuple "message"
			data = cPickle.dumps(message) # then "pickle" (serialize) that message into byte-code data
			self.__mySocket.send(data)
		except Exception, e:
			print e
			print 'Disconnected from Display Server(%s)' % self.__host
			self.__connected = False


	def isConnected(self):
		return self.__connected

if __name__ == '__main__':

	dc = DisplayClient('localhost')

	dc.sendDemoText()

	for i in range(5):
		# 1st update
		gvX = [25,40]
		gvY = [25,40]
		gvTheta = [math.pi/2,math.pi/2]
		dc.update(2,gvX,gvY,gvTheta)
		time.sleep(.5)

		# 2nd update
		gvX = [25,40]
		gvY = [75,60]
		gvTheta = [0,0]
		dc.update(2,gvX,gvY,gvTheta)
		time.sleep(.5)

		# 3rd update
		gvX = [75,60]
		gvY = [75,60]
		gvTheta = [-math.pi/2,-math.pi/2]
		dc.update(2,gvX,gvY,gvTheta)
		time.sleep(.5)

		# 4th update
		gvX = [75,60]
		gvY = [25,40]
		gvTheta = [-math.pi,-math.pi]
		dc.update(2,gvX,gvY,gvTheta)
		time.sleep(.5)

	print 'Done with demo'


