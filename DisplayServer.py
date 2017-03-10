# DisplayServer.py
# Assignment 2

# Author: Alessandro Lira

import socket, select, cPickle, threading, random, math, sys, os

from graphics import *

import platform

if platform.system() == 'Windows':
	import msvcrt
else:
	import termios, fcntl


class DisplayServer(threading.Thread):

	def __init__(self):
		print 'entered DS init method'
		threading.Thread.__init__(self)

		self.__myHostName = socket.gethostname()

		self.__colors = ['blue','red','green','cyan', 'magenta',
		'orange','purple','brown','gray','black']

		#[PLACEHOLDER] see if this runs any different without the 'Nones'
		self.__numVehicles = None

		self.__gvX = None
		self.__gvY = None
		self.__gvTheta = None

		self.__maxHistoryLength = 1000
		self.__histories = []

		self.__trace = False
		self.__demo = False #bolean used to determine is connected displayClient is running the demo

		# self.__connection_list = []
		# self.__recv_buffer = 4096 # should be exponent of 2
		# #host = socket.gethostname()
		# host = "0.0.0.0"
		# self.__port = 1635

		# self.__server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		# self.__server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		# self.__server_socket.bind((host,self.__port)) # bind socket to local address of machine at specified port #
		# self.__server_socket.listen(10) # backlog value for max queued connections, must be atleast 0

		# # Add server socket (this socket) to list of readable connections
		# self.__connection_list.append(self.__server_socket)

		# print "DisplayServer started on port " + str(self.__port)

		self.initializeDisplay()

		self.quit = False

	def setVehicleData(self, numVehicles, gvX, gvY, gvTheta):
			self.__numVehicles = numVehicles
			self.__gvX = gvX
			self.__gvY = gvY
			self.__gvTheta = gvTheta
			#[PLACEHOLDER]
			print gvTheta

	def clear(self):
		#[PLACEHOLDER]
		self.__histories = []
		self.displayWindow.delete('all')

	def setTrace(self, value):
		self.__trace = value # should be a boolean!

	def setDemo(self, value):
		self.__demo = value

	def updateHistories(self):
		print 'entered update histories'
		# PART 1: check numVehicles
		# if num vehicles same:
		#    proceed to next part
		# else
		#    reset all histories
		#
		# PART 2: check current length of histories
		# if history NOT max length
		#    append new positions
		# else 
		#    'pop' oldest history postions, and add newest positions to front
				#[PLACEHOLDER]
		#print self.__histories
		#print 'histories num GVs: %i | self.numGVs: %i' % (len(self.__histories),self.__numVehicles)
		if (len(self.__histories) != self.__numVehicles):
			# reset histories to have proper length of vehicles
			self.__histories = []
			for i in range(self.__numVehicles):
				# add new vehicle history at end of history list
				self.__histories.append([]) 
				# add most recent vehicle position at beginning of that 'blank' history
				self.__histories[i].insert(0,(self.__gvX[i],self.__gvY[i])) 
		else: # numVehicles still matches number of GV histories stored in histories
			if len(self.__histories[0]) < self.__maxHistoryLength:
				for i in range(len(self.__histories)):
					self.__histories[i].insert(0,(self.__gvX[i],self.__gvY[i]))
			else: # history length has reached max lenth, pop off oldest value
				for i in range(len(self.__histories)):
					self.__histories[i].pop()
					self.__histories[i].insert(0,(self.__gvX[i],self.__gvY[i]))




	def repaint(self):
		print 'entered repaint in DS'
		# clears entire display window
		print 'entered repaint in DS2'
		self.displayWindow.delete('all')
		

		# stringy =  'currentTheta = %f' % (self.__gvTheta[0]*180/math.pi)
		# stringy = self.displayWindow.getKey()
		# print stringy
		# cPos = Text(Point(250,250),stringy)
		# cPos.draw(self.displayWindow)

		print 'made it past DisplayWindow.delete(all)'
		self.drawVehicles()
		print 'made it past call draw vehicles'
		
		if self.__trace:
			self.drawVehicleTrails()

		self.displayWindow.update()

	def initializeDisplay(self):
		# make a display windows
		hostIP = socket.gethostbyname(socket.gethostname())
		self.displayWindow = GraphWin('Display Client started on %s' % hostIP, 750, 750, False)

		# # set window coordinate system to cartesian with (250,250) as origin
		self.displayWindow.setCoords(0,0,750,750)

		#[PLACEHOLDER]
		currentPos = 'Placeholder postion: (X,Y)'
		cPos = Text(Point(375,375),currentPos)
		cPos.draw(self.displayWindow)

		self.displayWindow.update()

		print 'finished initializeDisplay method()'

	def drawVehicles(self):
		#[PLACEHOLDER]
		print 'entered drawVehicles'
		
		for i in range(self.__numVehicles):
			posX = self.__gvX[i]*5
			posY = self.__gvY[i]*5
			theta = self.__gvTheta[i]


			headPoint = Point(posX,posY) # current location of GV

			# draw vehicle as arrow head
			tailPoint = Point(posX-math.cos((theta))*10,
				posY-math.sin((theta))*10)
			print 'tailPoint scaled location (%f,%f)' % (tailPoint.getX(),tailPoint.getY())
			headLine = Line(tailPoint,headPoint) 
			headLine.setFill(self.__colors[i%10])
			headLine.setArrow('last')
			headLine.draw(self.displayWindow)
			print 'made it past draw headline'

		if self.__demo:
			str1 = 'If you can see two squares being traced by two vehicles then'
			str2 = 'Display Client has succefully connected to Display Server'
			text1 = Text(Point(250,452),str1)
			text2 = Text(Point(250,440),str2)
			text1.draw(self.displayWindow)
			text2.draw(self.displayWindow)

	def drawVehicleTrails(self):
		#[PLACHOLDER]
		if len(self.__histories) == 0:
			return None

		for i in range(len(self.__histories)):
			xyPointList = []
			for xyTuple in self.__histories[i]:
				point = Point(xyTuple[0]*5,xyTuple[1]*5)
				xyPointList.append(point)
			
			# make a polygon object from a list of all the path vertices
			# of GV[i], then draw that polygon in the DisplayWindow
			xyPath = Polygon(xyPointList) 
			xyPath.setOutline(self.__colors[i%10])
			xyPath.draw(self.displayWindow)

	def startQuit(self):
		self.quit = True

	def run(self):
		print 'starting up the ole DS thread'
		while 1:
			print '----------------------------DS RUNNING BABY!!----------------------------'

class MessageListener(threading.Thread):

	def __init__(self, displayServer):
		threading.Thread.__init__(self)
		
		self.__displayServer = displayServer

		self.__connection_list = []
		self.__recv_buffer = 4096 # should be exponent of 2
		#host = socket.gethostname()
		host = "0.0.0.0"
		self.__port = 1635

		self.__server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		self.__server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.__server_socket.bind((host,self.__port)) # bind socket to local address of machine at specified port #
		self.__server_socket.listen(10) # backlog value for max queued connections, must be atleast 0

		# Add server socket (this socket) to list of readable connections
		self.__connection_list.append(self.__server_socket)

		print "DisplayServer started on port " + str(self.__port)

	def run(self):
		print "entered ml run method"

		while 1:
			# check to see if user has initiated quit sequence
			if self.__displayServer.quit:
				print 'ML: quit condition caught'
				break
			#print 'strted ds run top while loop'
			# Get the list of sockets which are ready to be read using 'select()'
			read_sockets, write_sockets, error_sockets = select.select(self.__connection_list,[],[])

			for sock in read_sockets:
				# New Connection
				if sock == self.__server_socket:
					# Handle the case in which there is a new connection recieved through server_socket
					sock_new, addr = self.__server_socket.accept()
					self.__connection_list.append(sock_new)
					print "Client (%s, %s) connected" % addr

				 	#self.broadcast_data(sock_new, "[%s:%s] entered DisplayServer\n" % addr)

				# data incoming from DisplayClient
				else:
					# proccess data received
					try:
						#In Windows, sometimes when a TCP program closes abruptly,
						# a "Connection reset by peer" exception will be thrown
						data = sock.recv(self.__recv_buffer)

						#if data: # checks to see if data is null before proceeding
						#	self.broadcast_data(sock, "\r" + '<' + str(sock.getpeername()) + '> ' + data) 
						if data:
							message = cPickle.loads(data)
							#print 'message was :'
							#print message
							print 'current gvX: '
							#print self.__gvX

							if (len(message) == 1): # scenario where a command is given, NOT vehicle data
								print 'DEBUG: Message was length 1'
								if message[0] == 'clear':
									self.__displayServer.clear()
								elif message[0] == 'traceon':
									self.__displayServer.setTrace(True)
								elif message[0] == 'traceoff':
									self.__displayServer.setTrace(False)
								elif message[0] == 'demo':
									self.__displayServer.setDemo(True)

							elif (len(message) == 4): # length of vehicle data message
								print 'DEBUG: Message was length 4'
								self.__displayServer.setVehicleData(message[0],message[1],message[2],message[3])
								#[PLACEHOLDER] update histories
								print 'called repaint in DS'
								self.__displayServer.updateHistories()
								self.__displayServer.repaint()
								#time.sleep(.01)

							else:
								print 'DEBUG: Message was not length 1 or 4'

					except:
						#self.broadcast_data(sock, 'Client (%s, %s) is offline' % addr)
						print "DisplayClient (%s, %s) is offline" % (addr, random.randint(1,10))
						#sock.close()
						#self.__connection_list.remove(sock)
						continue

		self.__server_socket.close()
		#[PLACEHOLDER]
		#sys.exit()

# Adapted from python documentation at:
# https://docs.python.org/2/faq/library.html#how-do-i-get-a-single-keypress-at-a-time
class KeyListenerUnix(threading.Thread):
	def __init__(self, displayServer):
	    	threading.Thread.__init__(self)

		self.__displayServer = displayServer

		self.__fd = sys.stdin.fileno()

		self.__oldterm = termios.tcgetattr(self.__fd)
		self.__newattr = termios.tcgetattr(self.__fd)
		self.__newattr[3] = self.__newattr[3] & ~termios.ICANON & ~termios.ECHO
		termios.tcsetattr(self.__fd, termios.TCSANOW, self.__newattr)

		self.__oldflags = fcntl.fcntl(self.__fd, fcntl.F_GETFL)
		fcntl.fcntl(self.__fd, fcntl.F_SETFL, self.__oldflags | os.O_NONBLOCK)

	def run(self):
		try:
			while 1:
				if self.__displayServer.quit:
					print 'KL: quit condition caught'
					break
				try:
					c = sys.stdin.read(1)
					print "Got character", repr(c)
					if (c=='q' or c=='Q'):
						self.__displayServer.startQuit()
				except IOError: pass
		finally:
			termios.tcsetattr(self.__fd, termios.TCSAFLUSH, self.__oldterm)
			fcntl.fcntl(self.__fd, fcntl.F_SETFL, self.__oldflags)

class KeyListenerWindows(threading.Thread):
	def __init__(self, displayServer):
		threading.Thread.__init__(self)
		self.__displayServer = displayServer
		
	def run(self):
		while 1:
			if self.__displayServer.quit:
				print 'KL: quit condition caught'
				break
			try:
				c = msvcrt.getch()
				print "Got character", repr(c)
				if (c=='q' or c=='Q'):
					self.__displayServer.startQuit()
					#break
			except IOError: pass
	
if __name__ == '__main__':
	
	ds = DisplayServer()
	ml = MessageListener(ds)


	if platform.system() == 'Windows':
		kl = KeyListenerWindows(ds)
	else:
		kl = KeyListenerUnix(ds)
	
	print 'calling ds run method'
	kl.start()
	#ds.start()
	ml.run()

	kl.join()
	

