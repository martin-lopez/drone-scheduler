import math, time, random, itertools

from IllegalArgumentException import *
from FollowingController import *
from VehicleController import *
from LeadingController import *
from RandomController import *
from GroundVehicle import *
from DisplayClient import *
from Control import *
from FlightVehicle import *
from FlightController import *
from Route import *

class ATC:
	tturn = (1.0 / (math.pi / 4) * math.pi / 2) / 1.0 #turn time
	tdeliver = 12

	def __init__(self):
		self.vehicles = []
		self.numVehicles = 0
		self.startLoc = (5,5)
		self.vmax = 5.0
		self.vmin = 1.0

	def canHandleMore(self):
		for i in self.vehicles:
			if i.isAvailable():
				return True
		return False
		# fv = next((x for x in self.vehicles if x.isAvailable()), None)
		# print(fv)
		# if (fv == None):
		# 	return False
		# else:
		# 	return True

	def handleRequest(self,destination,time):
		if (time < 0):
			raise IllegalArgumentException("time < 0")
		if (destination[0] < 6 or destination[0] > 100):
			raise IllegalArgumentException("destination x-coordinate out of range")
		if (destination[1] < 6 or destination[1] > 100):
			raise IllegalArgumentException("destination y-coordinate out of range")

		v = self.vmax
		time = time
		route = Route((5,5), (destination[0],5), destination, time, v)
		while (not self.checkRoutes(route)):
			v -= 1
			if v < self.vmin:
				time += 1
				v = self.vmax
			route = Route((5,5), (destination[0],5), destination, time, v)
		fv = next(x for x in self.vehicles if x.isAvailable())
		fv.setAvailable(False)
		fv.setRoute(route)

	def addVehicle(self,vehicle):
		self.vehicles.append(vehicle)
		self.numVehicles += 1

	def rmVehicle(self):
		if (len(self.vehicles) < 1):
			raise IllegalArgumentException("No vehicles")

		fv = next((x for x in self.vehicles if x.isAvailable()), None)
		if (fv == None):
			raise IllegalArgumentException("No available vehicles")
		self.vehicles.remove(fv)
		self.numVehicles -= 1

	def checkRoutes(self,route):
		if (not isinstance(route, Route)):
			raise IllegalArgumentException("Input must be a Route")
		routes = []
		for i in self.vehicles:
			if i.getRoute() != None:
				routes.append(i.getRoute())
		if (len(routes) < 1):
			return True
		for i in routes:
			tistart = i.getStartTime()
			tjstart = route.getStartTime()
			if (tistart == tjstart):
				return False
			istart = i.getStart()
			jstart = route.getStart()
			iint = i.getIntersection()
			jint = route.getIntersection()
			iend = i.getEnd()
			jend = route.getEnd()
			iv = i.getVelocity()
			jv = i.getVelocity()
			tiend = tistart + 2 * iv * ((iint[0] - istart[0]) + (iend[1] - iint[1]))
			tjend = tjstart + 2 * jv * ((jint[0] - jstart[0]) + (jend[1] - jint[1]))
			if (tiend == tjend):
				return False
			if (iend[1] > jend[1] and iend[0] < jend[0]):
				tiint = tistart + iv * ((iint[0] - istart[0]) + (jend[1] - istart[1]))
				tjint = tjstart + jv * ((jint[0] - jstart[0]) + (jend[1] - jint[1]) + (jend[0] - iend[0])) + ATC.tdeliver
				if (tiint == tjint):
					return False
			if (jend[1] > iend[1] and jend[0] < iend[0]):
				tiint = tistart + iv * ((iint[0] - istart[0]) + (iend[1] - iint[1]) + (iend[0] - jend[0])) + ATC.tdeliver
				tjint = tjstart + jv * ((jint[0] - jstart[0]) + (iend[1] - jstart[1]))
				if (tiint == jint):
					return False
			if (iend[0] == jend[0]):
				if (iend[1] > jend[1]):
					tiint = tistart + iv * ((iint[0] - istart[0]) + (jend[1] - istart[1]))
					tjint = tjstart + jv * ((jint[0] - jstart[0]) + (jend[1] - jint[1])) + ATC.tdeliver
					if (tiint == tjint):
						return False
				elif (jend[1] > iend[1]):
					tiint = tistart + iv * ((iint[0] - istart[0]) + (iend[1] - iint[1])) + ATC.tdeliver
					tjint = tjstart + jv * ((jint[0] - jstart[0]) + (iend[1] - jstart[1]))
					if (tiint == tjint):
						return False
				else:
					if (tistart > tjstart):
						tiint = tistart + iv * ((iint[0] - istart[0]) + (iend[1] - iint[1]))
						tjint1 = tjstart + jv * ((jint[0] - jstart[0]) + (jend[1] - jint[1]))
						tjint2 = tjstart + jv * ((jint[0] - jstart[0]) + (jend[1] - jint[1]))+ ATC.tdeliver
						if (tjint1 < tiint and tiint < tjint2):
							return False
					else:
						tiint1 = tistart + iv * ((iint[0] - istart[0]) + (iend[1] - iint[1]))
						tiint2 = tistart + iv * ((iint[0] - istart[0]) + (iend[1] - iint[1])) + ATC.tdeliver
						tjint = tjstart + jv * ((jint[0] - jstart[0]) + (jend[1] - jint[1]))
						if (tiint1 < tjint and tjint < tiint2):
							return False
			if (iend[1] == jend[1]):
				if (iend[0] > jend[0]):
					tiint = tistart + iv * ((iint[0] - istart[0]) + (iend[1] - iint[1]) + (iend[0] - jend[0])) + ATC.tdeliver
					tjint = tjstart + jv * ((jint[0] - jstart[0]) + (jend[1] - jint[1]))
					if (tiint == tjint):
						return False
				if (jend[0] > iend[0]):
					tiint = tistart + iv * ((iint[0] - istart[0]) + (iend[1] - iint[1]))
					tjint = tjstart + jv * ((jint[0] - jstart[0]) + (jend[1] - jint[1]) + (jend[0] - iend[0])) + ATC.tdeliver
					if (tiint == tjint):
						return False
		return True

	def getVMax(self):
		return self.vmax

	def getVMin(self):
		return self.vmin

	def setVMax(self, v):
		if (v < 0):
			raise IllegalArgumentException("Proposed vmax < 0")
		if (v < self.vmin):
			raise IllegalArgumentException("Proposed vmax < vmin")

		self.vmax = v

	def setVMin(self, v):
		if (v < 0):
			raise IllegalArgumentException("Proposed vmin < 0")
		if (v > self.vmax):
			raise IllegalArgumentException("Proposed vmin > vmax")

		self.vmin = v
