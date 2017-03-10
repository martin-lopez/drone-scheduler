# Authors: Martin Lopez, Patrick Lowe
# 16.35 Final Project
# May 9, 2016

import math, threading

from Simulator import *
from GroundVehicle import *
from IllegalArgumentException import *
from FlightController import *

class FlightVehicle(GroundVehicle):
    def __init__(self,pos,vel):
        super(FlightVehicle,self).__init__(pos[0:3],vel[0],vel[1],vel[2])
	if (len(pos) != 4):
		raise IllegalArgumentException("Incorrect size Pos array")
	if (len(vel) != 4):
		raise IllegalArgumentException("Incorrect size Vel array")

        # set FlightVehicle geometry
        self.__maxThrust = 10
        self.__numRotors = 4
        self.__wingSpan = 1.5

        # define useful/required variables
        self.__route = None
        self.__sim = None
        self.__availability = True
        self.__flightControl = None

        # set initial position and velocities
        self.__x = pos[0]
        self.__y = pos[1]
        self.__z = pos[3]
        self.__dx = vel[0]
        self.__dy = vel[1]
        self.__dz = vel[3]
        self.__theta = pos[2]
        self.__dtheta = vel[2]

        self.clampPosition()
        self.clampVelocity()

        # self lock for vehicle state
        self.fv_lock = threading.RLock()

        # create availability lock
        self.av_lock = threading.RLock()

        self.__vehicleID = GroundVehicle.totalNumVehicles
        GroundVehicle.totalNumVehicles += 1

    def isAvailable(self):
        return self.__availability

    def setAvailable(self,av):
        self.av_lock.acquire()
        self.__availability = av
        self.av_lock.release()

    def setRoute(self,route):
        self.__route = route

    def getRoute(self):
        return self.__route

    def getPosition(self):
        return [self.__x,self.__y,self.__theta,self.__z]

    def getVelocity(self):
        return [self.__dx,self.__dy,self.__dtheta,self.__dz]

    def setPosition(self,pos):
	if (len(pos) != 4):
		raise IllegalArgumentException("new Pos array must be of length 4")
        self.__x = pos[0]
        self.__y = pos[1]
        self.__theta = pos[2]
        self.__z = pos[3]

        self.clampPosition()

    def setVelocity(self,vel):
	if (len(vel) != 4):
		raise IllegalArgumentException("new Vel array must be of Length 4")
        self.__dx = vel[0]
        self.__dy = vel[1]
        self.__dtheta = vel[2]
        self.__dz = vel[3]

        self.clampVelocity()

    def setFlightController(self,fc):
        self.__flightControl = fc

    def controlVehicle(self,c):
        speed = c.getSpeed()

        # change dtheta to supplied rotational velocity
        self.__dtheta = c.getRotVel()

        self.fv_lock.acquire()
        # control when the vehicle is climbing/descending
        if (self.__flightControl.getIsClimb()):
            self.__dz = speed
            self.__dx = 0
            self.__dy = 0

        elif (self.__flightControl.getIsDescent()):
            self.__dz = speed
            self.__dx = 0
            self.__dy = 0

        # modify internal dx and dy values
        else:
            self.__dx = speed * math.cos(self.__theta)
            self.__dy = speed * math.sin(self.__theta)
            self.__dz = 0

        self.clampVelocity()
        self.fv_lock.release()  # end critical region

    def advance(self,sec,msec):
        t = sec + msec * 1e-3;

        self.fv_lock.acquire()  # start critical region

        # Assuming that  dx,  dy, dz, and  dtheta was set beforehand by controlVehicle()
        s = math.sqrt(self.__dx * self.__dx + self.__dy * self.__dy + self.__dz * self.__dz)

        # If the drone is climbing or landing, use the following model
        if (abs(self.__dz) > 0):
            self.__z = self.__z + self.__dz * t

            self.__theta = self.__theta + self.__dtheta * t

            rtheta = ((self.__theta - math.pi) % (2 * math.pi))
            if (rtheta < 0):
                rtheta += 2 * math.pi

            self.__theta = rtheta - math.pi;

        elif (abs(self.__dtheta) > 1e-3):  # The following model is not well defined when dtheta = 0
            # Circle center and radius
            r = s / self.__dtheta

            xc = self.__x - r * math.sin(self.__theta)
            yc = self.__y + r * math.cos(self.__theta)

            self.__theta = self.__theta + self.__dtheta * t

            rtheta = ((self.__theta - math.pi) % (2 * math.pi))
            if (rtheta < 0):
                rtheta += 2 * math.pi

            self.__theta = rtheta - math.pi;

            # Update Values
            self.__x = xc + r * math.sin(self.__theta)
            self.__y = yc - r * math.cos(self.__theta)
            self.__dx = s * math.cos(self.__theta)
            self.__dy = s * math.sin(self.__theta)

        else:  # Straight motion. No change in theta.
            self.__x = self.__x + self.__dx * t
            self.__y = self.__y + self.__dy * t

        self.clampPosition()
        self.clampVelocity()

        self.fv_lock.release()  # end critical region


