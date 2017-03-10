# Authors: Martin Lopez, Patrick Lowe
# 16.35 Final Project
# May 6, 2016

import math, threading

from Simulator import *
from FlightVehicle import *
from Route import *
from Control import *
from VehicleController import *
from IllegalArgumentException import *

class FlightController(VehicleController):

    def __init__(self,sim,fv):
        if (type(sim).__name__!='Simulator'):
            raise IllegalArgumentException("sim must be a Simulator object")
        if (type(fv).__name__ != 'FlightVehicle'):
            raise IllegalArgumentException("fv must be a FlightVehicle object")
        super(FlightController,self).__init__(sim,fv)

        self.__sim = sim
        self.__fv = fv
        self.__route = None

        # set velocity, altitude and angle constraints
        self.__maxForwardVel = 5
        self.__minForwardVel = 1.0
        self.__maxUpVel = 5.0
        self.__minUpVel = -5.0
        self.__maxRotSpeed = math.pi/4
        self.__opperatingAlt = 30
        # always a 90-degree turn radius at the intersection of the segments
        self.__turnAngle = math.pi/2

        # define useful control variables
        self.__controllerInitialized = False
        self.__isClimb = None
        self.__isDescent = None
        self.__isTurning = None
        self.__doneFirstSeg = None
        self.__doneSecondSeg = None
        self.__climbTime = None
        self.__turnTime = None
        self.__timeOfManueverStart = None
        self.__climbSpeed = None
        self.__firstSegTime = []
        self.__secondSegTime = []
        self.__climbOmega = []
        self.__numIterations = 1

        self.__fv.setFlightController(self)

    def initializeController(self):

        minTurnRadius = self.__minForwardVel / self.__maxRotSpeed
        turnLength = minTurnRadius * self.__turnAngle
        self.__turnTime = turnLength / self.__minForwardVel

        self.__route = self.__fv.getRoute()
        fvPos = self.__fv.getPosition()

        # get the start, intersection and end coordinates of the FlightVehicle's Route
        if (self.__numIterations == 1):
            start = self.__route.getStart()
            startX = start[0]
            startY = start[1]
            intersect =self.__route.getIntersection()
            intersectX = intersect[0]
            intersectY = intersect[1]
            end = self.__route.getEnd()
            endX = end[0]
            endY = end[1]
        else:
            start = self.__route.getEnd()
            startX = start[0]
            startY = start[1]
            end = self.__route.getStart()
            endX = end[0]
            endY = end[1]
            if (fvPos[2] > math.pi / 3 and fvPos[2] < 2 * math.pi / 3):
                intersectX = endX
                intersectY = startY
            else:
                intersectX = startX
                intersectY = endY

        ForwardVel = self.__route.getVelocity()

        # compute relevant lengths
        firstSegDist = math.sqrt((intersectX-startX)*(intersectX-startX)+(intersectY-startY)*(intersectY-startY))
        secondSegDist = math.sqrt((endX-intersectX)*(endX-intersectX)+(endY-intersectY)*(endY-intersectY))
        firstStrtLength = firstSegDist - (minTurnRadius * math.tan(self.__turnAngle/2))
        secondStrtLength = secondSegDist - (minTurnRadius * math.tan(self.__turnAngle/2))

        #compute relevant times
        self.__firstSegTime.append(firstStrtLength / ForwardVel)
        self.__secondSegTime.append(secondStrtLength / ForwardVel)

        # calculate initial turn angle if neccessary
        xDiff = intersectX - startX
        yDiff = intersectY - startY

        if (math.fabs(xDiff) < 1e-6):
            # edge cases for xDiff ~= 0
            if yDiff > 0:
                desiredTheta = math.pi / 2
            else:
                desiredTheta = -math.pi / 2
        else:
            desiredTheta = math.atan2(yDiff, xDiff)

        # need change in angle
        desiredTheta = VehicleController.normalizeAngle(desiredTheta)
        desiredOmega = VehicleController.normalizeAngle(desiredTheta - fvPos[2])

        initialOmegaTime = desiredOmega/self.__maxRotSpeed
        climbTime = self.__opperatingAlt/self.__maxUpVel

        # bound takeoff/descent time by the max of initialOmegaTime and climbTime
        if (climbTime > initialOmegaTime):
            self.__climbTime = climbTime
            self.__climbSpeed = self.__maxUpVel
            self.__climbOmega.append(desiredOmega*(initialOmegaTime/climbTime)/2)
        else:
            self.__climbTime = initialOmegaTime
            self.__climbSpeed = self.__maxUpVel*(climbTime/initialOmegaTime)
            self.__climbOmega.append(desiredOmega)

        # start with the initial climb of the FlightVehicle to operating altitude
        self.__isClimb = True
        self.__isDescent = False
        self.__isTurning = False
        self.__doneFirstSeg = False
        self.__doneSecondSeg = False
        self.__fv.setFlightController(self)

        currentTime = self.__sim.getCurrentSec() + self.__sim.getCurrentMSec()*1e-3
        self.__timeOfManueverStart = currentTime
        self.__controllerInitialized = True

    def getIsClimb(self):
        return self.__isClimb

    def getIsDescent(self):
        return self.__isDescent


    def getControl(self,sec,msec):

        controlTime = sec + msec*1e-3
        nextControl = None

        if (not self.__fv.isAvailable()):

            if (not self.__controllerInitialized):
                self.initializeController()

            route = self.__route

            if(self.__isClimb):
                if(controlTime - self.__timeOfManueverStart < self.__climbTime):
                    nextControl = Control(self.__climbSpeed,self.__climbOmega[self.__numIterations-1])

                else:
                    self.__isClimb = False
                    self.__timeOfManueverStart = controlTime
                    nextControl = Control(route.getVelocity(),0)

            elif(self.__isDescent):
                if(controlTime - self.__timeOfManueverStart < self.__climbTime):
                    nextControl = Control(-self.__climbSpeed,0)

                else:
                    self.__isDescent = False
                    self.__timeOfManueverStart = controlTime

                    if (self.__numIterations == 2):
                        nextControl = Control(0,0)
                        self.__fv.setPosition([5,5,0,0])
                        self.__fv.setAvailable(True)
                        self.__fv.setRoute(None)
                        self.__numIterations = 1
                        self.__controllerInitialized = False
                        self.__firstSegTime = []
                        self.__secondSegTime = []
                        self.__climbOmega = []
                        self.__climbTime = None
                        self.__climbSpeed = None

                    else:
                        self.__isClimb = True
                        self.__controllerInitialized = False
                        self.__numIterations += 1
                        nextControl = None

            elif(self.__isTurning):
                if(controlTime - self.__timeOfManueverStart < self.__turnTime):
                    nextControl = Control(self.__minForwardVel,self.__maxRotSpeed)

                else:
                    self.__isTurning = False
                    self.__timeOfManueverStart = controlTime
                    nextControl = Control(route.getVelocity(),0)

            elif(not self.__doneFirstSeg):
                if(controlTime - self.__timeOfManueverStart < self.__firstSegTime[self.__numIterations-1]):
                    nextControl = Control(route.getVelocity(),0)

                else:
                    self.__doneFirstSeg = True
                    self.__isTurning = True
                    self.__timeOfManueverStart = controlTime
                    nextControl = Control(self.__minForwardVel, self.__maxRotSpeed)

            elif(not self.__doneSecondSeg):
                if(controlTime - self.__timeOfManueverStart < self.__secondSegTime[self.__numIterations-1]):
                    nextControl = Control(route.getVelocity(),0)

                else:
                    if (self.__numIterations == 1):
                        self.__doneSecondSeg = True
                        self.__isDescent = True
                        self.__timeOfManueverStart = controlTime
                        nextControl = Control(-self.__climbSpeed, 0)

                    else:
                        self.__doneSecondSeg = True
                        self.__isDescent = True
                        self.__timeOfManueverStart = controlTime
                        nextControl = Control(-self.__climbSpeed,0)

        return nextControl
