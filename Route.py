# Authors: Martin Lopez, Patrick Lowe
# 16.35 Final Project
# May 9, 2016

import math

from IllegalArgumentException import *

class Route:

    def __init__(self,start,mid,end,T,vel):
        if (start[0]<5 or start[0]>100):
            raise IllegalArgumentException("Starting x-coordinate out of range")
        if (start[1]<5 or start[1]>100):
            raise IllegalArgumentException("Starting y-coordinate out of range")
        if (mid[0]<5 or mid[0]>100):
            raise IllegalArgumentException("Intersection x-coordinate out of range")
        if (mid[1]<5 or mid[1]>100):
            raise IllegalArgumentException("Intersection y-coordinate out of range")
        if (end[0]<5 or end[0]>100):
            raise IllegalArgumentException("Ending x-coordinate out of range")
        if (end[1]<5 or end[1]>100):
            raise IllegalArgumentException("Ending y-coordinate out of range")
        if (vel < 1 or vel > 5):
            raise IllegalArgumentException("Velocity out of range")

        self.__X1 = start[0]
        self.__Y1 = start[1]
        self.__X2 = mid[0]
        self.__Y2 = mid[1]
        self.__X3 = end[0]
        self.__Y3 = end[1]
        self.__startTime = T
        self.__vel = vel

    def getStart(self):
        return [self.__X1,self.__Y1]

    def getIntersection(self):
        return [self.__X2,self.__Y2]

    def getEnd(self):
        return [self.__X3,self.__Y3]

    def getStartTime(self):
        return self.__startTime

    def getVelocity(self):
        return self.__vel
