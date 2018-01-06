#!/usr/bin/python

import rospy
import numpy

from geometry_msgs.msg import Twist

class Movement:
    'a class to handle simple motion'

    def __init__(self, movementPublisher):
        self.movementPublisher = movementPublisher

    def moveForward(self, speed):
        self.movementPublisher.publish(self.getForwardTwist(speed))

    def rotate(self, speed, isClockwise = True):
        self.movementPublisher.publish(self.getRotationTwist(speed, isClockwise))

    def getStoppingDistance(self):
        # TODO
        return 0

    #def moveDistance(speed, distance):
    @staticmethod
    def getForwardTwist(speed):
        twist = Twist()

        twist.linear.x =  speed
        twist.linear.y = 0
        twist.angular.z = 0    

        return twist

    @staticmethod
    def getRotationTwist(speed, isClockwise = True):
        twist = Twist()

        twist.linear.x = 0
        twist.linear.y = 0
        twist.angular.x = 0
        if isClockwise == True:
            twist.angular.z = -speed 
        else:
            twist.angular.z = speed 

        return twist