#!/usr/bin/python
import rospy
import math
from sensor_msgs.msg import LaserScan

class ObstaclesDecetor:
    def __init__(self, laserTopicName):
        self.subscriber = rospy.Subscriber(laserTopicName, LaserScan, self.onNewScan)

    def onNewScan(self, data):
        self.scan = data

    def getDistance(self):
        return self.scan.ranges[len(self.scan.ranges) / 2]

    #Gets the distance from an object at a certain angle
    def getDistance_angle(self, angle):
        inc = self.scan.angle_increment / math.pi * 180
        index_offset = math.floor(angle/inc)

        distance = self.scan.ranges[int(len(self.scan.ranges) / 2 - index_offset)] 
        print distance
        return distance

    #Gets the distance from an object at a range of angles, starting from -angle to +angle
    def getDistance_angleRange(self, angle):
        inc = self.scan.angle_increment / math.pi * 180
        index_offset = math.floor(angle/inc)

        midIndex = math.floor(len(self.scan.ranges) / 2) 

        maxIndex = midIndex + index_offset
        minIndex = midIndex - index_offset

        scans_inrange = self.scan.ranges[minIndex, maxIndex]
        return max(scans_inrange)




        
