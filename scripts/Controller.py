#!/usr/bin/python

import sys

import rospy
import numpy
import math

from Movement import Movement
from Poses import Poses
from Obstacles import ObstaclesDecetor
from Arm import Arm

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from ass2.srv import *

#def onNewPose(pose):
   # print pose

# def spinToRed(msg):
#     diff = msg.data

#     global movement

#     speed = math.fabs(diff/1000)
#     if diff > 20:
#         movement.rotate(speed, True)
#     elif diff < 20:
#         movement.rotate(speed, False)
#     else:
#         print "Just fine"

def init_node():
    publisher = rospy.Publisher('cmd_vel',Twist, queue_size=1)
    rospy.init_node('movement_controller',anonymous=True)

    global movement, poses, obstaclesDecetor, getRedItem, rate, arm
    rate = rospy.Rate(10)
    movement = Movement(publisher)
    arm = Arm()
    poses = Poses()
    obstaclesDecetor = ObstaclesDecetor("/scan")

    print "waiting for service..."
    rospy.wait_for_service('/get_red_item')
    print "done"
    getRedItem = rospy.ServiceProxy("/get_red_item", GetRedItem)

    while not rospy.is_shutdown():
        command = int(raw_input('Enter command number. '))
        handleCommand(command)

def handleCommand(command):
    if command == 1:
        moveDistance(0.1, 0.5)
    elif command == 2:
        degrees = float(raw_input("Insert desired degrees (clockwise) to rotate. can be negative\n"))
        rotateDegrees(0.2, degrees)
    elif command == 3:
        print getDistanceToRedItem()
    elif command == 4:
        #print findRedItem(0.1)
        reach_red_item()
    else:
        print "unknown command"
    

def getDistanceToRedItem():
    coordinates = get_red_item_coordinates()
    return numpy.linalg.norm(coordinates)


def reach_red_item():
    coordinates = get_red_item_coordinates()

    src_frame = 'kinect2_depth_optical_frame'
    target_frame = 'base_footprint'
    transformed_coordinates = Poses.transform_point(src_frame, target_frame, coordinates)

    target_point = Poses.offset_point(transformed_coordinates)
    arm.set_pose(target_point)



def get_red_item_coordinates():
    coordinates = rospy.wait_for_message("/object_detection/coordinates", Vector3)
    return [coordinates.x, coordinates.y, coordinates.z]

def findRedItem(speed, clockwise = True):
    totalDegrees = 0

    global poses, movement, getRedItem, rate
    previousPose = poses.getCurrentPose()

    while totalDegrees < 360 and not getRedItem().has_red_item:
        currentPose = poses.getCurrentPose()
        diff = poses.getAngleDistance(previousPose, currentPose, clockwise)

        totalDegrees += diff
        previousPose = currentPose

        movement.rotate(speed, clockwise)
        rate.sleep()
        rate.sleep()
        rate.sleep()

    rospy.sleep(2)
    return getDistanceToRedItem()


def getAngleToRedItem():
    global getRedItem

    response = getRedItem()
    if (not response.has_red_item):
        return None
    
    return response.angle_offset

def moveDistance(speed, distance):
    global obstaclesDecetor
    print obstaclesDecetor.getDistance()
    #if obstaclesDecetor.getDistance_angleRange(20) <= distance:
    if obstaclesDecetor.getDistance() <= distance:
        print "too close!"
        return

    distanceTraveled = 0
    global movement, poses

    startingPosition = poses.getCurrentPose()

    while distanceTraveled + movement.getStoppingDistance() < distance:
        currentPose = poses.getCurrentPose()
        distanceTraveled = poses.getDistanceBetweenPoses(startingPosition, currentPose)

        movement.moveForward(speed)
        rate.sleep()

def rotateDegrees(speed, degressToRotate):
    clockwise = degressToRotate > 0
    degressToRotate = math.fabs(degressToRotate)
    totalDegrees = 0

    global poses, movement
    previousPose = poses.getCurrentPose()

    while totalDegrees < degressToRotate:
        currentPose = poses.getCurrentPose()
        diff = poses.getAngleDistance(previousPose, currentPose, clockwise)

        totalDegrees += diff
        previousPose = currentPose

        movement.rotate(speed, clockwise)
        rate.sleep()

if __name__ == '__main__':
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass