#!/usr/bin/python

import rospy
import numpy
import math

from tf import TransformListener
import tf.transformations

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point, Quaternion

class Poses:

    def onNewPose(self, data):
        self.poseWithConvariance = data.pose

    def __init__(self):
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #rospy.sleep(1)
        self.odom_listener = rospy.Subscriber('mobile_base_controller/odom', Odometry, self.onNewPose)

    def getCurrentPose(self):
       # return self.tf_buffer.lookup_transform("base_footprint", "odom", rospy.Time(0)).transform
        return self.poseWithConvariance.pose

    @staticmethod
    def getPosePosition(pose):
        position = pose.position

        return numpy.array([position.x, position.y, position.z])

    @staticmethod
    def getDistanceBetweenPoses(pose1, pose2):
        vector1 = Poses.getPosePosition(pose1)
        vector2 = Poses.getPosePosition(pose2)

        return numpy.linalg.norm(vector1 - vector2)

    @staticmethod 
    def getPoseEuler(pose):
        (x, y, z) = Poses.quaternionToEuler(pose.orientation.x, pose.orientation.w, pose.orientation.z, pose.orientation.w )
        return x

    @staticmethod 
    def getAngleDistance(pose1, pose2, clockwise = True):
        angle1 = Poses.getPoseEuler(pose1)
        angle2 = Poses.getPoseEuler(pose2)

        diff = angle2 - angle1
        if diff < 0:
            diff += 360

        if clockwise:
            return diff

        return 360 - diff if diff != 0 else 0

    @staticmethod 
    def quaternionToEuler(x, y, z, w):
        ysqr = y * y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))
        
        return (X, Y, Z)

    @staticmethod
    def transform_point(src_frame, target_frame, point, tf_listener=None):
        if tf_listener is None:
            tf_listener = TransformListener()

        p = PointStamped()
        p.header.frame_id = src_frame
        p.header.stamp = rospy.Time(0)
        p.point = Point(*point)

        tf_listener.waitForTransform(target_frame, src_frame, rospy.Time(0), rospy.Duration(4.0))
        newp = tf_listener.transformPoint(target_frame, p)
        p = newp.point

        return numpy.array([p.x, p.y, p.z])

    @staticmethod
    def offset_point(point):
        x = numpy.array(point)
        xy_dir = x[:2] / numpy.linalg.norm(x[:2])
        x[:2] -= 0.2 * xy_dir
        angle = numpy.arctan2(*xy_dir[::-1])
        quat = tf.transformations.quaternion_about_axis(-angle, [0, 0, 1])
        quat = Quaternion(*quat)

        return x




        