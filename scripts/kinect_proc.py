#!/usr/bin/python
import sys
import rospy

import cv2
import numpy as np
import struct
import ctypes
import math

import cv_helper

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

from ass2.srv import GetRedItem, GetRedItemResponse

import sensor_msgs.point_cloud2 as pc2

def handleRequest(req):
    global last_red_item_cooridnates

    response = GetRedItemResponse()
    
    if last_red_item_cooridnates is None:
        response.has_red_item = False
        return response

    response.has_red_item = True
    response.x = last_red_item_cooridnates[0]
    response.y = last_red_item_cooridnates[1]
    response.z = last_red_item_cooridnates[2]

    return response


def on_new_img(msg):
    print "got new image"
    global red_mask_publisher, object_coordinates_publisher, object_view_publisher
    global cv_bridge, last_red_item_cooridnates

    bgr = get_bgr_from_pc2_msg(msg)

    reds = cv_helper.maskRed(bgr)
    red_mask_publisher.publish(cv_bridge.cv2_to_imgmsg(reds))

    contour = cv_helper.find_largest_contour(reds)
    if contour is None:
        last_red_item_cooridnates = None
        object_view_publisher.publish(cv_bridge.cv2_to_imgmsg(bgr, encoding="bgr8"))

        print "found nothing"
        return

    center = cv_helper.find_contour_center(contour)

    cv2.circle(reds, (center[0], center[1]), 7, (255, 255, 255), -1)
    object_view_publisher.publish(cv_bridge.cv2_to_imgmsg(bgr, encoding="bgr8"))

    transform = get_transform(msg, center)
    print "found red object at  ", transform

    coordinates = Vector3(x = transform[0], y = transform[1], z = transform[2])
    last_red_item_cooridnates = transform
    object_coordinates_publisher.publish(coordinates)


def get_bgr_from_pc2_msg(pc):
    x = np.frombuffer(pc.data, 'uint8').reshape(-1, 8, 4)
    bgr = x[:pc.height*pc.width, 4, :3].reshape(pc.height, pc.width, 3)

    return bgr

def get_transform(cloud, point):
    coordiante = [int(point[0]), int(point[1])]
    generator = pc2.read_points(cloud, field_names=['x', 'y', 'z'], uvs=[coordiante])

    return generator.next()
    
def init_node():
    global red_mask_publisher, object_coordinates_publisher, object_view_publisher, cv_bridge, last_red_item_cooridnates
    last_red_item_cooridnates = None

    rospy.init_node('kinect_listener', anonymous=True)

    rospy.Subscriber("/kinect2/qhd/points", PointCloud2, on_new_img)
    rospy.Subscriber("/kinect2/hd/points", PointCloud2, on_new_img)
    red_mask_publisher = rospy.Publisher("/kinect2/red_mask", Image, queue_size=1)
    object_coordinates_publisher = rospy.Publisher("/object_detection/coordinates", Vector3, queue_size=1)
    object_view_publisher = rospy.Publisher('/object_detection/image_view', Image, queue_size=1)

    rospy.Service('/get_red_item', GetRedItem, handleRequest)

    rate = rospy.Rate(0.5)

    cv_bridge = CvBridge()

    print "I'm up!"
    rospy.spin()

if __name__ == '__main__':
    try:
        init_node()
        
    except rospy.ROSInterruptException:
        pass
    