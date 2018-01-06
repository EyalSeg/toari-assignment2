#!/usr/bin/python

#############################
# this file is old and out of use
# it remains here in case some functionality might be useful in the future.
############################

import sys
import rospy

import cv2
import numpy as np
import cv_helper

from ass2.srv import *

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

def onNewImage(msg):
    global cv_bridge, publisher, frameHasRedItem, redItem_angleOffset

    try:
        img = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        print img[0][0]
        item = find_object(img)

        if item is None:
            frameHasRedItem = False
            return
        
        frameHasRedItem = True
        (height, width) = img.shape[:2]
        # distance_from_center_width = item.pt[0] - width/2

        redItem_angleOffset = img_coordiante_to_angle(img, item.pt)
        msg = Float32()
        msg.data = redItem_angleOffset
        publisher.publish(msg)

        # im_with_keypoints = cv2.drawKeypoints(img, [item], np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # cv2.imshow("Keypoints", im_with_keypoints)
        # cv2.waitKey(0)

        
    except CvBridgeError as e:
      print(e)
    

def handleRequest(req):
    global frameHasRedItem, redItem_angleOffset
    response = GetRedItemResponse()

    response.has_red_item = frameHasRedItem
    if frameHasRedItem:
        response.angle_offset = redItem_angleOffset

    return response
    
def init_node():
    global cv_bridge, publisher, frameHasRedItem, redItem_angleOffset
    cv_bridge = CvBridge()

    rospy.init_node('Camera_listener', anonymous=True)
    
    publisher = rospy.Publisher('object_detection/red_objects', Float32, queue_size=1)
    rospy.Service('get_red_item', GetRedItem, handleRequest)
    #rospy.Subscriber("/front_camera/image_raw", Image, onNewImage)
    rospy.Subscriber("/kinect2/qhd/image_color", Image, onNewImage)
    rate = rospy.Rate(0.5)

    rate.sleep()
    rospy.spin()


def find_object(img):
   
    masked_img = cv_helper.maskRed(img)
    blobs = cv_helper.findBlobs(masked_img)

    if blobs is None:
        return None

    maxBlob = max(blobs, key =lambda p: p.size)


    return maxBlob


def img_coordiante_to_angle(img, coordinate):
    (height, width) = img.shape[:2]
    camera_degrees = 90.0

    distance_from_center_width = coordinate[0] - width/2

    return camera_degrees / width * distance_from_center_width



if __name__ == '__main__':
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass
    