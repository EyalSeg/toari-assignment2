#!/usr/bin/python
import sys

import cv2
import numpy as np

def maskByColor(image, lower_color, upper_color):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    mask = cv2.inRange(img_hsv, lower_color, upper_color)

    return mask

def maskRed(image):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # The red spectrum has two ranges because HSV is circular
    mask1 = cv2.inRange(img_hsv, np.array([0, 70, 70]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(img_hsv, np.array([170, 60, 40]), np.array([180, 255, 255]))

    unified = cv2.bitwise_or(mask1, mask2)

    return unified

def findCircles(image):
    image_cvt = blurImage(image)

    circles = cv2.HoughCircles(image_cvt,cv2.cv.CV_HOUGH_GRADIENT,1,
                             param1=50,param2=30,minRadius=75,maxRadius=0, minDist = 150)

    if (circles is None):
        return []
    
    circles = np.uint16(np.around(circles))
    return circles

def drawCircle(image, circle):
    # draw the outer circle
    cv2.circle(image,(circle[0],circle[1]),circle[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(image,(circle[0],circle[1]),2,(0,0,255),3)

def findBlobs(image):
    params = cv2.SimpleBlobDetector_Params()

    params.filterByInertia = False
    params.filterByConvexity = False

    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 500000000000

    params.filterByCircularity = False

    params.filterByColor = True
    params.blobColor = 255

    detector = cv2.SimpleBlobDetector(params)

    keypoints = detector.detect(image)

    print "found ", len(keypoints)
    if len(keypoints) == 0:
        return None

    return keypoints

def find_largest_contour(img):
    (cnts, _) = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if (len(cnts) == 0):
        return None
    
    return max(cnts, key = cv2.contourArea)

def find_contour_center(contour):
    M = cv2.moments(contour)
    x= int(M["m10"] / M["m00"])
    y = int(M["m01"] / M["m00"])

    return (x, y)

def findLargestBlob(image):
    blobs = findBlobs(image)

    if blobs is None:
        return None
    
    return max(blobs, key =lambda p: p.size)

def blurImage(image):
    image_cvt = cv2.medianBlur(image,5)
    image_cvt = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    return image_cvt

def drawKeypoints(image, keypoints):
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return im_with_keypoints
