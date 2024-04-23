#!/bin/python3.6
"""
This file holds the draw functions for the AprilTag detection.

Functions:
- drawCorners: Draws lines between the corners of the AprilTag on the image.
- drawCenter: Draws the center of the AprilTag on the image.
- drawName: Draws the tag family name on the image.
- distance: Calculates the distance between two points.

Usage:
- Functions to be imported to to the main script for drawing the AprilTag detection.
"""

import cv2
import numpy as np

def drawCorners(img, tag):
    # extract the bounding box (x, y)-coordinates for the AprilTag # and convert each of the (x, y)-coordinate pairs to integers 
    (ptA, ptB, ptC, ptD) = tag.corners
    ptB = (int(ptB[0]), int(ptB [1])) 
    ptC = (int(ptC[0]), int(ptC [1])) 
    ptD = (int(ptD[0]), int(ptD [1])) 
    ptA = (int(ptA[0]), int(ptA [1]))
    # draw the bounding box of the AprilTag detection 
    cv2.line(img, ptA, ptB, (255, 255,   0), 1) 
    cv2.line(img, ptB, ptC, (255,   0,   0), 1) 
    cv2.line(img, ptC, ptD, (  0, 255,   0), 1) 
    cv2.line(img, ptD, ptA, (  0,   0, 255), 1)
    return img

def drawCenter(img, tag):
    (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
    cv2.circle(img, (cX, cY), 2, (255, 0, 255), -1)
    return img

def drawName(img, tag, corners):
    # draw the tag family on the image
    tagFamily = tag.tag_family.decode("utf-8")
    cv2.putText(img, tagFamily, (corners[0][0], corners[0][1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return img, tagFamily

def distance(p1:tuple, p2:tuple):
    return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)