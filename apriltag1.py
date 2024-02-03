#!/bin/python3.6

# based on PyImageSearch video https://www.youtube.com/watch?v=H77ieFq5mQ8

import apriltag
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="path to input image containing AprilTag")
ap.add_argument("-d", "--display", required=True, help="enable/disable display")
args = vars (ap.parse_args())

#load the input image and convert it to grayscale
print("[INFO] loading image...")
image = cv2.imread(args["image"])
display = bool(args["display"])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# define the AprilTags detector options and then detect the AprilTags # in the input image
print("[INFO] detecting AprilTags...")
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
results = detector.detect(gray)
print("[INFO] {} total AprilTags detected".format(len(results)))

#loop over the AprilTag detection results
for r in results:
    # extract the bounding box (x, y)-coordinates for the AprilTag # and convert each of the (x, y)-coordinate pairs to integers (ptA, ptB, ptC, ptD) = r.corners
    ptB = (int(ptB[0]), int(ptB [1])) 
    ptC = (int(ptC[0]), int(ptC [1])) 
    ptD = (int(ptD[0]), int(ptD [1])) 
    ptA = (int(ptA[0]), int(ptA [1]))

    # draw the bounding box of the AprilTag detection 
    cv2.line(image, ptA, ptB, (0, 255, 0), 2) 
    cv2.line(image, ptB, ptC, (0, 255, 0), 2) 
    cv2.line(image, ptC, ptD, (0, 255, 0), 2) 
    cv2.line(image, ptD, ptA, (0, 255, 0), 2)

    # draw the center (x, y)-coordinates of the AprilTag (cX, cY) = (int(r.center[0]), int(r.center[1]))
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

    # draw the tag family on the image
    tagFamily = r.tag_family.decode("utf-8")
    cv2.putText(image, tagFamily, (ptA[0], ptA[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    print(f"[INFO] tag family: {tagFamily}")

if display:
    cv2.imshow("Image", image)
    cv2.waitKey(0)
else:
    base, extension = args["image"].rsplit('.', 1)
    outimage = f"{base}_out.{extension}"
    cv2.imwrite(outimage, image)