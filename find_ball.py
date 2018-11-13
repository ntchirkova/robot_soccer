#!/usr/bin/env python

import os, sys
from os import path
import csv
import cv2 as cv
import numpy as np

resize = (160, 120)
boxlist = []

def rad2box(x, y, rad):
    """
    Returns the coordinates of the upper left and
    bottom right coordinates of the bounding box
    for the ball coordinates based on
    the radius of the ball and center coordinates
    """
    offset = float(rad)
    box = [x - offset, y + offset, x + offset, y - offset]
    return box

def getAngleDist(x,radius):
    """
    Returns the angle and distance to the ball from
    the neato's current position
    """
    BALL_DI = 7.5 #ball is 7.5 inches in diameter.
    FOV = 60. #Field of view in degrees.
    FOCAL = 150.*12./BALL_DI #The first number is the measured width in pixels of a picture taken at the second number's distance (inches).
    center = resize[0]/2
    difference = int(x) - center
    distance = BALL_DI * FOCAL / float(2.*radius)
    #Because the camera isn't a 1:1 camera, it has a 60 degree FoV, which makes angle calculations easier because angle
    #is directly proportional to distance from center.
    angle = float(difference)/160. * (FOV/2.) #scale to half of FoV

    return angle, distance

for filename in os.listdir("data"):
    if filename.endswith(".png") or filename.endswith(".JPG") or filename.endswith(".jpg"):
        print(str(filename))
        filepath = os.path.join("data", filename)
        base = cv.imread(filepath, 1)
        base = cv.resize(base, resize, interpolation = cv.INTER_AREA)
        img = cv.medianBlur(base.copy(),5)
        img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        lower = np.array([40, 10, 10])
        upper = np.array([110, 255, 255])

        img = cv.inRange(img, lower, upper)
        img = cv.erode(img, None, iterations=3)
        img = cv.dilate(img, None, iterations=2)

        contours = cv.findContours(img.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # Draw bounding circle around largest contour
        if len(contours) > 0:
            largest_contour = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)

            # Draw circles on image to represent the ball
            if radius > 10:
                # cv.circle(base, (int(x), int(y)), int(radius),
                #     (0, 255, 255), 2)
                # cv.circle(base, (int(x), int(y)), 5,
                #     (0, 0, 255), -1)
                print("coord:" + str(x) + "," + str(y) + " radius:" + str(radius))
                angle, dist = getAngleDist(float(x), float(radius))
                print("angle:" + str(angle) + " distance:" + str(dist))

                box = rad2box(float(x), float(y), float(radius))
                box.append(radius)
                box.append(float(angle))
                box.append(dist)
                cv.rectangle(base, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255,0,0), 2)
                boxlist.append(box)


                cv.line(base, (int(x), int(y)), (160, 240), (255,0,0), 1, 8, 0)

            else:
                print("Radius too small!")
        continue

    else:
        continue



with open('ball_info.csv', mode='w') as ball_info:
    ball_writer = csv.writer(ball_info, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for i in boxlist:
        ball_writer.writerow([i[0]] + [i[1]] + [i[2]] + [i[3]] + [i[4]] + [i[5]] + [i[6]])



cv.imshow('detected circles', base)
cv.waitKey(25)
cv.destroyAllWindows()
