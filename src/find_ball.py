#!/usr/bin/env python

import cv2
import numpy as np

resize = (640, 480)
boxlist = []

def getAngleDist(x, radius):
	"""
	Returns the angle and distance to the ball from
	the neato's current position
	"""
	FOV = 62.2 #Field of view in degrees.
	BALL_DI = 190.5 #ball is 7.5 inches in diameter, so 190.5 mm
	FOCAL_LENGTH = 3.04 #mm
	IMAGE_HEIGHT = resize[1]
	SENSOR_HEIGHT = 2.76 #mm

	angle = ((x - float(resize[0]/2)) / resize[0]) * (FOV / 2.)

	distance = (FOCAL_LENGTH * BALL_DI * IMAGE_HEIGHT) / (radius * 2 * SENSOR_HEIGHT)

	# BALL_DI = 7.5 #ball is 7.5 inches in diameter.
	# FOV = 62.2 #Field of view in degrees.
	# FOCAL = 150.*12./BALL_DI #The first number is the measured width in pixels of a picture taken at the second number's distance (inches).
	# center = resize[0]/2
	# difference = int(x) - center
	# distance = BALL_DI * FOCAL / float(2.*radius)
	# #Because the camera isn't a 1:1 camera, it has a 60 degree FoV, which makes angle calculations easier because angle
	# #is directly proportional to distance from center.
	# angle = float(difference)/160. * (FOV/2.) #scale to half of FoV

	return angle, distance

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

def nothing(x):
	pass


def find_ball(base):
	img = cv2.medianBlur(base.copy(),5)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	cv2.namedWindow('image')

	# create trackbars for color change
	cv2.createTrackbar('HL','image',42,255,nothing)
	cv2.createTrackbar('HU','image',84,255,nothing)
	cv2.createTrackbar('SL','image',27,255,nothing)
	cv2.createTrackbar('SU','image',255,255,nothing)
	cv2.createTrackbar('VL','image',41,255,nothing)
	cv2.createTrackbar('VU','image',255,255,nothing)


	while(1):
		rectimg = base.copy()
		# get current positions of four trackbars
		hl = int(cv2.getTrackbarPos('HL','image'))
		hu = int(cv2.getTrackbarPos('HU','image'))
		sl = int(cv2.getTrackbarPos('SL','image'))
		su = int(cv2.getTrackbarPos('SU','image'))
		vl = int(cv2.getTrackbarPos('VL','image'))
		vu = int(cv2.getTrackbarPos('VU','image'))

		lower = np.array([hl, sl, vl])
		upper = np.array([hu, su, vu])

		outimg = cv2.inRange(img.copy(), lower, upper)
		outimg = cv2.erode(outimg, None, iterations=3)
		outimg = cv2.dilate(outimg, None, iterations=2)

		contours = cv2.findContours(outimg.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None

		# Draw bounding circle around largest contour
		if len(contours) > 0:
			largest_contour = max(contours, key=cv2.contourArea)
			((contour_x, contour_y), contour_r) = cv2.minEnclosingCircle(largest_contour)

			# Draw circles on image to represent the ball
			if contour_r > 10:
				#print "coord:" + str(x) + "," + str(y) + " radius:" + str(radius)
				angle, dist = getAngleDist(float(contour_x), float(contour_r))
				#print "angle:" + str(angle) + " distance:" + str(dist)

				box = rad2box(float(contour_x), float(contour_y), float(contour_r))
				box.append(contour_r)
				box.append(float(angle))
				box.append(dist)
				cv2.rectangle(rectimg, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255,0,0), 2)
				boxlist.append(box)

				#cv2.line(base, (int(x), int(y)), (160, 240), (255, 0, 0), 1, 8, 0)
				visimg = cv2.cvtColor(outimg,cv2.COLOR_GRAY2RGB)
				vis = np.concatenate((visimg, rectimg), axis=1)

		gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
		# gray = cv2.bitwise_and(gray, gray, mask= outimg)
		# detect circles in the image
		circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)
		 
		# ensure at least some circles were found
		if circles is not None:
			# convert the (x, y) coordinates and radius of the circles to integers
			circles = np.round(circles[0, :]).astype("int")
		 
			# loop over the (x, y) coordinates and radius of the circles
			for (x, y, r) in circles:
				min_x = x - r
				max_x = x + r

				if (max_x > contour_x > min_x):
					# draw the circle in the output image, then draw a rectangle
					# corresponding to the center of the circle
					cv2.circle(rectimg, (x, y), r, (0, 255, 0), 4)
					cv2.rectangle(rectimg, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
		 
			# show the output image
			visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
			vis = np.concatenate((visimg, rectimg), axis=1)
		else:
			visimg = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
			vis = np.concatenate((visimg,rectimg), axis=1)


		cv2.imshow('image', vis)
		k = cv2.waitKey(1) & 0xFF
		if k == 27:
		    break
	cv2.destroyAllWindows()

def check_channels(img):

	img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	b = img.copy()
	# set green and red channels to 0
	b[:, :, 1] = 0
	b[:, :, 2] = 0


	g = img.copy()
	# set blue and red channels to 0
	g[:, :, 0] = 0
	g[:, :, 2] = 0

	r = img.copy()
	# set blue and green channels to 0
	r[:, :, 0] = 0
	r[:, :, 1] = 0


	# RGB - Blue
	cv2.imshow('B-RGB', b)

	cv2.waitKey(0)

	# RGB - Green
	cv2.imshow('G-RGB', g)
	cv2.waitKey(0)

	# RGB - Red
	cv2.imshow('R-RGB', r)

	cv2.waitKey(0)



if __name__ == '__main__':
	img = cv2.imread('ball_distraction.png')
	find_ball(img)
	#check_channels(img)