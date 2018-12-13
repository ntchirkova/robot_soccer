#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Image

import random

import sys

import cv2
from cv_bridge import CvBridge

from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
import math

from helper_functions import angle_diff, add_angles, angle_normalize, calibrate_for_lighting

import time

resize = (640, 480)


#State variables
LOOK_FOR_BALL = 0
CHECK_BALL_LOCATION = 1
MOVE_TO_BALL = 2
HIT_BALL = 3


class RobotSoccer():

    def __init__(self, da):

        rospy.init_node('robot_soccer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        self.rate = rospy.Rate(20)

        rospy.Subscriber("/camera/image_raw", Image, self.camera_cb)
        rospy.Subscriber("/odom", Odometry, self.set_location)

        rospy.Timer(rospy.Duration(0.01), self.look_for_ball)

        self.img_flag = False
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.desired_angle = da
        self.angle_threshold = 2
        self.FOC = 503.0332069533456
        self.ball_width = .1905 # in meters
        self.cx_offset = 321.5712473375021
        self.startup = False # Turn to true when we start getting information
        self.i = 0
        self.hough_params = [100,50,200,60,50,100]  #[90,100,200,100,50,100]

        self.ball_loc = [0., 0.]
        self.ball_loc_count = 0

        self.state = LOOK_FOR_BALL


    def publish_cmd_vel(self, x = 0, z = 0):
        """
        Publish the given speed and angular velocity
        """
        outmsg = Twist()
        outmsg.linear.x = x
        outmsg.angular.z = z
        self.pub.publish(outmsg)


    def set_location(self, odom):
        """
        Convert pose (geometry_msgs.Pose) to a (x, y, theta) tuple
        Constantly being called as it is the callback function for this node's subscription

        odom is Neato ROS' nav_msgs/Odom msg composed of pose and orientation submessages
        """
        if self.img_flag:
            self.startup = True # We have started to receive information
        pose = odom.pose.pose
        orientation_tuple = (pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = angles[2]
        return (pose.position.x, pose.position.y, angles[2])


    def camera_cb(self, img):
        img = self.bridge.imgmsg_to_cv2(img, desired_encoding="rgb8")
        img = cv2.resize(img, resize, interpolation=cv2.INTER_AREA)
        img = np.array(img)
        self.img = img
        self.img_flag = True


    def rad2box(self, x, y, radius):
        """
        Returns the coordinates of the upper left and
        bottom right coordinates of the bounding box
        for the ball coordinates based on
        the radius of the ball and center coordinates
        """
        box = [int(x - radius), int(y + radius), int(x + radius), int(y - radius)]
        return box

    def getAngleDist(self, x, radius):
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

    def getAngleDist2(self, x, widthP):
        """
        Returns angle and distance based on x coordinate and width in pixels.
        """
        Z = 2 * self.FOC * self.ball_width / widthP
        X = 2 * (x - self.cx_offset) / self.FOC
        dist = math.sqrt(X**2+Z**2)
        angle = math.atan2(X,Z)

        return angle, dist

    def random_walk(self):
        """
        Get random linear and angular speeds and send them to publisher
        """
        x = 0.25
        z = random.random()*2 - 1
        self.publish_cmd_vel(x, z)
        time.sleep(0.1)

    def turn_timed(self, angle):
        """
        Turns a given angle, based on the speed and timing information
        """
        angle_vel = 28.23 # degrees per second
        turn_time = angle/28.23
        twist = self.make_twist(0, .5)
        start = datetime.now()
        self.pub.publish(twist)
        while True:
            delta_t = datetime.now()-start
            delta_s = delta_t.total_seconds()
            if delta_s > turn_time:
                print(delta_s)
                break

        self.pub.publish(self.stop)

    def move_forward_timed(self, distance):
        """
        Takes a distance in meters and moves it forward, using time info. Works under the
        timing that 0.5 cmd_vel = 1 ft/s.
        """
        speed = 0.5
        m2ft = 0.3048
        dist_ft = distance/m2ft
        sec = dist_ft

        start = datetime.now()
        go = self.make_twist(speed, 0)
        self.pub.publish(go)

        while(1):
            delta_t = datetime.now()-start
            delta_s = delta_t.total_seconds()
            if delta_s > sec:
                print(delta_s)
                break

        self.pub.publish(self.stop)

    def turn_odom(self, angle, tolerance = 0.01, angular_speed = 0.2, angle_max = 0.5):
        """
        Turn a given angle, using odometry information
        """
        if abs(angle) > angle_max:
            angle = angle_max * np.sign(angle)
        angle = -angle
        start_theta = self.theta
        end_theta = add_angles(start_theta, angle)
        if (angle_diff(end_theta, self.theta) > 0):
            turn_direction = 1
        else:
            turn_direction = -1
        print("current theta: %f , desired theta: %f" % (self.theta, end_theta))
        if abs(self.theta - end_theta) > tolerance:
            if (angle_diff(end_theta, self.theta) > 0):
                turn_direction = 1
            else:
                turn_direction = -1
            self.publish_cmd_vel(0, angular_speed*turn_direction)
            print("current theta: %f , desired theta: %f" % (self.theta, end_theta))
            self.rate.sleep()

    def move_dist_odom(self, forward, tolerance = 0.05, linear_speed = 0.1):
        """
        Move a given distance forward, using odometry information
        """
        print("moving forward %f meters!" % forward)
        start_x = self.x
        start_y = self.y
        end_x = start_x + math.cos(self.theta) * forward
        end_y = start_y + math.sin(self.theta) * forward
        print("current x: %f , current y: %f , desired x: %f , desired y: %f" % (self.x, self.y, end_x, end_y))
        while (abs(self.x - end_x) > tolerance) or (abs(self.y - end_y) > tolerance):
            self.publish_cmd_vel(linear_speed, 0)
            print("current x: %f , current y: %f , desired x: %f , desired y: %f" % (self.x, self.y, end_x, end_y))
            self.rate.sleep()
        self.publish_cmd_vel()

    def turn_and_forward(self, angle, forward):
        """
        Turn and move forward a given amount
        """
        center_tolerance = 0.1
        self.turn_odom(angle)
        if abs(angle) < 0.01:
            print("moving forward!")
            self.publish_cmd_vel(0.5, 0)
            time.sleep(0.3)
        self.publish_cmd_vel()
            #self.move_dist_odom(forward)

    def move_backwards(self):
        """ Move the robot backwards 1 meter so it can see where the ball went. 
        """
        sec = 1
        start = datetime.now()
        self.publish_cmd_vel(-1.)

        while(1):
            delta_t = datetime.now()-start
            delta_s = delta_t.total_seconds()
            if delta_s > sec:
                print(delta_s)
                break
        self.publish_cmd_vel()

    def hit_ball(self, sec = 0.5):
        """
        Full speed forward for (default) 0.5 seconds, then back up to see where the ball went. 
        """
        start = datetime.now()
        self.publish_cmd_vel(2)

        while(1):
            delta_t = datetime.now()-start
            delta_s = delta_t.total_seconds()
            if delta_s > sec:
                print(delta_s)
                break
        self.publish_cmd_vel()
        self.state = LOOK_FOR_BALL
        self.move_backwards()

    def nothing(self, val):
        pass

    def check_ball_loc(self, angle, dist):
        angle_certainty_tol = 0.2
        dist_certainty_tol = 0.3
        if ((abs(self.ball_loc[0] - angle) < angle_certainty_tol) and (abs(self.ball_loc[1] - dist) < dist_certainty_tol)):
            self.ball_loc_count += 1
            self.ball_loc[0] = (self.ball_loc[0] + angle) / 2.
            self.ball_loc[1] = (self.ball_loc[1] + dist) / 2.
            return True
        self.ball_loc[0] = angle
        self.ball_loc[1] = dist
        return False

    def look_for_ball(self, val):
        """
        Checks if we have a new image, if we do, see if ball is in image. If ball is, move towards it, otherwise 
        random walk.
        """
        print(self.state)
        distance_threshold = 0.3
        if self.img_flag:
            self.img_flag = False
            found, angle, dist = self.find_ball(self.img)
            if found:
                good_loc = self.check_ball_loc(angle, dist)
                print(good_loc)
                if self.ball_loc_count > 2:
                    self.state = MOVE_TO_BALL
                    if self.ball_loc[1] < distance_threshold:
                        self.state = HIT_BALL
                    self.turn_and_forward(self.ball_loc[0], self.ball_loc[1])
            else:
                if self.state == HIT_BALL:
                    self.hit_ball()
                else:
                    self.state = LOOK_FOR_BALL
                    self.random_walk()

    def find_ball(self, base, calibrate = False):
        """
        Returns flag for whether ball was successfully found, and then the angle and distance if it was.
        """
        if calibrate:
            # create window with trackbars to adjust hough circle params
            cv2.namedWindow('image')

            # create trackbars for color change
            cv2.createTrackbar('dp','image',self.hough_params[0],100,self.nothing)
            cv2.createTrackbar('min_dist','image',self.hough_params[1],200,self.nothing)
            cv2.createTrackbar('param1','image',self.hough_params[2],200,self.nothing)
            cv2.createTrackbar('param2','image',self.hough_params[3],200,self.nothing)
            cv2.createTrackbar('minr','image',self.hough_params[4],200,self.nothing)
            cv2.createTrackbar('maxr','image',self.hough_params[5],200,self.nothing)

            self.hough_params[0] = int(cv2.getTrackbarPos('dp','image'))
            self.hough_params[1] = int(cv2.getTrackbarPos('min_dist','image'))
            self.hough_params[2] = int(cv2.getTrackbarPos('param1','image'))
            self.hough_params[3] = int(cv2.getTrackbarPos('param2','image'))
            self.hough_params[3] = int(cv2.getTrackbarPos('minr','image'))
            self.hough_params[3] = int(cv2.getTrackbarPos('maxr','image'))


        crop_img = base[150:, :]
        img = cv2.medianBlur(crop_img.copy(),5)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([42, 27, 41])
        upper = np.array([84, 255, 255])

        outimg = cv2.inRange(img.copy(), lower, upper)
        outimg = cv2.erode(outimg, None, iterations=3)
        outimg = cv2.dilate(outimg, None, iterations=2)

        contours = cv2.findContours(outimg.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # Draw bounding circle around largest contour
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            ((contour_x, contour_y), contour_r) = cv2.minEnclosingCircle(largest_contour)

            # If the largest contour is reasonably big
            if contour_r > 10:

                # detect circles in the image
                dp = self.hough_params[0] / 50.
                if (dp < 1):
                    dp = 1.
                gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
                circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp, self.hough_params[1], self.hough_params[2], self.hough_params[3])

                # ensure at least some circles were found
                if (circles is not None) and (type(circles[0][0]) is not np.float64):
                    # convert the (x, y) coordinates and radius of the circles to integers
                    (circles) = np.round(circles[0, :]).astype("int")

                    # loop over the (x, y) coordinates and radius of the circles
                    for (x, y, r) in circles:

                        # check that the contour radius and circle radius are similar
                        if abs(r - contour_r) < 30:
                            # check that the circle center is near the contour center
                            if ((x + r > contour_x > x - r) and (y + r > contour_y > y - r)):

                                # get the angle and distance of the ball 
                                angle, dist = self.getAngleDist2(float(contour_x), float(contour_r))

                                # draw the visualizations on the output image
                                # circles
                                cv2.circle(crop_img, (x, y), r, (0, 255, 0), 4)
                                cv2.rectangle(crop_img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                                # contour rectangle
                                box = self.rad2box(contour_x, contour_y, contour_r)
                                cv2.rectangle(crop_img, (box[0], box[1]), (box[2], box[3]), (255,0,0), 2)

                                # show the output image
                                visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
                                vis = np.concatenate((visimg, crop_img), axis=1)
                                cv2.imshow('image', vis)
                                cv2.waitKey(1)
                                return True, angle, dist
        visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
        vis = np.concatenate((visimg,crop_img), axis=1)
        cv2.imshow('image', vis)
        cv2.waitKey(1)
        return False, 0, 0

    def move_to_ball(self, angle):
        if angle < self.desired_angle: #move left
            self.publish_cmd_vel(.1,.1)
        else: #move right
            self.publish_cmd_vel(.1,-.1)

    def run(self):
        #base = cv2.imread("../data/testballcenter.jpg", 1)
        #found, angle, dist = self.find_ball(base)
        if not self.startup:
            self.rate.sleep()  # wait to start until we're getting information
        while not rospy.is_shutdown():
            # if self.img_flag:
            #     found, angle, dist = self.find_ball(self.img)
            #     dist_inches = dist / 25.4
            #     angle_rad = math.radians(angle)
            #     if found:
            #         print("angle: %f ,  distance_inches: %f " % (angle, dist_inches))
            #         self.turn_and_forward(angle_rad, dist)
            #         # if angle >= (self.desired_angle - self.angle_threshold) and angle <= (self.desired_angle + self.angle_threshold):
            #         #     #annas code
            #         # else:
            #         #     self.move_to_ball(angle)
            #     else:
            #         self.publish_cmd_vel()
            #     self.img_flag = False
            self.rate.sleep()


if __name__ == "__main__":
  rs = RobotSoccer(0.3)
  rs.run()
