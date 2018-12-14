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
from datetime import datetime
import operator

resize = (640, 480)

import numpy as np

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return [rho, phi]

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return [x, y]

#State variables
LOOK_FOR_BALL = 0
CHECK_BALL_LOCATION = 1
ALIGN_WITH_GOAL = 2
MOVE_TO_BALL = 3
HIT_BALL = 4




class RobotSoccer():

    def __init__(self):

        rospy.init_node('robot_soccer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        self.rate = rospy.Rate(20)

        rospy.Subscriber("/camera/image_rect_color", Image, self.camera_cb)
        rospy.Subscriber("/odom", Odometry, self.set_location)

        rospy.Timer(rospy.Duration(0.01), self.look_for_ball)

        self.img_flag = False
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0
        self.theta = 0
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
        Returns the angle in degrees and distance to the ball in mm from
        the neato's current position
        """
        FOV = 62.2 #Field of view in degrees.
        BALL_DI = 190.5 #ball is 7.5 inches in diameter, so 190.5 mm
        FOCAL_LENGTH = 3.04 #mm
        IMAGE_HEIGHT = resize[1]
        SENSOR_HEIGHT = 2.76 #mm

        angle = ((x - float(resize[0]/2)) / resize[0]) * (FOV / 2.)

        distance = (FOCAL_LENGTH * BALL_DI * IMAGE_HEIGHT) / (radius * 2 * SENSOR_HEIGHT)

        return angle, distance

    def random_walk(self):
        """
        Get random linear and angular speeds and send them to publisher
        """
        x = 0.25
        z = random.random()*2 - 1
        self.publish_cmd_vel(x, z)
        time.sleep(0.1)

    def turn_odom(self, angle, tolerance = 0.01, angular_speed = 0.05, angle_max = 0.5):
        """
        Turn a given in radians angle, using odometry information
        """
        angle = -angle
        start_theta = self.theta
        end_theta = add_angles(start_theta, angle)
        if (angle_diff(end_theta, self.theta) > 0):
            turn_direction = 1
        else:
            turn_direction = -1
        print("current theta: %f , desired theta: %f" % (self.theta, end_theta))
        while abs(self.theta - end_theta) > tolerance:
            if abs(self.theta - end_theta) > 0.3:
                turn_speed = 0.3
            else:
                turn_speed = angular_speed
            if (angle_diff(end_theta, self.theta) > 0):
                turn_direction = 1
            else:
                turn_direction = -1
            self.publish_cmd_vel(0, turn_speed*turn_direction)
            print("current theta: %f , desired theta: %f" % (self.theta, end_theta))
            self.rate.sleep()
        self.publish_cmd_vel()

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
        self.move_dist_odom(forward)
        self.publish_cmd_vel()

    def move_backwards(self):
        """ Move the robot backwards, so that the ball is in its vision after being hit. 
        """
        sec = 1
        start = datetime.now()
        self.publish_cmd_vel(-.25)

        while(1):
            delta_t = datetime.now()-start
            delta_s = delta_t.total_seconds()
            if delta_s > sec:
                print(delta_s)
                break
        self.publish_cmd_vel()

    def hit_ball(self, sec = 2.5):
        """
        Full speed forward for (default) 2.5 seconds, then back up to see where the ball went.
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
        for i in range(5):
            self.rate.sleep()
        self.move_backwards()

    def nothing(self, val):
        """
        Dummy function for trackbars
        """
        pass

    def check_ball_loc(self, angle, dist):
        """
        Compares the given angle and distance to the previously found angle and distance. If they are similar enough within 
        tolerance, average the current and previous, and add one to the counter. If not, reset the previous to the current, 
        and zero the counter. We want to have several similar measurements in a row before we take it as true data. 
        """
        angle_certainty_tol = 0.1
        dist_certainty_tol = 0.3
        if ((abs(self.ball_loc[0] - angle) < angle_certainty_tol) and (abs(self.ball_loc[1] - dist) < dist_certainty_tol)):
            self.ball_loc_count += 1
            self.ball_loc[0] = (self.ball_loc[0] + angle) / 2
            self.ball_loc[1] = (self.ball_loc[1] + dist) / 2
            return True
        self.ball_loc[0] = angle
        self.ball_loc[1] = dist
        self.ball_loc_count = 0
        return False

    def center_on_ball(self):
        """
        Centers the robot on the ball and moves forward slowly, keeping centered, until it is "close enough".
        """
        while(1):
            found, angle, dist = self.find_ball(self.img)
            if found:
                self.check_ball_loc(angle, dist)
            if self.ball_loc_count > 2:
                self.ball_loc_count = 0
                if self.ball_loc[0] < 0.1:
                    self.turn_and_forward(self.ball_loc[0], 0.1)
                else:
                    self.turn_odom(self.ball_loc[0])
                if self.ball_loc[1] < 0.5:
                    break


    def look_for_ball(self, val):
        """
        Checks if we have a new image, if we do, see if ball is in image. If ball is, move towards it, otherwise
        random walk.
        """
        distance_threshold = 0.4
        print("state: %d" % self.state)
        if self.state == HIT_BALL:
            self.hit_ball()
            self.state = LOOK_FOR_BALL
            return
        if self.img_flag:
            self.img_flag = False
            found, angle, dist = self.find_ball(self.img)
            cone1, cone2, angle_goal, dist_goal = self.find_goal(self.img)
            if found:
                self.check_ball_loc(angle, dist)
            if self.ball_loc_count > 2:
                if cone1 and cone2:
                    self.ball_loc_count = 0
                    print("Ball Angle: %f Ball Distance: %f Goal Angle: %f Goal Dist: %f" %(self.ball_loc[0], self.ball_loc[1], angle_goal, dist_goal))
                    self.get_in_position(self.ball_loc[0], self.ball_loc[1], angle_goal, dist_goal, distance_threshold)
                    found, angle, dist = self.find_ball(self.img)
                    self.center_on_ball()
                    self.state = HIT_BALL
                elif cone1:
                    self.ball_loc_count = 0
                    print("Ball Angle: %f Ball Distance: %f Goal Angle: %f Goal Dist: %f" %(self.ball_loc[0], self.ball_loc[1], angle_goal, dist_goal))
                    self.get_in_position(self.ball_loc[0], self.ball_loc[1], angle_goal, dist_goal, distance_threshold)
                    self.center_on_ball()
                    self.state = HIT_BALL
        else:
            self.state = LOOK_FOR_BALL
            self.random_walk()

    def get_in_position(self, angle_b, dist_b, angle_g, dist_g, distance_threshold):
        """
        Positions the robot in the prime position to hit the ball between the goalposts. 
        angle_b, dist_b : angle and distance of ball. 
        angle_g, dist_g : angle and distance of goal
        distance_threshold: how far we want to be from the ball when we run to hit it. 
        """

        frame_shift = math.pi/2
        ball_pos = pol2cart(dist_b, angle_b+frame_shift)
        goal_pos = pol2cart(dist_g, angle_g+frame_shift)
        ball_pos[0] = -ball_pos[0]
        goal_pos[0] = -goal_pos[0]
        v = (goal_pos[0] - ball_pos[0], goal_pos[1] - ball_pos[1])
        norm = np.linalg.norm(v)
        u = (v[0]/norm, v[1]/norm)

        desired_pos = (ball_pos[0] - distance_threshold*u[0], ball_pos[1] - distance_threshold*u[1])
        initial_turn = math.atan2(desired_pos[1], desired_pos[0])
        self.turn_odom(initial_turn)

        dist_to_desired = (math.sqrt(desired_pos[0]**2 + desired_pos[1]**2))
        self.move_dist_odom(dist_to_desired)

        norm_angle = math.atan2(u[1],u[0])
        last_turn = math.pi/2 - abs(norm_angle) + abs(initial_turn)

        self.turn_odom(-last_turn)

        while(1):
            found, angle, dist = self.find_ball(self.img)
            if found:
                self.turn_odom(angle)
                if angle < 0.1:
                    break
            else:
                self.turn_odom(-last_turn/10)


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
                        if abs(r - contour_r) < (contour_r / 3):
                            # check that the circle center is near the contour center
                            if ((x + r > contour_x > x - r) and (y + r > contour_y > y - r)):

                                # get the angle and distance of the ball
                                angle, dist = self.getAngleDist(float(contour_x), float(contour_r))

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

                                # adjust angle and dist for accuracy and units
                                scale = angle/5.
                                angle = angle + scale
                                angle = math.radians(angle) #radians
                                dist = dist / 1000. # meters

                                return True, angle, dist

        visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
        vis = np.concatenate((visimg,crop_img), axis=1)
        cv2.imshow('image', vis)
        cv2.waitKey(1)
        return False, 0, 0

    def check_if_cone(self, contour):
        """ Takes in a contour, and checks to see if it's a cone.
            This is done pretty roughly - it finds the vertices of the given contour
            and then counts them. If there are four or five, it counts that as a cone, because
            in my testing cones usually came up with four or five verticies. 
            Outputs boolean for if cone, and then the minimum and maximum x and y values in the vertices. 
        """
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        max_x = 0
        max_y = 0
        min_x = 1000
        min_y = 1000
        if len(approx) in [4,5]:
            for point in approx:
                if point[0][0] > max_x:
                    max_x = point[0][0]
                if point[0][1] > max_y:
                    max_y = point[0][1]
                if point[0][0] < min_x:
                    min_x = point[0][0]
                if point[0][1] < min_y:
                    min_y = point[0][1]
            return True, (max_x, max_y), (min_x, min_y)
        else: 
            return False, (0, 0), (0, 0)



    def find_goal(self, base):
        """
        Tries to locate the goal within the given image. If it finds two cones, outputs the angle (rads)
        and distance (m) to the center point between the cones. If it finds one cone, it outputs the angle and 
        distance to that cone. It also outputs two booleans for whether cones have been found, one for each cone. 

        """

        crop_img = base[150:, :]
        img = cv2.medianBlur(crop_img.copy(),5)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([116, 122, 97])
        upper = np.array([143, 255, 255])

        outimg = cv2.inRange(img.copy(), lower, upper)
        outimg = cv2.erode(outimg, None, iterations=3)
        outimg = cv2.dilate(outimg, None, iterations=2)

        contours = cv2.findContours(outimg.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        cone1 = False
        cone2 = False

        # Draw bounding circle around largest contour
        if len(contours) > 0:
            contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
            largest_contour = contours_sorted[0]
            cone1, corner1, minvals1 = self.check_if_cone(largest_contour)

            xdiff1 = corner1[0] - minvals1[0]
            ydiff1 = corner1[1] - minvals1[1]

            dist1_inches = 2160*np.power(ydiff1,-0.992)
            dist1_m = dist1_inches / 39.37

            angle1, trash = self.getAngleDist(corner1[0], 1)

            if len(contours) > 1:
                second_largest_contour = contours_sorted[1]
                cone2, corner2, minvals2 = self.check_if_cone(second_largest_contour)

                if cone1 and cone2:

                    xdiff2 = corner2[0] - minvals2[0]
                    ydiff2 = corner2[1] - minvals2[1]
                    dist2_inches = 2160*np.power(ydiff2,-0.992)
                    dist2_m = dist2_inches / 39.37

                    center = (int((corner1[0] + corner2[0])/2), int((corner1[1] + corner2[1])/2))

                    center_angle, trash = self.getAngleDist(center[0], 1)
                    center_dist = (dist1_m + dist2_m) / 2
                    cv2.circle(crop_img, center, 5, (0, 255, 0), 4)
                    print(center_angle, center_dist)
                    visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
                    vis = np.concatenate((visimg,crop_img), axis=1)
                    cv2.imshow('image', vis)
                    cv2.waitKey(1)
                    center_angle_rad = math.radians(center_angle)
                    return cone1, cone2, center_angle_rad, center_dist

            visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
            vis = np.concatenate((visimg,crop_img), axis=1)
            cv2.imshow('image', vis)
            cv2.waitKey(1)
            angle1_rad = math.radians(angle1)
            return cone1, cone2, angle1_rad, dist1_m

        visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
        vis = np.concatenate((visimg,crop_img), axis=1)
        cv2.imshow('image', vis)
        cv2.waitKey(1)
        return False, False, 0, 0




    def run(self):
        if not self.startup:
            self.rate.sleep()  # wait to start until we're getting information
        for i in range(10):
            self.rate.sleep()
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
  rs = RobotSoccer()
  rs.run()
