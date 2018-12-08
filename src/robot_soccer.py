#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Image

import sys

import cv2
from cv_bridge import CvBridge

from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
import math

from helper_functions import angle_diff, add_angles, angle_normalize

resize = (640, 480)
boxlist = []

class RobotSoccer():

    def __init__(self, da):

        rospy.init_node('robot_soccer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        self.rate = rospy.Rate(20)

        rospy.Subscriber("/camera/image_raw", Image, self.camera_cb)
        rospy.Subscriber("/odom", Odometry, self.set_location)

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


    def rad2box(self, x, y, rad):
        """
        Returns the coordinates of the upper left and
        bottom right coordinates of the bounding box
        for the ball coordinates based on
        the radius of the ball and center coordinates
        """
        offset = float(rad)
        box = [x - offset, y + offset, x + offset, y - offset]
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
        angle = math.arctan2(X/Z)

        return angle, dist

    def random_walk(self):
        """
        Get random linear and angular speeds and send them to publisher
        """
        x = random.random()*2 - 1
        z = random.random()*2 - 1
        self.publish_cmd_vel(x, z)

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

    def turn_odom(self, angle, tolerance = 0.01, angular_speed = 0.2):
        """
        Turn a given angle, using odometry information
        """
        angle = -angle
        start_theta = self.theta
        end_theta = add_angles(start_theta, angle)
        if (angle_diff(end_theta, self.theta) > 0):
            turn_direction = 1
        else:
            turn_direction = -1
        while abs(self.theta - end_theta) > tolerance:
            if (angle_diff(end_theta, self.theta) > 0):
                turn_direction = 1
            else:
                turn_direction = -1
            self.publish_cmd_vel(0, angular_speed*turn_direction)
            print("current theta: %f , desired theta: %f" % (self.theta, end_theta))
            self.rate.sleep()
        self.publish_cmd_vel()

    def move_dist_odom(self, forward, tolerance = 0.05, linear_speed = 0.1):
        """
        Move a given distance forward, using odometry information
        """
        start_x = self.x
        start_y = self.y
        end_x = start_x + math.cos(self.theta) * forward
        end_y = start_y + math.sin(self.theta) * forward
        while (abs(self.x - end_x) > tolerance) or (abs(self.y - end_y) > tolerance):
            self.publish_cmd_vel(linear_speed, 0)
            print("current x: %f , current y: %f , desired x: %f , desired y: %f" % (self.x, self.y, end_x, end_y))
            self.rate.sleep()
        self.publish_cmd_vel()

    def turn_and_forward(self, angle, forward):
        """
        Turn and move forward a given amount
        """
        print("moving!")
        self.turn_odom(angle)
        self.move_dist_odom(forward)


    def find_ball(self, base):
        """
        Returns flag for whether ball was successfully found, and then the angle and distance if it was.
        """
        try:
            rectimg = base.copy()
            img = cv2.medianBlur(base.copy(),5)
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

                            # Draw circles on image to represent the ball
                            if contour_r > 10:
                                #print "coord:" + str(x) + "," + str(y) + " radius:" + str(radius)
                                angle, dist = self.getAngleDist2(float(contour_x), float(contour_r))
                                #print "angle:" + str(angle) + " distance:" + str(dist)

                                box = self.rad2box(float(contour_x), float(contour_y), float(contour_r))
                                box.append(contour_r)
                                box.append(float(angle))
                                box.append(dist)
                                cv2.rectangle(rectimg, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255,0,0), 2)
                                boxlist.append(box)

                                #cv2.line(base, (int(x), int(y)), (160, 240), (255, 0, 0), 1, 8, 0)
                                visimg = cv2.cvtColor(outimg,cv2.COLOR_GRAY2RGB)
                                vis = np.concatenate((visimg, rectimg), axis=1)


                                # show the output image
                                visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
                                vis = np.concatenate((visimg, rectimg), axis=1)
                                cv2.imshow('image', vis)
                                cv2.waitKey(1)
                                return True, angle, dist
            visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
            vis = np.concatenate((visimg,rectimg), axis=1)
            cv2.imshow('image', vis)
            cv2.waitKey(1)
            return False, 0, 0
        except AttributeError:
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
            if self.img_flag:
                found, angle, dist = self.find_ball(self.img)
                print(found)
                dist_inches = dist / 25.4
                angle_rad = math.radians(angle)
                if found:
                    print("angle: %f ,  distance_inches: %f " % (angle, dist))
                    #self.turn_and_forward(angle_rad, dist)
                    # if angle >= (self.desired_angle - self.angle_threshold) and angle <= (self.desired_angle + self.angle_threshold):
                    #     #annas code
                    # else:
                    #     self.move_to_ball(angle)
                else:
                    self.publish_cmd_vel()
                self.img_flag = False
            self.rate.sleep()


if __name__ == "__main__":
  #desired_angle = int(raw_input("What is your desired angle for kick?"))
  #rs = RobotSoccer(desired_angle)
  rs = RobotSoccer(0.3)
  rs.run()
