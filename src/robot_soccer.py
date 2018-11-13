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

resize = (160, 120)
boxlist = []

class RobotSoccer():

    def __init__(self):

        rospy.init_node('robot_soccer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
       
        self.rate = rospy.Rate(2)

        rospy.Subscriber("/camera/image_raw", Image, self.camera_cb)
        rospy.Subscriber("/odom", Odometry, self.set_location)

        self.img_flag = False
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0
        self.theta = 0

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
        img = cv2.resize(img, (160, 120), interpolation=cv2.INTER_AREA)
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

    def getAngleDist(self, x,radius):
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


    def random_walk(self):
        """
        Get random linear and angular speeds and send them to publisher
        """
        x = random.random()*2 - 1
        z = random.random()*2 - 1
        self.publish_cmd_vel(x, z)

    def turn_and_forward(self, angle, forward):
        """
        Turn and move forward a given amount
        """
        angular_speed = 0.2
        linear_speed = 0.2
        angle_tolerance = 0.05
        linear_tolerance = 0.1
        start_x = self.x
        start_y = self.y
        start_theta = self.theta
        end_theta = start_theta + angle
        if (0 <= angle <= 180):
            turn_direction = 1
        else:
            turn_direction = -1
        end_x = start_x + (math.cos(abs(angle))*turn_direction)
        end_y = start_y + math.sin(abs(angle))
        while abs(self.theta - end_theta) > angle_tolerance:
            self.publish_cmd_vel(0, angular_speed*turn_direction)
            print("current theta: %f , desired theta: %f" % (self.theta, end_theta))
        while (abs(self.x - end_x) > linear_tolerance) or (abs(self.y - end_y) > linear_tolerance):
            self.publish_cmd_vel(linear_speed, 0)
            print("current x: %f , current y: %f , desired x: %f , desired y: %f" % (self.x, self.y, end_x, end_y))
        self.publish_cmd_vel()


    def find_ball(self, base):
        """
        Returns flag for whether ball was successfully found, and then the angle and distance if it was.
        """
        try:
            img = cv2.medianBlur(base.copy(),5)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            lower = np.array([40, 10, 10])
            upper = np.array([110, 255, 255])

            img = cv2.inRange(img, lower, upper)
            img = cv2.erode(img, None, iterations=3)
            img = cv2.dilate(img, None, iterations=2)

            contours = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            # Draw bounding circle around largest contour
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

                # Draw circles on image to represent the ball
                if radius > 10:
                    print "coord:" + str(x) + "," + str(y) + " radius:" + str(radius)
                    angle, dist = self.getAngleDist(float(x), float(radius))
                    print "angle:" + str(angle) + " distance:" + str(dist)

                    box = self.rad2box(float(x), float(y), float(radius))
                    box.append(radius)
                    box.append(float(angle))
                    box.append(dist)
                    cv2.rectangle(base, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255,0,0), 2)
                    boxlist.append(box)

                    cv2.line(base, (int(x), int(y)), (160, 240), (255, 0, 0), 1, 8, 0)
                    cv2.imshow('detected circles', base)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    return True, angle, dist
            return False, 0, 0
        except AttributeError:
            return False, 0, 0



    def run(self):
        #base = cv2.imread("../data/testballcenter.jpg", 1)
        #found, angle, dist = self.find_ball(base)
        while not rospy.is_shutdown():
            # if self.img_flag:
            #     found, angle, dist = self.find_ball(self.img)
            #     self.img_flag = False
            self.turn_and_forward(0.1, 1)
            self.rate.sleep()


if __name__ == "__main__":
  rs = RobotSoccer()
  rs.run()