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

resize = (160, 120)
boxlist = []

class RobotSoccer():

    def __init__(self, da):

        rospy.init_node('robot_soccer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        self.rate = rospy.Rate(2)

        rospy.Subscriber("/camera/image_raw", Image, self.camera_cb)

        self.img_flag = False
        self.bridge = CvBridge
        self.desired_angle = da
        self.angle_threshold = 2

    def publish_cmd_vel(x, z):
        """
        Publish the given speed and angular velocity
        """
        outmsg = Twist()
        outmsg.linear.x = x
        outmsg.angular.z = z
        self.pub.publish(outmsg)


    def setLocation(self, odom):
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

    def move_to_ball(self, angle):
        if angle < self.desired_angle: #move left
            self.publish_cmd_vel(.1,.1)
        else: #move right
            self.publish_cmd_vel(.1,-.1)

    def run(self):
        #base = cv2.imread("../data/testballcenter.jpg", 1)
        #found, angle, dist = self.find_ball(base)
        while not rospy.is_shutdown():
            if self.img_flag:
                found, angle, dist = self.find_ball(self.img)
                if found:
                    if angle >= (self.desired_angle - self.angle_threshold) and angle <= (self.desired_angle + self.angle_threshold):
                        #annas code
                    else:
                        self.move_to_ball(angle)
                self.img_flag = False
            self.rate.sleep()


if __name__ == "__main__":
  desired_angle = int(raw_input("What is your desired angle for kick?"))
  rs = RobotSoccer(desired_angle)
  rs.run()
