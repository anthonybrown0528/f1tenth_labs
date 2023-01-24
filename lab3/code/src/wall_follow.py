#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 5
kd = 0
ki = 0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # specify publisher queue size
        queue_size = 1000

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=queue_size)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.

        angle_offset = angle - data.angle_min

        desired = angle_offset / data.angle_increment
        desired = int(desired)

        # return distance at specified angle
        return data.ranges[desired]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        derivative = 0.0
        angle = 0.0

        derivative = error - prev_error
        integral += error

        # Compute PID terms
        p = kp * error
        i = ki * integral
        d = kd * derivative

        prev_error = error

        # Compute control signal
        angle = p + i + d

        if angle >= math.radians(0) and angle <= math.radians(10):
            velocity = 1.5
        elif angle >= math.radians(0) and angle <= math.radians(10):
            velocity = 1.0
        else:
            velocity = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 

        # angle pointing to car's left
        angle_right = -math.pi + math.radians(90)

        # angle sampled from car's right side
        sample_angle = angle_right + math.radians(70)

        a = self.getRange(data, sample_angle)
        b = self.getRange(data, angle_right)

        theta = sample_angle - angle_right

        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))

        # current distance from wall
        distance = b * math.cos(alpha)
        # prediction of next distance
        next_distance = distance + CAR_LENGTH * math.sin(alpha)

        return leftDist - next_distance

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
