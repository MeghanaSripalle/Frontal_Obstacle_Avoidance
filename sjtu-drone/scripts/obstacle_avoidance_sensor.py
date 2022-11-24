#!/usr/bin/env python
# importing the necessary libraries

import os
import sys
import cv2
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import copy
from functools import cmp_to_key
import rospy
import tf
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, Empty
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range

bridge = CvBridge()
rospy.init_node("Obstacle_Avoidance", anonymous=True)  # initialize ros node


frontal_distance = 0


def cb_distance(data):
    global frontal_distance, fov
    frontal_distance = data.range


rospy.Subscriber("/drone/sonar", Range, cb_distance)

image = Image()
imu = Imu()
vel = Twist()
position = Pose()
flag = False
pos = np.zeros(3)
orient = np.zeros(3)
roll = pitch = yaw = 0.0
kp = 0.5
width_drone = 0.517  # width of the drone in meters
# minimum safe distance to the side so that the drone can reroute in case of a V shaped obstacle
minimum_safe_distance = (width_drone/2) + 0.25


def cb_img(data):
    global image, flag
    image = data
    flag = True


def cb_imu(data):
    global imu
    imu = data


def cb_pose(data):
    global position, pos, orient
    position = data
    pos[0] = position.position.x
    pos[1] = position.position.y
    pos[2] = position.position.z

    quaternion = (
        position.orientation.x,
        position.orientation.y,
        position.orientation.z,
        position.orientation.w
    )


def get_rotation(data):
    global roll, pitch, yaw
    orientation_q = data.pose.pose.orientation
    orientation = [orientation_q.x, orientation_q.y,
                   orientation_q.z, orientation_q.w]
    euler = tf.transformations.euler_from_quaternion(orientation)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]


rospy.Subscriber("/drone/front_camera/image_raw", Image, cb_img)
rospy.Subscriber("/drone/imu", Imu, cb_imu)
rospy.Subscriber("/drone/gt_pose", Pose, cb_pose)
rospy.Subscriber('/odom', Odometry, get_rotation)
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
vel_ctrl_pub = rospy.Publisher("/drone/vel_mode", Bool, queue_size=10)
takeoff_pub = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
land_pub = rospy.Publisher('drone/land', Empty, queue_size=1)

rospy.sleep(0.5)


class Point:
    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y



def pub_vel(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    vel.linear.x = linear_x
    vel.linear.y = linear_z
    vel.linear.z = linear_z
    vel.angular.x = angular_x
    vel.angular.y = angular_y
    vel.angular.z = angular_z
    vel_pub.publish(vel)


def start_vel_ctrl():
    ctrl = Bool(True)
    vel_ctrl_pub.publish(ctrl)
    print("Started Velocity Control")


def takeoff():
    takeoff_pub.publish(Empty())
    print("Take off")


def land():
    land_pub.publish(Empty())
    print("Landing!")


def rotate(angle, speed, direction):

    lin_x = 0.0
    lin_y = 0.0
    lin_z = 0.0

    ang_x = 0.0
    ang_y = 0.0
    ang_z = 0.0

    relative_angle_degree = angle

    angle = math.radians(angle)
    angular_speed = math.radians(abs(speed))
    # angular_speed = 0.1 #1 radian per second

    if direction == 'left':
        ang_z = abs(angular_speed)
    elif direction == 'right':
        ang_z = -abs(angular_speed)

    cur_angle = 0
    r = 10
    rate = rospy.Rate(r)

    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Drone rotates")
        # Publishing the velocity to drone
        pub_vel(lin_x, lin_y, lin_z, ang_x, ang_y, ang_z)

        t1 = rospy.Time.now().to_sec()
        cur_angle = (t1-t0)*speed
        rate.sleep()

        if (cur_angle >= relative_angle_degree):
            rospy.loginfo("Rotated By Angle")
            break

    print("Current angle: ", cur_angle)
    # stop rotation about the z axis
    ang_z = 0.0
    pub_vel(lin_x, lin_y, lin_z, ang_x, ang_y, ang_z)


def Rotate(angle):

    target_rad = angle*math.pi/180
    rate = rospy.Rate(10)
    while True:
        ang_z = kp * (target_rad-yaw)
        pub_vel(0, 0, 0, 0, 0, ang_z)
        print("target={} current:{}", angle, yaw)
        rate.sleep()


def droneRotation():

    Rotate(45)


def objectDetection():
    dist = frontal_distance

    if dist < 1:
        return True

    return False


def stopDrone():
    pub_vel(0, 0, 0, 0, 0, 0)


def slowDown():
    return 0.25


def steerLeft(left_distance, left_angle):

    temp = ((width_drone/2)/left_distance)
    theta_radians = math.asin(temp)
    theta = theta_radians*(180.0/math.pi)
    angle_to_rotate = theta + left_angle
    return angle_to_rotate


def steerRight(right_distance, right_angle):

    temp = ((width_drone/2)/right_distance)
    theta_radians = math.asin(temp)
    theta = theta_radians*(180.0/math.pi)  # theta is degrees
    angle_to_rotate = theta + right_angle
    return angle_to_rotate

# function to determine if the side distances are safe or not


def criticalSafeSideCondition(left_side_distance, right_side_distance):
    if left_side_distance <= minimum_safe_distance or right_side_distance <= minimum_safe_distance:
        return False
    return True


def moveForward(speed):

    pub_vel(speed, 0, 0, 0, 0, 0)
    rate = rospy.Rate(10)

    dist_flag = False
    first_distance = 0
    distance = 0

    while True:
        rospy.loginfo("Drone moves forwards.")
        pub_vel(speed, 0, 0, 0, 0, 0)

        rate.sleep()

        distance = frontal_distance

        if objectDetection():
            rospy.loginfo("Obstacle detected.Moving towards obstacle.")
            if not dist_flag:
                first_distance = distance
                dist_flag = True

            if distance >= 0.3 and distance < 1:
                speed = slowDown()
                continue
            else:
                break

    # finally, stop  the drone when the obstacle is detected and near enough for edge detection
    stopDrone()
    locateEdges(first_distance, distance)


def locateEdges(first_distance, distance):

    threshold = 1  # 0.3/(math.cos(math.radians(45)))  # trial threshold
    left_edge = False  # left edge flag
    right_edge = False  # right edge flag
    left_angle = 0
    right_angle = 0
    prev_left_angle = 0
    prev_right_angle = 0
    left_dist = 0
    prev_left_dist = 0
    right_dist = 0
    prev_right_dist = 0
    print("Distance: ", distance)

    # rotating the drone to the left degree by degree
    while left_angle <= 45:
        print('Rotating left')
        rotate(1, 10, 'left')
        prev_left_dist = left_dist
        prev_left_angle = left_angle
        left_angle += 1
        left_dist = frontal_distance
        # print("Left Distance:", left_dist)
        if left_dist > distance + threshold:
            print("Left Distance After Edge Detection:", prev_left_dist)
            print("Left Edge is Found.")
            left_edge = True  # left edge is found
            break

    rotate(left_angle, 10, 'right')  # rotate to the initial position

    # rotating the drone to the right degree by degree
    while right_angle <= 45:
        print('Rotating right')
        rotate(1, 10, 'right')
        prev_right_dist = right_dist
        prev_right_angle = right_angle
        right_angle += 1
        right_dist = frontal_distance
        # print('Right distance:', right_dist)
        # print('Right Angle: ',right_angle)
        if right_dist > distance + threshold:
            print("Right Distance After Edge Detection:", prev_right_dist)
            print("Right Edge is Found.")
            right_edge = True  # right edge is found
            break

    # rotate to the initial position after complete edge detection
    rotate(right_angle, 10, 'left')

    if left_edge:
        print("The left edge is at degrees: ", prev_left_angle)

    if right_edge:
        print("The right edge is at degrees: ", prev_right_angle)

    # calculation of the distance of the object on the left and right side of the UAV
    triangulation(prev_left_angle, prev_right_angle,
                  prev_left_dist, prev_right_dist, left_edge, right_edge)


def faultTolerance():
    return


def triangulation(left_angle, right_angle, left_distance, right_distance, left_edge, right_edge):

    # h_1 = left_distance, h_2 = right_distance,left_length = distance of the object on the left, right_length = distance of the object on the right

    # test case 1: visible edge on the left
    if left_edge and not right_edge:
        phi_1 = 90 - left_angle
        left_length = left_distance*(math.cos(abs(phi_1)))
        left_frontal_distance = left_distance*(math.sin(abs(phi_1)))

        # angle to rotate by to the left to take a left turn is calculated here
        angle_to_rotate = steerLeft(left_distance, left_angle)
        if angle_to_rotate < 90:
            rotate(angle_to_rotate, 10, "left")
            moveForward(0.5)
    elif right_edge and not left_edge:  # test case 2: visible edge on the right
        phi_2 = 90 - right_angle
        right_length = right_distance*(math.cos(abs(phi_2)))
        right_frontal_distance = right_distance*(math.sin(abs(phi_2)))

        angle_to_rotate = steerRight(right_distance, right_angle)
        if angle_to_rotate < 90:
            rotate(angle_to_rotate, 10, "right")
            moveForward(0.5)
    elif right_edge and left_edge: #test case 3 : when both the edges are visible
        # finding the minimum out of left edge length and right edge length to choose which edge to steer towards
        phi_1 = 90 - left_angle
        left_length = left_distance*(math.cos(abs(phi_1)))

        phi_2 = 90 - right_angle
        right_length = right_distance*(math.cos(abs(phi_2)))

        if left_length <= right_length:
            print("Turn Left!")
            turn_angle = steerLeft(left_distance, left_angle)
            if turn_angle < 90:
                rotate(turn_angle, 10, "left")
                moveForward(0.5)
        else:
            print("Turn Right!")
            turn_angle = steerRight(right_distance, right_angle)
            if turn_angle < 90:
                rotate(turn_angle, 10, "right")
                moveForward(0.5)


def main():
    takeoff()  # Takeoff the drone
    print('Z for takeoff')
    start_vel_ctrl()  # Start Velocity control mode
    rospy.sleep(2)  # Delay so that drone can get height
    goal_coordinates = np.array([14.285401, -0.058154, 0.718870])
    pub_vel(0.5,0,0,0,0,0)

    while not rospy.is_shutdown():
        pub_vel(0.5,0,0,0,0,0)
        # if goal is reached
        if (pos == goal_coordinates).all():
            print("The drone has reached the goal!")
            break

        # Main Algorithm

        # move forward till the obstacle distance is below 1 meter
        moveForward(0.5)


        # pub_vel(lin_x,lin_y,lin_z,ang_x,ang_y,ang_z)
    stopDrone()
    land()  # Landing


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
