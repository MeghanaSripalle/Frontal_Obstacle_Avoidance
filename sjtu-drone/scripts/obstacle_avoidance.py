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

    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]


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


# p0 = Point(0, 0)


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
    elif right_edge and left_edge:
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


def detectCorners(gray_img):
    # using good Features to Track for corner detection with shi tomasi method

    feature_params = dict(maxCorners=100, qualityLevel=0.3,
                          minDistance=7, blockSize=7)

    corners = cv2.goodFeaturesToTrack(gray_img, mask=None, **feature_params)
    return corners


def lucasKanadeOpticalFlow(prev_frame, cur_frame, prev_gray, cur_gray, corners):

    frame = cur_frame.copy()
    mask = np.zeros_like(prev_frame)
    color = np.random.randint(0, 255, (100, 3))

    opticalflow_params = dict(winSize=(15, 15), maxLevel=2, criteria=(
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    detected_points, status, errors = cv2.calcOpticalFlowPyrLK(
        prev_gray, cur_gray, corners, None, **opticalflow_params)

    # points/features for which flow has been found
    cur_good = detected_points[status == 1]
    prev_good = corners[status == 1]

    for i, (new, old) in enumerate(zip(cur_good, prev_good)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (a, b), (c, d), color[i].tolist(), 2)

        frame = cv2.circle(frame, (a, b), 5,
                           color[i].tolist(), -1)

    img = cv2.add(frame, mask)


def detectKeypoints(frame):
    # SIFT detector

    sift_detector = cv2.xfeatures2d.SIFT_create()
    keypoints, descriptors = sift_detector.detectAndCompute(frame, None)

    return keypoints, descriptors


def bruteForceMatching(descriptors_m, descriptors_n, curFrame, prevFrame, keypoints_m, keypoints_n):

    bf = cv2.BFMatcher()
    matchings = bf.knnMatch(descriptors_n, descriptors_m, k=2)

    # With the ratio test
    matched = []
    matched_show = []

    for i in range(len(matchings)):
        if len(matchings[i]) == 1:
            matched_show.append([matchings[i][0]])
            matched.append(matchings[i][0])
            continue
        if matchings[i][0].distance < 0.75*matchings[i][1].distance:
            matched_show.append([matchings[i][0]])
            matched.append(matchings[i][0])

    return matched


def sizeFiltering(curFrame, prevFrame, keypoints_m, keypoints_n, matched_keypoints):
    updated_matched_keypoints = []
    for match in matched_keypoints:
        if keypoints_n[match.queryIdx].size < keypoints_m[match.trainIdx].size:
            updated_matched_keypoints.append(match)

    return updated_matched_keypoints


def orientation(p, q, r):
    val = ((q.y - p.y) * (r.x - q.x) -
           (q.x - p.x) * (r.y - q.y))
    if val == 0:
        return 0  # collinear
    elif val > 0:
        return 1  # clock wise
    else:
        return 2  # counterclock wise


def Left_index(points):

    mini = 0
    for i in range(1, len(points)):
        if points[i].x < points[mini].x:
            mini = i
        elif points[i].x == points[mini].x:
            if points[i].y > points[mini].y:
                mini = i
    return mini

# To check if point q lies on the line segment 'pr'


def onSegment(p, q, r):

    return (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y))

# The Jarvis Wrapping Algorithm


def convexHull(points, n):

    if n < 3:
        return []
    
    print("Convex Hull Building.")
    l = Left_index(points)

    hull = []

    p = l
    q = 0
    while (True):

        # Add current point to result
        hull.append(p)

        q = (p + 1) % n

        for i in range(n):

            # If i is more counterclockwise
            # than current q, then update q
            if (orientation(points[p], points[i], points[q]) == 2):
                q = i

            if (p != i and orientation(points[p], points[i], points[q]) == 0 and onSegment(points[p], points[q], points[i])):
                q = i

        p = q

        # While we don't come to first point
        if (p == l):
            print("Loop done")
            break

    convex_hull_points = []

    for each in hull:
        convex_hull_points.append((int(points[each].x), int(points[each].y)))
    
    print(convex_hull_points)
    return convex_hull_points


# utility function to see the convex hull of the obstacl formed
def showConvexHull(convex_hull_points, frame, frame_name):

    for i in range(len(convex_hull_points)-1):

        frame = cv2.line(
            frame, convex_hull_points[i], convex_hull_points[i+1], (255, 0, 0), 2)

    frame = cv2.line(
        frame, convex_hull_points[-1], convex_hull_points[0], (255, 0, 0), 2)

 # calculating the area of the convex hull for ratio check


def calculateArea(convex_hull_points):
    area = 0
    points = convex_hull_points
    # points.append(points[0])

    for j in range(len(points)-1):
        row = []
        row.append(points[j])
        row.append(points[j+1])
        det_set = np.array(row)
        area += np.linalg.det(det_set)

    row = []
    row.append(points[-1])
    row.append(points[0])
    det_set = np.array(row)
    area += np.linalg.det(det_set)

    return (1/2)*area

# finding the minimum x and y points along with the maximum x and y points to form a bounding box


def findCoordinates(points, n):
    min_x = sys.maxsize
    max_x = 0
    min_y = sys.maxsize
    max_y = 0

    for i in range(n):
        if points[i].x < min_x:
            min_x = points[i].x
        if points[i].y < min_y:
            min_y = points[i].y
        if points[i].x > max_x:
            max_x = points[i].x
        if points[i].y > max_y:
            max_y = points[i].y

    coordinates = [int(min_x), int(min_y), int(max_x), int(max_y)]
    return coordinates

# constructing the bounding box


def constructBoundary(points, n):
    coordinates = findCoordinates(points, n)
    boundary = [(coordinates[0], coordinates[1]), (coordinates[2], coordinates[1]),
                (coordinates[2], coordinates[3]), (coordinates[0], coordinates[3])]

    return boundary

# displaying the bounding box


def showBoundary(boundary, frame, frame_name):

    for i in range(len(boundary)-1):

        frame = cv2.line(frame, boundary[i], boundary[i+1], (255, 0, 0), 2)

    frame = cv2.line(frame, boundary[-1], boundary[0], (255, 0, 0), 2)

# calculating the area of the bounding box


def calculateBoundaryArea(boundary):
    width = boundary[1][0] - boundary[0][0]
    length = boundary[2][1] - boundary[1][1]
    area = width*length
    return area

# checking the ratios of the areas of the obstacles in the previous frame and current frame


def calcAreaRatio(prev_area, cur_area):
    if prev_area == 0:
        return 0

    return float(cur_area)/float(prev_area)

# calculating the summation of the ratio of the sizes of the keypoints in the previous frame and current frame


def calcSizeRatio(keypoints, keypoints_n, keypoints_m):
    ratio = 0
    if len(keypoints) == 0:
        return ratio
    for keypoint in keypoints:
        ratio += float((keypoints_m[keypoint.trainIdx].size) /
                       (keypoints_n[keypoint.queryIdx].size))

    rat = ratio/len(keypoints)
    return rat

# a utility function for processing the imaging using methods like image thresholding,morphological gradient


def imagePreprocessing(frame):

    # Using adaptive thresholding followed by contour detection

    directory = r'/home/meghana/Desktop'
    os.chdir(directory)

    newFrame = frame.copy()
    cur = cv2.adaptiveThreshold(
        newFrame, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    # ret,cur = cv2.threshold(newFrame, 130, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # kernelSizes = [(3,3)]

    # for kernelSize in kernelSizes:
    #    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernelSize)
    #    cur = cv2.morphologyEx(cur, cv2.MORPH_GRADIENT, kernel)

    _, contours, hierarchy = cv2.findContours(
        cur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # detecting max area contour
    max_area_contour = max(contours, key=cv2.contourArea)
    convex_hull = cv2.convexHull(max_area_contour)

    n = convex_hull.ravel()
    coordinates = []
    i = 0
    for j in n:
        if (i % 2 == 0):
            coordinates.append(Point(n[i], n[i + 1]))
        i = i + 1

    return coordinates

# calculating the safety boundary zones around the obstacle


def calculateZones(points, n, frame):

    coordinates = findCoordinates(points, n)
    # coordinates = [int(min_x),int(min_y),int(max_x),int(max_y)]
    pu = coordinates[3]
    pr = coordinates[2]
    pd = coordinates[1]
    pl = coordinates[0]
    h, w = frame.shape[:2]  # height and width of frame

    left_zone = [[0, pu], [pl, pu], [0, pd], [pl, pd]]
    right_zone = [[pr, pu], [w, pu], [pr, pd], [w, pd]]
    up_zone = [[pl, 0], [pr, 0], [pl, pd], [pr, pd]]
    down_zone = [[pl, pu], [pr, pu], [pl, h], [pr, h]]

    l_width = pl
    r_width = w - pr
    u_height = pd
    d_height = h - pu

    tau = [l_width, r_width, u_height, d_height]
    return tau


def obstacleAvoidance(freeZone, sizeRatio, areaRatio):
    left_zone = freeZone[0]
    right_zone = freeZone[1]
    top_zone = freeZone[2]
    bottom_zone = freeZone[3]

    print('Avoid the Obstacle')

    # Maneuver Behaviour = (Mrl,Mud)
    maneuverBehaviour = [0, 0]
    if areaRatio <= 2 and sizeRatio <= 1.5:

        if left_zone == 0 and right_zone == 0:
            maneuverBehaviour[0] = 0
        else:
            maneuverBehaviour[0] = max(left_zone, right_zone)

        # if top_zone == 0 and bottom_zone == 0:
        #     maneuverBehaviour[1] = 0
        # else:
        #     maneuverBehaviour[1] = max(top_zone,bottom_zone)

        if maneuverBehaviour[0] == 0:
            # if maneuverBehaviour[1] == top_zone:
            #     print('I')
            # elif maneuverBehaviour[1] == bottom_zone:
            #     print('K')
            print('H')
        else:
            if maneuverBehaviour[0] == left_zone:
                print('A')
            elif maneuverBehaviour[0] == right_zone:
                print('D')

        # Checking avaliable top and bottom zones
        # if maneuverBehaviour[1] == 0:
        #     if maneuverBehaviour[0] == right_zone:
        #         print('D')
        #     elif maneuverBehaviour[0] == left_zone:
        #         print('A')
        # else:
        #     if maneuverBehaviour[1] == top_zone:
        #         print('I')
        #     elif maneuverBehaviour[1] == bottom_zone:
        #         print('K')

        # if maneuverBehaviour == [0,0]:
        #     print('H')

    else:
        print('H')
    print(maneuverBehaviour)


def main():
    # takeoff()  # Takeoff the drone
    print('Z for takeoff')
    # start_vel_ctrl()  # Start Velocity control mode
    # rospy.sleep(2)  # Delay so that drone can get height
    prev_image = []
    goal_coordinates = np.array([14.285401, -0.058154, 0.718870])
    # pub_vel(0.25,0,0,0,0,0)

    while not rospy.is_shutdown():
        # pub_vel(0.25,0,0,0,0,0)
        if flag:
            # if goal is reached
            if (pos == goal_coordinates).all():
                print("The drone has reached the goal!")
                break

            # To convert from ros image to opencv image
            cv_image = bridge.imgmsg_to_cv2(
                image, desired_encoding='passthrough')
            curFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # train

            if len(prev_image) == 0:
                prev_image = cv_image
                continue

            prevFrame = cv2.cvtColor(prev_image, cv2.COLOR_BGR2GRAY)  # query
            prev_image = cv_image

            quaternion = (
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w)

            # # convert quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            # Main Algorithm

            # move forward till the obstacle distance is below 1 meter
            # moveForward(0.5)

            # Finding the keypoints in both the frames
            keypoints_n, descriptors_n = detectKeypoints(prevFrame)
            keypoints_m, descriptors_m = detectKeypoints(curFrame)
            # Conditions for when we have no detected keypoints
            if descriptors_n is None or descriptors_m is None:
                continue

            # frame = curFrame.copy()
            # frame = cv2.drawKeypoints(frame, keypoints_n, 0, (0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            # frame = cv2.drawKeypoints(frame, keypoints_m, 0, ( 255,0, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # directory = r'/home/meghana/Desktop/Results'
            # os.chdir(directory)
            # cv2.imwrite('Keypoints.jpg',frame)
            # continue

            # Matching of the keypoints
            matched_keypoints = bruteForceMatching(descriptors_m, descriptors_n, curFrame, prevFrame, keypoints_m,keypoints_n)
            # Condition for when there are no matched keypoints
            if len(matched_keypoints) == 0:
                continue
            
            # frame = curFrame.copy()
            # frame = cv2.drawMatches(prevFrame, keypoints_n, curFrame, keypoints_m, matched_keypoints, None, flags=2)
            # directory = r'/home/meghana/Desktop/Results'
            # os.chdir(directory)
            # cv2.imwrite('Matches.jpg',frame)
            # continue

            # #Size Filtering
            updated_matched_keypoints = sizeFiltering(curFrame,prevFrame,keypoints_m,keypoints_n,matched_keypoints)
            # Condition for when there are no matched keypoints after size filtering
            if len(updated_matched_keypoints) == 0:
                continue

            # points_n consists of the matched keypoints in the previous frame
            # points_m consists of the matched keypoints in the current frame
            points_n = []
            points_m = []

            for keypoint in updated_matched_keypoints:

                points_n.append(Point(keypoints_n[keypoint.queryIdx].pt[0],keypoints_n[keypoint.queryIdx].pt[1]))
                points_m.append(Point(keypoints_m[keypoint.trainIdx].pt[0],keypoints_m[keypoint.trainIdx].pt[1]))

    
              # Finding the convex hull/ object of interest
            # convex_hull_points_n = convexHull(points_n,len(points_n))
            # convex_hull_points_m = convexHull(points_m,len(points_m))
            # if len(convex_hull_points_n) == 0 or len(convex_hull_points_m) == 0:
            #     continue

            # Finding the  object of interest
            convex_hull_points_m = constructBoundary(points_m,len(points_m))
            convex_hull_points_n = constructBoundary(points_n,len(points_n))
            if len(convex_hull_points_n) == 0 or len(convex_hull_points_m) == 0:
                continue

            # Calculating the area of the convex hulls in the 2 different frames
            area_prev_frame = calculateBoundaryArea(convex_hull_points_n)
            area_cur_frame = calculateBoundaryArea(convex_hull_points_m)

            state = False
            sizeRatio = calcSizeRatio(updated_matched_keypoints,keypoints_n,keypoints_m)
            areaRatio = calcAreaRatio(area_prev_frame,area_cur_frame)
            boundary = convex_hull_points_m

            # directory = r'/home/meghana/Desktop/Results'
            # os.chdir(directory)
            # for i in range(len(boundary)-1):
            #         curFrame = cv2.line(curFrame,boundary[i],boundary[i+1],(255,0,0),2)

            # curFrame = cv2.line(curFrame,boundary[-1],boundary[0],(255,0,0),2)
            # cv2.imwrite('Result.jpg',curFrame)

            if(sizeRatio >=1.2 and areaRatio >=1.35):
                state = True

                # coordinates = imagePreprocessing(curFrame)
                # bounding_box = constructBoundary(coordinates,len(coordinates))

                # #Describing the different boundary zones
                # boundary_zones_processing = calculateZones(coordinates,len(coordinates),curFrame)
                #boundary_zones_detection = calculateZones(points_m,len(points_m),curFrame)

                #obstacleAvoidance(boundary_zones_detection,sizeRatio,areaRatio)

                print("Obstacle has to be avoided!")
                for i in range(len(boundary)-1):
                    curFrame = cv2.line(curFrame,boundary[i],boundary[i+1],(255,0,0),2)

                curFrame = cv2.line(curFrame,boundary[-1],boundary[0],(255,0,0),2)
                # cv2.imwrite('Obstacle.jpg',curFrame)
                # stopDrone()

    
            # pub_vel(lin_x,lin_y,lin_z,ang_x,ang_y,ang_z)
            cv2.namedWindow('Current frame')
            cv2.imshow('Current frame', curFrame)
            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()
    # stopDrone()
    # land()  # Landing


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
