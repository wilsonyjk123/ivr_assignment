#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from Cryptodome import Math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import message_filters



class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing1', anonymous=True)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = message_filters.Subscriber("/image_topic1", Image)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera2/image_raw and use callback function to recieve data
        self.image_sub2 = message_filters.Subscriber("/image_topic2", Image)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        timesync = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 15)
        timesync.registerCallback(self.callback1)
        # initialize a publisher to send joints' angular position to the robot
        self.joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.joint1_pos = rospy.Publisher("/joint1_vision2_pos", Float64, queue_size=10)
        self.joint3_pos = rospy.Publisher("/joint3_vision2_pos", Float64, queue_size=10)
        self.joint4_pos = rospy.Publisher("/joint4_vision2_pos", Float64, queue_size=10)
        self.joints_pos = rospy.Publisher("/joints_vision2_pos", Float64MultiArray, queue_size=10)


        self.last_projection = [0,1]
        self.last_ja1 = 0
        self.last_ja3 = 0
        self.clockwise = 1


        self.time_initial = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')

    def detect_red(self, image):
        # Isolate the blue colour in the image as a binary image
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        if (M['m00'] == 0):
            return self.detect_blue(image)
        else:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return np.array([cx, cy])

        # Detecting the centre of the green circle

    # Detecting the centre of the green circle
    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

        # Detecting the centre of the blue circle

    # Detecting the centre of the blue circle
    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if (M['m00'] == 0):
            return self.detect_yellow(image)
        else:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return np.array([cx, cy])

    # Detecting the centre of the yellow circle
    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_green(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 4 / np.sqrt(dist)

    def detect_joint1_angle(self):
        a = self.pixel2meter(self.cv_image1)
        b = self.pixel2meter(self.cv_image2)
        yellow1 = a * self.detect_yellow(self.cv_image1)
        yellow2 = b * self.detect_yellow(self.cv_image2)
        blue1 = a * self.detect_blue(self.cv_image1)
        blue2 = b * self.detect_blue(self.cv_image2)

        projection = np.array([yellow2[0]-blue2[0],yellow1[0]-blue1[0]])
        y_axis = [0,1]

        link2 = np.array([blue2[0] - yellow2[0], yellow1[0] - blue1[0], yellow1[1] - blue1[1]])
        z_axis = [0, 0, 1]

        ja3 = np.arccos(
            np.dot(link2, z_axis)
            /
            (
                    np.sqrt(link2[0] ** 2 + link2[1] ** 2 + link2[2] ** 2) * np.sqrt(
                z_axis[0] ** 2 + z_axis[1] ** 2 + z_axis[2] ** 2)
            )
        )

        ja1 = np.arccos(
            np.dot(projection, y_axis)
            /
            (
                    np.sqrt(projection[0] ** 2 + projection[1] ** 2 ) * np.sqrt(
                y_axis[0] ** 2 + y_axis[1] ** 2 )
            )
        )

        if(blue2[0]>yellow2[0]):
            ja1 = ja1
            return ja1
        else:
            ja1 = -ja1
            return ja1





    def detect_joint3_angle(self):
        a = self.pixel2meter(self.cv_image1)
        b = self.pixel2meter(self.cv_image2)
        yellow1 = a * self.detect_yellow(self.cv_image1)
        yellow2 = b * self.detect_yellow(self.cv_image2)
        blue1 = a * self.detect_blue(self.cv_image1)
        blue2 = b * self.detect_blue(self.cv_image2)

        projection = np.array([yellow2[0]-blue2[0],yellow1[0]-blue1[0]])
        y_axis = [0,1]

        link2 = np.array([blue2[0] - yellow2[0], yellow1[0] - blue1[0], yellow1[1] - blue1[1]])
        z_axis=[0,0,1]

        ja3 = np.arccos(
            np.dot(link2, z_axis)
            /
            (
                    np.sqrt(link2[0] ** 2 + link2[1] ** 2 + link2[2] ** 2) * np.sqrt(
                z_axis[0] ** 2 + z_axis[1] ** 2 + z_axis[2] ** 2)
            )
        )

        ja1 = np.arccos(
            np.dot(projection, y_axis)
            /
            (
                    np.sqrt(projection[0] ** 2 + projection[1] ** 2 ) * np.sqrt(
                y_axis[0] ** 2 + y_axis[1] ** 2 )
            )
        )
        if(ja1<np.pi/2 or ja1 > -np.pi/2):
            if(yellow1[0]>blue1[0]):
                ja3 = ja3
            else:
                ja3 = -ja3
        if(ja1>np.pi/2 or ja1 < -np.pi/2):
            if (yellow1[0] > blue1[0]):
                ja3 = -ja3
            else:
                ja3 = ja3
        return ja3

    def detect_joint4_angle(self):
        a = self.pixel2meter(self.cv_image1)
        b = self.pixel2meter(self.cv_image2)
        yellow1 = a * self.detect_yellow(self.cv_image1)
        yellow2 = b * self.detect_yellow(self.cv_image2)
        red1 = a * self.detect_red(self.cv_image1)
        red2 = b * self.detect_red(self.cv_image2)
        blue1 = a * self.detect_blue(self.cv_image1)
        blue2 = b * self.detect_blue(self.cv_image2)

        projection = np.array([yellow2[0]-blue2[0],yellow1[0]-blue1[0]])
        y_axis = [0,1]

        link2 = np.array([blue2[0] - yellow2[0], yellow1[0] - blue1[0], yellow1[1] - blue1[1]])
        link3 = np.array([red2[0]-blue2[0],blue1[0]-red1[0],blue1[1]-red1[1]])

        ja4 = np.arccos(
            np.dot(link2, link3)
            /
            (
                    np.sqrt(link2[0] ** 2 + link2[1] ** 2 + link2[2] ** 2) * np.sqrt(
                link3[0] ** 2 + link3[1] ** 2 + link3[2] ** 2)
            )
        )

        if(red2[0]-blue2[0]>0 and blue1[0]-yellow2[0]>0):
            return ja4
        elif(red2[0]-blue2[0]>0 and blue1[0]-yellow2[0]<0):
            return -ja4
        elif(red2[0]-blue2[0] <0 and blue1[0] - yellow2[0]>0):
            return -ja4
        elif (red2[0] - blue2[0] < 0 and blue1[0] - yellow2[0] < 0):
            return ja4


    # def detect_joint1_angle(self):
    #     a = self.pixel2meter(self.cv_image1)
    #     b = self.pixel2meter(self.cv_image2)
    #     yellow1 = a * self.detect_yellow(self.cv_image1)
    #     yellow2 = b * self.detect_yellow(self.cv_image2)
    #     blue1 = a * self.detect_blue(self.cv_image1)
    #     blue2 = b * self.detect_blue(self.cv_image2)
    #
    #     projection = np.array([yellow2[0] - blue2[0], yellow1[0] - blue1[0]])
    #     last_projection = self.last_projection
    #
    #     ja = np.arccos(
    #         np.dot(projection, last_projection)
    #         /
    #         (
    #                 np.sqrt(projection[0] ** 2 + projection[1] ** 2) * np.sqrt(
    #             last_projection[0] ** 2 + last_projection[1] ** 2)
    #         )
    #     )
    #
    #     if (self.last_ja1 >= np.pi - 0.15 or self.last_ja1 <= np.pi + 0.15):
    #         ja1 = self.last_ja1 - ja
    #         self.clockwise = 0
    #         self.last_ja1 = ja1
    #         self.last_projection = projection
    #         return ja1
    #     elif (self.last_ja1 >= -np.pi - 0.15 or self.last_ja1 <= -np.pi + 0.15):
    #         ja1 = self.last_ja1 + ja
    #         self.clockwise = 1
    #         self.last_ja1 = ja1
    #         self.last_projection = projection
    #         return ja1
    #
    #     elif (self.clockwise == 1):
    #         ja1 = self.last_ja1 + ja
    #         self.last_ja1 = ja1
    #         self.last_projection = projection
    #         return ja1
    #     elif(self.clockwise == 0):
    #         ja1 = self.last_ja1 - ja
    #         self.last_ja1 = ja1
    #         self.last_projection = projection
    #         return ja1







    def callback1(self, data1, data2):
        joint1_val = Float64()
        joint1_val.data = (np.pi) * np.sin((np.pi / 28) * (rospy.get_time() - self.time_initial))
        joint3_val = Float64()
        joint3_val.data = (np.pi / 2) * np.sin((np.pi / 20) * (rospy.get_time() - self.time_initial))
        joint4_val = Float64()
        joint4_val.data = (np.pi / 2) * np.sin((np.pi / 18) * (rospy.get_time() - self.time_initial))
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
        except CvBridgeError as e:
            print(e)

            # self.joints = Float64MultiArray()
            # self.joints.data = self.detect_joint_angles(self.cv_image1)


            #
            # self.vector_z = Float64MultiArray()
            # self.vector_z.data = self.detect_z2()
            #
            # self.joint3 = Float64()
            # self.joint3.data = self.detect_joint3_angle()
            #
            # self.joint4 = Float64()
            # self.joint4.data = self.detect_joint4_angle()
            #
            # self.joints = Float64MultiArray()
            # self.joints.data = self.detect_joints_angle()
            # Recieve the image
        self.joint1 = Float64()
        self.joint1.data = self.detect_joint1_angle()

        self.joint3 = Float64()
        self.joint3.data = self.detect_joint3_angle()

        self.joint4 = Float64()
        self.joint4.data = self.detect_joint4_angle()
            # Publish the results
        try:
            self.joint1_pub.publish(joint1_val)
            self.joint3_pub.publish(joint3_val)
            self.joint4_pub.publish(joint4_val)

            self.joint1_pos.publish(self.joint1)
            self.joint3_pos.publish(self.joint3)
            self.joint4_pos.publish(self.joint4)
            # self.vector_z_pos.publish(self.vector_z)
            # self.joint3_pos.publish(self.joint3)
            # self.joint4_pos.publish(self.joint4)
            # self.joints_pos.publish(self.joints)
        except CvBridgeError as e:
            print(e)

    # call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
     main(sys.argv)