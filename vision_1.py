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
        self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.joint2_pos = rospy.Publisher("/joint2_vision1_pos", Float64, queue_size=10)
        self.joint3_pos = rospy.Publisher("/joint3_vision1_pos", Float64, queue_size=10)
        self.joint4_pos = rospy.Publisher("/joint4_vision1_pos", Float64, queue_size=10)
        self.joints_pos = rospy.Publisher("/joints_vision1_pos", Float64MultiArray, queue_size=10)

        self.green = rospy.Publisher("/green", Float64MultiArray, queue_size=10)
        self.yellow = rospy.Publisher("/yellow", Float64MultiArray, queue_size=10)

        self.vector_z_pos = rospy.Publisher("/detect_z_pos",Float64MultiArray,queue_size=10)

        self.time_initial = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')

        self.green1 = np.array([402,545])
        self.green2 = np.array([402,545])
        self.yellow1 = np.array([399,430])
        self.yellow2 = np.array([399, 430])


        self.last_red_image1 = [0, 0]
        self.last_red_image2 = [0, 0]
        self.last_blue_image1 = [0, 0]
        self.last_blue_image2 = [0, 0]

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
    # def detect_red(self, image):
    #     # Isolate the blue colour in the image as a binary image
    #     mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    #     # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    #     kernel = np.ones((5, 5), np.uint8)
    #     mask = cv2.dilate(mask, kernel, iterations=3)
    #     # Obtain the moments of the binary image
    #     M = cv2.moments(mask)
    #     # Calculate pixel coordinates for the centre of the blob
    #     if M['m00'] != 0:
    #         cx = int(M['m10'] / M['m00'])
    #         cy = int(M['m01'] / M['m00'])
    #         if image is self.cv_image1:
    #             self.last_red_image1[0] = cx
    #             self.last_red_image1[1] = cy
    #             return np.array([cx, cy])
    #         else:
    #             self.last_red_image2[0] = cx
    #             self.last_red_image2[1] = cy
    #             return np.array([cx, cy])
    #     # this is in case red is blocked by green
    #     else:
    #         if image is self.cv_image1:
    #             return np.array(self.last_red_image1)
    #         else:
    #             return np.array(self.last_red_image2)


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
    # def detect_blue(self, image):
    #     mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    #     kernel = np.ones((5, 5), np.uint8)
    #     mask = cv2.dilate(mask, kernel, iterations=3)
    #     M = cv2.moments(mask)
    #     # Calculate pixel coordinates for the centre of the blob
    #     if M['m00'] != 0:
    #         cx = int(M['m10'] / M['m00'])
    #         cy = int(M['m01'] / M['m00'])
    #         if self.cv_image1 is image:
    #             self.last_blue_image1 = np.array([cx, cy])
    #             return np.array([cx, cy])
    #         else:
    #             self.last_blue_image2 = np.array([cx, cy])
    #             return np.array([cx, cy])
    #     # this is in case red is blocked by green
    #     else:
    #         if self.cv_image1 is image:
    #             return self.last_blue_image1
    #         else:
    #             return self.last_blue_image2

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

    # Calculate the conversion from pixel to meter
    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_green(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 4 / np.sqrt(dist)

    def detect_joint2_angle(self):
        a = self.pixel2meter(self.cv_image1)
        b = self.pixel2meter(self.cv_image2)
        yellow1 = self.yellow1
        yellow2 = self.yellow2
        blue1 = self.detect_blue(self.cv_image1)
        blue2 = self.detect_blue(self.cv_image2)

        vector_z = np.array([0, 0, 3.2])
        vector_x = np.array([3.2, 0, 0])
        vector_y = np.array([0, 3.2, 0])
        vector_z1 = np.array([blue2[0] - yellow2[0], blue1[0]-yellow1[0], yellow1[1] - blue1[1]])
        vector_x1 = np.cross(vector_y, vector_z1)
        ja2 = np.arccos(
            np.dot(vector_x, vector_x1)
            /
            (
                    np.sqrt(vector_x[0] ** 2 + vector_x[1] ** 2 + vector_x[2] ** 2) * np.sqrt(
                vector_x1[0] ** 2 + vector_x1[1] ** 2 + vector_x1[2] ** 2)
            )
        )

        if (blue2[0] >= yellow2[0]):
            return ja2
        else:
            return -ja2

    def detect_joint3_angle(self):
        a = self.pixel2meter(self.cv_image1)
        b = self.pixel2meter(self.cv_image2)
        yellow1 = self.yellow1
        yellow2 = self.yellow2
        blue1 = self.detect_blue(self.cv_image1)
        blue2 = self.detect_blue(self.cv_image2)

        vector_z = np.array([0, 0, 3.2])
        vector_x = np.array([3.2, 0, 0])
        vector_y = np.array([0, -3.2, 0])
        vector_z1 = np.array([blue2[0] - yellow2[0], yellow1[0]-blue1[0], yellow1[1] - blue1[1]])
        vector_x1 = np.cross(vector_y, vector_z1)
        vector_y1 = np.cross(vector_x1, vector_z1)
        ja3 = np.arccos(
            np.dot(vector_y, vector_y1)
            /
            (
                    np.sqrt(vector_y[0] ** 2 + vector_y[1] ** 2 + vector_y[2] ** 2) * np.sqrt(
                vector_y1[0] ** 2 + vector_y1[1] ** 2 + vector_y1[2] ** 2)
            )
        )
        if (blue1[0]<=yellow1[0]):
            return -ja3+np.pi
        else:
            return ja3-np.pi


    def detect_joint4_angle(self):
        a = self.pixel2meter(self.cv_image1)
        b = self.pixel2meter(self.cv_image2)
        red1 = self.detect_red(self.cv_image1)
        red2 = self.detect_red(self.cv_image2)
        yellow1 = self.yellow1
        yellow2 = self.yellow2
        blue1 = self.detect_blue(self.cv_image1)
        blue2 = self.detect_blue(self.cv_image2)
        link2 = np.array([blue2[0] - yellow2[0], yellow1[0] - blue1[0], yellow1[1] - blue1[1]])
        link3 = np.array([red2[0]-blue2[0],blue1[0]-red1[0],blue1[1]-red1[1]])

        vector_y = np.array([0, 3.2, 0])
        vector_x1 = np.cross(vector_y, link2)

        ja4 = np.arccos(
            np.dot(link2, link3)
            /
            (
                    np.sqrt(link2[0] ** 2 + link2[1] ** 2 + link2[2] ** 2) * np.sqrt(
                link3[0] ** 2 + link3[1] ** 2 + link3[2] ** 2)
            )
        )

        if(np.dot(link3,vector_x1)>0):
            return ja4
        else:
            return -ja4


    def detect_z2(self):
        a = self.pixel2meter(self.cv_image2)
        b = self.pixel2meter(self.cv_image1)
        yellow1 = a * self.detect_yellow(self.cv_image1)
        yellow2 = b * self.detect_yellow(self.cv_image2)
        blue1 = a * self.detect_blue(self.cv_image1)
        blue2 = b * self.detect_blue(self.cv_image2)

        vector_z = np.array([0, 0, 3.2])
        vector_x = np.array([3.2, 0, 0])
        vector_y = np.array([0, 3.2, 0])
        vector_z2 = np.array([blue2[0] - yellow2[0], blue1[0] - yellow1[0], yellow1[1] - blue1[1]])
        vector_x1 = np.cross(vector_y, vector_z2)
        ja2 = np.arccos(
            np.dot(vector_x, vector_x1)
            /
            (
                    np.sqrt(vector_x[0] ** 2 + vector_x[1] ** 2 + vector_x[2] ** 2) * np.sqrt(
                vector_x1[0] ** 2 + vector_x1[1] ** 2 + vector_x1[2] ** 2)
            )
        )

        return vector_z2



    # Recieve data from camera 1, process it, and publish
    def callback1(self, data1, data2):
        joint2_val = Float64()
        joint2_val.data = (np.pi / 2) * np.sin((np.pi / 15) * (rospy.get_time() - self.time_initial))
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

        self.joint2 = Float64()
        self.joint2.data = self.detect_joint2_angle()

        self.vector_z = Float64MultiArray()
        self.vector_z.data = self.detect_z2()

        self.joint3 = Float64()
        self.joint3.data = self.detect_joint3_angle()

        self.joint4 = Float64()
        self.joint4.data = self.detect_joint4_angle()

        self.green_pos = Float64MultiArray()
        self.green_pos.data = self.detect_green(self.cv_image1)

        self.yellow_pos = Float64MultiArray()
        self.yellow_pos.data = self.detect_yellow(self.cv_image1)
        # self.joints = Float64MultiArray()
        # self.joints.data = self.detect_joints_angle()
        # Recieve the image


        # Publish the results
        try:
            self.joint2_pub.publish(joint2_val)
            self.joint3_pub.publish(joint3_val)
            self.joint4_pub.publish(joint4_val)
            self.joint2_pos.publish(self.joint2)
            self.vector_z_pos.publish(self.vector_z)
            self.joint3_pos.publish(self.joint3)
            self.joint4_pos.publish(self.joint4)

            self.yellow.publish(self.yellow_pos)
            self.green.publish(self.green_pos)
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
