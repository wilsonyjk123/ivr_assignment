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

class forward_kinematics:
    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('control_processing1', anonymous=True)

        self.joint1_sub = rospy.Subscriber("/joint1_vision2_pos", Float64,self.callback1)
        self.joint3_sub = rospy.Subscriber("/joint3_vision2_pos", Float64,self.callback2)
        self.joint4_sub = rospy.Subscriber("/joint4_vision2_pos", Float64,self.callback3)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # timesync = message_filters.TimeSynchronizer([self.joint1_sub, self.joint3_sub,self.joint4_sub], 10)
        # timesync.registerCallback(self.callback)

        self.coordinates_pub = rospy.Publisher("/coordinates", Float64MultiArray, queue_size=10)

        # hardcode the position of yellow sphere and green sphere
        self.green1 = np.array([402,543])
        self.green2 = np.array([402,543])
        self.yellow1 = np.array([400,440])
        self.yellow2 = np.array([400,440])

    def calculate_final_matrix(self):
        # o0 to o1 frame
        transform_0_1 = np.array([
            [np.cos(self.joint1), -np.sin(self.joint1), 0, 0],
            [np.sin(self.joint1), np.cos(self.joint1), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        # o1 to o2 frame
        transform_1_2 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 4],
            [0, 0, 0, 1]
        ])
        #o2 to o3
        transform_2_3 = np.array([
            [1, 0, 0, 0],
            [0, np.cos(self.joint3), -np.sin(self.joint3), 0],
            [0, np.sin(self.joint3), np.cos(self.joint3), 0],
            [0, 0, 0, 1]
        ])
        #o3 to o4
        transform_3_4 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 3.2],
            [0, 0, 0, 1]
        ])

        transform_4_5 = np.array([
            [np.cos(self.joint4), 0, np.sin(self.joint4), 0],
            [0, 1, 0, 0],
            [-np.sin(self.joint4), 0, np.cos(self.joint4), 0],
            [0, 0, 0, 1]
        ])

        transform_5_6 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 2.8],
            [0, 0, 0, 1]
        ])

        final_transformed = np.dot(
            np.dot(np.dot(np.dot(np.dot(transform_0_1, transform_1_2), transform_2_3), transform_3_4), transform_4_5),
            transform_5_6)

        final_transformed = np.array(final_transformed)
        x = final_transformed[0][3]
        y = final_transformed[1][3]
        z = final_transformed[2][3]

        return np.array([x,y,z])

    def cal_jacobian(self, x, y, z):
        jacobian = np.zeros([3, 3])
        jacobian[0][0] = (14 * np.cos(x + z) - 14 * np.cos(x-z) + 16 * np.sin(x + y) - 16 * np.sin(
            x - y) + 7 * np.sin(x + y + z) - 7 * np.sin(x - y + z) + 7 * np.sin(x + y - z) - 7 * np.sin(x - y - z)) / 10
        jacobian[0][1] = (16 * np.sin(x + y) - 16 * np.sin(x - y) + 7 * np.sin(x + y + z) - 7 * np.sin(
            x - y + z) + 7 * np.sin(x + y - z) - 7 * np.sin(x - y - z)) / 10
        jacobian[0][2] = (14 * np.cos(x + z) - 14 * np.cos(x - z) + 7 * np.sin(x + y + z) - 7 * np.sin(
            x - y + z) + 7 * np.sin(x + y - z) - 7 * np.sin(x - y - z)) / 10

        jacobian[1][0] = (-16 * np.cos(x + y) + 16 * np.cos(x - y) - 7 * np.cos(x + y + z) + 7 * np.cos(
            x - y + z) - 7 * np.cos(x + y - z) + 7 * np.cos(x - y - z) + 14 * np.sin(x + z) - 14 * np.sin(x - z)) / 10
        jacobian[1][1] = (-16 * np.cos(x + y) + 16 * np.cos(x - y) - 7 * np.cos(x + y + z) + 7 * np.cos(
            x - y + z) - 7 * np.cos(x + y - z) + 7 * np.cos(x - y - z)) / 10
        jacobian[1][2] = (-7 * np.cos(x + y + z) + 7 * np.cos(x - y + z) - 7 * np.cos(x + y - z) + 7 * np.cos(
            x - y - z) + 14 * np.sin(x + z) - 14 * np.sin(x - z)) / 10

        jacobian[2][0] = 0
        jacobian[2][1] = (-16 * np.sin(y) - 7 * np.sin(y + z) - 7 * np.sin(y - z)) / 5
        jacobian[2][2] = (-7 * np.sin(y + z) - 7 * np.sin(y - z)) / 5
        return jacobian

    def callback1(self,data):
        self.joint1 = Float64()
        self.joint1 = float(data.data)
    def callback2(self,data):
        self.joint3 = Float64()
        self.joint3 = float(data.data)
    def callback3(self,data):
        self.joint4 = Float64()
        self.joint4 = float(data.data)

        self.coordinates = Float64MultiArray()
        self.coordinates.data = self.calculate_final_matrix()

        try:
            print(self.coordinates)
            self.coordinates_pub.publish(self.coordinates)
        except CvBridgeError as e:
            print(e)

    # def callback(self,data1,data2,data3):
    #
    #     self.joint1 = float(data1)
    #     self.joint3 = float(data2)
    #     self.joint4 = float(data3)
    #
    #     self.coordinates = Float64MultiArray()
    #     self.coordinates.data = self.calculate_final_matrix(self.joint1,self.joint3,self.joint4)
    #
    #     try:
    #         print(self.coordinates)
    #         self.coordinates_pub.publish(self.coordinates)
    #     except CvBridgeError as e:
    #         print(e)


# call the class
def main(args):
  fk = forward_kinematics()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)