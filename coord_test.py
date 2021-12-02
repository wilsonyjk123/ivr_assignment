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
      rospy.init_node('image_processing2', anonymous=True)
      # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
      # self.image_sub1 = rospy.Subscriber("/image_topic1", Image, self.callback)
      self.image_sub1 = message_filters.Subscriber("/image_topic1", Image)
      # receive image from second camera
      self.image_sub2 = message_filters.Subscriber("/image_topic2", Image)

      self.red_pub = rospy.Publisher("/red", Float64MultiArray, queue_size=10)
      # Synchronize subscriptions into one callback
      timesync = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
      timesync.registerCallback(self.callback1)
      # initialize a publisher to send joints' angular position to a topic called joints_pos

      # initialize the bridge between openCV and ROS
      self.time_initial = rospy.get_time()
      # initialize errors
      self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
      self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
      self.bridge = CvBridge()

      #hardcode yellow and blue
      self.green1 = np.array([402, 543])
      self.green2 = np.array([402, 543])
      self.yellow1 = np.array([400, 440])
      self.yellow2 = np.array([400, 440])

      #record the last position of red and blue
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
      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0:
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
          if image is self.cv_image1:
              self.last_red_image1[0] = cx
              self.last_red_image1[1] = cy
              return np.array([cx, cy])
          else:
              self.last_red_image2[0] = cx
              self.last_red_image2[1] = cy
              return np.array([cx, cy])
      # this is in case red is blocked by green
      else:
          if image is self.cv_image1:
              return np.array(self.last_red_image1)
          else:
              return np.array(self.last_red_image2)



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
  def detect_blue(self, image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0:
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
          if self.cv_image1 is image:
              self.last_blue_image1 = np.array([cx, cy])
              return np.array([cx, cy])
          else:
              self.last_blue_image2 = np.array([cx, cy])
              return np.array([cx, cy])
      # this is in case red is blocked by green
      else:
          if self.cv_image1 is image:
              return self.last_blue_image1
          else:
              return self.last_blue_image2

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def pixel2meter(self, image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_yellow(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos) ** 2)
      return 4 / np.sqrt(dist)


  def detect_red_c(self):

      a = self.pixel2meter(self.cv_image1)
      b = self.pixel2meter(self.cv_image2)
      red1 = a * self.detect_red(self.cv_image1)
      red2 = b * self.detect_red(self.cv_image2)

      yellow1 = a * self.yellow1
      yellow2 = b * self.yellow2

      blue1 = a * self.detect_blue(self.cv_image1)
      blue2 = b * self.detect_blue(self.cv_image2)

      green1 = a * self.green1
      green2 = b * self.green2

      red_coordinates = np.array([red2[0]-green2[0],red1[0]-green1[0],green1[1]-red1[1]])
      print(red_coordinates)
      return red_coordinates


  # Recieve data, process it, and publish
  def callback1(self,data1,data2):

    # Recieve the image
      try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
      except CvBridgeError as e:
            print(e)

      self.redco = Float64MultiArray()
      self.redco.data = self.detect_red_c()

      # Publish the results
      try:
          print(self.redco)
          self.red_pub.publish(self.redco)
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