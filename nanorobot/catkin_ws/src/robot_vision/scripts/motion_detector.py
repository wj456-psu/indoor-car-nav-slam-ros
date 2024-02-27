#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError


class motionDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # Create cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "cv_bridge_image", Image, queue_size=1)

        # Setting parameters: minimum area, threshold
        self.minArea = rospy.get_param("~minArea",   500)
        self.threshold = rospy.get_param("~threshold", 25)

        self.firstFrame = None
        self.text = "Unoccupied"

        # Initialize subscribers who subscribe to image data in rgb format, where the topic name of the image topic can be remapped in the launch file
        self.image_sub = rospy.Subscriber(
            "input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        # Use cv_bridge to convert ROS image data to OpenCV image format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as e:
            print(e)

        # Create a grayscale image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # Use two frames of images for comparison to detect areas of moving objects
        if self.firstFrame is None:
            self.firstFrame = gray
            return
        frameDelta = cv2.absdiff(self.firstFrame, gray)
        thresh = cv2.threshold(frameDelta, self.threshold,
                               255, cv2.THRESH_BINARY)[1]

        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts, hierarchy = cv2.findContours(
            thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            # Ignored if the detected area is smaller than the set value
            if cv2.contourArea(c) < self.minArea:
                continue

            # Frame the recognized objects on the output screen
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (50, 255, 50), 2)
            self.text = "Occupied"

        # Print the current status and timestamp information on the output screen
        cv2.putText(frame, "Status: {}".format(self.text), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Convert the recognized image into a ROS message and publish it
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        # Initialize the ros node
        rospy.init_node("motion_detector")
        rospy.loginfo("motion_detector node is started...")
        rospy.loginfo("Please subscribe the ROS image.")
        motionDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down motion detector node.")
        cv2.destroyAllWindows()
