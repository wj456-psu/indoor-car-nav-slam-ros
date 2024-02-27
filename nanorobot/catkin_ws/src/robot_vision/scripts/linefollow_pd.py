#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class LineFollow:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "cv_bridge_image", Image, queue_size=1)
        self.adjusted_image_pub = rospy.Publisher(
            "adjusted_image", Image, queue_size=1)

        self.image_sub = rospy.Subscriber(
            "input_rgb_image", Image, self.image_callback, queue_size=1)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.twist = Twist()
        self.logmark = True
        self.prev_error = 0.0  # Initialize previous error to zero

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as e:
            print(e)

        # Adjust image brightness using histogram equalization
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv[:, :, 2] = cv2.equalizeHist(hsv[:, :, 2])
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # Resize the frame
        frame_resized = cv2.resize(
            frame, (160, 120), interpolation=cv2.INTER_CUBIC)

        # Convert resized frame to HSV color space
        hsv_resized = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the blue color
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Threshold the resized HSV image to get only blue colors
        mask = cv2.inRange(hsv_resized, lower_blue, upper_blue)

        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours of the blue objects
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.twist.linear.x = 0
        self.twist.angular.z = 0

        # Create a copy of the resized frame for drawing contour
        frame_contour = frame_resized.copy()

        if len(contours) > 0:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Draw contour on the resized frame
            cv2.drawContours(
                frame_contour, [largest_contour], -1, (0, 255, 0), 2)

            # Calculate the size of the largest contour
            contour_area = cv2.contourArea(largest_contour)

            # Adjust the linear velocity based on the size of the contour
            if contour_area > 1600:
                self.twist.linear.x = 0.15  # Move forward faster when a large region is detected
            else:
                self.twist.linear.x = 0.05  # Move forward at a steady pace

            # Calculate the centroid of the largest contour
            M = cv2.moments(largest_contour)
            cx = int(M['m10'] / M['m00'])

            # Calculate the deviation from the center of the image
            deviation_x = cx - 80

            # PD control parameters
            kp = 2.0 # Proportional gain
            kd = 1.0  # Derivative gain

            # Calculate the error and derivative of the error
            error = deviation_x
            delta_error = error - self.prev_error

            # Calculate the control signal
            control_signal = kp * error + kd * delta_error

            # Adjust the control signal based on the deviation
            max_control_signal = 0.6
            deviation_factor = 1.0  # Increase this factor to make it turn harder
            adjusted_control_signal = control_signal + \
                (deviation_factor * deviation_x)

            # Limit the adjusted control signal to a maximum value for smoother movements
            adjusted_control_signal = max(-max_control_signal,
                                          min(max_control_signal, adjusted_control_signal))

            # Update the angular velocity
            self.twist.angular.z = -adjusted_control_signal

            # Update the previous error
            self.prev_error = error

            self.logmark = True
        else:
            if self.logmark:
                self.logmark = False
                rospy.loginfo("Unrecognized Line in vision")

        self.pub_cmd.publish(self.twist)

        if len(contours) == 0:
            rospy.loginfo("No Line in vision")

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(
            mask, encoding="passthrough"))
        self.adjusted_image_pub.publish(
            self.bridge.cv2_to_imgmsg(frame_contour, encoding="bgr8"))

    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        rospy.init_node("line_follow")
        LineFollow()
        rospy.loginfo("Line follow is started.")
        rospy.loginfo("Please subscribe to the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down line follow node.")
        cv2.destroyAllWindows()
