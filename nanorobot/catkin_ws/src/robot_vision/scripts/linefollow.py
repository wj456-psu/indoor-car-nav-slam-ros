#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
# from dynamic_reconfigure.server import Server
# from robot_vision.cfg import LineFollowConfig


def rgb_to_bound(r, g, b, tolerance):
    rgb = np.uint8([[[r, g, b]]])
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    hue = hsv[0][0][0]
    lower_color = np.array([hue - tolerance, 50, 50])
    upper_color = np.array([hue + tolerance, 255, 255])
    return lower_color, upper_color


def pre_processing(img):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    lab_planes = cv2.split(lab)
    lab_planes[0] = clahe.apply(lab_planes[0])
    lab = cv2.merge(lab_planes)
    img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    return img


class LineFollow:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "/cv_image/frame/compressed", CompressedImage, queue_size=1
        )
        self.mask_v_pub = rospy.Publisher("/cv_image/mask_v", Image, queue_size=1)
        self.mask_pub = rospy.Publisher("/cv_image/mask", Image, queue_size=1)
        self.mask_filtered_pub = rospy.Publisher(
            "/cv_image/mask_filtered", Image, queue_size=1
        )

        self.image_sub = rospy.Subscriber(
            "input_rgb_image", Image, self.image_callback, queue_size=1
        )
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        # self.srv = Server(LineFollowConfig, self.dyn_reconf_callback)

        self.twist = Twist()
        self.last_line_detection_time = rospy.Time.now()
        self.stop = False

        self.color_tolerance = rospy.get_param("~color_tolerance", 10)
        self.bound_r = rospy.get_param("~bound_r", 140)
        self.bound_g = rospy.get_param("~bound_g", 20)
        self.bound_b = rospy.get_param("~bound_b", 40)
        self.lower_color, self.upper_color = rgb_to_bound(
            self.bound_r, self.bound_g, self.bound_b, self.color_tolerance
        )

    # def dyn_reconf_callback(self, config, level):
    #     rospy.loginfo(config)
    #     self.color_tolerance = config["color_tolerance"]
    #     self.bound_r = config["bound_r"]
    #     self.bound_g = config["bound_g"]
    #     self.bound_b = config["bound_b"]
    #     self.lower_color, self.upper_color = rgb_to_bound(
    #         self.bound_r, self.bound_g, self.bound_b, self.color_tolerance
    #     )
    #     return config

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Apply preprocessing steps
        frame = cv2.resize(cv_image, (160, 120), interpolation=cv2.INTER_AREA)
        frame = pre_processing(frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv)
        _, mask_h = cv2.threshold(
            h, self.lower_color[0], self.upper_color[0], cv2.THRESH_BINARY
        )
        _, mask_s = cv2.threshold(
            s, self.lower_color[1], self.upper_color[1], cv2.THRESH_BINARY
        )
        _, mask_v = cv2.threshold(
            v, self.lower_color[2], self.upper_color[2], cv2.THRESH_BINARY
        )
        mask = cv2.bitwise_and(mask_h, mask_s)
        _, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
        mask_filtered = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_filtered = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_filtered = cv2.morphologyEx(
            mask_filtered, cv2.MORPH_CLOSE, kernel, iterations=3
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        mask_filtered = cv2.morphologyEx(mask_filtered, cv2.MORPH_OPEN, kernel)

        # Find contours of the mask
        contours, _ = cv2.findContours(
            mask_filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        self.twist.linear.x = 0
        self.twist.angular.z = 0

        # Check if any contours were found
        if contours:
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 2, cv2.LINE_AA)
            # Filter contours based on position in the frame
            height, width = frame.shape[:2]
            bottom_area = int(height * 0.8)
            filtered_contours = [contour for contour in contours if cv2.boundingRect(contour)[1] + cv2.boundingRect(contour)[3] >= bottom_area]

            # Check if any filtered contours remain
            if filtered_contours:
                self.stop = False
                self.last_line_detection_time = rospy.Time.now()
                cv2.drawContours(
                    frame, filtered_contours, -1, (255, 0, 0), 2, cv2.LINE_AA
                )
                # Get the largest filtered contour
                largest_contour = max(filtered_contours, key=cv2.contourArea)
                contour_area = cv2.contourArea(largest_contour)
                cv2.putText(
                    frame,
                    f"Area: {contour_area:.1f}",
                    (5, 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (255, 255, 255),
                    1,
                    cv2.LINE_8,
                )

                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    error = center_x - (width / 2)

                    self.twist.linear.x = 0.1 

                    self.twist.angular.z = -0.02 * error

                    cv2.putText(
                        frame,
                        f"Speed: {self.twist.linear.x:.2f} {self.twist.angular.z:.2f}",
                        (5, height - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                        cv2.LINE_8,
                    )

                    self.pub_cmd.publish(self.twist)
                else:
                    rospy.loginfo("Invalid moments - m00 is zero.")
                    if self.stop is False:
                        self.pub_cmd.publish(self.twist)
                        self.stop = True
            else:
                rospy.loginfo("No contours found.")
                if self.stop is False:
                    self.pub_cmd.publish(self.twist)
                    self.stop = True

        if rospy.Time.now() - self.last_line_detection_time > rospy.Duration(5) and self.stop is True:
            self.twist.angular.z = -0.5
            self.pub_cmd.publish(self.twist)

        # Publish the images
        frame_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.image_pub.publish(frame_msg)
        mask_v_msg = self.bridge.cv2_to_imgmsg(mask_v, "mono8")
        self.mask_v_pub.publish(mask_v_msg)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.mask_pub.publish(mask_msg)
        mask_filtered_msg = self.bridge.cv2_to_imgmsg(mask_filtered, "mono8")
        self.mask_filtered_pub.publish(mask_filtered_msg)


if __name__ == "__main__":
    rospy.init_node("line_follow")
    LineFollow()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
