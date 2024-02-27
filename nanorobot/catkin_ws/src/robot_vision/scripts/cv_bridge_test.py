#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def single_scale_retinex(img, sigma):
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(img, (0, 0), sigma)

    # Calculate the logarithm of the original image and blurred image
    log_img = np.log1p(img.astype(np.float32))
    log_blurred = np.log1p(blurred.astype(np.float32))

    # Calculate the difference between the log images
    ssr = cv2.subtract(log_img, log_blurred)

    # Normalize the output image
    ssr = cv2.normalize(ssr, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)

    return ssr


class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.frame_pub = rospy.Publisher(
            "/cv_image/frame", Image, queue_size=1)
        self.out_pub = rospy.Publisher(
            "/cv_image/retinex", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        frame = cv2.resize(cv_image, (160, 120), interpolation=cv2.INTER_CUBIC)

        # Split the image into B, G, R channels
        b, g, r = cv2.split(frame)

        # Apply Single-scale Retinex on each channel
        b_enhanced = single_scale_retinex(b, sigma=10)
        g_enhanced = single_scale_retinex(g, sigma=10)
        r_enhanced = single_scale_retinex(r, sigma=10)

        # Merge the enhanced channels back into an RGB image
        retinex = cv2.merge((b_enhanced, g_enhanced, r_enhanced))

        try:
            self.frame_pub.publish(self.bridge.cv2_to_imgmsg(
                frame, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
        try:
            self.out_pub.publish(self.bridge.cv2_to_imgmsg(
                retinex, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
