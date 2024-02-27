#!/usr/bin/env python

import time

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


def compressed_image_callback(msg):
    try:
        # Convert the CompressedImage to OpenCV format using CvBridge
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg)

        # Convert the OpenCV image to an Image message
        image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        # Publish the Image message on the new topic
        image_pub.publish(image_msg)

    except Exception as e:
        rospy.logerr(e)


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("compressed_image_converter")

    # Create a publisher for the new Image topic
    image_pub = rospy.Publisher("/converted_image", Image, queue_size=1)

    # Subscribe to the CompressedImage topic
    image_topic_param = "/rosapi/nnrb_image_topic"
    while not rospy.has_param(image_topic_param):
        time.sleep(1)
    sub_topic = rospy.get_param(image_topic_param)
    rospy.Subscriber(sub_topic, CompressedImage, compressed_image_callback)

    rospy.loginfo("Started image converter node!")

    # Start the ROS spin loop
    rospy.spin()
