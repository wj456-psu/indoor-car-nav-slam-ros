#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2


def image_callback(image_msg):
    # Convert ROS Image message to OpenCV image
    cv_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    # Compress the image using JPEG encoding and convert it to a CompressedImage message
    compressed_image_msg = CvBridge().cv2_to_compressed_imgmsg(
        cv_image, dst_format="jpg"
    )

    # Publish the compressed image
    compressed_image_pub.publish(compressed_image_msg)


if __name__ == "__main__":
    rospy.init_node("image_compressor_node")

    # Set the topic names for subscribing and publishing
    # darknet_ros -> '/darknet_ros/detection_image' yolov3_pytorch_ros -> '/detections_image_topic'
    image_sub_topic = rospy.get_param("~image_sub_topic")
    image_pub_topic = rospy.get_param("~image_pub_topic")

    # Create a subscriber for the raw image
    image_sub = rospy.Subscriber(image_sub_topic, Image, image_callback)

    # Create a publisher for the compressed image
    compressed_image_pub = rospy.Publisher(
        image_pub_topic, CompressedImage, queue_size=1
    )

    rospy.loginfo("Started image compressor node!")
    rospy.loginfo(f"{image_sub_topic} -> {image_pub_topic}")

    # Spin the ROS node
    rospy.spin()
