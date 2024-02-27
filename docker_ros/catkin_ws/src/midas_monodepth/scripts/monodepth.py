#!/usr/bin/env python

# import time

import cv2
import numpy as np
import rospy
import torch
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class MiDaS:
    def __init__(self) -> None:
        model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)
        self.midas = torch.hub.load("intel-isl/MiDaS", model_type)
        self.device = (
            torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        )
        self.midas.to(self.device)
        self.midas.eval()
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = self.midas_transforms.small_transform

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/converted_image", Image, self.image_callback, queue_size=1
        )
        self.image_pub = rospy.Publisher("/depth_image", Image, queue_size=1)
        rospy.loginfo("Started MiDaS!")

    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # start = time.time()

        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Apply input transforms
        input_batch = self.transform(img).to(self.device)

        # Prediction and resize to original resolution
        with torch.no_grad():
            prediction = self.midas(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()

        depth_map = cv2.normalize(
            depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F
        )

        # end = time.time()
        # totalTime = end - start

        # fps = 1 / totalTime

        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        depth_map = (depth_map * 255).astype(np.uint8)
        depth_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_MAGMA)

        # print('depth_map.shape = ',depth_map.shape)
        dim = (192 * 3, 108 * 4)
        # img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
        depth_map = cv2.resize(depth_map, dim, interpolation=cv2.INTER_AREA)
        depth_map = cv2.cvtColor(depth_map, cv2.COLOR_RGB2BGR)

        depth_map_msg = self.bridge.cv2_to_imgmsg(depth_map)
        self.image_pub.publish(depth_map_msg)

        # cv2.putText(
        #     img,
        #     f"FPS: {int(fps)}",
        #     (20, 70),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     1.5,
        #     (0, 255, 0),
        #     2,
        # )


def main():
    rospy.init_node("monodepth")
    MiDaS()
    rospy.spin()


if __name__ == "__main__":
    main()
