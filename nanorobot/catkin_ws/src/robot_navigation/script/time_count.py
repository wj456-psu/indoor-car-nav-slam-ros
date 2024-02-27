#!/usr/bin/env python

import rospy
import time

def main():
    # Initialize the ROS node with a unique name
    rospy.init_node('timer_node', anonymous=True)

    # Get the current time
    start_time = time.time()

    # Main loop
    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - start_time

        # Print the elapsed time in seconds
        rospy.loginfo("=== TIME ===: %.2f seconds", elapsed_time)

        # Sleep for a while to control the loop rate (1 Hz in this example)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

