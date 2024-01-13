#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

def image_points_publisher():
    rospy.init_node('image_points_publisher', anonymous=True)
    image_points_pub = rospy.Publisher('/image_points', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        # Generate random image points
        num_points = 4
        image_points = np.random.rand(num_points, 2) * 100  # Random points in a 100x100 space

        # Create Float32MultiArray message
        image_points_msg = Float32MultiArray()
        image_points_msg.data = image_points.flatten().tolist()

        # Publish the message
        image_points_pub.publish(image_points_msg)

        # Sleep for a while before publishing the next set of points
        time.sleep(1.0)

if __name__ == '__main__':
    try:
        image_points_publisher()
    except rospy.ROSInterruptException:
        pass
