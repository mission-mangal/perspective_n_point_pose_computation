#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class PoseEstimationNode:
    def __init__(self):
        rospy.init_node('pose_estimation_node', anonymous=True)
        self.bridge = CvBridge()

        # Replace these with your actual values
        # Replace these values with your actual camera calibration parameters
        fx = 500.0
        fy = 500.0
        cx = 320.0
        cy = 240.0

        k1 = 0.1
        k2 = 0.05
        p1 = 0.01
        p2 = -0.02
        k3 = 0.0
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.distortion_coefficients = np.array([k1, k2, p1, p2, k3])

        # Object points in the arrow mark board's coordinate system
        self.object_points = np.array([[0, 0, 0], [30, 0, 0], [30, 20, 0], [0, 20, 0]], dtype=np.float32)

        self.image_points_subscriber = rospy.Subscriber('/image_points', Float32MultiArray, self.image_points_callback)

    def image_points_callback(self, image_points_msg):
        # Convert the received Float32MultiArray to a NumPy array
        image_points = np.array(image_points_msg.data, dtype=np.float32).reshape(-1, 2)

        # Solve for pose using solvePnP
        _, rvecs, tvecs = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.distortion_coefficients)

        # Calculate the rotation matrix from the rotation vector
        R, _ = cv2.Rodrigues(rvecs)

        # Extract yaw angle from rotation matrix
        yaw_angle = np.degrees(np.arctan2(R[1, 0], R[0, 0]))

        # Calculate the Euclidean distance in the x-y plane
        distance_xy = np.linalg.norm(tvecs[0][:2])

        # Display the results
        rospy.loginfo(f"Yaw angle to be turned: {yaw_angle:.2f} degrees")
        rospy.loginfo(f"Distance to be traveled in x-y plane: {distance_xy:.2f} units (e.g., centimeters)")

if __name__ == '__main__':
    try:
        node = PoseEstimationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
