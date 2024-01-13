# Pose Estimation using PnP (Perspective-n-Point)

## Overview

This README provides a conceptual overview of Pose Estimation using PnP, focusing on the transformation matrix that converts 3D world points to 2D image points in the camera frame. It addresses the scenario where the transformation matrix is unknown but can be estimated using camera parameters and distortion coefficients.

## Transformation Process

Consider the following transformation process:

### 1. World Points (3D Coordinates - x, y, z):

The object is represented by 3D points in a world coordinate frame.

### 2. Unknown Transformation Matrix:

There exists a transformation matrix that converts these world points to 2D image points in the camera frame. However, the matrix is initially unknown.

### 3. Camera Parameters and Distortion Coefficients:

Camera parameters (intrinsic matrix) and distortion coefficients are known. These parameters define the camera's characteristics.

### 4. Conversion to Camera Image Frame:

Using the known camera parameters and distortion coefficients, the 3D world points are transformed into 2D image points in the camera image frame.

### 5. Known Image Points (x, y):

Corresponding 2D image points are also known and represent the projections of the world points onto the image plane.

### 6. PnP Estimation:

Pose Estimation using PnP is employed to estimate the unknown transformation matrix. This involves finding the rotation and translation vectors that best align the known 2D image points with the transformed 3D points.

### 7. Transformation Matrix Obtained:

The PnP algorithm yields a transformation matrix that accurately converts 3D world points to 2D image points in the camera frame.

### 8. Pose Information:

The rotation vector provides information about the yaw angle (orientation), and the translation vector gives insight into the distance to travel in the camera frame.

For more info refer - https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html


# Robot Perception ROS Workspace

This ROS workspace, named `robot_perception_ws`, contains a package named `robot_perception` with nodes for image points publishing and pose estimation.

## Getting Started

These instructions will guide you through setting up and running the nodes in this ROS workspace.

### Prerequisites

- ROS (Robot Operating System) installed. You can follow the installation instructions [here](http://wiki.ros.org/noetic/Installation).

### Installation

1. **Create a ROS Workspace:**
    ```bash
    mkdir -p ~/robot_perception_ws/src
    cd ~/robot_perception_ws/src
    catkin_init_workspace
    ```

2. **Create a Package and Nodes:**
    ```bash
    cd ~/robot_perception_ws/src
    catkin_create_pkg robot_perception rospy std_msgs sensor_msgs cv_bridge
    ```

3. **Place Nodes in the Package:**
    - Create two scripts, `image_points_publisher.py` and `pose_estimation_node.py`, inside the `src/robot_perception` directory.

4. **Edit CMakeLists.txt:**
    - Open `CMakeLists.txt` in `robot_perception` and modify it as follows:
        ```cmake
        find_package(catkin REQUIRED COMPONENTS
          rospy
          std_msgs
          sensor_msgs
          cv_bridge
        )

        catkin_python_setup()

        catkin_package()

        install(PROGRAMS
          scripts/image_points_pub.py
          scripts/pnp_sub.py
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        ```

5. **Build the Workspace:**
    ```bash
    cd ~/robot_perception_ws
    catkin_make
    ```

6. **Source the Workspace:**
    ```bash
    source devel/setup.bash
    ```

## Running the Nodes

Open multiple terminals:
1. Terminal 1: 
    ```bash
    roscore
    ```
2. Terminal 2: 
    ```bash
    rosrun robot_perception image_points_pub.py
    ```
3. Terminal 3: 
    ```bash
    rosrun robot_perception pnp_sub.py
    ```

Adjust the names, scripts, and dependencies based on your project's requirements.

Also, here I have generated some random 2d points to simulate the process, but in real time it has to retrieve those 2d points from the object detected in the image.



