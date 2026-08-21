# IRIS - Final Project 1: Road Lane Detection with OpenCV + Sending Data via Websockets

| **Key**  | Value             |
| -------- | ----------------- |
| **Name** | Hendra Manudinata |
| **NRP**  | 5027251051        |

This repository contains the code and resources for the IRIS final project focused on road lane detection using OpenCV and sending the detected lane data and turn degree via Websockets.

## Directory Structure

- `src/`: Contains the main source code for lane detection and WebSocket communication.
- `assets/`: Contains sample videos and images used for testing the lane detection algorithm.
- `OLD_FILES/`: Contains previous versions of the code and experiments. Please do not see.

## Todo List

- [x] Implement lane detection using OpenCV
- [ ] Optimize lane detection algorithm for accuracy
- [x] Calculate turn degree based on lane position
- [x] Set up WebSocket server to send lane data and turn degree
- [x] Test the entire pipeline with sample video input
- [x] Document the project in README.md

## Base Station

Check here: <https://github.com/manoedinata/fp1_bs_iris25>

## Usage

0.  Don't forget to source your ROS 2 workspace:

    ```bash
    # Change 'kilted' with your ROS 2 distro name
    source /opt/ros/kilted/setup.bash
    ```

1.  Clone the repository:

    ```bash
    git clone https://github.com/manoedinata/IRIS-FP1.git
    cd IRIS-FP1
    ```

2.  Build the project

    ```bash
    colcon build --packages-select lane_detector
    ```

3.  Run all nodes:

    3.a. In terminal 1, run the lane detector node:

    ```bash
    source install/setup.bash
    ros2 run lane_detector lane_detector_node
    ```

    3.b. In terminal 2, run the odometry node:

    ```bash
    source install/setup.bash
    ros2 run lane_detector odometry_node
    ```

    3.c. In terminal 3, run the camera publisher node:

    ```bash
    source install/setup.bash
    ros2 run lane_detector camera_publisher_node

    # Optional: You can change the video source parameter
    # ros2 run lane_detector camera_publisher_node --ros-args -p video_source:=<path_to_your_video_file_or_stream_url>
    ```
