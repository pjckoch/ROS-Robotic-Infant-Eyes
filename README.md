# ROS-Robotic-Infant-Eyes

## Description
This repository contains the catkin packages for the visual system of a robotic infant which I am working on for my bachelor thesis.

#### Note:
The ROS nodes in this repository can be analyzed regarding their timing. Therefore, you will also need the **ROS-Timing** repository (https://github.com/pjckoch/ROS-Timing.git). Furthermore, you will need to clone the **video_stream_opencv** repository (https://github.com/ros-drivers/video_stream_opencv.git. If you want to perform stereo matching, you will also need the **stereo_image_proc** package which is part of the ROS image_pipeline (https://github.com/ros-perception/image_pipeline.git). The patch included in this repository in the folder "patch" can be applied to the stereo_image_proc

This repository provides tools for capturing live stereo video from two USB cameras and performing various computer vision processing steps on the video stream.

There are three catkin packages in this repository: edge_detection, blob_detection and vision_launcher.

The **edge_detection package** contains one C++ program:
- **canny_detector.cpp**: An edge_detector which uses the Canny algorithm. It subscribes to an image stream and publishes the corresponding edge map. 

The **blob_detection** package contains one C++ program:
- **blob_detector.cpp**: A blob detector which uses OpenCV's SimpleBlobDetector algorithm. It subscribes to an image stream, filters for a user-specified color and then searches for blobs of a user-specified shape. It publishes the single-channel image with all detected blobs circled in green.

The **vision_launcher** package is only used for launching the robotic infant's visual system, i.e. the stereo image stream, the edge detector, the blob detector and the stereo matcher.

## How to use
1. `git clone` this repository and the repositories listed in the above **Note**.
2. `roslaunch vision_launcher piVisualStream.launch` on any machine connected to two USB cameras.
3. `roslaunch vision_launcher pcVisualProcessing.launch` on any machine of your choice. Keep in mind that visual processing is computationally expensive.

**Remark**: you can optionally pass parameters to the nodes when calling roslaunch. See below for a list of parameters.

