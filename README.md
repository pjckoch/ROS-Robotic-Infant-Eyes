# ROS-Robotic-Infant-Eyes
This repository contains the catkin packages for the visual system of a robotic infant which I am working on for my bachelor thesis.

**Note**: the ROS nodes in this repository can be analyzed regarding their timing. Therefore, you will also need the **ROS-Timing** repository (https://github.com/pjckoch/ROS-Timing.git). Furthermore, you will need to clone the **video_stream_opencv** repository (https://github.com/ros-drivers/video_stream_opencv.git. If you want to perform stereo matching, you will also need the **stereo_image_proc** package which is part of the ROS image_pipeline (https://github.com/ros-perception/image_pipeline.git). The patch included in this repository in the folder "patch" can be applied to the stereo_image_proc

This repository provides tools for streaming live stereo video and performing various computer vision processing steps.

There are three catkin packages in this repository: edge_detection, blob_detection and vision_launcher.

The **edge_detection package** contains one C++ program:
- **canny_detector.cpp**: An edge_detector which uses the canny algorithm. It subscribes to an image stream and publishes the corresponding edge map. 

The **blob_detection** package contains one C++ program:
- **blob_detector.cpp**:

The **vision_launcher** package is only used for launching the 

