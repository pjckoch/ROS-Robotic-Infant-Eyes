# ROS-Robotic-Infant-Eyes

## Description
This repository contains the catkin packages for the visual system of a robotic infant which I am working on for my bachelor thesis.

The repository provides tools for capturing live stereo video from two USB cameras and performing various computer vision processing steps on the video stream.

There are three catkin packages in this repository: edge_detection, blob_detection and vision_launcher.

The **edge_detection package** contains one C++ program:
- [canny_detector.cpp](edge_detection/src/canny_detector.cpp): An edge_detector which uses the Canny algorithm. It subscribes to an image stream and publishes the corresponding edge map. 

The **blob_detection** package contains one C++ program:
- [blob_detector.cpp](blob_detection/src/blob_detector.cpp): A blob detector which uses OpenCV's SimpleBlobDetector algorithm. It subscribes to an image stream, filters for a user-specified color and then searches for blobs of a user-specified shape. It publishes the single-channel image with all detected blobs circled in green.

The **vision_launcher** package is only used for launching the robotic infant's visual system, i.e. the stereo image stream, the edge detector, the blob detector and the stereo matcher.

Every step (capture, edge detection, blob detection, stereo matching) is implemented as a separate ROS node. This enables us to the spread the nodes across distributed system compontens. In my case, the USB camera driver runs on a Raspberry Pi Zero W, whereas the other two nodes run on a PC. Remember to use a common ROS_MASTER_URI on the different devices.

### Prerequisites:
The ROS nodes in this repository can be analyzed regarding their timing. Therefore, you will also need the [ROS-Timing]
- Clone this repository.
- Clone the [ROS-Timing](https://github.com/pjckoch/ROS-Timing.git) repository: Allows for timing analyzation and synchronization.
- Clone the [video_stream_opencv](https://github.com/ros-drivers/video_stream_opencv.git) repository: Used as USB camera driver.
- Clone the [ROS image_pipeline](https://github.com/ros-perception/image_pipeline.git): Necessary for computer vision related applications in ROS. Used e.g. for camera calibration and stereo matching. This [patch](patch/stereo_image_proc_timing_analysis.patch) can be applied to the stereo_image_proc to analyze timing.
- Calibrate your cameras: `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left  --no-service-check --approximate=0.1`. (`--size` refers to the number of internal corners of your chessboard, `--square` refers to the edge length of one square on the chessboard. For more information on the parameters, consult the [ROS wiki](http://wiki.ros.org/camera_calibration)).

## How to use
1. `roslaunch vision_launcher piVisualStream.launch` on any machine connected to two USB cameras. This will start the video stream
2. `roslaunch vision_launcher pcVisualProcessing.launch` on any machine of your choice. Keep in mind that visual processing is computationally expensive. If you wish to use only camera (without stereo matching), you will have to adapt the launch files.

**Remark**: you can optionally pass parameters to the nodes when calling roslaunch. See below for a list of parameters.

## ROS parameters

### piVisualStream.launch:
- **video_device_right** and **video_device_left**: The index for the right and left camera respectively. For example, index 0 corresponds to `/dev/video0`
- **image width** and **image height**: Specify the resolution to use.
- **image_ns**: The namespace under which the video stream is launched. The processing nodes will subscribe to images from the "stereo" namespace.
- **camera_name_right** and **camera_name_left**: Like the **image_ns**, these parameters determine the topic name under which the video stream is published.
- **fft_at_robot**: Set this to `true` if you want to perform the FFT directly on your robot. Keep in mind that it might run slower than on a PC.
- **depth**: Only for audio_common driver. Specifies the bit depth.

- **channels**: Only for audio_common driver. Number of channels to use. The audio_proc driver supports only 1 channel.

### pcAudioProcessing.launch:
