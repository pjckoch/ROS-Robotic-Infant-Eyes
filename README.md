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

Every step (capture, edge detection, blob detection, stereo matching) is implemented as a separate ROS node. This enables us to the spread the nodes across distributed system compontens. In my case, the USB camera driver runs on a Raspberry Pi Zero W, whereas the nodes run on a PC. Remember to use a common ROS_MASTER_URI on the different devices.

## Prerequisites
The ROS nodes in this repository can be analyzed regarding their timing. Therefore, you will also need the [ROS-Timing]
- Clone this repository.
- Clone the [ROS-Timing](https://github.com/pjckoch/ROS-Timing.git) repository: Allows for timing analyzation and synchronization.
- Clone the [video_stream_opencv](https://github.com/ros-drivers/video_stream_opencv.git) repository: Used as USB camera driver.
- Clone the [ROS image_pipeline](https://github.com/ros-perception/image_pipeline.git): Necessary for computer vision related applications in ROS. Used e.g. for camera calibration and stereo matching. This [patch](patch/stereo_image_proc_timing_analysis.patch) can be applied to the stereo_image_proc to analyze timing.
- Calibrate your cameras: `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left  --no-service-check --approximate=0.1`. (`--size` refers to the number of internal corners of your chessboard, `--square` refers to the edge length of one square on the chessboard. For more information on the parameters, consult the [ROS wiki](http://wiki.ros.org/camera_calibration)).

## How to use
1. `roslaunch vision_launcher piVisualStream.launch` on any machine connected to two USB cameras. This will start the video stream.
2. `roslaunch vision_launcher pcVisualProcessing.launch` on any machine of your choice. This will do the processing. Keep in mind that visual processing is computationally expensive. If you wish to use only camera (without stereo matching), you will have to adapt the launch files. 

**Remark**: You can optionally pass parameters to the nodes when calling roslaunch. See below for a list of parameters.

## ROS parameters

### [piVisualStream.launch](vision_launcher/launch/piVisualStream.launch)

- **video_device_right** and **video_device_left**: The index for the right and left camera respectively. For example, index 0 corresponds to `/dev/video0`
- **image width** and **image height**: Specify the resolution to use.
- **framerate**: The frame rate (FPS) at which image frames are grabbed from the camera.
- **image_ns**: The namespace under which the video stream is launched. The processing nodes will subscribe to images from the "stereo" namespace.
- **camera_name_right** and **camera_name_left**: Like the **image_ns**, these parameters determine the topic names under which the video stream is published. Moreover the names are used when searching for the camera info.
- **image_topic_name**: The default topic name would be `/<image_ns>/<camera_name_*>/image_raw`, but this topic name will be used by a republisher (see remarks below), so the original stream is remapped to `/<image_ns>/<camera_name_*>/<image_topic_name>`.
- **camera_info_path**: This points to the directory where the camera calibration data is stored.


### [pcVisualProcessing.launch](vision_launcher/launch/pcVisualProcessing.launch)

- **edge_on**: Choose whether to run the edge detector.
- **blob_on**: Choose whether to run the blob detector.
- **stereo_matching**: Choose whether to run the stereo matcher. Note that this is the only processing node that is deactivated by default, because it requires a camera calibration. The edge and blob detector can be run without calibration.
- **blob_shape** and **blob_color**: The blob detector will get its parameters from two yaml-files. One specifies the shape and one the color of the blob to detect. The parameters must must match existing file names (without the .yaml extension). This repository comes with a [circular](blob_detection/config/circular.yaml) shape and three colors ([red](blob_detection/config/red.yaml), [green](blob_detection/config/green.yaml) and [blue](blob_detection/config/blue.yaml)). You are free to add more configuration files.
- **edge_lowerThreshold**: This specifies the lower threshold for the Canny edge detector. See the [OpenCV Documentation](https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html) for further information.

## Remarks

- If running the visual system over distributed machines that are connected over WiFi, it is recommendable to use compressed image streams due to bandwidth limitations.
- Therefore the processing nodes are launched with the parameter  `image_transport:=compressed` by default.
- However, for stereo matching, this did not work. The stereo matcher keeps subscribing to the raw image stream.
- Hence, a republisher is used which runs on the machine that does the processing. It subscribes to the compressed image stream and republishes it in raw format for the stereo matcher

## License

This project is licensed under the 3-Clause-BSD-License (see the [LICENSE.md](LICENSE/LICENSE.md) for details). For third-party licenses, see [LICENSE-3RD-PARTY.md](LICENSE/LICENSE-3RD-PARTY.md).

