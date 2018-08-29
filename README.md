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

### Note:
The ROS nodes in this repository can be analyzed regarding their timing. Therefore, you will also need the [ROS-Timing](https://github.com/pjckoch/ROS-Timing.git) repository. Furthermore, you will need to clone the [video_stream_opencv](https://github.com/ros-drivers/video_stream_opencv.git) repository. If you want to perform stereo matching, you will also need the **stereo_image_proc** package which is part of the [ROS image_pipeline](https://github.com/ros-perception/image_pipeline.git). The patch included in this repository in the folder "patch" can be applied to the stereo_image_proc to analyze timing.

## How to use
1. `git clone` this repository and the repositories listed in the above **Note**.
2. `roslaunch vision_launcher piVisualStream.launch` on any machine connected to two USB cameras.
3. `roslaunch vision_launcher pcVisualProcessing.launch` on any machine of your choice. Keep in mind that visual processing is computationally expensive.

**Remark**: you can optionally pass parameters to the nodes when calling roslaunch. See below for a list of parameters.

## ROS parameters

### piVisualStream.launch:
- **video_device_right** and **video_device_left**: The index for the right and left camera respectively. For example, index 0 corresponds to `/dev/video0`
- **device**: If using the audio_proc driver, PyAudio will give an index to every sound device. If you set device to a valid index, the driver will capture from this device. If you do not set the device parameter or set it to an invalid index, the driver will print out all available input devices and choose the first one automatically. In case, you use the audio_common driver, the device parameter refers to the index that you get when running `arecord -l` from your command line.
- **sample_rate**: Sample rate in Hertz (Hz) with which you want to capture audio. It needs to be valid for the chosen device. If unsure which sample rate is supported, you can leave the device parameter empty and see what the driver prints to your terminal. Besides the device indices, it will print name and default sample rate for each input device.
- **buffer_size**: Only for audio_proc driver. The buffer size is also referred to as frames per buffer or chunk. It specifies how many frames are stored into one buffer. As the FFT is based on the Cooley-Tukey algorithm, performance is best if the buffer size is a power of two. Note that large values (>= 1024) result in higher spectral resolution. However if the value is very large (>= 8192), the plot might respond very slowly. A good trade-off is a value of 2048.
- **fft_at_robot**: Set this to `true` if you want to perform the FFT directly on your robot. Keep in mind that it might run slower than on a PC.
- **depth**: Only for audio_common driver. Specifies the bit depth.

- **channels**: Only for audio_common driver. Number of channels to use. The audio_proc driver supports only 1 channel.

### pcAudioProcessing.launch:
