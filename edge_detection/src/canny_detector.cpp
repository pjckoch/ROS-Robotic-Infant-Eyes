/*
The following is a summary of the licenses involved in this project.
Please also refer to the LICENSE folder in this github repository
for full licensing information.
LICENSE SUMMARY:
------------------------------------------
               BSD License
applies to:
- ros, Copyright (c) 2008, Willow Garage, Inc.
- std_msgs, Copyright (c) 2008, Willow Garage, Inc.
- sensor_msgs, Copyright (c) 2008, Willow Garage, Inc.
- image_transport, Copyright (c) 2009, Willow Garage, Inc. 
- cv_bridge, Copyright (c) 2011, Willow Garage, Inc.
- opencv2, Copyright (C) 2000-2015, Intel Corporation, all rights reserved.
          Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
          Copyright (C) 2015, OpenCV Foundation, all rights reserved.
          Copyright (C) 2015, Itseez Inc., all rights reserved.
------------------------------------------
*/

#include <ros/ros.h>

#include <timing_analysis/timing_analysis.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <iostream>

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>

class CannyDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher edge_pub_;
  image_transport::Publisher gray_pub_;
  image_transport::Publisher col_pub_;
  ros::Publisher time_pub_;
  std::string input_topic_name;
  std::string frame_id;
  const int ratio = 3;
  const int kernel_size = 3;
  int lowerThreshold;
  const int default_lowerThreshold = 50;
  int upperThreshold = ratio*default_lowerThreshold;


  // publish CV images as ROS sensor messages
  void publishImage(cv::Mat src, std_msgs::Header src_header, ros::Time callback_begin) {
      sensor_msgs::Image msg;
      cv_bridge::CvImage bridge;

      // convert CV image to ROS sensor message 
      bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, src);
      bridge.toImageMsg(msg);

      // copy message header of subscribed message
      msg.header = src_header;

      //timing analysis: end
      ros::Time callback_end = ros::Time::now();
      edge_pub_.publish(msg);
      publishDuration(src_header.stamp, callback_begin, callback_end, time_pub_);
  }
      

public:
  CannyDetector()
    : it_(nh_)
  {
      // obtain parameters from ROS parameter server
      nh_.getParam("image_topic_name", input_topic_name);
      nh_.getParam("/frame_id", frame_id);
      nh_.getParam("/detection_lowerThreshold", lowerThreshold);
      
      // check whether given threshold is valid
      if (lowerThreshold >= 0){
          upperThreshold = ratio*lowerThreshold;
          std::cout << std::endl << std::endl
          << "Lower threshold = " << lowerThreshold
          << "; Upper threshold = " << upperThreshold
          << std::endl << std::endl;
      }
      else {
          std::cout << std::endl << std::endl
          << "Please enter a valid threshold value >= 0"
          << std::endl << std::endl;
          lowerThreshold = default_lowerThreshold;
      }

      // set up subscriber and publisher
      image_sub_ = it_.subscribe(input_topic_name, 1,
        &CannyDetector::detect, this);
      edge_pub_ = it_.advertise("/edgeDetector/edge_map", 1);
      gray_pub_ = it_.advertise("/edgeDetector/gray_edges", 1);
      col_pub_ = it_.advertise("/edgeDetector/color_edges", 1);
      time_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("edgeDuration", 10);

  }


  void detect(const sensor_msgs::ImageConstPtr& msg)
  {

      // timing analysis: start
      ros::Time callback_begin = ros::Time::now();

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // declare CV Mat objects
      cv::Mat gray, edge_map;

      // convert to gray image
      cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY); 

      // Reduce noise
      cv::GaussianBlur(gray, edge_map, cv::Size(3,3), 0, 0);

      // Canny edge detector
      cv::Canny(edge_map, edge_map, lowerThreshold, upperThreshold, kernel_size);

      // Publish the edge map and the masked gray-scale image
      publishImage(edge_map, msg->header, callback_begin);

  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_detector");
    CannyDetector cd;
    ros::spin();
    return 0;
}
