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
- cv_bridge, Copyright (c) 2011, Willow Garage, Inc.
- opencv2, Copyright (C) 2000-2015, Intel Corporation, all rights reserved.
          Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
          Copyright (C) 2015, OpenCV Foundation, all rights reserved.
          Copyright (C) 2015, Itseez Inc., all rights reserved.
------------------------------------------
*/

#include <ros/ros.h>
#include <ros/console.h>

#include <timing_analysis/timing_analysis.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <vector>

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>

class BlobDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher time_pub_;
  std::string input_topic_name;
  std::string frame_id;
  cv::SimpleBlobDetector::Params params;
  // temporal variables to store parameters that need to be casted lateron
  int tmp_minRepeatability, tmp_blobColor;
  // color hue ranges for HSV thresholding
  int lLowH, lLowV, lLowS, lHighH, lHighV, lHighS;
  // in the case of red, a second range is required, as red = 0 and red = 180
  bool secondRange;
  int uLowH, uLowV, uLowS, uHighH, uHighV, uHighS;


  void publishImage(cv::Mat src, std_msgs::Header src_header, ros::Time callback_begin) {
      sensor_msgs::Image msg;
      cv_bridge::CvImage bridge;

      bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, src);
      bridge.toImageMsg(msg);
      
      // copy message header of subscribed message
      msg.header = src_header;

      // timing_analysis: end
      ros::Time callback_end = ros::Time::now();
      // fill msg header with frame id
      msg.header.frame_id = frame_id;


      image_pub_.publish(msg);
      publishDuration(src_header.stamp, callback_begin, callback_end, time_pub_);
  }



public:
  BlobDetector()
    : it_(nh_)
  {
      // get image input topic name from ROS parameter server
      nh_.getParam("image_topic_name", input_topic_name);
   
      // get frame ID for message header
      nh_.getParam("/frame_id", frame_id); 
      // get HSV-thresholds from ROS parameter server
      nh_.param("/lLowH", lLowH, 0);
      nh_.param("/lLowS", lLowS, 0);
      nh_.param("/lLowV", lLowV, 0);
      nh_.param("/lHighH", lHighH, 180);
      nh_.param("/lHighS", lHighS, 255);
      nh_.param("/lHighV", lHighV, 255);
      
      // if filtering for red or for two colors, a second range is needed
      nh_.param("/secondRange", secondRange, false); 
      nh_.param("/uLowH", uLowH, 0);
      nh_.param("/uLowS", uLowS, 0);
      nh_.param("/uLowV", uLowV, 0);
      nh_.param("/uHighH", uHighH, 180);
      nh_.param("/uHighS", uHighS, 255);
      nh_.param("/uHighV", uHighV, 255);

      //########## get Blob Parameters from ROS parameter server
      // thresholds
      nh_.param("/thresholdStep", params.thresholdStep, (float)10);
      nh_.param("/minThreshold", params.minThreshold, (float)10);
      nh_.param("/maxThreshold", params.maxThreshold, (float)220);
      nh_.param("/minRepeatability", tmp_minRepeatability, 2);
      nh_.param("/minDistBetweenBlobs", params.minDistBetweenBlobs, (float)10);
      
      // filter by color: 0 = dark; 255 = light
      nh_.param("/filterByColor", params.filterByColor, false);
      nh_.param("/blobColor", tmp_blobColor, 0);

      // filter by area: set minArea large enough to eliminate noise
      nh_.param("/filterByArea", params.filterByArea, false);
      nh_.param("/minArea", params.minArea, (float)25);
      nh_.param("/maxArea", params.maxArea, (float)(320*240));

      // filter by circularity: circularity = 4*pi*Area/(perimeter^2)
      nh_.param("/filterByCircularity", params.filterByCircularity, false);
      nh_.param("/minCircularity", params.minCircularity, 0.6f);
      nh_.param("/maxCircularity", params.maxCircularity, (float)1e37);

      // filter by inertia: area distribution around rotational axis
      // low moment of inertia -> 1; high moment of inertia -> 0
      nh_.param("/filterByInertia", params.filterByInertia, false);
      nh_.param("/minInertiaRatio", params.minInertiaRatio, 0.1f);
      nh_.param("/maxInertiaRatio", params.maxInertiaRatio, (float)1e37);

      // filter by convexity
      nh_.param("/filterByConvexity", params.filterByConvexity, false);
      nh_.param("/minConvexity", params.minConvexity, 0.95f);
      nh_.param("/maxConvexity", params.maxConvexity, (float)1e37);
      
      // cast temporary variables to correct parameter format
      params.blobColor = (uchar)tmp_blobColor;
      params.minRepeatability = (ulong)tmp_minRepeatability;
      //########### end: Blob Parameters

      // set up subscriber and publisher
      image_sub_ = it_.subscribe(input_topic_name, 1,
        &BlobDetector::detect, this);
      image_pub_ = it_.advertise("/blobDetector/blob", 1);
      time_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("blobDuration", 10);

      ROS_DEBUG_STREAM("Blob detection for " << input_topic_name <<  " running.\n");

  }

  void detect(const sensor_msgs::ImageConstPtr& msg)
  {
      // timing analysis: start
      ros::Time callback_begin = ros::Time::now();

      // convert sensor message to CV image
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


      // CV matrices
      cv::Mat src_HSV, dst, lower_hue_range, upper_hue_range, hue_img;

      // Convert to HSV as it delivers better color separation
      cv::cvtColor(cv_ptr->image, src_HSV, cv::COLOR_BGR2HSV);

      // Threshold images
      cv::inRange(src_HSV, cv::Scalar(lLowH, lLowS, lLowV), cv::Scalar(lHighH, lHighS, lHighV), lower_hue_range);

      // Combine thresholds if there are two color ranges
      if (secondRange) {
                cv::inRange(src_HSV, cv::Scalar(uLowH, uLowS, uLowV), cv::Scalar(uHighH, uHighS, uHighV), upper_hue_range);
          cv::addWeighted(lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, hue_img);
      }
      else
          hue_img = lower_hue_range;
      // Blur to avoid false positives
      cv::GaussianBlur(hue_img, hue_img, cv::Size(3, 3), 2, 2);

      // init blob detector with parameters from ROS parameter server
      cv::Ptr<cv::SimpleBlobDetector> bd = cv::SimpleBlobDetector::create(params);

      // store keypoints in vector
      std::vector<cv::KeyPoint> keypoints;

      // detect keypoints and draw green circle in same size as keypoint
      bd->detect(hue_img, keypoints);
      cv::drawKeypoints(hue_img, keypoints, dst, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      // Publish result image
      publishImage(dst, msg->header, callback_begin);
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_detector");
    BlobDetector bd;
    ros::spin();
    return 0;
}
