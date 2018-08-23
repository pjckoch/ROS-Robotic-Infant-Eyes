#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <std_msgs/Float32.h>


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
  const int ratio = 3;
  const int kernel_size = 3;
  int lowerThreshold;
  const int default_lowerThreshold = 50;
  int upperThreshold = ratio*default_lowerThreshold;


  // publish CV images as ROS sensor messages
  void publishImage(cv::Mat src, image_transport::Publisher pub, bool color, ros::Time stamp_begin, ros::Publisher time_pub_) {
      sensor_msgs::Image msg;
      cv_bridge::CvImage bridge;


      if (color == true)
          bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, src);
      else
          bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, src);
      bridge.toImageMsg(msg);

      msg.header.stamp = ros::Time::now();
      pub.publish(msg);
  }

    void publishDuration(ros::Time begin, ros::Time end, ros::Publisher pub) {
      // timing analysis
      std_msgs::Float32 msg;
      ros::Duration elapsed = end - begin;
      msg.data = elapsed.toSec();
      pub.publish(msg);
  }



public:
  CannyDetector()
    : it_(nh_)
  {
      // obtain parameters from ROS parameter server
      nh_.getParam("image_topic_name", input_topic_name);
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
      time_pub_ = nh_.advertise<std_msgs::Float32>("edgeDuration", 1);

  }


  void detect(const sensor_msgs::ImageConstPtr& msg)
  {

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
      cv::Mat gray, dst_gray, edges, edges_color;

      // convert to gray image
      cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY); 

      // Reduce noise
      cv::GaussianBlur(gray, edges, cv::Size(3,3), 0, 0);

      // Canny edge detector
      cv::Canny(edges, edges, lowerThreshold, upperThreshold, kernel_size);

      // mask gray image with edge map and copy to dst_gray
      gray.copyTo(dst_gray, edges);

      // mask original color image with edge map and copy to dst_color
      cv::cvtColor(edges, edges_color, CV_GRAY2BGR);

      // Publish the edge map and the masked gray-scale image
      publishImage(edges, edge_pub_, false, msg->header.stamp, time_pub_);

  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_detector");
    CannyDetector cd;
    ros::spin();
    return 0;
}
