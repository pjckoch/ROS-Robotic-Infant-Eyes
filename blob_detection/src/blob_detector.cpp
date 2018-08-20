#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <ros/console.h>

static const std::string OPENCV_WINDOW = "Blob";

class BlobDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  std::string input_topic_name;
  const int ratio = 3;
  const int kernel_size = 3;
  int lowerThreshold;
  const int default_lowerThreshold = 50;
  int upperThreshold = ratio*default_lowerThreshold;

  void publishImage(cv::Mat src, image_transport::Publisher pub) {
      sensor_msgs::Image msg;
      cv_bridge::CvImage bridge;

      bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, src);
      bridge.toImageMsg(msg);
      pub.publish(msg);
  }


public:
  BlobDetector()
    : it_(nh_)
  {
      // Subscribe to input video feed and publish output video feed
      nh_.getParam("image_topic_name", input_topic_name);
      nh_.getParam("/detection_lowerThreshold", lowerThreshold);
      
      // check whether given threshold is valid
      if (lowerThreshold >= 0){
          upperThreshold = ratio*lowerThreshold;
          std::cout << std::endl << std::endl
          << "Lower threshold =  " << lowerThreshold
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
      image_pub_ = it_.advertise("/edgeDetector/canny", 1);
    
      cv::namedWindow(OPENCV_WINDOW);
  }

  ~CannyDetector()
  {
      cv::destroyWindow(OPENCV_WINDOW);
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
      cv::Mat gray;
      cv::Mat dst, edges;

      // convert to gray image
      cvtColor(cv_ptr->image, gray, CV_BGR2GRAY); 

      // Reduce noise
      cv::GaussianBlur(gray, edges, cv::Size(3,3), 0, 0);

      // Canny edge detector
      cv::Canny(edges, edges, lowerThreshold, upperThreshold, kernel_size);

      // Overwrite original img with masked gray image
      gray.copyTo(cv_ptr->image, edges);

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

      // Publish the edge map
      publishImage(edges, image_pub_);
      image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_detector");
    CannyDetector cd;
    ros::spin();
    return 0;
}
