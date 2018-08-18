#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <ros/console.h>

static const std::string OPENCV_WINDOW = "Canny Edge Map";

class CannyDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  std::string input_topic_name;
  int threshold;
  const int default_threshold = 50;

public:
  CannyDetector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    nh_.getParam("image_topic_name", input_topic_name);
    nh_.getParam("/detection_threshold", threshold);
    // check whether given threshold is valid
    if (threshold >= 0 && threshold <= 100){
        std::cout << std::endl << std::endl << "Edge detection threshold set     to " << threshold << std::endl << std::endl;
    }
    else {
        std::cout << std::endl << std::endl << "Please enter a valid thresho    ld value between 0 and 100" << std::endl << std::endl;
    threshold = default_threshold;
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


    // variables
    const int ratio = 3;
    const int kernel_size = 3;

    cv::Mat gray;
    cv::Mat dst, edges;

    // convert to gray image
    cvtColor(cv_ptr->image, gray, CV_BGR2GRAY); 

    // Reduce noise
    cv::GaussianBlur(gray, edges, cv::Size(3,3), 0, 0);

    // Canny edge detector
    cv::Canny(edges, edges, threshold, threshold*ratio, kernel_size);

    // Mask gray image with Canny results and store it in dst matrix
    dst = cv::Scalar::all(0);
    gray.copyTo(dst, edges);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, dst);
    cv::waitKey(3);

    // Output modified video stream
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
