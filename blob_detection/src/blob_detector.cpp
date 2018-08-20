#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

#include <ros/console.h>


static const std::string OPENCV_WINDOW = "Blob Detector";

class BlobDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  std::string input_topic_name;
  cv::SimpleBlobDetector::Params params;
  // two parameters need to be casted
  int tmp_minRepeatability;
  int tmp_blobColor;

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
      // get image input topic name from ROS parameter server
      nh_.getParam("image_topic_name", input_topic_name);
     
      // get Blob Parameters from ROS parameter server
      nh_.param("/thresholdStep", params.thresholdStep, float(10));
      nh_.param("/minThreshold", params.minThreshold, float(10));
      nh_.param("/maxThreshold", params.maxThreshold, float(200));
      nh_.param("/minRepeatability", tmp_minRepeatability, 2);
      nh_.param("/minDistBetweenBlobs", params.minDistBetweenBlobs, float(10));
      nh_.param("/filterByColor", params.filterByColor, false);
      nh_.param("/blobColor", tmp_blobColor, 0);
      nh_.param("/filterByArea", params.filterByArea, false);
      nh_.param("/minArea", params.minArea, float(25));
      nh_.param("/maxArea", params.maxArea, float(320*240));
      nh_.param("/filterByCircularity", params.filterByCircularity, false);
      nh_.param("/minCircularity", params.minCircularity, 0.6f);
      nh_.param("/maxCircularity", params.maxCircularity, (float)1e37);
      nh_.param("/filterByInertia", params.filterByInertia, false);
      nh_.param("/minInertiaRatio", params.minInertiaRatio, 0.1f);
      nh_.param("/maxInertiaRatio", params.maxInertiaRatio, (float)1e37);
      nh_.param("/filterByConvexity", params.filterByConvexity, false);
      nh_.param("/minConvexity", params.minConvexity, 0.95f);
      nh_.param("/maxConvexity", params.maxConvexity, (float)1e37);
      
      params.blobColor = uchar(tmp_blobColor);
      params.minRepeatability = ulong(tmp_minRepeatability);


      // set up subscriber and publisher
      image_sub_ = it_.subscribe(input_topic_name, 1,
        &BlobDetector::detect, this);
      image_pub_ = it_.advertise("/blobDetector/blob", 1);
    


      cv::namedWindow(OPENCV_WINDOW);
  }

  ~BlobDetector()
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
      cv::Mat dst;//, result(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);

      // init blob detector with parameters from ROS parameter server
      cv::Ptr<cv::SimpleBlobDetector> bd = cv::SimpleBlobDetector::create(params);
      // store keypoints in vector
      std::vector<cv::KeyPoint> keypoints;

      // detect keypoints
      bd->detect(cv_ptr->image, keypoints);
      cv::drawKeypoints(cv_ptr->image, keypoints, dst, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


      // color palette
      /*std::vector<cv::Vec3b> palette;
      for (int i = 0; i < 65536; i++)
          palette.push_back(cv::Vec3b((uchar)rand(), (uchar)rand(), (uchar)rand()));

      int i = 0;
      // draw large circles around blobs
      for (std::vector<cv::KeyPoint>::iterator k = keypoints.begin(); k != keypoints.end(); ++k, ++i)
      cv::circle(dst, k->pt, (int)k->size, palette[i % 65536]);
      */
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, dst);
      cv::waitKey(3);

      // Publish the edge map
      publishImage(dst, image_pub_);
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_detector");
    BlobDetector cd;
    ros::spin();
    return 0;
}
