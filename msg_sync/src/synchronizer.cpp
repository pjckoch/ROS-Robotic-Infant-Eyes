#include <ros/ros.h>

#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <audio_proc/FFTData.h>
#include <audio_common_msgs/AudioData.h>


#include <image_transport/image_transport.h>

class SynchronizeNodes {

  ros::NodeHandle nh_;
  image_transport::Image_transport it_;
  
  ros::Publisher audio_pub;
  ros::Publisher fft_pub;
  image_transport::Publisher img_c_pub1;
  image_transport::Publisher img_c_pub2;
  image_transport::Publisher img_d_pub;
  image_transport::Publisher img_e_pub1;
  image_transport::Publisher img_e_pub2;
  image_transport::Publisher img_b_pub1;
  image_transport::Publisher img_b_pub2;






  void callback(const audioConstPtr& audio,
                const fftConstPtr& fft,
                const ImageConstPtr& img_color1,
                const ImageConstPtr& img_color2,
                const ImageConstPtr& img_depth,
                const ImageConstPtr& img_edges1,
                const ImageConstPtr& img_edges2,
                const ImageConstPtr& img_blobs1,
                const ImageConstPtr& img_blobs2
                ros::Publisher& audio_pub;
                ros::Publisher& fft_pub;
                image_transport::Publisher& img_c_pub1,
                image_transport::Publisher& img_c_pub2,
                image_transport::Publisher& img_d_pub,
                image_transport::Publisher& img_e_pub1,
                image_transport::Publisher& img_e_pub2,
                image_transport::Publisher& img_b_pub1,
                image_transport::Publisher& img_b_pub2) {

                // republish everything in sync
                audio_pub.publish(audio);
                fft_pub.publish(fft);
                img_c_pub1.publish(img_color1);
                img_c_pub2.publish(img_color2);
                img_d_pub.publish(img_depth);
                img_e_pub1.publish(img_edges1);
                img_e_pub2.publish(img_edges2);
                img_b_pub1.publish(img_blobs1);
                img_b_pub2.publish(img_blobs2);
  }


  public:
    SynchronizeNodes()
      : it_(nh_)
    {
        // get parameters from ROS parameter server        
        nh_.param<int>("~queue_size", queue_size_, 50);

        nh_.param<std::string>("/audio_topic", audio_top, "/audio");
        nh_.param<std::sting>("/fft_topic", fft_top, "/fftData");
        nh_.param<std::string>("/img_topic_left", img_c_top1, "/stereo/left/image_raw");
        nh_.param<std::string>("/img_topic_right", img_c_top2, "/stereo/right/image_raw");
        nh_.param<std::string>("img_topic_disparity", img_d_top, "/stereo/depth");
        nh_.param<std::string>("img_topic_edge_left", img_e_top1, "/stereo/left/edge_map");
        nh_.param<std::string>("img_topic_edge_right", img_e_top2, "/stereo/right/edge_map");
        nh_.param<std::string>("img_topic_blob_left", img_b_top1, "/stereo/left/blob");
        nh_.param<std::string>("img_topic_blob_right", img_b_top2, "/stereo/right/blob");
        // subscribers
        message_filters::Subscriber<AudioData> audio_sub(nh_, audio_top1, 1);
        message_filters::Subscriber<FFTData> fft_sub(nh_, fft_top. 1);
        message_filters::Subscriber<sensor_msgs::Image> img_c_sub1(it_, img_c_top1, 1);
        message_filters::Subscriber<sensor_msgs::Image> img_c_sub2(it_, img_c_top2, 1);
        message_filters::Subscriber<sensor_msgs::Image> img_d_sub(it_, img_d_top, 1);
        message_filters::Subscriber<sensor_msgs::Image> img_e_sub1(it_, img_e_top1, 1);
        message_filters::Subscriber<sensor_msgs::Image> img_e_sub2(it_, img_e_top2, 1);
        message_filters::Subscriber<sensor_msgs::Image> img_b_sub1(it_, img_b_top1, 1);
        message_filters::Subscriber<sensor_msgs::Image> img_b_sub2(it_, img_b_top2, 1);


  typedef message_filters::sync_policies::ApproximateTime<AudioData, FFTData,
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
      sensor_msgs::Image> MySyncPolicy;

  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;


  std::string s = "/sync";

        // publishers
        audio_pub = nh_.advertise(audio_top + s, 1);
        fft_pub = nh_.advertise(fft_top + s, 1);
        img_c_pub1 = it_.advertise(img_c_top1 + s, 1);
        img_c_pub2 = it_.advertise(img_c_top2 + s, 1);
        img_d_pub = it_.advertise(img_d_top + s, 1);
        img_e_pub1 = it_.advertise(img_e_top1 + s, 1);
        img_e_pub2 = it_.advertise(img_e_top2 + s, 1);
        img_b_pub1 = it_.advertise(img_b_top1 + s, 1);
        img_b_pub2 = it_.advertise(img_b_top2 + s, 1);

        sync_.reset(new Sync(MySyncPolicy(queue_size_),
                    audio_sub, fft_sub, img_c_sub1, img_c_sub2,
                    img_d_sub, img_e_sub1, img_e_sub2, img_b_sub1,
                    img_b_sub2);
        sync_->registerCallback(boost::bind(&SynchronizeNodes::callback, this, _1, _2);
   };
