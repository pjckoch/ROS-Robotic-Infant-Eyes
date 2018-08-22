#include <ros/ros.h>

#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>

class SynchronizeNodes {

  ros::NodeHandle nh_;
  image_transport::Image_transport it_;
  ros::Publisher audio_pub;
  ros::Publisher fft_pub;
  
  std::string audio_top = "/audio"
  std::sting fft_top = "/fft"
  std::string img_c_top1 = "/stereo/left/image_rect"
  std::string img_c_top2 = "/stereo/right/image_rect"
  std::string img_d_top = "/stereo/depth"
  std::string img_e_top1 = "/stereo/left/edge_map"
  std::string img_e_top2 = "/stereo/right/edge_map"
  std::string img_b_top1 = "/stereo/left/blob"
  std::string img_b_top2 = "/stereo/right/blob"

  message_filters::Subscriber<AudioData> audio_sub;
  message_filters::Subscriber<FFTData> fft_sub;
  message_filters::Subscriber<sensor_msgs::Image> img_c_sub1;
  message_filters::Subscriber<sensor_msgs::Image> img_c_sub2;
  message_filters::Subscriber<sensor_msgs::Image> img_d_sub;
  message_filters::Subscriber<sensor_msgs::Image> img_e_sub1;
  message_filters::Subscriber<sensor_msgs::Image> img_e_sub2;
  message_filters::Subscriber<sensor_msgs::Image> img_b_sub1;
  message_filters::Subscriber<sensor_msgs::Image> img_b_sub2;

  typedef message_filters::sync_policies::ApproximateTime<AudioData, FFTData,
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, 
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
      sensor_msgs::Image> MySyncPolicy;

  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  image_transport::Publisher img_c_pub1;
  image_transport::Publisher img_c_pub2;
  image_transport::Publisher img_d_pub;
  image_transport::Publisher img_e_pub1;
  image_transport::Publisher img_e_pub2;
  image_transport::Publisher img_b_pub1;
  image_transport::Publisher img_b_pub2;






  void callback(//const audioConstPtr& audio,
              //const fftConstPtr& fft,
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
        audio_sub.subscribe(nh_, audio_top, 10);
        fft_sub.subscribe(nh_, fft_top, 10);
        img_c_sub1.subscribe(nh_, img_c_top1, 1);
        img_c_sub2.subscribe(nh_, img_c_top2, 1);
        img_d_sub.subscribe(nh_, img_d_top, 1);
        img_e_sub1.subscribe(nh_, img_e_top1, 1);
        img_e_sub2.subscribe(nh_, img_e_top2, 1);
        img_b_sub1.subscribe(nh_, img_b_top1, 1);
        img_b_sub2.subscribe(nh_, img_b_top2, 1);

        sync_.reset(new Sync(MySyncPolicy(10),
                    audio_sub, fft_sub, img_c_sub1, img_c_sub2,
                    img_d_sub, img_e_sub1, img_e_sub2, img_b_sub1,
                    img_b_sub2);
        sync_->registerCallback(boost::bind(&SynchronizeNodes::callback, this, _1, _2);
   };
