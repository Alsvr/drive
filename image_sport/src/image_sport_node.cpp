#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "boost/thread.hpp"
#include "image_transport/image_transport.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

image_transport::Publisher pub;
image_transport::Subscriber sub;

void imageCallback(const sensor_msgs::ImageConstPtr &tem_msg)
{
  cv::Mat img;
  double fps = 1;
  ros::Time begin = ros::Time::now();
  static ros::Time next;
  fps = 1.0 / (begin - next).toSec();
  next = begin;

  ROS_INFO("cam fps is %2.0f \n", fps);
  try
  {
    img = cv_bridge::toCvShare(tem_msg, "mono8")->image;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    pub.publish(msg);
    //img = cv_bridge::toCvShare(tem_msg, "bgr8")->image;
    cv::imshow("call_view", img);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_sport_node");
  ros::NodeHandle n;
  cv::namedWindow("call_view");
  cv::startWindowThread();
  image_transport::ImageTransport it(n);

  sub = it.subscribe("/usb_cam/image_raw", 10, imageCallback);
  pub = it.advertise("/gray/image_raw", 10);

  ROS_INFO("image \n");
  ros::spin();
  // ros::MultiThreadedSpinner spinner(2); // Use 4 threads; // Use 4 threads
  // spinner.spin();
  return 0;
}
