#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h" 
#include "boost/thread.hpp" 
#include "image_transport/image_transport.h"    
   
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/highgui/highgui.hpp>    
#include <iostream> 

  void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)
  {
    cv::Mat img;
    ROS_INFO("into call\n");
    try
    {
      img = cv_bridge::toCvShare(tem_msg, "mono8")->image;
      //img = cv_bridge::toCvShare(tem_msg, "bgr8")->image;
      cv::imshow("call_view", img);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());
    }
   
  }

int main(int argc,char** argv )
{
 ros::init(argc,argv,"image_sport_node");
 ros::NodeHandle  n;
 cv::namedWindow("call_view");
 cv::startWindowThread();
 image_transport::ImageTransport it(n);
 image_transport::Subscriber sub;
 sub= it.subscribe("/usb_cam/image_raw", 10, imageCallback);
 ROS_INFO("image \n");

 ros::spin();
 return 0;
}



